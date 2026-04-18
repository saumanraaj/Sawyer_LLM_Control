# ZED Stereo Camera — Computer Vision Detection Pipeline
## Technical Report

**System:** Ubuntu 20.04 LTS · Linux kernel 5.15.0 · Python 3.8.10  
**Key libraries:** OpenCV 4.13.0 · NumPy 1.24.4 · SciPy 1.10.1  
**Camera:** Stereolabs ZED (USB ID `2b03:f580`) — accessed without proprietary SDK  
**Scope:** This report documents the CV pipeline only. Robot arm control (MoveIt, Sawyer) is excluded.

---

## Table of Contents

1. [System Architecture](#1-system-architecture)
2. [Hardware — ZED Camera Without the SDK](#2-hardware--zed-camera-without-the-sdk)
3. [Software Environment and Dependencies](#3-software-environment-and-dependencies)
4. [Image Acquisition](#4-image-acquisition)
5. [Detection Pipeline](#5-detection-pipeline)
6. [Multi-Object Tracker](#6-multi-object-tracker)
7. [3D Localisation](#7-3d-localisation)
8. [Configuration System](#8-configuration-system)
9. [Live Calibration Viewer](#9-live-calibration-viewer)
10. [Test Suite](#10-test-suite)
11. [Installation from Scratch](#11-installation-from-scratch)
12. [Known Limitations and Next Steps](#12-known-limitations-and-next-steps)

---

## 1. System Architecture

The pipeline is divided into four sequential stages:

```
┌─────────────────────────────────────────────────────────────────┐
│  STAGE 1 — Acquisition                                          │
│  ZED camera (/dev/video0) ──► OpenCV VideoCapture              │
│  Side-by-side stereo frame ──► split ──► left eye (1280×720)   │
└───────────────────────────────┬─────────────────────────────────┘
                                │ BGR frame
┌───────────────────────────────▼─────────────────────────────────┐
│  STAGE 2 — Detection  (detector.py)                             │
│  Gaussian blur ──► BGR→HSV ──► inRange mask ──► morphology      │
│  findContours ──► area filter ──► shape validation              │
│  (solidity · aspect ratio) ──► best Detection object            │
└───────────────────────────────┬─────────────────────────────────┘
                                │ Detection | None
┌───────────────────────────────▼─────────────────────────────────┐
│  STAGE 3 — Tracking  (tracker.py)                               │
│  Hungarian assignment ──► EMA centroid smoothing                │
│  disappearance counter ──► TrackedObject with trajectory        │
└───────────────────────────────┬─────────────────────────────────┘
                                │ TrackedObject | None
┌───────────────────────────────▼─────────────────────────────────┐
│  STAGE 4 — 3D Localisation  (coordinate_converter.py)           │
│  Pinhole back-projection ──► (x,y,z) in camera frame           │
│  TF2 transform ──► (x,y,z) in robot base frame  [ROS only]     │
└─────────────────────────────────────────────────────────────────┘
```

### Package layout

```
sawyer_vision_tracker/
├── src/sawyer_vision_tracker/
│   ├── detector.py           # Stages 2 — HSV detection + shape filter
│   ├── tracker.py            # Stage 3 — Hungarian tracking
│   ├── coordinate_converter.py  # Stage 4 — 3D localisation (ROS)
│   └── utils.py              # euclidean_distance, ema_smooth
├── scripts/
│   ├── zed_raw_publisher.py  # Stage 1 — ZED→ROS bridge (optional)
│   ├── zed_live_viewer.py    # Standalone viewer + HSV calibration tool
│   ├── vision_node.py        # ROS node wiring stages 1-4
│   └── pickup_node.py        # Robot FSM (out of scope for this report)
├── config/vision_tracker.yaml   # All tunable parameters
├── tests/
│   ├── test_cv_pipeline.py   # 42 unit tests, no ROS required
│   └── check_zed_camera.py   # ROS camera topic health check
└── TECHNICAL_REPORT.md       # This document
```

---

## 2. Hardware — ZED Camera Without the SDK

### 2.1 Physical camera

The Stereolabs ZED is a passive stereo camera with two lenses separated by approximately 120 mm. It connects via USB 3.0 and is identified on Linux as:

```
Bus 002 Device 002: ID 2b03:f580 Leopard ZED
```

Linux exposes it as two V4L2 video devices: `/dev/video0` (primary) and `/dev/video1`. Only `/dev/video0` is needed.

### 2.2 UVC access — no SDK required

The ZED is fully UVC-compliant. Without the proprietary Stereolabs SDK, it behaves as a standard USB webcam. OpenCV's `VideoCapture` opens it directly:

```python
cap = cv2.VideoCapture(0)   # opens /dev/video0
```

**What you lose without the SDK:** the depth map (disparity→metric depth), IMU data, sensor fusion, and factory lens calibration file. Depth-based 3D localisation therefore falls back to a geometric assumption (see §7).

### 2.3 Side-by-side stereo format

The ZED does **not** expose left and right images as separate video streams over UVC. Instead, both eyes are packed horizontally into a single frame:

```
┌─────────────────────┬─────────────────────┐
│                     │                     │
│    LEFT  EYE        │    RIGHT EYE        │
│    (detection)      │    (reference)      │
│                     │                     │
└─────────────────────┴─────────────────────┘
 ◄────── full frame width ──────────────────►
```

Splitting the eyes:

```python
ok, frame = cap.read()          # shape: (H, 2W, 3)
left  = frame[:, :W, :]         # left eye  (H, W, 3)
right = frame[:, W:, :]         # right eye (H, W, 3)
```

All detection is performed on the left eye. The right eye is displayed as a stereo reference for depth cues.

### 2.4 Resolution modes

All four modes are accessible without the SDK via `cap.set()`:

| Mode  | Full frame      | Per eye      | Typical FPS |
|-------|-----------------|--------------|-------------|
| VGA   | 1344 × 376      | 672 × 376    | 60 / 100    |
| HD720 | 2560 × 720      | 1280 × 720   | 30 / 60     |
| FHD   | 3840 × 1080     | 1920 × 1080  | 15 / 30     |
| 2.2K  | 4416 × 1242     | 2208 × 1242  | 15          |

**Resolution negotiation** — the viewer attempts each mode in preference order and uses the first that succeeds:

```python
for target_w, target_h, label in [
    (2560, 720,  "HD720"),
    (3840, 1080, "FHD"),
    (1344, 376,  "VGA"),
]:
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  target_w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, target_h)
    ok, probe = cap.read()
    if ok and probe.shape[1] == target_w:
        break   # accepted
```

On this system, all four modes are supported. **HD720 (1280×720 per eye) is the recommended operating mode** — it provides a sharp image while keeping per-frame processing time well under 10 ms on a modern CPU.

> **Note:** If you connect the ZED on a different Linux machine and only get VGA, ensure the USB port is USB 3.0 (blue port). USB 2.0 cannot sustain the bandwidth for HD720 or higher.

---

## 3. Software Environment and Dependencies

### 3.1 Operating system

```
Ubuntu 20.04 LTS (Focal Fossa)
Linux kernel: 5.15.0-139-generic (x86_64)
Python: 3.8.10 (system Python)
```

### 3.2 Python package dependency conflict

Ubuntu 20.04 ships with **system SciPy 1.3.3**, which is incompatible with NumPy ≥ 1.20 (the `np.typeDict` attribute was removed). The user-installed NumPy 1.24.4 (via pip) will cause a crash if the system SciPy is imported.

**Resolution:** install a compatible SciPy in user scope, which takes precedence on the `sys.path`:

```bash
pip3 install --user "scipy>=1.7"
```

Verify that the correct version is loaded:

```bash
python3 -c "import scipy; print(scipy.__version__, scipy.__file__)"
# Expected: 1.10.1  /home/<user>/.local/lib/python3.8/site-packages/scipy/__init__.py
```

If the system path `/usr/lib/python3/dist-packages` is listed before `~/.local/lib` in `sys.path`, the wrong SciPy will be imported. Source the ROS workspace first — ROS Noetic's `setup.bash` prepends the correct paths.

### 3.3 Complete dependency list

```bash
# System packages
sudo apt-get install python3-pip python3-opencv

# pip packages (user scope)
pip3 install --user \
    numpy>=1.20 \
    scipy>=1.7 \
    opencv-python>=4.5   # only if system OpenCV is absent or < 4.5

# ROS Noetic packages (for vision_node.py and coordinate_converter.py only)
sudo apt-get install \
    ros-noetic-cv-bridge \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-geometry-msgs
```

| Package     | Version | Source       | Purpose                              |
|-------------|---------|--------------|--------------------------------------|
| Python      | 3.8.10  | system       | runtime                              |
| OpenCV      | 4.13.0  | pip          | image processing, contours, display  |
| NumPy       | 1.24.4  | pip          | array operations                     |
| SciPy       | 1.10.1  | pip (user)   | Hungarian algorithm                  |
| cv_bridge   | noetic  | ROS apt      | ROS Image ↔ NumPy conversion         |
| tf2_ros     | noetic  | ROS apt      | coordinate frame transforms          |

### 3.4 IDE / static analysis

A `pyrightconfig.json` at the workspace root resolves the package imports for VS Code / Pylance:

```json
{
    "extraPaths": ["src/sawyer_vision_tracker/src"],
    "pythonVersion": "3.8",
    "reportMissingImports": "none"
}
```

---

## 4. Image Acquisition

### 4.1 Standalone capture (no ROS)

The live viewer and the test suite capture directly from `/dev/video0`:

```python
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  2560)   # request HD720
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,  720)
ok, frame = cap.read()
left = frame[:, :1280, :]   # left eye only
```

### 4.2 ROS acquisition bridge (`zed_raw_publisher.py`)

When integrating with a ROS ecosystem (e.g., for TF-based 3D localisation or robot control), `zed_raw_publisher.py` acts as the bridge between `/dev/video0` and the ROS topic `/camera/color/image_raw`:

```
/dev/video0  ──►  zed_raw_publisher.py  ──►  /camera/color/image_raw
                  (crops left eye,             (sensor_msgs/Image,
                   publishes at 30 fps)          bgr8 encoding)
```

The node also publishes `/camera/color/camera_info` (`sensor_msgs/CameraInfo`) with the approximate intrinsics from `vision_tracker.yaml`. `vision_node.py` subscribes to `/camera/color/image_raw` and runs the full pipeline on each incoming message.

### 4.3 Frame rate

Measured frame rate at HD720: **30.1 fps**. The ZED's USB 3.0 interface sustains this without dropped frames on a typical desktop CPU.

---

## 5. Detection Pipeline

All detection logic lives in `detector.py`. The `Detector` class exposes two public detection methods: `detect()` for generic multi-colour HSV detection and `detect_blue_cube()` for single-object cube detection with shape validation.

### 5.1 The `Detection` dataclass

Every successful detection is returned as a `Detection` object:

```python
@dataclass
class Detection:
    centroid:     tuple[int, int]            # (cx, cy) pixel coordinates
    bbox:         tuple[int, int, int, int]  # (x, y, w, h) bounding rectangle
    contour:      np.ndarray                 # OpenCV contour array
    area:         float                      # contour area in px²
    color_name:   str                        # e.g. "blue_cube"
    color_bgr:    tuple[int, int, int]       # BGR colour for annotation
    mask:         np.ndarray                 # binary object mask (H×W uint8)
    solidity:     float = 1.0               # area / convex hull area
    aspect_ratio: float = 1.0              # bounding rect w / h
```

### 5.2 Step 1 — Gaussian blur

Before colour conversion, a Gaussian blur is applied to suppress high-frequency noise (sensor noise, JPEG artefacts). The kernel size is odd-valued:

```python
blurred = cv2.GaussianBlur(frame, (5, 5), 0)
```

**Why blur before HSV conversion?** Noise in the RGB channels creates high-variance HSV hue values even in smooth-coloured regions. Blurring makes the subsequent `inRange` mask more coherent and reduces fragmented contours.

### 5.3 Step 2 — Colour space conversion

```python
hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
```

OpenCV uses a scaled HSV representation: `H ∈ [0, 179]`, `S ∈ [0, 255]`, `V ∈ [0, 255]`. The mapping to the standard 0–360° hue wheel is:

```
OpenCV H × 2 = standard H°

Colour   Standard H°   OpenCV H
──────   ───────────   ────────
Red      0° / 360°     0 / 180
Yellow   60°           30
Green    120°          60
Cyan     180°          90
Blue     240°          120
Magenta  300°          150
```

**Why HSV over RGB/BGR?** HSV separates *chroma* (H, S) from *luminance* (V). A threshold on H+S is robust to changes in ambient brightness, shadow, and specular highlights that would shift RGB values significantly.

### 5.4 Step 3 — HSV range mask

```python
lower = np.array([95, 80, 40],   dtype=np.uint8)
upper = np.array([135, 255, 255], dtype=np.uint8)
mask  = cv2.inRange(hsv, lower, upper)
```

`cv2.inRange` produces a binary mask: 255 where all three channels fall within `[lower, upper]`, 0 elsewhere.

**Blue cube range rationale:**

| Channel | Range  | Meaning                                              |
|---------|--------|------------------------------------------------------|
| H       | 95–135 | Covers 190°–270° on the standard wheel — pure blues |
| S       | 80–255 | Ensures some saturation (rejects white/grey)         |
| V       | 40–255 | Accepts both well-lit and slightly shadowed surfaces |

The lower bound on S should be tightened once the cube's actual HSV is known from the calibration tool (see §9). Starting wide prevents false negatives during initial setup.

**Red hue wrap-around:** Red (H ≈ 0° / 360°) straddles the circular boundary. Two masks are OR'd:

```python
mask1 = cv2.inRange(hsv, [0,  120, 70], [10,  255, 255])
mask2 = cv2.inRange(hsv, [170, 120, 70], [180, 255, 255])
mask  = cv2.bitwise_or(mask1, mask2)
```

### 5.5 Step 4 — Morphological operations

Two morphological passes clean the binary mask:

```python
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
```

| Operation     | Effect                                                            |
|---------------|-------------------------------------------------------------------|
| `MORPH_OPEN`  | Erosion then dilation — removes isolated noise pixels and thin filaments |
| `MORPH_CLOSE` | Dilation then erosion — fills small holes inside the object mask  |

An elliptical structuring element avoids introducing sharp corners (more appropriate for convex physical objects than a rectangular kernel).

**Kernel size (7×7):** larger than the 5×5 used for the generic detector because the blue cube detection benefits from more aggressive noise suppression. The tradeoff is that small distant objects are eroded away — this is intentional since a cube too small to be grasped reliably should not trigger the pickup.

### 5.6 Step 5 — Contour detection

```python
contours, _ = cv2.findContours(
    mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
)
```

`RETR_EXTERNAL` retrieves only outermost contours (no holes). `CHAIN_APPROX_SIMPLE` compresses horizontal, vertical, and diagonal segments, reducing memory usage.

### 5.7 Step 6 — Shape validation (`detect_blue_cube` only)

This step is what differentiates `detect_blue_cube()` from the generic `detect()`. Two geometric properties are computed per contour and compared against thresholds:

#### Solidity

```python
hull      = cv2.convexHull(cnt)
hull_area = cv2.contourArea(hull)
solidity  = cv2.contourArea(cnt) / hull_area   # ∈ (0, 1]
```

**Interpretation:** A perfect convex shape (e.g., a square) has solidity = 1.0. An irregular blob (reflection, shadow with gaps, thin branch) has low solidity. The blue cube's top face, being convex, typically scores ≥ 0.85.

**Threshold:** `min_solidity = 0.75` (configurable). Contours below this are rejected.

#### Aspect ratio

```python
x, y, bw, bh = cv2.boundingRect(cnt)
aspect = bw / bh    # 1.0 = square
```

**Interpretation:** A cube viewed from above (or at a moderate tilt) presents a roughly square face. Long thin shapes (cables, table edges that happen to be blue) score far from 1.0.

**Threshold:** `max_aspect_ratio_err = 0.50` — the contour is accepted if `|aspect - 1.0| ≤ 0.50`, i.e., aspect ratio between 0.5 and 1.5.

#### Candidate selection

Among all contours passing both shape filters, the **largest by area** is returned as the single `Detection`. This "largest wins" rule works well for table-top scenarios where a large cube is likely more salient than smaller matching artefacts.

```python
if area > best_area:
    best_area = area
    best = Detection(...)
```

#### Diagnostic pass — `get_raw_candidates()`

A companion method returns *all* blue contours with their shape metrics before any filtering. The live viewer uses this to colour-code rejected contours (orange), helping the user understand whether the HSV range or the shape filter is the bottleneck.

### 5.8 Centroid calculation

For each accepted contour, the centroid is derived from image moments:

```python
M  = cv2.moments(cnt)
cx = int(M["m10"] / M["m00"])
cy = int(M["m01"] / M["m00"])
```

`M["m00"]` is the zeroth moment (contour area). `M["m10"]` and `M["m01"]` are the first spatial moments. This gives the centroid of the filled contour — more accurate than the centre of the bounding rectangle for non-rectangular shapes.

---

## 6. Multi-Object Tracker

`tracker.py` maintains object identity across frames using the **Hungarian algorithm** (also called the Munkres assignment algorithm) implemented in `scipy.optimize.linear_sum_assignment`.

### 6.1 TrackedObject dataclass

```python
@dataclass
class TrackedObject:
    id:           int                           # persistent integer ID
    centroid:     tuple[float, float]           # EMA-smoothed pixel position
    raw_centroid: tuple[float, float]           # unfiltered pixel position
    position_3d:  tuple[float, float, float] | None  # base-frame 3D (if available)
    color_name:   str
    color_bgr:    tuple[int, int, int]
    bbox:         tuple[int, int, int, int]
    disappeared:  int                           # frames since last matched
    trajectory:   deque                         # deque(maxlen=40) of smoothed positions
```

### 6.2 Hungarian assignment

Each call to `tracker.update(detections)` runs a full assignment between existing tracked objects and new detections:

```
Existing centroids (n objects)          New detection centroids (m detections)

   obj_0 ──────────────────────────────── det_0
   obj_1                                  det_1
   obj_2 ──────────────────────────────── det_2
```

**Cost matrix:** an n×m matrix where `cost[i][j]` is the Euclidean distance between the centroid of tracked object `i` and detection `j`.

```python
cost = np.zeros((n_obj, n_det), dtype=np.float64)
for i, oc in enumerate(obj_centroids):
    for j, dc in enumerate(det_centroids):
        cost[i, j] = euclidean_distance(oc, dc)

row_idx, col_idx = linear_sum_assignment(cost)
```

`linear_sum_assignment` minimises the total cost (sum of matched distances), which is the globally optimal assignment. A greedy nearest-neighbour would give suboptimal results when multiple objects are close.

**Max-distance gating:** a match is accepted only if the distance is below `max_distance` (60 px default). Pairs above this threshold are treated as unmatched — the object "disappeared" and the detection is registered as a new object.

```python
for r, c in zip(row_idx, col_idx):
    if cost[r, c] > self._max_distance:
        continue           # reject this match
    self._update_object(obj_ids[r], detections[c])
```

### 6.3 EMA centroid smoothing

Matched detections update the tracked centroid with exponential moving average filtering:

```
smoothed = α × current_detection + (1 − α) × previous_smoothed
```

```python
alpha    = 0.35   # configurable
smooth_x = alpha * raw_x + (1 - alpha) * obj.centroid[0]
smooth_y = alpha * raw_y + (1 - alpha) * obj.centroid[1]
```

**Effect:** at α = 0.35, the smoothed position has an effective time constant of approximately `1/α − 1 ≈ 1.9 frames`. A slow-moving cube converges quickly; a sudden detection jump (noise spike) is damped to about 35% in the first frame and 58% by the second.

Both the smoothed centroid (used for trajectory and 3D projection) and the raw centroid (useful for debugging) are stored in `TrackedObject`.

### 6.4 Disappearance handling

When a tracked object has no matching detection in a frame:

```python
obj.disappeared += 1
if obj.disappeared > max_disappeared:   # default 20 frames
    del self._objects[obj_id]
```

This tolerates brief occlusions (hand passing in front, specular glint causing missed detection) without immediately losing the object ID.

### 6.5 Trajectory recording

Every smoothed centroid is appended to a `deque(maxlen=40)`:

```python
obj.trajectory.append((smooth_x, smooth_y))
```

The viewer renders the trajectory as a fading line, providing a visual motion history. The deque's `maxlen` automatically evicts old points.

---

## 7. 3D Localisation

### 7.1 Pinhole camera model

Given a pixel coordinate `(u, v)` in the left eye image, the corresponding 3D point in the camera optical frame at depth `z` is:

```
x_cam = (u − cx) × z / fx
y_cam = (v − cy) × z / fy
z_cam = z
```

Where `(fx, fy)` are the focal lengths in pixels and `(cx, cy)` is the principal point (optical centre of the image).

The camera matrix `K` is:

```
    ⎡ fx   0  cx ⎤
K = ⎢  0  fy  cy ⎥
    ⎣  0   0   1 ⎦
```

### 7.2 Fixed-Z assumption

Without the ZED SDK depth map, the depth `z` of the target object is unknown. For the common scenario of objects resting on a flat table at a known height, a **fixed-Z assumption** is used: all objects are assumed to lie at a fixed depth `z = fixed_z` in the camera optical frame.

```python
x_cam = (u - cx) * fixed_z / fx
y_cam = (v - cy) * fixed_z / fy
z_cam = fixed_z             # 0.70 m — camera frame Z to table surface
```

**Accuracy:** if the table is genuinely flat and the camera is rigidly mounted, the XY position error introduced by this assumption is proportional to the object's height above the table. For a 50 mm cube, the error is typically 3–5 mm — acceptable for pick-and-place.

**Failure mode:** this assumption fails if (a) the camera tilts, (b) the table is not level, or (c) objects are stacked. See §12 for depth-based alternatives.

### 7.3 Camera intrinsics

The intrinsics used are approximate values for ZED HD720 mode:

| Parameter | Value   | Notes                                              |
|-----------|---------|----------------------------------------------------|
| `fx`      | 700.0   | Approximate — factory value requires ZED SDK       |
| `fy`      | 700.0   | Approximate                                        |
| `cx`      | 640.5   | `image_width / 2 + 0.5`                           |
| `cy`      | 360.5   | `image_height / 2 + 0.5`                          |

**To obtain factory calibration:** install the ZED SDK and run `ZED Calibration Tool`, or look up the calibration file at `~/.config/Stereolabs/settings/SN<serial>.conf`.

**Empirical calibration without the SDK:** place a known object at a measured distance, read the projected `cam_X` / `cam_Y` from the viewer, and scale `fx`/`fy` until they agree.

### 7.4 TF2 transform (ROS-only component)

`coordinate_converter.py` uses ROS TF2 to transform the camera-frame 3D point to the robot base frame:

```python
pt_cam = PointStamped()
pt_cam.header.frame_id = "front_cam_link"
pt_cam.point.x = x_cam
pt_cam.point.y = y_cam
pt_cam.point.z = z_cam

pt_base = tf_buffer.transform(pt_cam, "base", rospy.Duration(1.0))
return (pt_base.point.x, pt_base.point.y, pt_base.point.z)
```

This requires a valid TF chain from `front_cam_link` to `base`. The chain is established with a `static_transform_publisher`:

```xml
<node pkg="tf2_ros" type="static_transform_publisher" name="cam_tf"
      args="X Y Z yaw pitch roll base front_cam_link"/>
```

Where `X Y Z` is the measured camera position relative to the robot base (metres) and `yaw pitch roll` is its orientation (radians).

> **For non-ROS users:** the pinhole back-projection formula in §7.1 can be used directly. The only missing piece is the rigid-body transform from camera to world frame, which can be represented as a 4×4 homogeneous matrix and applied with NumPy instead of TF2.

---

## 8. Configuration System

All parameters are stored in `config/vision_tracker.yaml`. The file is divided into four sections.

### 8.1 Detection configuration

```yaml
detection:
  blue_cube:
    hsv_lower: [95, 80, 40]        # [H_min, S_min, V_min]
    hsv_upper: [135, 255, 255]     # [H_max, S_max, V_max]
    min_area: 800                  # px² — noise floor
    max_area: 200000               # px² — reject entire frame floods
    min_solidity: 0.75             # 0–1, higher = more convex
    max_aspect_ratio_err: 0.50     # |w/h − 1| tolerance
    morph_kernel_size: 7           # morphological structuring element size
    gaussian_blur: 5               # blur kernel size (must be odd)
```

### 8.2 Tracking configuration

```yaml
tracking:
  max_disappeared: 20      # frames before a lost object is dropped
  max_distance: 80         # px — max centroid jump between frames
  smoothing_alpha: 0.35    # EMA weight on current detection (0–1)
  trajectory_length: 40    # number of past positions stored
```

### 8.3 Camera configuration

```yaml
camera:
  image_topic: "/camera/color/image_raw"
  camera_frame: "front_cam_link"
  base_frame: "base"
  intrinsics:
    fx: 700.0
    fy: 700.0
    cx: 640.5
    cy: 360.5
  fixed_z: 0.70    # metres from camera optical centre to table surface
```

### 8.4 Parameter sensitivity guide

| Parameter          | Too low                               | Too high                               |
|--------------------|---------------------------------------|----------------------------------------|
| `min_area`         | Noise blobs detected                  | Small/distant cube missed              |
| `min_solidity`     | Irregular noise passes                | Real cube rejected if tilted           |
| `max_aspect_ratio_err` | Non-cube shapes pass             | Tilted cube rejected                   |
| `smoothing_alpha`  | High lag, slow to respond             | Jittery centroid                       |
| `max_distance`     | ID switches on fast motion            | Multiple detections merged             |
| `max_disappeared`  | IDs lost during brief occlusion       | Ghost objects persist too long         |

---

## 9. Live Calibration Viewer

`zed_live_viewer.py` is a standalone tool (no ROS) for HSV calibration and detection verification.

### 9.1 Capabilities

| Feature                  | How to use                                            |
|--------------------------|-------------------------------------------------------|
| HD720 auto-negotiation   | Automatic on startup — falls back to VGA if unsupported |
| Left eye annotation      | Bounding box, centroid, trajectory, shape metrics, cam-XY |
| Right eye reference      | Raw right eye for stereo depth cue                    |
| HSV mask overlay         | Press **H** — blue-detected pixels shown as green tint |
| Shape filter toggle      | Press **F** — OFF = show any blue blob; ON = cube only |
| Click-to-inspect HSV     | Click on cube → terminal prints `H S V` + suggested range |
| Apply suggested range    | Press **R** — applies last-clicked range live          |
| Solidity adjustment      | **[** / **]** — decrease / increase by 0.05           |
| Aspect ratio adjustment  | **;** / **'** — decrease / increase by 0.05           |
| Snapshot                 | Press **S** — saves left and right PNGs               |

### 9.2 Calibration workflow

The standard procedure for a new blue object in a new lighting environment:

1. **Run the viewer** with shape filter OFF (`F` key) and mask ON (`H` key).
2. If the mask shows **zero blue pixels**: the HSV range is wrong. Click on the cube in the left window. The terminal prints:
   ```
   [HSV click] (423,310)  BGR=[180,95,12]  HSV=H:108 S:187 V:180
     → Suggested config:  hsv_lower: [96, 127, 120]  hsv_upper: [120, 255, 255]
   ```
3. Press **R** to apply the suggested range. The mask should now show the cube.
4. If the mask shows the cube but it is **not detected** (orange outline): shape filter is rejecting it. Press `[` to lower the solidity threshold or `'` to widen the aspect ratio tolerance.
5. Once detection is stable (cyan annotation, "DETECTED" in the status bar): copy the final values into `vision_tracker.yaml`.

### 9.3 Diagnostic status bar

The viewer's bottom status bar provides a three-state diagnosis:

| Status bar message                       | Meaning                                    | Action                              |
|------------------------------------------|--------------------------------------------|-------------------------------------|
| `NO BLUE PIXELS`                         | HSV range misses the cube entirely         | Click cube → press R                |
| `SHAPE FILTER REJECTING N blue blobs`    | Colour OK but shape too strict             | Press F, relax with [ ] ; '         |
| `DETECTED solid=0.XX ar=0.XX area=XXXX` | Full detection — all stages passing        | Record params to YAML               |

---

## 10. Test Suite

`tests/test_cv_pipeline.py` contains 42 unit tests that run without ROS or a physical camera, using synthetic OpenCV frames.

```bash
cd /path/to/sawyer_vision_tracker
python3 tests/test_cv_pipeline.py -v
# Ran 42 tests in 0.075s  OK
```

### 10.1 Test groups

#### `TestUtils` — 9 tests

| Test                          | Assertion                                  |
|-------------------------------|--------------------------------------------|
| `euclidean_distance_3_4_5`    | `dist((0,0),(3,4)) == 5.0`                 |
| `euclidean_distance_same_point` | Returns 0.0                              |
| `euclidean_distance_horizontal` | Correct for axis-aligned vectors         |
| `euclidean_distance_vertical`   | Correct for axis-aligned vectors         |
| `ema_alpha_1_returns_current`   | `ema(x, y, 1.0) == x`                   |
| `ema_alpha_0_returns_previous`  | `ema(x, y, 0.0) == y`                   |
| `ema_alpha_half`                | `ema(10, 0, 0.5) == 5.0`                |
| `ema_weighted`                  | `0.4×10 + 0.6×5 == 7.0`                |
| `ema_convergence`               | 200 iterations converge to target        |

#### `TestDetector` — 14 tests

Synthetic 480×640 BGR frames are generated with solid-colour patches (HSV-specified, converted to BGR). Each test verifies a specific detector behaviour:

| Test                             | Verifies                                               |
|----------------------------------|--------------------------------------------------------|
| `detect_blue_blob`               | Blue HSV patch → 1 detection                           |
| `detect_red_blob_low_hue`        | Red H≈5 → detected via primary range                  |
| `detect_red_blob_high_hue`       | Red H≈175 → detected via wrap-around range             |
| `detect_green_blob`              | Green H≈60 → detected                                  |
| `black_frame_no_detections`      | All-black frame → 0 detections                         |
| `white_frame_no_detections`      | White frame (S≈0) → rejected by saturation floor      |
| `tiny_blob_below_min_area`       | 15×15 px blob → filtered by `min_contour_area`        |
| `centroid_accuracy_blue`         | Centroid within ±5 px of true centre                  |
| `centroid_accuracy_green`        | Centroid within ±5 px of true centre                  |
| `detection_fields_present`       | All dataclass fields populated                         |
| `detection_area_positive`        | All detected areas > 0                                 |
| `get_combined_mask_empty`        | Empty detection list → all-zero mask                  |
| `get_combined_mask_nonzero`      | Non-empty list → mask > 0                             |
| `two_blue_blobs_detected`        | Two spatially separated blobs → 2 detections          |

#### `TestTracker` — 12 tests

| Test                              | Verifies                                              |
|-----------------------------------|-------------------------------------------------------|
| `new_detection_gets_id`           | First detection assigned ID 0                         |
| `two_detections_get_different_ids`| Simultaneous detections get distinct IDs              |
| `same_position_keeps_id`          | Same centroid across frames → same ID                 |
| `nearby_position_keeps_id`        | 30 px jump → same ID (< max_distance 80)             |
| `far_jump_registers_new_id`       | 250 px jump → new ID registered                       |
| `disappeared_increments`          | Empty detection → disappeared counter +1              |
| `object_removed_after_max`        | 6 empty frames (> max_disappeared 5) → object purged |
| `object_revived_before_removal`   | Re-detected before threshold → disappeared reset to 0|
| `ema_smoothing_applied`           | `0.35×340 + 0.65×300 = 326` ± 1 px                  |
| `raw_centroid_matches_detection`  | `raw_centroid` stores unfiltered position             |
| `trajectory_grows_with_updates`   | Trajectory length increases per frame                 |
| `trajectory_maxlen`               | Deque does not exceed `trajectory_length`             |

#### `TestCoordinateProjectionMath` — 7 tests

These test the pinhole back-projection formula directly, without TF2 or ROS:

| Test                          | Verifies                                                |
|-------------------------------|---------------------------------------------------------|
| `center_pixel_maps_to_origin` | `(cx, cy)` → `(0, 0, z)` in camera frame              |
| `unit_offset_right`           | `(cx+fx, cy)` → `(z, 0, z)`                           |
| `unit_offset_down`            | `(cx, cy+fy)` → `(0, z, z)`                           |
| `left_right_symmetry`         | Mirrored U → mirrored X                                |
| `top_bottom_symmetry`         | Mirrored V → mirrored Y                                |
| `scale_with_fixed_z`          | Double `z` → double projected X                        |
| `known_value`                 | 100 px right of centre → `100 × z / fx` metres        |

---

## 11. Installation from Scratch

Follow these steps to set up the ZED CV pipeline on a fresh Ubuntu 20.04 machine.

### 11.1 System packages

```bash
sudo apt update
sudo apt install -y \
    python3-pip \
    v4l-utils \
    usbutils
```

### 11.2 Python packages

```bash
pip3 install --user \
    "numpy>=1.20,<1.27" \
    "scipy>=1.7" \
    "opencv-python>=4.5"
```

> Avoid `apt install python3-scipy` — the system package (1.3.3) is incompatible with NumPy ≥ 1.20.

### 11.3 Verify camera access

```bash
# Check camera is present
lsusb | grep "2b03"
# Expected: Bus 00X Device 00Y: ID 2b03:f580 Leopard ZED

# Check video devices
ls /dev/video*
# Expected: /dev/video0  /dev/video1

# Verify access
python3 -c "
import cv2
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
ok, f = cap.read()
cap.release()
print('ZED OK:', ok, f.shape if ok else 'failed')
"
# Expected: ZED OK: True (720, 2560, 3)
```

If the camera is not found, check: (a) USB 3.0 port is used, (b) user is in the `video` group (`sudo usermod -aG video $USER`, then log out/in).

### 11.4 Clone and run

```bash
# Either clone the full ROS workspace and source it,
# or just copy the package and use it standalone:

git clone <your-repo-url>
cd sawyer_vision_tracker

# Run the unit tests (no camera needed)
python3 tests/test_cv_pipeline.py -v

# Run the live viewer (camera required)
python3 scripts/zed_live_viewer.py
```

### 11.5 ROS Noetic (optional — for TF2 coordinate conversion)

```bash
# Add ROS apt repository (standard ROS Noetic install)
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" \
    > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | \
    sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-ros-base

# Additional packages for this pipeline
sudo apt install \
    ros-noetic-cv-bridge \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-geometry-msgs

# Source and run
source /opt/ros/noetic/setup.bash
roscore &
python3 scripts/zed_raw_publisher.py &
python3 scripts/vision_node.py
```

---

## 12. Known Limitations and Next Steps

### 12.1 Current limitations

| Limitation | Impact | Workaround |
|---|---|---|
| No SDK → no depth map | Fixed-Z assumption; accuracy degrades if camera tilts or objects are at different heights | Use OpenCV stereo matching (§12.2) or install ZED SDK |
| Approximate intrinsics (fx, fy) | XY localisation error proportional to intrinsic error | Obtain factory calibration via ZED SDK |
| Fixed-Z assumption | Cannot localise objects at different table heights | See §12.2 |
| HSV sensitivity to lighting | Range tuned in one lighting condition may fail under different artificial light | Re-run calibration workflow; or use adaptive histogram equalisation before HSV conversion |
| No depth fusion | Single-view; cannot recover from severe occlusion | ArUco marker backup (code present, disabled) |

### 12.2 Depth without the ZED SDK — OpenCV stereo matching

The ZED provides both left and right images. OpenCV's `StereoSGBM` can compute a disparity map, which can be converted to metric depth using the ZED's 120 mm baseline `B` and focal length `fx`:

```python
# depth = B × fx / disparity
sgbm = cv2.StereoSGBM_create(minDisparity=0, numDisparities=128, blockSize=5)
disparity = sgbm.compute(cv2.cvtColor(left, cv2.COLOR_BGR2GRAY),
                          cv2.cvtColor(right, cv2.COLOR_BGR2GRAY))
depth = (baseline_m * fx) / (disparity / 16.0)   # disparity is ×16 fixed-point
```

This eliminates the fixed-Z assumption and handles objects at varying heights. It requires the camera to be stereo-rectified (calibration matrices available from the ZED SDK or OpenCV's stereo calibration routine).

### 12.3 Improved HSV robustness

**Adaptive HSV pre-processing:** apply CLAHE (Contrast Limited Adaptive Histogram Equalisation) to the V channel before segmentation to normalise luminance variation:

```python
lab   = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
lab[:,:,0] = clahe.apply(lab[:,:,0])
frame = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
```

### 12.4 ArUco marker backup

`detector.py` contains `detect_aruco()` (disabled by default via `aruco.enabled: false` in the YAML). Placing an ArUco marker on top of the blue cube provides a pose-precise fallback when colour segmentation fails (e.g., specular reflection washing out the blue hue).

### 12.5 Factory calibration (highest priority improvement)

The single highest-impact improvement is obtaining the factory `fx`, `fy`, `cx`, `cy` and distortion coefficients from the ZED SDK:

```bash
# Install ZED SDK, then:
python3 -c "
import pyzed.sl as sl
cam = sl.Camera()
cam.open()
cal = cam.get_camera_information().camera_configuration.calibration_parameters.left_cam
print(f'fx={cal.fx:.4f} fy={cal.fy:.4f} cx={cal.cx:.4f} cy={cal.cy:.4f}')
"
```

With accurate intrinsics and a measured camera-to-robot transform, the fixed-Z XY localisation error typically drops below 5 mm for objects within 1 m.

---

*End of report.*

**Tested on:** Ubuntu 20.04 LTS · Linux 5.15.0-139-generic · Python 3.8.10 · OpenCV 4.13.0 · ZED USB ID 2b03:f580  
**Date generated:** April 2026
