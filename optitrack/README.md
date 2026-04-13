vision_engine
=============

OptiTrack V120:Trio tracking + 3D visualization pipeline.
Writers: Nguyen Nguyen (Alan), Sauman Raaj


What this does
--------------

Connects to the OptiTrack system over NatNet, reads rigid body positions
and quaternions in real time, and plots their 3D traces in a matplotlib
window. Supports tracking one or multiple shiny balls at the same time,
each with its own color trail.


Files you need
--------------

For the 3D trace to work, you need these files (and nothing else):

    vision_engine/
        cv/
            __init__.py              <- package init
            optitrack_client.py      <- NatNet client, talks to Motive
            transforms.py           <- quaternion/euler math, Y-up to Z-up
        scripts/
            run_3d_trace.py         <- the 3D trace renderer (main script)
        requirements.txt            <- pip dependencies

The other files (cs100_model.py, depth_estimator.py, scene_state_publisher.py,
run_calibration.py, run_cs100_tracker.py, config/, utils.py) are for the
CS-100 L-shape tracker and scene publisher. You don't need them for
the ball tracing.


Setup
-----

    pip install -r requirements.txt

That's it. You need numpy, opencv-python, matplotlib, and pyyaml.
Standard stuff, no weird dependencies.


How to run it
-------------

Demo mode (no hardware, simulates 2 balls):

    cd vision_engine
    python scripts/run_3d_trace.py --demo

Track a single rigid body on the live system:

    python scripts/run_3d_trace.py --body rigid_body_12 --ip 192.168.0.101

Track two specific bodies:

    python scripts/run_3d_trace.py --body rigid_body_12 rigid_body_13 --ip 192.168.0.101

Track everything that shows up:

    python scripts/run_3d_trace.py --all-bodies --ip 192.168.0.101

If you want Z-up instead of the raw Y-up from Motive, add --zup:

    python scripts/run_3d_trace.py --body rigid_body_12 --ip 192.168.0.101 --zup

Other options:

    --rate 60       update rate in Hz (default 30)
    --trail 5000    max trail points per body (default 3000)


How it works
------------

The OptiTrack client (optitrack_client.py) opens a socket to Motive's
NatNet stream, parses rigid body packets, and returns a dict of
RigidBodyState objects (name, id, position, quaternion, timestamp).

run_3d_trace.py calls get_rigid_bodies() on every animation frame,
pulls out the position for each body, appends it to a per-body deque,
and redraws the trail line + current dot in matplotlib. New bodies that
appear are auto-registered with their own color.

Coordinate system by default is raw Motive Y-up:
    X = right, Y = up (height), Z = toward cameras

With --zup it converts to robotics convention:
    X = right, Y = forward, Z = up


Notes
-----

- The IP default (192.168.0.101) is our lab OptiTrack server. Change it
  if yours is different.
- Body names like "rigid_body_12" come from Motive. Check Motive to see
  what your bodies are called.
- Close the matplotlib window or Ctrl+C to stop.
- Terminal prints position every 1 second so you can sanity check without
  staring at the plot.
