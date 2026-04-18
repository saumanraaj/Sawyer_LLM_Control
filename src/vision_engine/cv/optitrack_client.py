"""
vision_engine/cv/optitrack_client.py — NatNet Protocol Client for OptiTrack V120:Trio

Writers: Nguyen Nguyen (Alan), Sauman Raaj

Implements a pure-Python NatNet 3.x/4.x client that connects to OptiTrack Motive
and receives rigid body tracking data over UDP.

Architecture
------------
The OptiTrack V120:Trio is a self-contained 3-camera tracking bar. It connects via
USB to a Windows PC running Motive, which streams data via NatNet over the network.

NatNet uses two UDP channels:
  - Command socket (unicast, default port 1510): request/response for server info
    and model definitions.
  - Data socket (multicast or unicast, default port 1511): frame data broadcast
    (rigid body poses, markers, etc.).

Coordinate System
-----------------
OptiTrack/Motive defaults to Y-up. This client converts to Z-up by default:
    x_zup =  x_yup
    y_zup = -z_yup
    z_zup =  y_yup

Set convert_to_zup=False to receive raw Y-up values (useful for debugging).

Usage
-----
    from cv.optitrack_client import OptiTrackClient

    client = OptiTrackClient(server_ip="192.168.0.101")
    client.start()

    bodies = client.get_rigid_bodies()
    for name, state in bodies.items():
        print(f"{name}: pos={state.position}")

    client.stop()
"""

import socket
import struct
import threading
import time
import numpy as np
from dataclasses import dataclass
from typing import Dict, Optional, Callable

from cv.transforms import position_yup_to_zup, quaternion_yup_to_zup


# --- Data Structures ---

@dataclass
class RigidBodyState:
    """State of a single tracked rigid body at a moment in time."""
    name: str
    id: int
    position: np.ndarray       # (3,) — XYZ in meters
    quaternion: np.ndarray     # (4,) — (x, y, z, w) Hamilton convention
    timestamp: float           # Unix time (seconds)
    tracking_valid: bool       # Motive's tracking validity flag


# --- NatNet Message Types ---

NAT_CONNECT            = 0
NAT_SERVERINFO         = 1
NAT_REQUEST            = 2
NAT_RESPONSE           = 3
NAT_REQUEST_MODELDEF   = 4
NAT_MODELDEF           = 5
NAT_REQUEST_FRAMEOFDATA = 6
NAT_FRAMEOFDATA        = 7
NAT_UNRECOGNIZED       = 100


# --- NatNet Client ---

class OptiTrackClient:
    """
    Pure-Python NatNet client for receiving rigid body data from OptiTrack Motive.

    Tested with Motive 3.3.4.1 (NatNet 4.2) and V120:Trio.
    """

    def __init__(
        self,
        server_ip: str,
        local_ip: str = "0.0.0.0",
        multicast_ip: str = "239.255.42.99",
        command_port: int = 1510,
        data_port: int = 1511,
        convert_to_zup: bool = True,
    ):
        self.server_ip = server_ip
        self.local_ip = local_ip
        self.multicast_ip = multicast_ip
        self.command_port = command_port
        self.data_port = data_port
        self.convert_to_zup = convert_to_zup

        # Server info (populated after start)
        self.server_app_name: str = ""
        self.server_app_version: tuple = (0, 0, 0, 0)
        self.natnet_version: tuple = (0, 0, 0, 0)

        # Rigid body name lookup: id -> name
        self._id_to_name: Dict[int, str] = {}

        # Latest rigid body states
        self._rigid_bodies: Dict[str, RigidBodyState] = {}
        self._lock = threading.Lock()

        # Counters
        self._frame_count: int = 0
        self._raw_data_packets: int = 0
        self._parse_errors: int = 0
        self._first_parse_error: str = ""

        # Callback
        self._callback: Optional[Callable] = None

        # Sockets and threads
        self._command_socket: Optional[socket.socket] = None
        self._data_socket: Optional[socket.socket] = None
        self._command_thread: Optional[threading.Thread] = None
        self._data_thread: Optional[threading.Thread] = None
        self._running = False

    # --- Public API ---

    def start(self) -> None:
        """Connect to Motive and begin receiving data."""
        print(f"[OptiTrack] Connecting to {self.server_ip}...")

        # Command socket (unicast)
        self._command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._command_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._command_socket.bind(("", 0))
        self._command_socket.settimeout(3.0)

        # Data socket (bind INADDR_ANY to receive both unicast and multicast)
        self._data_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._data_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._data_socket.bind(("", self.data_port))

        # Join multicast group (harmless if Motive uses unicast)
        try:
            mreq = struct.pack(
                "4s4s",
                socket.inet_aton(self.multicast_ip),
                socket.inet_aton("0.0.0.0"),
            )
            self._data_socket.setsockopt(
                socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq
            )
        except OSError as e:
            print(f"[OptiTrack] Multicast join failed ({e}), unicast only.")

        self._data_socket.settimeout(1.0)

        # Start listener threads
        self._running = True
        self._data_thread = threading.Thread(
            target=self._data_listener, daemon=True, name="NatNet-Data"
        )
        self._command_thread = threading.Thread(
            target=self._command_listener, daemon=True, name="NatNet-Command"
        )
        self._data_thread.start()
        self._command_thread.start()

        # Send connection request
        self._send_command(NAT_CONNECT, b"")

        # Wait for server info
        time.sleep(0.5)
        if self.server_app_name:
            version_str = ".".join(str(v) for v in self.server_app_version)
            natnet_str = ".".join(str(v) for v in self.natnet_version)
            print(f"[OptiTrack] Server: {self.server_app_name} v{version_str}, "
                  f"NatNet v{natnet_str}")
        else:
            print("[OptiTrack] Warning: No server response. "
                  "Check IP and network connectivity.")

        # Request model definitions (rigid body names)
        self._send_command(NAT_REQUEST_MODELDEF, b"")
        time.sleep(0.5)

        if self._id_to_name:
            for rb_id, name in self._id_to_name.items():
                print(f"[OptiTrack] Rigid body: '{name}' (id={rb_id})")
        else:
            print("[OptiTrack] Warning: No rigid body definitions received.")

        # Wait for first frame data
        time.sleep(0.5)
        if self._raw_data_packets == 0:
            print("[OptiTrack] Warning: No data packets received on port "
                  f"{self.data_port}.")
            print("  Check Motive: View > Data Streaming > Broadcast Frame Data = ON")
            print("  If using Unicast, ensure this machine's IP is in Motive's "
                  "unicast target list.")
        else:
            print(f"[OptiTrack] Data flowing: {self._raw_data_packets} packets, "
                  f"{self._frame_count} frames parsed")
            if self._parse_errors > 0:
                print(f"[OptiTrack] Parse errors: {self._parse_errors} "
                      f"(first: {self._first_parse_error})")

    def stop(self) -> None:
        """Stop receiving data and close sockets."""
        self._running = False

        if self._data_thread and self._data_thread.is_alive():
            self._data_thread.join(timeout=2.0)
        if self._command_thread and self._command_thread.is_alive():
            self._command_thread.join(timeout=2.0)

        for sock in (self._data_socket, self._command_socket):
            if sock:
                try:
                    sock.close()
                except Exception:
                    pass

        print(f"[OptiTrack] Disconnected. "
              f"Packets={self._raw_data_packets}, "
              f"Frames={self._frame_count}, "
              f"Errors={self._parse_errors}")

    def get_rigid_bodies(self) -> Dict[str, RigidBodyState]:
        """Get the latest rigid body states (thread-safe copy)."""
        with self._lock:
            return dict(self._rigid_bodies)

    def get_frame_count(self) -> int:
        """Return the number of successfully parsed frames."""
        return self._frame_count

    def set_callback(self, fn: Callable[[Dict[str, RigidBodyState]], None]) -> None:
        """Set a callback called on every new frame (runs in data thread)."""
        self._callback = fn

    # --- Socket Communication ---

    def _send_command(self, msg_type: int, payload: bytes) -> None:
        """Send a NatNet command message to the server."""
        header = struct.pack("<HH", msg_type, len(payload))
        self._command_socket.sendto(
            header + payload,
            (self.server_ip, self.command_port),
        )

    def _data_listener(self) -> None:
        """Background thread: receive data packets on the data socket."""
        while self._running:
            try:
                data, addr = self._data_socket.recvfrom(65536)
                if len(data) < 4:
                    continue
                self._raw_data_packets += 1

                msg_type = struct.unpack_from("<H", data, 0)[0]
                if msg_type == NAT_FRAMEOFDATA:
                    try:
                        self._parse_frame_data(data, 4)
                    except (struct.error, IndexError, ValueError) as e:
                        self._parse_errors += 1
                        if self._parse_errors <= 3:
                            self._first_parse_error = str(e)
                            print(f"[OptiTrack] Frame parse error #{self._parse_errors}: {e}")

            except socket.timeout:
                continue
            except OSError:
                if self._running:
                    continue
                break

    def _command_listener(self) -> None:
        """Background thread: receive command responses on the command socket."""
        while self._running:
            try:
                data, addr = self._command_socket.recvfrom(65536)
                if len(data) < 4:
                    continue
                msg_type = struct.unpack_from("<H", data, 0)[0]

                if msg_type == NAT_SERVERINFO:
                    self._parse_server_info(data, 4)
                elif msg_type == NAT_MODELDEF:
                    self._parse_model_def(data, 4)
            except socket.timeout:
                continue
            except OSError:
                if self._running:
                    continue
                break

    # --- Server Info Parser ---

    def _parse_server_info(self, data: bytes, offset: int) -> None:
        """Parse NAT_SERVERINFO: app name + version + NatNet version."""
        try:
            app_name_raw = data[offset:offset + 256]
            null_idx = app_name_raw.find(b'\x00')
            self.server_app_name = (
                app_name_raw[:null_idx].decode("utf-8", errors="replace")
                if null_idx >= 0
                else app_name_raw.decode("utf-8", errors="replace").strip()
            )
            offset += 256
            self.server_app_version = struct.unpack_from("4B", data, offset)
            offset += 4
            self.natnet_version = struct.unpack_from("4B", data, offset)
        except (struct.error, IndexError):
            pass

    # --- Model Definition Parser ---

    def _parse_model_def(self, data: bytes, offset: int) -> None:
        """Parse NAT_MODELDEF response. Handles dataset types 0-6 (NatNet 4.2)."""
        try:
            natnet_major = self.natnet_version[0] if self.natnet_version[0] > 0 else 4

            num_datasets = struct.unpack_from("<i", data, offset)[0]
            offset += 4

            if num_datasets < 0 or num_datasets > 100:
                print(f"[OptiTrack] Model def: bad num_datasets={num_datasets}, "
                      f"dumping first 64 bytes:")
                self._hex_dump(data, 0, 64)
                return

            for ds_idx in range(num_datasets):
                if offset + 4 > len(data):
                    break

                dataset_type = struct.unpack_from("<i", data, offset)[0]
                offset += 4

                if dataset_type == 0:
                    offset = self._skip_markerset_def(data, offset)
                elif dataset_type == 1:
                    offset = self._parse_rigid_body_def(data, offset, natnet_major)
                elif dataset_type == 2:
                    offset = self._skip_skeleton_def(data, offset, natnet_major)
                elif dataset_type == 3:
                    offset = self._skip_force_plate_def(data, offset)
                elif dataset_type == 4:
                    offset = self._skip_device_def(data, offset)
                elif dataset_type == 5:
                    offset = self._skip_camera_def(data, offset)
                elif dataset_type == 6:
                    offset = self._skip_asset_def(data, offset, natnet_major)
                else:
                    print(f"[OptiTrack] Model def: unknown dataset type {dataset_type} "
                          f"at ds_idx={ds_idx}, offset={offset}")
                    break

        except (struct.error, IndexError, ValueError) as e:
            print(f"[OptiTrack] Model def parse error: {e}")
            print(f"  Parsed so far: {dict(self._id_to_name)}")
            self._hex_dump(data, max(0, offset - 16), 48)

    def _parse_rigid_body_def(self, data: bytes, offset: int,
                              natnet_major: int = 4) -> int:
        """Parse rigid body definition. NatNet 4.0+ has per-body marker data."""
        name, offset = self._read_cstring(data, offset)

        rb_id = struct.unpack_from("<i", data, offset)[0]
        offset += 4
        parent_id = struct.unpack_from("<i", data, offset)[0]
        offset += 4
        offset += 12  # offset position (3 floats)

        # NatNet 4.0+: per-body marker data
        if natnet_major >= 4 and offset + 4 <= len(data):
            num_markers = struct.unpack_from("<i", data, offset)[0]
            offset += 4
            offset += num_markers * 12  # marker positions
            offset += num_markers * 4   # active labels

        self._id_to_name[rb_id] = name
        print(f"[OptiTrack] Model def: rigid body '{name}' id={rb_id}")
        return offset

    def _skip_markerset_def(self, data: bytes, offset: int) -> int:
        """Skip a marker set definition."""
        name, offset = self._read_cstring(data, offset)
        num_markers = struct.unpack_from("<i", data, offset)[0]
        offset += 4
        for _ in range(num_markers):
            _, offset = self._read_cstring(data, offset)
        return offset

    def _skip_skeleton_def(self, data: bytes, offset: int,
                           natnet_major: int = 4) -> int:
        """Skip a skeleton definition."""
        _, offset = self._read_cstring(data, offset)
        offset += 4  # skeleton ID
        num_bodies = struct.unpack_from("<i", data, offset)[0]
        offset += 4
        for _ in range(num_bodies):
            offset = self._parse_rigid_body_def(data, offset, natnet_major)
        return offset

    def _skip_force_plate_def(self, data: bytes, offset: int) -> int:
        """Skip a force plate definition (NatNet 3.0+)."""
        offset += 4  # ID
        _, offset = self._read_cstring(data, offset)  # serial number
        offset += 4  # width
        offset += 4  # length
        offset += 48  # origin (3x4 matrix)
        offset += 48  # calibration matrix
        offset += 48  # corners (4 x 3 floats)
        offset += 4   # plate type
        num_channels = struct.unpack_from("<i", data, offset)[0]
        offset += 4
        for _ in range(num_channels):
            _, offset = self._read_cstring(data, offset)
        return offset

    def _skip_device_def(self, data: bytes, offset: int) -> int:
        """Skip a device definition (NatNet 3.0+)."""
        offset += 4  # ID
        _, offset = self._read_cstring(data, offset)  # name
        offset += 4  # serial
        offset += 4  # device type
        num_channels = struct.unpack_from("<i", data, offset)[0]
        offset += 4
        for _ in range(num_channels):
            _, offset = self._read_cstring(data, offset)
        return offset

    def _skip_camera_def(self, data: bytes, offset: int) -> int:
        """Skip a camera definition (NatNet 3.0+)."""
        _, offset = self._read_cstring(data, offset)  # name
        offset += 12  # position (3 floats)
        offset += 16  # orientation (4 floats)
        return offset

    def _skip_asset_def(self, data: bytes, offset: int,
                        natnet_major: int = 4) -> int:
        """Skip an asset definition (NatNet 4.1+)."""
        _, offset = self._read_cstring(data, offset)  # name
        offset += 4  # asset type
        offset += 4  # asset ID
        num_rb = struct.unpack_from("<i", data, offset)[0]
        offset += 4
        for _ in range(num_rb):
            offset = self._parse_rigid_body_def(data, offset, natnet_major)
        num_markers = struct.unpack_from("<i", data, offset)[0]
        offset += 4
        for _ in range(num_markers):
            _, offset = self._read_cstring(data, offset)
            offset += 4  # marker ID
        return offset

    # --- Frame Data Parser ---

    def _parse_frame_data(self, data: bytes, offset: int) -> None:
        """
        Parse NAT_FRAMEOFDATA — the main data path at ~120Hz.

        NatNet 4.x format:
          frame_number -> marker_sets -> other_markers(=0) -> rigid_bodies -> ...
        """
        now = time.time()
        natnet_major = self.natnet_version[0] if self.natnet_version[0] > 0 else 4

        offset += 4  # frame number

        # Marker sets (skip)
        num_marker_sets = struct.unpack_from("<i", data, offset)[0]
        offset += 4
        for _ in range(num_marker_sets):
            _, offset = self._read_cstring(data, offset)
            num_markers = struct.unpack_from("<i", data, offset)[0]
            offset += 4
            offset += num_markers * 12

        # Other markers (legacy, =0 in NatNet 4.x)
        num_other_markers = struct.unpack_from("<i", data, offset)[0]
        offset += 4
        offset += num_other_markers * 12

        # Rigid bodies
        num_rigid_bodies = struct.unpack_from("<i", data, offset)[0]
        offset += 4

        new_bodies: Dict[str, RigidBodyState] = {}

        for _ in range(num_rigid_bodies):
            if offset + 32 > len(data):
                break

            rb_id = struct.unpack_from("<i", data, offset)[0]
            offset += 4

            px, py, pz = struct.unpack_from("<fff", data, offset)
            offset += 12

            qx, qy, qz, qw = struct.unpack_from("<ffff", data, offset)
            offset += 16

            # NatNet 2.x: per-body marker data (removed in 3.0)
            if natnet_major < 3:
                n_body_markers = struct.unpack_from("<i", data, offset)[0]
                offset += 4
                offset += n_body_markers * 12  # positions
                offset += n_body_markers * 4   # IDs
                offset += n_body_markers * 4   # sizes

            # Mean marker error (float32)
            tracking_valid = True
            if offset + 4 <= len(data):
                offset += 4

            # Params (int16) — bit 0 = tracking valid
            if offset + 2 <= len(data):
                params = struct.unpack_from("<H", data, offset)[0]
                offset += 2
                tracking_valid = bool(params & 0x0001)

            # Convert coordinates
            pos = np.array([px, py, pz], dtype=np.float64)
            quat = np.array([qx, qy, qz, qw], dtype=np.float64)

            if self.convert_to_zup:
                pos = position_yup_to_zup(pos)
                quat = quaternion_yup_to_zup(quat)

            name = self._id_to_name.get(rb_id, f"rigid_body_{rb_id}")

            new_bodies[name] = RigidBodyState(
                name=name,
                id=rb_id,
                position=pos,
                quaternion=quat,
                timestamp=now,
                tracking_valid=tracking_valid,
            )

        if new_bodies:
            with self._lock:
                self._rigid_bodies = new_bodies
            self._frame_count += 1

            if self._callback is not None:
                try:
                    self._callback(new_bodies)
                except Exception as e:
                    print(f"[OptiTrack] Callback error: {e}")

    # --- Helpers ---

    @staticmethod
    def _read_cstring(data: bytes, offset: int) -> tuple:
        """Read null-terminated C string. Returns (string, new_offset)."""
        end = data.index(b'\x00', offset)
        s = data[offset:end].decode("utf-8", errors="replace")
        return s, end + 1

    @staticmethod
    def _hex_dump(data: bytes, offset: int, length: int = 48) -> None:
        """Print hex dump of a data region for debugging."""
        end = min(offset + length, len(data))
        chunk = data[offset:end]
        hex_str = " ".join(f"{b:02x}" for b in chunk)
        ascii_str = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        print(f"  Hex @{offset}: {hex_str}")
        print(f"  ASCII:    {ascii_str}")
