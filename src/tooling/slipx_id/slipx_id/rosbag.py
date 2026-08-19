# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""rosbag2 without ROS (ADR-0040).

Reads the two rosbag2 storage formats directly: sqlite3 with the standard
library, MCAP through the ``mcap`` extra when it is installed. Payloads are
CDR, decoded by hand for exactly the message set the manoeuvre library
records; a topic whose type is not in the set is refused by name, never
skipped, because a recording the fitter half-understands produces a fit
that looks complete and is missing a signal.

The same module writes bags, so the synthetic self-test can pass through
the identical decoder a real recording meets. The writer produces the
sqlite3 storage with a metadata file in rosbag2's own format.

CDR here means the XCDR1 little-endian encoding ROS 2 uses on the wire:
a four-byte encapsulation header, then fields aligned to their own size
relative to the start of the payload, strings as a length-prefixed,
NUL-terminated byte run, sequences as a count followed by elements.
"""

from __future__ import annotations

import math
import sqlite3
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

from .channels import Channel

# The message set. Everything the manoeuvre library records, and nothing
# else: an addition here is an addition to the procedures, not a convenience.
IMU_TYPE = "sensor_msgs/msg/Imu"
POSE_TYPE = "geometry_msgs/msg/PoseStamped"
ODOMETRY_TYPE = "nav_msgs/msg/Odometry"
JOINT_STATE_TYPE = "sensor_msgs/msg/JointState"
DRIVE_TYPE = "ackermann_msgs/msg/AckermannDriveStamped"

SUPPORTED_TYPES = (
    IMU_TYPE,
    POSE_TYPE,
    ODOMETRY_TYPE,
    JOINT_STATE_TYPE,
    DRIVE_TYPE,
)


# ------------------------------------------------------------------- CDR


class _CdrReader:
    def __init__(self, data: bytes) -> None:
        if len(data) < 4:
            raise ValueError("CDR payload shorter than its header")
        if data[0] != 0x00 or data[1] != 0x01:
            raise ValueError(
                f"unsupported CDR encapsulation {data[0]:#04x} {data[1]:#04x};"
                f" this reader speaks little-endian XCDR1 (what ROS 2 writes)"
            )
        self._data = data
        self._offset = 4

    def _align(self, size: int) -> None:
        misaligned = (self._offset - 4) % size
        if misaligned:
            self._offset += size - misaligned

    def _read(self, format_char: str, size: int) -> float:
        self._align(size)
        (value,) = struct.unpack_from("<" + format_char, self._data, self._offset)
        self._offset += size
        return value

    def int32(self) -> int:
        return self._read("i", 4)

    def uint32(self) -> int:
        return self._read("I", 4)

    def float32(self) -> float:
        return self._read("f", 4)

    def float64(self) -> float:
        return self._read("d", 8)

    def string(self) -> str:
        length = self.uint32()
        if length == 0:
            return ""
        raw = self._data[self._offset : self._offset + length - 1]
        self._offset += length
        return raw.decode("utf-8")

    def float64_array(self, count: int) -> Tuple[float, ...]:
        return tuple(self.float64() for _ in range(count))

    def float64_sequence(self) -> Tuple[float, ...]:
        return self.float64_array(self.uint32())

    def string_sequence(self) -> Tuple[str, ...]:
        return tuple(self.string() for _ in range(self.uint32()))

    def stamp_ns(self) -> int:
        sec = self.int32()
        nanosec = self.uint32()
        return sec * 1_000_000_000 + nanosec


class _CdrWriter:
    def __init__(self) -> None:
        self._data = bytearray(b"\x00\x01\x00\x00")

    def _align(self, size: int) -> None:
        misaligned = (len(self._data) - 4) % size
        if misaligned:
            self._data.extend(b"\x00" * (size - misaligned))

    def _write(self, format_char: str, size: int, value) -> None:
        self._align(size)
        self._data.extend(struct.pack("<" + format_char, value))

    def int32(self, value: int) -> None:
        self._write("i", 4, value)

    def uint32(self, value: int) -> None:
        self._write("I", 4, value)

    def float32(self, value: float) -> None:
        self._write("f", 4, value)

    def float64(self, value: float) -> None:
        self._write("d", 8, value)

    def string(self, value: str) -> None:
        raw = value.encode("utf-8")
        self.uint32(len(raw) + 1)
        self._data.extend(raw)
        self._data.append(0)

    def float64_array(self, values: Iterable[float]) -> None:
        for value in values:
            self.float64(value)

    def float64_sequence(self, values: Sequence[float]) -> None:
        self.uint32(len(values))
        self.float64_array(values)

    def string_sequence(self, values: Sequence[str]) -> None:
        self.uint32(len(values))
        for value in values:
            self.string(value)

    def stamp(self, stamp_ns: int) -> None:
        self.int32(stamp_ns // 1_000_000_000)
        self.uint32(stamp_ns % 1_000_000_000)

    def bytes(self) -> bytes:
        return bytes(self._data)


# --------------------------------------------------------------- messages


@dataclass(frozen=True)
class ImuSample:
    stamp_ns: int
    angular_velocity: Tuple[float, float, float]
    linear_acceleration: Tuple[float, float, float]


@dataclass(frozen=True)
class PoseSample:
    stamp_ns: int
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class JointSample:
    stamp_ns: int
    names: Tuple[str, ...]
    velocities: Tuple[float, ...]


@dataclass(frozen=True)
class DriveSample:
    stamp_ns: int
    steering_angle: float
    speed: float
    acceleration: float


def _yaw_of(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(
        2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)
    )


def decode_imu(data: bytes) -> ImuSample:
    r = _CdrReader(data)
    stamp = r.stamp_ns()
    r.string()  # frame_id
    r.float64_array(4)  # orientation, unused: the pose owns the heading
    r.float64_array(9)
    angular = (r.float64(), r.float64(), r.float64())
    r.float64_array(9)
    linear = (r.float64(), r.float64(), r.float64())
    r.float64_array(9)
    return ImuSample(stamp, angular, linear)


def encode_imu(sample: ImuSample) -> bytes:
    w = _CdrWriter()
    w.stamp(sample.stamp_ns)
    w.string("imu")
    w.float64_array((0.0, 0.0, 0.0, 1.0))
    w.float64_array((0.0,) * 9)
    w.float64_array(sample.angular_velocity)
    w.float64_array((0.0,) * 9)
    w.float64_array(sample.linear_acceleration)
    w.float64_array((0.0,) * 9)
    return w.bytes()


def decode_pose_stamped(data: bytes) -> PoseSample:
    r = _CdrReader(data)
    stamp = r.stamp_ns()
    r.string()  # frame_id
    x, y = r.float64(), r.float64()
    r.float64()  # z
    qx, qy, qz, qw = r.float64(), r.float64(), r.float64(), r.float64()
    return PoseSample(stamp, x, y, _yaw_of(qx, qy, qz, qw))


def encode_pose_stamped(sample: PoseSample) -> bytes:
    w = _CdrWriter()
    w.stamp(sample.stamp_ns)
    w.string("map")
    w.float64_array((sample.x, sample.y, 0.0))
    half = 0.5 * sample.yaw
    w.float64_array((0.0, 0.0, math.sin(half), math.cos(half)))
    return w.bytes()


def decode_odometry(data: bytes) -> PoseSample:
    r = _CdrReader(data)
    stamp = r.stamp_ns()
    r.string()  # frame_id
    r.string()  # child_frame_id
    x, y = r.float64(), r.float64()
    r.float64()  # z
    qx, qy, qz, qw = r.float64(), r.float64(), r.float64(), r.float64()
    # The twist and both covariances follow and are not consumed: body
    # velocity is reconstructed from the pose, the same way for every
    # source, so an odometry whose twist disagrees with its own pose cannot
    # split the fit in two.
    return PoseSample(stamp, x, y, _yaw_of(qx, qy, qz, qw))


def decode_joint_state(data: bytes) -> JointSample:
    r = _CdrReader(data)
    stamp = r.stamp_ns()
    r.string()  # frame_id
    names = r.string_sequence()
    r.float64_sequence()  # position
    velocities = r.float64_sequence()
    return JointSample(stamp, names, velocities)


def encode_joint_state(sample: JointSample) -> bytes:
    w = _CdrWriter()
    w.stamp(sample.stamp_ns)
    w.string("")
    w.string_sequence(sample.names)
    w.float64_sequence(())
    w.float64_sequence(sample.velocities)
    w.float64_sequence(())
    return w.bytes()


def decode_drive(data: bytes) -> DriveSample:
    r = _CdrReader(data)
    stamp = r.stamp_ns()
    r.string()  # frame_id
    steering_angle = r.float32()
    r.float32()  # steering_angle_velocity
    speed = r.float32()
    acceleration = r.float32()
    return DriveSample(stamp, steering_angle, speed, acceleration)


def encode_drive(sample: DriveSample) -> bytes:
    w = _CdrWriter()
    w.stamp(sample.stamp_ns)
    w.string("")
    w.float32(sample.steering_angle)
    w.float32(0.0)
    w.float32(sample.speed)
    w.float32(sample.acceleration)
    w.float32(0.0)  # jerk
    return w.bytes()


_DECODERS = {
    IMU_TYPE: decode_imu,
    POSE_TYPE: decode_pose_stamped,
    ODOMETRY_TYPE: decode_odometry,
    JOINT_STATE_TYPE: decode_joint_state,
    DRIVE_TYPE: decode_drive,
}


# ------------------------------------------------------------ bag reading


@dataclass(frozen=True)
class BagTopic:
    name: str
    type: str
    messages: Tuple[Tuple[int, bytes], ...]  # (receive time [ns], payload)


def _sqlite_files(path: Path) -> List[Path]:
    if path.is_file():
        return [path] if path.suffix == ".db3" else []
    return sorted(path.glob("*.db3"))


def _mcap_files(path: Path) -> List[Path]:
    if path.is_file():
        return [path] if path.suffix == ".mcap" else []
    return sorted(path.glob("*.mcap"))


def read_bag(path) -> Dict[str, BagTopic]:
    """Every topic in the bag, undecoded. The decoding happens per topic in
    :func:`bag_channels`, where a wrong type can be refused with its name."""
    root = Path(path)
    if not root.exists():
        raise FileNotFoundError(f"no bag at {root}")

    sqlite_files = _sqlite_files(root)
    if sqlite_files:
        return _read_sqlite(sqlite_files)
    mcap_files = _mcap_files(root)
    if mcap_files:
        return _read_mcap(mcap_files)
    raise ValueError(
        f"{root} contains neither a .db3 nor a .mcap file; rosbag2's two "
        f"storage formats are what this reader speaks (ADR-0040)"
    )


def _read_sqlite(files: List[Path]) -> Dict[str, BagTopic]:
    collected: Dict[str, Tuple[str, List[Tuple[int, bytes]]]] = {}
    for file in files:
        connection = sqlite3.connect(f"file:{file.as_posix()}?mode=ro", uri=True)
        try:
            topics = {
                topic_id: (name, type_name)
                for topic_id, name, type_name in connection.execute(
                    "SELECT id, name, type FROM topics"
                )
            }
            for topic_id, timestamp, data in connection.execute(
                "SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp, id"
            ):
                name, type_name = topics[topic_id]
                entry = collected.setdefault(name, (type_name, []))
                entry[1].append((timestamp, bytes(data)))
        finally:
            connection.close()
    return {
        name: BagTopic(name, type_name, tuple(messages))
        for name, (type_name, messages) in collected.items()
    }


def _read_mcap(files: List[Path]) -> Dict[str, BagTopic]:
    try:
        from mcap.reader import make_reader
    except ImportError as exc:  # pragma: no cover - environment-specific
        raise ImportError(
            "this bag uses rosbag2's MCAP storage, which needs the mcap "
            "extra: pip install 'slipx[mcap]'"
        ) from exc

    collected: Dict[str, Tuple[str, List[Tuple[int, bytes]]]] = {}
    for file in files:
        with file.open("rb") as handle:
            reader = make_reader(handle)
            for schema, channel, message in reader.iter_messages():
                type_name = schema.name if schema is not None else ""
                entry = collected.setdefault(channel.topic, (type_name, []))
                entry[1].append((message.log_time, bytes(message.data)))
    return {
        name: BagTopic(name, type_name, tuple(sorted(messages)))
        for name, (type_name, messages) in collected.items()
    }


# ------------------------------------------------------------ bag writing


def write_bag(directory, topics: Mapping[str, Tuple[str, Sequence[Tuple[int, bytes]]]]) -> Path:
    """Write a rosbag2 directory (sqlite3 storage plus metadata.yaml).

    ``topics`` maps a topic name to its type and its (receive time [ns],
    CDR payload) messages. Returns the directory.
    """
    root = Path(directory)
    root.mkdir(parents=True, exist_ok=True)
    database = root / f"{root.name}_0.db3"
    if database.exists():
        raise FileExistsError(f"{database} already exists; refusing to overwrite")

    connection = sqlite3.connect(database)
    try:
        connection.execute(
            "CREATE TABLE schema(schema_version INTEGER PRIMARY KEY, "
            "ros_distro TEXT NOT NULL)"
        )
        connection.execute(
            "INSERT INTO schema(schema_version, ros_distro) VALUES (3, 'jazzy')"
        )
        connection.execute(
            "CREATE TABLE metadata(id INTEGER PRIMARY KEY, "
            "metadata_version INTEGER NOT NULL, metadata TEXT NOT NULL)"
        )
        connection.execute(
            "CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT NOT NULL, "
            "type TEXT NOT NULL, serialization_format TEXT NOT NULL, "
            "offered_qos_profiles TEXT NOT NULL)"
        )
        connection.execute(
            "CREATE TABLE messages(id INTEGER PRIMARY KEY, "
            "topic_id INTEGER NOT NULL, timestamp INTEGER NOT NULL, "
            "data BLOB NOT NULL)"
        )
        for index, (name, (type_name, _)) in enumerate(
            sorted(topics.items()), start=1
        ):
            connection.execute(
                "INSERT INTO topics(id, name, type, serialization_format, "
                "offered_qos_profiles) VALUES (?, ?, ?, 'cdr', '')",
                (index, name, type_name),
            )
        topic_ids = {
            name: index
            for index, name in enumerate(sorted(topics), start=1)
        }
        for name, (_, messages) in sorted(topics.items()):
            for stamp, payload in messages:
                connection.execute(
                    "INSERT INTO messages(topic_id, timestamp, data) "
                    "VALUES (?, ?, ?)",
                    (topic_ids[name], stamp, payload),
                )
        connection.commit()
    finally:
        connection.close()

    _write_metadata(root, database.name, topics)
    return root


def _write_metadata(
    root: Path,
    database_name: str,
    topics: Mapping[str, Tuple[str, Sequence[Tuple[int, bytes]]]],
) -> None:
    stamps = [
        stamp for _, messages in topics.values() for stamp, _ in messages
    ]
    start = min(stamps) if stamps else 0
    duration = (max(stamps) - start) if stamps else 0
    count = len(stamps)

    lines = [
        "rosbag2_bagfile_information:",
        "  version: 5",
        "  storage_identifier: sqlite3",
        f"  duration:",
        f"    nanoseconds: {duration}",
        f"  starting_time:",
        f"    nanoseconds_since_epoch: {start}",
        f"  message_count: {count}",
        "  topics_with_message_count:",
    ]
    for name, (type_name, messages) in sorted(topics.items()):
        lines.extend(
            [
                "    - topic_metadata:",
                f"        name: {name}",
                f"        type: {type_name}",
                "        serialization_format: cdr",
                '        offered_qos_profiles: ""',
                f"      message_count: {len(messages)}",
            ]
        )
    lines.extend(
        [
            '  compression_format: ""',
            '  compression_mode: ""',
            "  relative_file_paths:",
            f"    - {database_name}",
            "  files:",
            f"    - path: {database_name}",
            "      starting_time:",
            f"        nanoseconds_since_epoch: {start}",
            "      duration:",
            f"        nanoseconds: {duration}",
            f"      message_count: {count}",
        ]
    )
    (root / "metadata.yaml").write_text("\n".join(lines) + "\n", encoding="utf-8")


# ------------------------------------------------- channels out of a bag


@dataclass(frozen=True)
class TopicMap:
    """Which topic carries which signal, and what the wheels are called.

    ``wheel_names`` maps the fixed FL, FR, RL, RR order to the joint names
    the car's encoder publisher uses.
    """

    pose: str
    imu: str
    wheels: str
    drive: str
    wheel_names: Mapping[str, str]

    def wheel_name(self, wheel: str) -> str:
        try:
            return self.wheel_names[wheel]
        except KeyError:
            raise KeyError(
                f"the topic map names no joint for wheel {wheel}; "
                f"wheel_names needs all four of FL, FR, RL, RR"
            ) from None


def _require_topic(bag: Dict[str, BagTopic], name: str, kinds: Tuple[str, ...]) -> BagTopic:
    if name not in bag:
        available = ", ".join(sorted(bag)) or "none"
        raise ValueError(
            f"the bag has no topic '{name}' (topics present: {available})"
        )
    topic = bag[name]
    if topic.type not in kinds:
        raise ValueError(
            f"topic '{name}' is a {topic.type}, and this reader consumes "
            f"{' or '.join(kinds)} there; a half-understood recording is "
            f"worse than a refusal (ADR-0040)"
        )
    return topic


def _stamped(decoded, receive_ns: int) -> Tuple[int, object]:
    stamp = decoded.stamp_ns
    return (stamp if stamp > 0 else receive_ns, decoded)


def _channel(pairs: List[Tuple[int, float]], origin_ns: int) -> Channel:
    """Sort by stamp, drop exact duplicate stamps (first wins), rebase."""
    pairs.sort(key=lambda item: item[0])
    times: List[float] = []
    values: List[float] = []
    previous: Optional[int] = None
    for stamp, value in pairs:
        if previous is not None and stamp == previous:
            continue
        previous = stamp
        times.append((stamp - origin_ns) * 1e-9)
        values.append(value)
    return Channel(tuple(times), tuple(values))


def bag_channels(path, topic_map: TopicMap) -> Dict[str, Channel]:
    """The fitter's channel set out of one recording.

    Timestamps are the messages' own header stamps (falling back to the
    receive time when a stamp is zero), rebased so the earliest consumed
    message is t = 0. Alignment across channels is by timestamp everywhere
    downstream, so nothing here resamples anything.
    """
    bag = read_bag(path)

    pose_topic = _require_topic(bag, topic_map.pose, (POSE_TYPE, ODOMETRY_TYPE))
    imu_topic = _require_topic(bag, topic_map.imu, (IMU_TYPE,))
    wheels_topic = _require_topic(bag, topic_map.wheels, (JOINT_STATE_TYPE,))
    drive_topic = _require_topic(bag, topic_map.drive, (DRIVE_TYPE,))

    decode_pose = (
        decode_pose_stamped if pose_topic.type == POSE_TYPE else decode_odometry
    )
    poses = [_stamped(decode_pose(d), t) for t, d in pose_topic.messages]
    imus = [_stamped(decode_imu(d), t) for t, d in imu_topic.messages]
    joints = [
        _stamped(decode_joint_state(d), t) for t, d in wheels_topic.messages
    ]
    drives = [_stamped(decode_drive(d), t) for t, d in drive_topic.messages]

    stamps = [t for t, _ in poses + imus + joints + drives]
    if not stamps:
        raise ValueError("the bag's mapped topics carry no messages")
    origin = min(stamps)

    channels: Dict[str, Channel] = {
        "pose.x": _channel([(t, p.x) for t, p in poses], origin),
        "pose.y": _channel([(t, p.y) for t, p in poses], origin),
        "pose.yaw": _channel([(t, p.yaw) for t, p in poses], origin),
        "imu.ax": _channel(
            [(t, m.linear_acceleration[0]) for t, m in imus], origin
        ),
        "imu.ay": _channel(
            [(t, m.linear_acceleration[1]) for t, m in imus], origin
        ),
        "imu.yaw_rate": _channel(
            [(t, m.angular_velocity[2]) for t, m in imus], origin
        ),
        "cmd.steer": _channel(
            [(t, d.steering_angle) for t, d in drives], origin
        ),
        "cmd.accel": _channel(
            [(t, d.acceleration) for t, d in drives], origin
        ),
    }

    for wheel in ("FL", "FR", "RL", "RR"):
        joint = topic_map.wheel_name(wheel)
        pairs: List[Tuple[int, float]] = []
        for t, sample in joints:
            if joint not in sample.names:
                raise ValueError(
                    f"joint '{joint}' (wheel {wheel}) is missing from a "
                    f"JointState on '{topic_map.wheels}'; it carries "
                    f"{list(sample.names)}"
                )
            index = sample.names.index(joint)
            if index >= len(sample.velocities):
                raise ValueError(
                    f"joint '{joint}' has no velocity in a JointState on "
                    f"'{topic_map.wheels}'; encoder speeds are what the fit "
                    f"consumes"
                )
            pairs.append((t, sample.velocities[index]))
        channels[f"wheel.{wheel}"] = _channel(pairs, origin)

    return channels


# ------------------------------------------- recordings in and out of bags

#: The topic names the synthetic bags use, and a plausible default for a
#: real car; a session file overrides any of it.
DEFAULT_TOPIC_MAP = TopicMap(
    pose="/pose",
    imu="/imu",
    wheels="/joint_states",
    drive="/drive",
    wheel_names={
        "FL": "wheel_front_left",
        "FR": "wheel_front_right",
        "RL": "wheel_rear_left",
        "RR": "wheel_rear_right",
    },
)


def write_recording(recording, directory, topic_map: TopicMap = DEFAULT_TOPIC_MAP) -> Path:
    """Write a :class:`ManoeuvreRecording` as a real rosbag2 directory.

    This is how the synthetic self-test reaches the bag path: the recording
    becomes actual CDR in an actual sqlite3 bag, and the fit then reads it
    with the same decoder a team's recording meets.
    """

    def stamps_of(channel: Channel) -> List[int]:
        return [int(round(t * 1e9)) for t in channel.times]

    pose_x = recording.channel("pose.x")
    pose_y = recording.channel("pose.y")
    pose_yaw = recording.channel("pose.yaw")
    poses = [
        (
            stamp,
            encode_pose_stamped(
                PoseSample(stamp, pose_x.values[i], pose_y.values[i],
                           pose_yaw.values[i])
            ),
        )
        for i, stamp in enumerate(stamps_of(pose_x))
    ]

    ax = recording.channel("imu.ax")
    ay = recording.channel("imu.ay")
    yaw_rate = recording.channel("imu.yaw_rate")
    imus = [
        (
            stamp,
            encode_imu(
                ImuSample(
                    stamp,
                    (0.0, 0.0, yaw_rate.values[i]),
                    (ax.values[i], ay.values[i], 0.0),
                )
            ),
        )
        for i, stamp in enumerate(stamps_of(ax))
    ]

    wheel_channels = {
        wheel: recording.channel(f"wheel.{wheel}")
        for wheel in ("FL", "FR", "RL", "RR")
    }
    names = tuple(
        topic_map.wheel_name(wheel) for wheel in ("FL", "FR", "RL", "RR")
    )
    joints = [
        (
            stamp,
            encode_joint_state(
                JointSample(
                    stamp,
                    names,
                    tuple(
                        wheel_channels[w].values[i]
                        for w in ("FL", "FR", "RL", "RR")
                    ),
                )
            ),
        )
        for i, stamp in enumerate(stamps_of(wheel_channels["FL"]))
    ]

    steer = recording.channel("cmd.steer")
    accel = recording.channel("cmd.accel")
    drives = [
        (
            stamp,
            encode_drive(
                DriveSample(stamp, steer.values[i], 0.0, accel.values[i])
            ),
        )
        for i, stamp in enumerate(stamps_of(steer))
    ]

    return write_bag(
        directory,
        {
            topic_map.pose: (POSE_TYPE, poses),
            topic_map.imu: (IMU_TYPE, imus),
            topic_map.wheels: (JOINT_STATE_TYPE, joints),
            topic_map.drive: (DRIVE_TYPE, drives),
        },
    )


def read_recording(
    path,
    bench,
    *,
    topic_map: TopicMap = DEFAULT_TOPIC_MAP,
    name: Optional[str] = None,
    dt: float = 1.0e-3,
):
    """One recording out of one bag, in the shape every stage consumes.

    ``dt`` is the forward model's step for any stage that replays this
    recording; it is a property of the fit, not of the bag.
    """
    from .synthetic import ManoeuvreRecording

    channels = bag_channels(path, topic_map)
    return ManoeuvreRecording(
        name=name or Path(path).name, dt=dt, bench=bench, channels=channels
    )
