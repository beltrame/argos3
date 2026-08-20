#!/usr/bin/env python3
"""
A stub external estimator, used by the external_estimator regression test.

It speaks the same socket protocol as a real sidecar (see
CExternalEstimatorMedium::Update) but runs no SLAM: it replies with a
pose derived arithmetically from the tick, so the test can assert the
exact value that comes out of <odometry implementation="external">.
This is what lets the ARGoS half of the integration be tested with no
Docker, no ROS and no estimator installed.

It doubles as the reference implementation of the protocol: a real
sidecar differs only in where the pose comes from.

Usage:
  stub_estimator.py --socket /tmp/argos_test.sock [--valid-from 5]
"""
import argparse
import os
import socket
import struct
import sys

MAGIC = b"AEBR"
ACK = b"ACK\0"

# The pose the stub reports for a given tick. The test asserts exactly
# these numbers, so they must stay in sync with loop_functions.cpp.
POSE_X_PER_TICK = 0.001
POSE_Y = 2.0
POSE_Z = 3.0
TWIST = (0.1, 0.2, 0.3, 0.4, 0.5, 0.6)


def recv_exactly(sock, n):
    """Reads exactly n bytes, or returns None if the peer hung up."""
    chunks = []
    got = 0
    while got < n:
        b = sock.recv(min(65536, n - got))
        if not b:
            return None
        chunks.append(b)
        got += len(b)
    return b"".join(chunks)


def read_robot(conn):
    """Consumes one robot's record; returns (id, wheel_pose or None)."""
    (id_len,) = struct.unpack("<B", recv_exactly(conn, 1))
    robot = recv_exactly(conn, id_len).decode()

    (has_frame,) = struct.unpack("<B", recv_exactly(conn, 1))
    if has_frame:
        w, h, _fov = struct.unpack("<IIf", recv_exactly(conn, 12))
        recv_exactly(conn, w * h * 3)
        (has_depth,) = struct.unpack("<B", recv_exactly(conn, 1))
        if has_depth:
            recv_exactly(conn, w * h * 4)

    (has_scan,) = struct.unpack("<B", recv_exactly(conn, 1))
    if has_scan:
        (n_points,) = struct.unpack("<I", recv_exactly(conn, 4))
        # 26 bytes per point: f32 x y z intensity, f64 offset_ns, u8 tag, u8 line
        recv_exactly(conn, n_points * 26)

    wheels = None
    (has_wheels,) = struct.unpack("<B", recv_exactly(conn, 1))
    if has_wheels:
        wheels = struct.unpack("<7d", recv_exactly(conn, 56))

    struct.unpack("<6d", recv_exactly(conn, 48))  # imu
    return robot, wheels


def handle_tick(conn, valid_from, echo_wheels):
    head = recv_exactly(conn, 4 + 4 + 4 + 1 + 4)
    if head is None:
        return False
    magic, tick, _tps, _lockstep, n_robots = struct.unpack("<4sIIBI", head)
    if magic != MAGIC:
        raise RuntimeError("bad magic %r: protocol mismatch" % (magic,))

    robots = [read_robot(conn) for _ in range(n_robots)]

    # A real estimator needs to initialize before it can report a pose;
    # the test asserts the sensor stays invalid until it does.
    valid = 1 if tick >= valid_from else 0
    out = bytearray(ACK)
    out += struct.pack("<I", len(robots))
    for robot, wheels in robots:
        name = robot.encode()
        # In echo mode the "estimate" is the wheel-encoder pose ARGoS
        # dead-reckoned and sent, so the test can check that integration
        # (and its centimetre-to-metre conversion) against ground truth.
        if echo_wheels:
            if wheels is None:
                raise RuntimeError("%s sent no wheel pose, but --echo-wheels "
                                   "was requested: is differential_steering "
                                   "declared on the robot?" % robot)
            pose = wheels
        else:
            pose = (tick * POSE_X_PER_TICK, POSE_Y, POSE_Z, 1.0, 0.0, 0.0, 0.0)
        out += struct.pack("<B", len(name)) + name
        out += struct.pack("<I", tick)
        out += struct.pack("<7d", *pose)
        out += struct.pack("<6d", *TWIST)
        out += struct.pack("<B", valid)
    conn.sendall(bytes(out))
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--socket", required=True)
    ap.add_argument("--valid-from", type=int, default=5,
                    help="first tick for which a pose is reported as valid")
    ap.add_argument("--echo-wheels", action="store_true",
                    help="report the received wheel-encoder pose as the "
                         "estimate, instead of an arithmetic one")
    args = ap.parse_args()

    if os.path.exists(args.socket):
        os.unlink(args.socket)
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(args.socket)
    os.chmod(args.socket, 0o777)
    server.listen(1)
    # ARGoS retries its connect, but tell the harness we are ready anyway
    sys.stderr.write("stub_estimator listening on %s\n" % args.socket)
    sys.stderr.flush()

    conn, _ = server.accept()
    ticks = 0
    try:
        while handle_tick(conn, args.valid_from, args.echo_wheels):
            ticks += 1
    finally:
        conn.close()
        server.close()
        if os.path.exists(args.socket):
            os.unlink(args.socket)
    sys.stderr.write("stub_estimator saw %d ticks\n" % ticks)


if __name__ == "__main__":
    main()
