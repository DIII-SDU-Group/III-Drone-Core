"""Framing of the charger-gripper board's status stream (real drone)."""

import importlib.util
from pathlib import Path

NODE = Path(__file__).resolve().parents[2] / "scripts/payload/charger_gripper_node/charger_gripper_node.py"
spec = importlib.util.spec_from_file_location("charger_gripper_node", NODE)
node = importlib.util.module_from_spec(spec)
spec.loader.exec_module(node)

FIRST, LAST, LENGTH = 0xAA, 0x55, 6


def frame(payload: bytes) -> bytes:
    assert len(payload) == LENGTH - 2
    return bytes([FIRST]) + payload + bytes([LAST])


def take(pending: bytearray, data: bytes):
    return node.take_status_frames(pending, data, FIRST, LAST, LENGTH)


def test_every_frame_waiting_in_one_read_is_taken():
    # One byte per 10 ms tick capped the node at ~11 frames/s (2026-10-06).
    pending = bytearray()
    frames, dropped = take(pending, frame(b"\x01\x02\x03\x04") + frame(b"\x05\x06\x07\x08"))
    assert frames == [frame(b"\x01\x02\x03\x04"), frame(b"\x05\x06\x07\x08")]
    assert dropped == 0 and not pending


def test_a_frame_split_across_reads_completes_on_the_next_read():
    pending = bytearray()
    data = frame(b"\x01\x02\x03\x04")
    assert take(pending, data[:3]) == ([], 0)
    assert take(pending, data[3:]) == ([data], 0)


def test_bytes_before_a_first_byte_are_skipped():
    pending = bytearray()
    assert take(pending, b"\x00\x11" + frame(b"\x01\x02\x03\x04")) == ([frame(b"\x01\x02\x03\x04")], 0)


def test_a_wrong_last_byte_drops_the_frame_and_the_stream_resynchronises():
    pending = bytearray()
    bad = bytes([FIRST]) + b"\x01\x02\x03\x04" + b"\x00"
    frames, dropped = take(pending, bad + frame(b"\x05\x06\x07\x08"))
    assert frames == [frame(b"\x05\x06\x07\x08")]
    assert dropped == 1
