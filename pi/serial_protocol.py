"""HOM1 serial framing shared by the Pi bridge and its tests."""

from __future__ import annotations

MAX_FRAME = 128


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode(kind: str, sequence: int, *fields: object) -> bytes:
    if len(kind) != 1 or not kind.isascii() or not kind.isupper():
        raise ValueError("invalid frame type")
    if not 0 <= sequence <= 65535:
        raise ValueError("invalid sequence")
    body = "|".join(("HOM1", kind, str(sequence), *(str(field) for field in fields))).encode("ascii")
    frame = body + f"|{crc16(body):04X}\n".encode("ascii")
    if len(frame) > MAX_FRAME:
        raise ValueError("frame too long")
    return frame


def decode(line: bytes) -> tuple[str, int, list[str]]:
    if len(line) > MAX_FRAME or not line.endswith(b"\n"):
        raise ValueError("invalid frame length")
    body, separator, checksum = line[:-1].rpartition(b"|")
    if not separator or len(checksum) != 4:
        raise ValueError("missing checksum")
    try:
        expected = int(checksum, 16)
        parts = body.decode("ascii").split("|")
        if len(parts) < 3 or parts[0] != "HOM1" or len(parts[1]) != 1:
            raise ValueError("invalid header")
        sequence = int(parts[2])
    except (UnicodeError, ValueError) as error:
        raise ValueError("invalid frame") from error
    if not 0 <= sequence <= 65535 or crc16(body) != expected:
        raise ValueError("invalid sequence or checksum")
    return parts[1], sequence, parts[3:]


class Decoder:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.dropping = False
        self.errors = 0

    def feed(self, data: bytes) -> list[tuple[str, int, list[str]]]:
        frames = []
        for byte in data:
            if byte == 10:
                if not self.dropping and self.buffer:
                    try:
                        frames.append(decode(bytes(self.buffer) + b"\n"))
                    except ValueError:
                        self.errors += 1
                self.buffer.clear()
                self.dropping = False
            elif not self.dropping and byte != 13:
                if len(self.buffer) >= MAX_FRAME - 1:
                    self.buffer.clear()
                    self.dropping = True
                    self.errors += 1
                else:
                    self.buffer.append(byte)
        return frames
