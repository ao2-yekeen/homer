import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from serial_protocol import Decoder, decode, encode


class SerialProtocolTests(unittest.TestCase):
    def test_round_trip_and_chunked_input(self):
        frame = encode("O", 17, 1234, 0.1, 0.2, 0, 0, 0)
        self.assertEqual(decode(frame), ("O", 17, ["1234", "0.1", "0.2", "0", "0", "0"]))
        decoder = Decoder()
        self.assertEqual(decoder.feed(frame[:8]), [])
        self.assertEqual(decoder.feed(frame[8:]), [decode(frame)])

    def test_corruption_and_oversize_resynchronize(self):
        frame = encode("R", 1, 42)
        decoder = Decoder()
        self.assertEqual(decoder.feed(frame.replace(b"42", b"43") + b"x" * 140 + b"\n" + frame), [decode(frame)])
        self.assertEqual(decoder.errors, 2)

    def test_reject_invalid_output(self):
        with self.assertRaises(ValueError):
            encode("T", 1, "x" * 200)
        with self.assertRaises(ValueError):
            encode("T", 65536, 0, 0)


if __name__ == "__main__":
    unittest.main()
