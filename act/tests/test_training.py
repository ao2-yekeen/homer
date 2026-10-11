"""Verify future-action labels and training-only normalization."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

import numpy as np
from PIL import Image

MODULE = Path(__file__).resolve().parents[1] / "training/train.py"
spec = importlib.util.spec_from_file_location("act_train", MODULE)
training = importlib.util.module_from_spec(spec)
spec.loader.exec_module(training)


class DatasetTests(unittest.TestCase):
    def test_future_labels_pad_without_crossing_episodes(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            Image.new("RGB", (16, 12), (255, 0, 0)).save(root / "image.jpg")
            rows = [{"image": "image.jpg", "overview_image": "image.jpg"}] * 3
            first = np.repeat(np.arange(3, dtype=np.float32)[:, None], 6, axis=1)
            second = first + 100
            data = training.DemoDataset([(root, rows, first), (root, rows, second)], chunk_size=3)
            self.assertEqual(len(data), 4)
            self.assertEqual(data[0]["action"][:, 0].tolist(), [1, 2, 2])
            self.assertEqual(data[0]["action_is_pad"].tolist(), [False, False, True])
            self.assertEqual(data[1]["action"][:, 0].tolist(), [2, 2, 2])
            self.assertEqual(data[1]["action_is_pad"].tolist(), [False, True, True])
            self.assertEqual(data[2]["observation.state"][0].item(), 100)
            self.assertEqual(tuple(data[0]["observation.images.wrist"].shape), (3, 120, 160))
            self.assertGreater(data[0]["observation.images.wrist"][0].mean(), 0.9)

    def test_stats_only_use_supplied_train_episodes_and_label_offset(self):
        ticks = np.repeat(np.array([[0], [10], [20]], dtype=np.float32), 6, axis=1)
        stats = training.training_stats([(None, [None] * 3, ticks)])
        self.assertEqual(stats["observation.state"]["mean"][0].item(), 5)
        self.assertEqual(stats["action"]["mean"][0].item(), 15)
        self.assertEqual(stats["action"]["std"][0].item(), 5)


if __name__ == "__main__":
    unittest.main()
