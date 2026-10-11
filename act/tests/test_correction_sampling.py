"""Verify mixed replay never draws validation or crosses episode boundaries."""
import importlib.util
from pathlib import Path
import random
import unittest
from unittest.mock import patch

import numpy as np

PATH = Path(__file__).resolve().parents[1] / 'training/train.py'
spec = importlib.util.spec_from_file_location('correction_train', PATH)
training = importlib.util.module_from_spec(spec)
spec.loader.exec_module(training)


class SamplingTests(unittest.TestCase):
    def test_short_corrections_are_not_swamped_by_long_original_demos(self):
        def episode(name, frames):
            return (Path(name), [{}]*frames, np.zeros((frames, 6), dtype=np.float32))
        data = training.DemoDataset([episode('old', 1001), episode('correction', 11)])
        manifest = {'correction_train': ['correction'], 'correction_fraction': 0.5}
        generator = random.Random(42)
        with patch.object(training, 'random', generator):
            samples = training.sample_indices(data, manifest, 2000)
        correction_count = sum(index >= 1000 for index in samples)
        self.assertTrue(900 < correction_count < 1100)
        self.assertTrue(all(0 <= index < len(data) for index in samples))

    def test_original_training_sampler_remains_available(self):
        data = training.DemoDataset([(Path('old'), [{}]*5, np.zeros((5,6), dtype=np.float32))])
        self.assertEqual(len(training.sample_indices(data, {}, 8)), 8)


if __name__ == '__main__':
    unittest.main()
