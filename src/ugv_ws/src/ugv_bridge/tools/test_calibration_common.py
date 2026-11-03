"""保护两个标定报告原有的序号统计合同。"""
import unittest

from calibration_common import sequence_quality


class SequenceQualityTest(unittest.TestCase):
    def test_wrap_gaps_duplicates_and_reset(self):
        for modulus in (1 << 16, 1 << 32):
            with self.subTest(modulus=modulus):
                result = sequence_quality([modulus-1, 0, 0, 3, 2], modulus)
                self.assertEqual(result, {'transitions': 4, 'duplicates': 1, 'gap_events': 1,
                                          'skipped_values': 2, 'backward_steps': 1})
                renamed = sequence_quality([3, 2], modulus, backward_key='backward_or_reset_steps')
                self.assertEqual(renamed['backward_or_reset_steps'], 1)
                self.assertNotIn('backward_steps', renamed)

    def test_empty_and_single_record(self):
        for values in ([], [7]):
            self.assertTrue(all(v == 0 for v in sequence_quality(values, 1 << 16).values()))


if __name__ == '__main__':
    unittest.main()
