import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MAIN = ROOT / "main"


class RawSdPreEraseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = (MAIN / "raw_sd_segment_recorder.c").read_text(
            encoding="utf-8"
        )
        cls.header = (MAIN / "raw_sd_segment_recorder.h").read_text(
            encoding="utf-8"
        )
        cls.app = (MAIN / "main.c").read_text(encoding="utf-8")

    def test_capacity_uses_all_sectors_after_lba2048(self):
        self.assertIn(
            "recorder->data_capacity_sectors =",
            self.source,
        )
        self.assertIn(
            "recorder->card.csd.capacity - RAW_SD_DATA_START_LBA",
            self.source,
        )
        self.assertNotIn("RAW_SD_DATA_CAPACITY_SECTORS", self.source)
        self.assertNotIn("MAX_RECORD_FRAMES", self.app)

    def test_previous_end_is_loaded_from_two_validated_superblocks(self):
        for token in (
            "RAW_SD_SUPERBLOCK_LBA_A",
            "RAW_SD_SUPERBLOCK_LBA_B",
            "sdmmc_read_sectors",
            "raw_sd_superblock_is_valid",
            "generation",
            "next_write_lba",
        ):
            self.assertIn(token, self.source)

    def test_abnormal_power_loss_is_detected_from_last_segment(self):
        for token in (
            "raw_sd_segment_is_valid",
            "last_segment.metadata_generation > selected->generation",
            "RAW_SD_SEGMENT_CLOSED",
            "RAW_SD_SEGMENT_FAILED",
            "Previous run did not close normally",
        ):
            self.assertIn(token, self.source)

    def test_erase_range_is_au_aligned_and_has_small_fixed_margin(self):
        for token in (
            "card.ssr.alloc_unit_kb",
            "RAW_SD_ERASE_FALLBACK_AU_BYTES",
            "RAW_SD_ERASE_MARGIN_BYTES",
            "round_up_u64",
            "SDMMC_ERASE_ARG",
        ):
            self.assertIn(token, self.source + self.header)
        self.assertRegex(
            self.source,
            r"sdmmc_erase_sectors\(\s*&recorder->card\s*,\s*0U\s*,",
        )

    def test_progress_is_written_only_at_run_boundaries(self):
        self.assertNotIn("checkpoint_write_progress", self.source)
        self.assertNotIn("next_metadata_checkpoint_bytes", self.source)
        self.assertNotIn("RAW_SD_PROGRESS_CHECKPOINT_BYTES", self.header)
        self.assertIn("write_superblocks(recorder)", self.source)

    def test_each_boot_still_starts_at_lba2048(self):
        begin_run = self.source.split("esp_err_t raw_sd_recorder_begin_run", 1)[1]
        begin_run = begin_run.split("esp_err_t raw_sd_recorder_open_segment", 1)[0]
        self.assertIn(
            "recorder->next_write_lba = RAW_SD_DATA_START_LBA",
            begin_run,
        )
        self.assertLess(
            begin_run.find("prepare_previous_run_range(recorder)"),
            begin_run.find("memset(&recorder->superblock"),
        )

    def test_card_full_accounting_includes_sector_padding_from_prior_segments(self):
        for token in (
            "capacity_bytes - recorder->superblock.physical_bytes_written",
            "recorder->write_buffer_used",
            "physically_available",
        ):
            self.assertIn(token, self.source)
        self.assertIn(
            "result == APPEND_OK && consumed != item_size",
            self.app,
        )


if __name__ == "__main__":
    unittest.main()
