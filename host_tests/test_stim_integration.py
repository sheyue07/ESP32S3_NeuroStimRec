import re
import subprocess
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MAIN = ROOT / "main"

CAPTURE_FILES = (
    "continuous_rx.c",
    "continuous_rx.h",
    "frame_sync.c",
    "frame_sync.h",
    "raw_sd_segment_recorder.c",
    "raw_sd_segment_recorder.h",
    "raw_sd_segment_format.c",
    "raw_sd_segment_format.h",
)


class StimIntegrationTests(unittest.TestCase):
    def test_adc_and_raw_sd_implementation_files_are_unchanged(self):
        paths = [str(Path("main") / name) for name in CAPTURE_FILES]
        result = subprocess.run(
            ["git", "diff", "--exit-code", "b7f3247", "--", *paths],
            cwd=ROOT,
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_existing_capture_gpio_contract_is_unchanged(self):
        rx = (MAIN / "continuous_rx.h").read_text(encoding="utf-8")
        app = (MAIN / "main.c").read_text(encoding="utf-8")
        sd = (MAIN / "raw_sd_segment_recorder.c").read_text(encoding="utf-8")
        self.assertIn("CONTINUOUS_RX_CLK_GPIO       GPIO_NUM_20", rx)
        self.assertIn("CONTINUOUS_RX_DATA_GPIO      GPIO_NUM_16", rx)
        self.assertRegex(rx, r"CONTINUOUS_RX_CLOCK_HZ\s+30000000U")
        self.assertIn("RECORD_SWITCH_GPIO          GPIO_NUM_7", app)
        for pin in (41, 42, 40, 39, 1, 2):
            self.assertIn(f"GPIO_NUM_{pin}", sd)
        self.assertIn("SDMMC_MAX_FREQ_KHZ 20000", sd)

    def test_stimulator_gpio_and_clock_contract(self):
        header = (MAIN / "stim_waveform.h").read_text(encoding="utf-8")
        source = (MAIN / "stim_waveform.c").read_text(encoding="utf-8")
        controller = (MAIN / "stim_controller.c").read_text(encoding="utf-8")
        self.assertIn("GPIO_NUM_19", header)
        self.assertIn("GPIO_NUM_8", header)
        self.assertIn("GPIO_NUM_17", header)
        self.assertIn("GPIO_NUM_15", header)
        self.assertIn("GPIO_NUM_6", controller)
        self.assertIn("LCD_CLK_SRC_PLL160M", source)
        self.assertRegex(source, r"lcd_ll_set_group_clock_coeff\([^;]+24\s*,\s*33\s*,\s*8\s*\)")
        self.assertRegex(source, r"lcd_ll_set_pixel_clock_prescale\([^;]+1\s*\)")
        self.assertIn("LCD_PCLK_IDX", source)
        self.assertIn("LCD_DATA_OUT0_IDX", source)
        self.assertIn("LCD_DATA_OUT1_IDX", source)

    def test_stimulator_uses_lcd_cam_gdma_not_fallback_peripherals(self):
        text = "\n".join(
            path.read_text(encoding="utf-8")
            for path in MAIN.glob("stim_*.c")
        ).lower()
        self.assertIn("gdma_new_ahb_channel", text)
        self.assertIn("gdma_trig_periph_lcd", text)
        for forbidden in ("spi3_host", "gptimer", "ledc_"):
            self.assertNotIn(forbidden, text)

    def test_gpio6_debounce_is_event_driven_and_100us(self):
        text = (MAIN / "stim_controller.c").read_text(encoding="utf-8")
        self.assertRegex(text, r"STIM_DEBOUNCE_US\s+100U")
        self.assertIn("GPIO_INTR_ANYEDGE", text)
        self.assertIn("xQueueSendFromISR", text)
        self.assertIn("esp_timer_restart", text)


if __name__ == "__main__":
    unittest.main()
