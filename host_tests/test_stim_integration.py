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
)


class StimIntegrationTests(unittest.TestCase):
    def test_adc_capture_implementation_files_are_unchanged(self):
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
        self.assertIn("GPIO_NUM_5", controller)
        self.assertIn("LCD_CLK_SRC_PLL160M", source)
        self.assertRegex(source, r"lcd_ll_set_group_clock_coeff\([^;]+12\s*,\s*33\s*,\s*4\s*\)")
        self.assertRegex(source, r"lcd_ll_set_pixel_clock_prescale\([^;]+2\s*\)")
        self.assertIn("LCD_PCLK_IDX", source)
        self.assertIn("LCD_DATA_OUT0_IDX", source)
        self.assertIn("LCD_DATA_OUT1_IDX", source)

    def test_lcd_239_startup_workaround_keeps_control_lines_idle(self):
        source = (MAIN / "stim_waveform.c").read_text(encoding="utf-8")
        self.assertIn("LCD-239", source)
        self.assertRegex(
            source,
            r"lcd_ll_set_command\(\s*dev\s*,\s*8\s*,\s*0x0303U\s*\)",
        )
        self.assertRegex(
            source,
            r"lcd_ll_set_phase_cycles\(\s*dev\s*,\s*2\s*,\s*0\s*,\s*1\s*\)",
        )

    def test_stimulator_uses_lcd_cam_gdma_not_fallback_peripherals(self):
        text = "\n".join(
            path.read_text(encoding="utf-8")
            for path in MAIN.glob("stim_*.c")
        ).lower()
        self.assertIn("gdma_new_ahb_channel", text)
        self.assertIn("gdma_trig_periph_lcd", text)
        for forbidden in ("spi3_host", "gptimer", "ledc_"):
            self.assertNotIn(forbidden, text)

    def test_gpio5_debounce_is_event_driven_and_100us(self):
        text = (MAIN / "stim_controller.c").read_text(encoding="utf-8")
        self.assertRegex(text, r"STIM_DEBOUNCE_US\s+100U")
        self.assertIn("GPIO_INTR_ANYEDGE", text)
        self.assertIn("xTaskNotifyFromISR", text)
        self.assertNotIn("xQueueSend", text)
        self.assertIn("esp_timer_restart", text)

    def test_waveform_completion_and_faults_are_reported_to_task(self):
        text = (MAIN / "stim_controller.c").read_text(encoding="utf-8")
        self.assertIn("STIM_NOTIFY_SEQUENCE_COMPLETE", text)
        self.assertIn("STIM_NOTIFY_STOP_ACTIVE", text)
        self.assertIn("fifo_underflow_errors", text)
        self.assertIn("unexpected_stop_errors", text)

    def test_partial_initialization_has_a_full_teardown_path(self):
        controller = (MAIN / "stim_controller.c").read_text(encoding="utf-8")
        waveform = (MAIN / "stim_waveform.c").read_text(encoding="utf-8")
        header = (MAIN / "stim_waveform.h").read_text(encoding="utf-8")
        self.assertIn("stim_controller_cleanup", controller)
        self.assertIn("stim_waveform_deinit", controller)
        self.assertIn("stim_waveform_deinit", waveform)
        self.assertIn("stim_waveform_deinit", header)

    def test_waveform_can_be_reinitialized_after_deinit(self):
        waveform = (MAIN / "stim_waveform.c").read_text(encoding="utf-8")
        init_body = waveform.split("esp_err_t stim_waveform_init", 1)[1]
        init_body = init_body.split("esp_err_t stim_waveform_request_enabled", 1)[0]
        self.assertIn("s_waveform.fatal = false", init_body)


if __name__ == "__main__":
    unittest.main()
