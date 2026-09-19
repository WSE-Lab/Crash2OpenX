import shutil
import subprocess

import pytest

from tools.validate_demo_run import validate


@pytest.mark.parametrize("source,expected", [("testsrc2=size=160x90:rate=10", True),
                                            ("color=c=black:size=160x90:rate=10", False)])
def test_checks_actual_mp4_when_jpeg_intermediates_are_absent(tmp_path, source, expected):
    if not shutil.which("ffmpeg"):
        pytest.skip("ffmpeg is required for video validation")
    subprocess.run(["ffmpeg", "-v", "error", "-f", "lavfi", "-i", source,
                    "-t", "2", "-c:v", "libx264", "-pix_fmt", "yuv420p",
                    str(tmp_path / "carla_rgb.mp4")], check=True)
    check = next(c for c in validate(tmp_path)["checks"] if c["name"] == "dynamic_video")
    assert check["passed"] is expected, check
