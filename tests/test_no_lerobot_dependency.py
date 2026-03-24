import os
import subprocess
import sys
from pathlib import Path


def test_import_without_lerobot():
    project_root = Path(__file__).resolve().parents[1]
    env = os.environ.copy()
    env["PYTHONPATH"] = str(project_root / "src")
    result = subprocess.run(
        [sys.executable, "-c", "import supre_robot_sdk, sys; assert 'lerobot' not in sys.modules; print('ok')"],
        cwd=project_root,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == "ok"
