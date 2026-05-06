# Repository Guidelines

## Project Structure & Module Organization

This is a Python package using a `src/` layout. Runtime code lives in `src/supre_robot_sdk/`, with the main facade in `core/robot.py`, shared interfaces and configuration in `core/`, and hardware adapters in `hardware/` (`eyou/`, `jodell/`, and `interpolation/`). Unit tests live in `tests/` and mirror package behavior with files named `test_*.py`. Example robot configuration is in `examples/robot_config.yaml`; design and implementation notes are in `docs/`.

## Build, Test, and Development Commands

- `python -m pip install -e ".[dev]"`: install the SDK in editable mode with pytest.
- `python -m pytest`: run the full unit test suite configured by `pyproject.toml`.
- `python -m pytest tests/test_robot.py`: run one focused test module.
- `python -m pip install -e .`: install runtime dependencies only.

The tests mock hardware bindings, so `eu_motor_py` and `jodell_gripper_py` are not required for normal unit testing.

## Coding Style & Naming Conventions

Use Python 3.10+ syntax and follow the existing style: 4-space indentation, type hints for public interfaces, `from __future__ import annotations` where forward references are useful, and clear `snake_case` names for functions, modules, variables, and test cases. Classes use `PascalCase`; private helpers use a leading underscore. Keep hardware adapter code isolated under `src/supre_robot_sdk/hardware/<vendor>/` and register new hardware types through the existing base hardware registry.

No formatter or linter is currently configured in `pyproject.toml`; keep edits PEP 8-compatible and consistent with nearby code.

## Testing Guidelines

Pytest is the project test framework. Add tests under `tests/` with filenames matching `test_*.py` and test functions named `test_<behavior>`. Prefer mocked or fake hardware classes over physical devices so tests stay deterministic and CI-friendly. Cover configuration parsing, hardware manager behavior, adapter edge cases, and public `SupreRobot` facade changes.

## Commit & Pull Request Guidelines

The current Git history uses very short lowercase commit subjects such as `init` and `first commit`. Continue with concise, imperative subjects, but make them more descriptive when possible, for example `add jodell gripper tests`.

Pull requests should include a short summary, the tests run, and any hardware/runtime dependency implications. Link related issues or design docs when changing control flow, configuration format, or hardware adapter behavior.

## Security & Configuration Tips

Do not commit machine-specific CAN bus settings, credentials, or private robot calibration data. Keep shareable examples in `examples/`, and document required local hardware bindings in the PR when adapter behavior depends on external packages.
