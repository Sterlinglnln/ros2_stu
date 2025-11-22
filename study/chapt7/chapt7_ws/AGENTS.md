# Repository Guidelines

## Project Structure & Modules
- Workspace root follows ROS 2 layout: source lives in `src/`.
- `src/auto_patrol_robot/` and `src/fishbot_application/` are Python packages (nodes live under their package folder; configs in `config/`; resources in `resource/`; unit/lint tests in `test/`).
- `src/fishbot_description/` and `src/fishbot_navigation2/` are CMake-based packages holding launch files (`launch/`), maps/configs (`maps/`, `config/`), and robot description assets (`urdf/`, `worlds/`).
- Build artifacts land in `build/`, `install/`, and `log/`; avoid hand-editing anything there.

## Build, Test, and Development Commands
- Source your ROS 2 setup (e.g., `source /opt/ros/<distro>/setup.bash`) before working.
- Build the full workspace from the root: `colcon build --symlink-install`.
- Re-source after building: `source install/setup.bash`.
- Run all tests: `colcon test` then check results with `colcon test-result --verbose`.
- Run targeted launch examples:
  - `ros2 launch fishbot_description display_robot.launch.py`
  - `ros2 launch fishbot_navigation2 navigation2.launch.py`
  - `ros2 run auto_patrol_robot patrol_node` (ensure `config/patrol_config.yaml` is reachable).

## Coding Style & Naming
- Python: follow PEP 8 with 4-space indents; prefer snake_case for variables/functions, PascalCase for classes; keep nodes minimal and configurable via YAML.
- CMake/launch files: keep indentation consistent (2 spaces for XML, 2-4 for CMake); prefer descriptive target and file names.
- Linting hooks exist via `ament_flake8` and `ament_pep257` (see `test/` in Python packages); keep docstrings concise and modules import-safe.

## Testing Guidelines
- Unit/lint tests reside in each package’s `test/` directory; mirror module names when adding tests (e.g., `test_my_node.py`).
- Use `colcon test --packages-select <pkg>` for faster iterations; ensure `colcon test-result` is clean before proposing changes.
- Add scenario coverage for new parameters, topics, services, and launch files; include realistic config samples under `config/`.

## Commit & Pull Request Practices
- Commit messages: concise, imperative mood (e.g., “Add patrol parameter validation”); group related changes per commit.
- Before opening a PR: include a short summary of behavior changes, mention affected packages, list test commands run, and attach logs or screenshots for launch/runtime features when helpful.
- Keep PRs focused; note new configs, topics, or frames introduced so reviewers can cross-check integrations.

## Security & Configuration Tips
- Never commit secrets or machine-specific paths; keep environment-specific overrides in local copies of YAML files.
- Validate any new launch file with `--show-args` to surface required parameters, and prefer sane defaults to prevent unsafe motion.
