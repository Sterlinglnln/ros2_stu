# Repository Guidelines

## Project Structure & Module Organization
This workspace follows the standard ROS 2 layout. Source packages live in `src/`. `fishbot_description` holds the robot model, with launch files under `launch/`, RViz and bridge configs in `config/`, and URDF/Xacro assets inside `urdf/`. Simulation assets, including Gazebo worlds, live in `worlds/`. Build artifacts appear in top-level `build/`, installed products in `install/`, and runtime logs in `log/`; never commit these directories.

## Build, Test, and Development Commands
- `source install/setup.bash`: load the workspace overlay before running ROS 2 executables.
- `colcon build --symlink-install`: rebuild packages; symlinks let you iterate on URDF and Python without reinstalls.
- `colcon test && colcon test-result --verbose`: execute any test suites and review failures.
- `ros2 launch fishbot_description display_robot.launch.py`: visualize the robot model in RViz for quick regressions.
- `ros2 launch fishbot_description gz_sim.launch.py`: start the Gazebo simulation defined in `worlds/`.

## Coding Style & Naming Conventions
Python launch files follow PEP 8: 4-space indentation, snake_case names, and module-level constants in all caps. Xacro and URDF files keep 4-space indentation and descriptive macro names (e.g., `base_xacro`, `caster_xacro`). New meshes or worlds should use lowercase, hyphenated filenames (e.g., `warehouse-small.world`). Prefer English comments; include bilingual notes only when mirroring existing documentation. Run `ament_lint_auto` locally if you add new Python modules.

## Testing Guidelines
Prioritize deterministic tests for robot description changes. Add Gazebo regression checks or RViz snapshots in `test/` directories when you introduce new hardware elements. Name test packages after the package under test (`fishbot_description_tests`). Use `colcon test --packages-select ...` for targeted runs and ensure `colcon test-result` is clean before opening a PR.

## Commit & Pull Request Guidelines
Use concise, imperative commit messages (`Add lidar sensor plugin`) and avoid batching unrelated changes. Reference relevant issue IDs in the body when available. Pull requests should summarize the impact, list validation steps (`colcon build`, `ros2 launch ...`), and attach RViz or Gazebo screenshots when visual output changes. Request a review from another maintainer and wait for CI/colcon checks to pass before merging.

## Simulation & Configuration Tips
Store reusable Gazebo or RViz configuration updates in `config/` and reference them via launch arguments. For new Xacro parameters, expose them with `DeclareLaunchArgument` so downstream agents can override values without editing the model.
