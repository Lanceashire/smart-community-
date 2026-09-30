# Open-source provenance

This repository integrates small, task-relevant parts of RoboMaster projects.

1. `src/sentry_motion_manager/` is copied from [Boombroke/Sentry26](https://github.com/Boombroke/Sentry26), commit `9f3ae17627cef5ca1f102a27668f8b5e78a2cdb1`. Its source files and `package.xml` identify Apache-2.0 and retain the original copyright notices. This repository configures it for a 0.22 m differential robot and passes its stamped output to Gazebo through `community_demo/cmd_bridge.py`; the original Livox, terrain, RM referee and serial modules are not included.
2. `src/community_demo/community_demo/bounded_pid.py` is a reduced Python adaptation of the bounded PID in [SMBU PolarBear's `pb_omni_pid_pursuit_controller`](https://github.com/SMBU-PolarBear-Robotics-Team/pb_omni_pid_pursuit_controller), commit `6f0ef65f52b65aef0bf0350926947f7813c5b568`, `src/pid.cpp`, copyright 2025 Lihan Chen, Apache-2.0. The full omnidirectional Nav2 plugin is inappropriate for this small differential chassis; this adaptation keeps only bounded translation and heading control.

3. `src/community_demo/gui/overview.config` adapts the plugin layout of Gazebo Sim 8.15.0's installed `gui/gui.config` (Open Source Robotics Foundation and Gazebo contributors, Apache-2.0) to a compact overview without editor side panels. The physical-pose publisher is Gazebo's built-in [OdometryPublisher](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1OdometryPublisher.html).

The generated world, route state machine, ROS bridge adapter, traffic-light colour baseline, physical-lap verifier and documentation are new code for this project. The repository root `LICENSE` contains Apache-2.0. No code or assets from `Gh0stown/AiC-2026` are used.
