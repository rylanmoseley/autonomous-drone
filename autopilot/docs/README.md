# Autopilot Documentation Index

This directory provides in-depth technical documentation for each subsystem and architectural component within the **HEXAGON Flight Autonomous Quadcopter Autopilot**.

---

## Subsystem Documentation

| Subsystem | Primary Responsibilities | Key Classes & Modules |
| :--- | :--- | :--- |
| [**Architecture & Overview**](architecture.md) | High-level system architecture, threading model, deterministic 20 Hz loop, coordinate frames, and IPC interfaces | [`AutopilotNode`](../autopilot_node.h) |
| [**Mission Core & State Machine**](mission_core.md) | Flight state transitions, mission sequencing, safety supervisors, tag survey registry, and network telemetry bridges | [`MissionStateMachine`](../mission_state_machine.h), [`MissionSequencer`](../mission_sequencer.h), [`SafetySupervisor`](../safety_supervisor.h), [`PerceptionCoordinator`](../perception_coordinator.h), [`TelemetryBridge`](../telemetry_bridge.h) |
| [**Path Planner**](path_planner.md) | 3D orthogonal gate traversal lines, standoff approach corridors, potential field obstacle avoidance, and kinematic saturation | [`PathPlanner`](../path-planner/path_planner.h), [`KinematicProfileFilter`](../path-planner/kinematic_profile_filter.h), [`ApproachCorridorPlanner`](../path-planner/approach_corridor_planner.h), [`OrthogonalPathGenerator`](../path-planner/orthogonal_path_generator.h), [`ObstacleAvoidancePotentialField`](../path-planner/obstacle_avoidance_potential_field.h), [`PhaseAlignmentEvaluator`](../path-planner/phase_alignment_evaluator.h) |
| [**Goal Estimator**](goal_estimator.md) | Stateful goal tracking, first-order alpha smoothing, spatial outlier glitch rejection, planar yaw symmetry projection, and telemetry formatting | [`GoalEstimator`](../goal-estimator/goal_estimator.h), [`GoalTrackFilter`](../goal-estimator/goal_track_filter.h), [`GoalRegistry`](../goal-estimator/goal_registry.h), [`GoalTelemetryFormatter`](../goal-estimator/goal_telemetry_formatter.h) |
| [**MAVLink Daemon**](mavlink_daemon.md) | Zero-dependency MAVLink v2 serialization, UDP communication, NED/NEU coordinate conversions, CRC-16 checksums, and FCU watchdogs | [`MavlinkDaemon`](../mavlink-daemon/mavlink_daemon.h), [`MavlinkPacketCodec`](../mavlink-daemon/mavlink_packet_codec.h), [`MavlinkMessageDispatcher`](../mavlink-daemon/mavlink_message_dispatcher.h), [`MavlinkNetworkChannel`](../mavlink-daemon/mavlink_network_channel.h), [`MavlinkFcuWatchdog`](../mavlink-daemon/mavlink_fcu_watchdog.h) |
| [**Simulation Subsystem**](simulation.md) | Closed-loop SIL testing harness, follower quadcopter model with lag/noise/drift, synthetic camera perception, and 3D rim/pylon collision detection | [`AutopilotSimHarness`](../sim/include/autopilot_sim_harness.h), [`FcuFollowerSimulator`](../sim/include/follower_simulator.h), [`SimCameraPerception`](../sim/include/sim_camera_perception.h), [`CollisionDetector`](../sim/include/collision_detector.h) |

---

## Architectural Principles

1. **Modular Subsystems**: Subsystems are separated into single-responsibility units with decoupled headers, dedicated unit tests, and minimal dependencies.
2. **Deterministic 20 Hz Real-Time Execution**: Periodic logic runs at 50 ms step intervals (`dt = 0.05`), clamping maximum step drift to protect trajectory integrity.
3. **Thread Safety**: Inter-thread communication between network listener threads (UI, Vision, MAVLink) and the flight logic thread is synchronized via fine-grained mutex guards.
4. **Authoritative Survey Ownership**: The Autopilot companion computer, not the flight controller, owns the global survey coordinate frame and tag map.
