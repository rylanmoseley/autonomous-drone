#!/usr/bin/env python3
"""
Unit and integration tests for the Python Web Simulator (sim_web.py).
Tests ctypes byte contract compatibility, kinematic follower physics,
3D geometric collision detection, and camera perception raycasting.
"""

import unittest
import math
import ctypes
import sys
import os

# Ensure autopilot/sim is in python import path
sim_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if sim_dir not in sys.path:
    sys.path.insert(0, sim_dir)

import sim_web


class TestCtypesPacketLayouts(unittest.TestCase):
    """Verify that ctypes packed struct sizes match shared C++ contract sizes."""

    def test_pose_struct_size(self):
        self.assertEqual(ctypes.sizeof(sim_web.Pose), 64)

    def test_flight_command_struct_size(self):
        self.assertEqual(ctypes.sizeof(sim_web.FlightCommand), 34)

    def test_vision_packet_struct_size(self):
        self.assertEqual(ctypes.sizeof(sim_web.VisionPacket), 588)

    def test_ui_command_packet_struct_size(self):
        self.assertEqual(ctypes.sizeof(sim_web.UICommandPacket), 45)

    def test_telemetry_packet_struct_size(self):
        self.assertEqual(ctypes.sizeof(sim_web.TelemetryPacket), 152)

    def test_tag_measurement_struct_size(self):
        self.assertEqual(ctypes.sizeof(sim_web.TagMeasurement), 35)

    def test_flight_command_binary_roundtrip(self):
        cmd = sim_web.FlightCommand()
        cmd.tx = 1.25
        cmd.ty = -2.5
        cmd.tz = 1.15
        cmd.tyaw = 0.785
        cmd.en = 1
        cmd.est = 0

        raw_bytes = ctypes.string_at(ctypes.byref(cmd), ctypes.sizeof(cmd))
        self.assertEqual(len(raw_bytes), 34)

        recovered = sim_web.FlightCommand.from_buffer_copy(raw_bytes)
        self.assertAlmostEqual(recovered.tx, 1.25, places=5)
        self.assertAlmostEqual(recovered.ty, -2.5, places=5)
        self.assertAlmostEqual(recovered.tz, 1.15, places=5)
        self.assertAlmostEqual(recovered.tyaw, 0.785, places=4)
        self.assertEqual(recovered.en, 1)
        self.assertEqual(recovered.est, 0)


class TestFollowerPhysics(unittest.TestCase):
    """Verify kinematic follower lag, saturation, wind drift, and vision correction."""

    def setUp(self):
        self.follower = sim_web.FollowerSimulator()
        self.follower.noise_enabled = False
        self.reset_follower(0.0, 0.0, 1.0, 0.0)

    def reset_follower(self, x=0.0, y=0.0, z=1.0, yaw=0.0):
        self.follower.true_x = x
        self.follower.true_y = y
        self.follower.true_z = z
        self.follower.true_yaw = yaw
        self.follower.vx = 0.0
        self.follower.vy = 0.0
        self.follower.vz = 0.0
        self.follower.feedback_x = x
        self.follower.feedback_y = y
        self.follower.feedback_z = z
        self.follower.feedback_yaw = yaw

    def test_step_response_approaches_target(self):
        cmd = sim_web.FlightCommand()
        cmd.tx = 2.0
        cmd.ty = 1.0
        cmd.tz = 1.5
        cmd.tyaw = 0.5
        cmd.en = 1

        for _ in range(100):
            self.follower.step(cmd, dt=0.05)

        self.assertAlmostEqual(self.follower.true_x, 2.0, delta=0.05)
        self.assertAlmostEqual(self.follower.true_y, 1.0, delta=0.05)
        self.assertAlmostEqual(self.follower.true_z, 1.5, delta=0.05)
        self.assertAlmostEqual(self.follower.true_yaw, 0.5, delta=0.05)

    def test_vector_acceleration_limiting(self):
        self.follower.max_acceleration = 5.0
        cmd = sim_web.FlightCommand()
        cmd.tx = 100.0
        cmd.ty = 100.0
        cmd.tz = 50.0
        cmd.en = 1

        dt = 0.01
        prev_vx, prev_vy, prev_vz = 0.0, 0.0, 0.0
        for i in range(50):
            self.follower.step(cmd, dt=dt)
            if i > 0:
                ax = (self.follower.vx - prev_vx) / dt
                ay = (self.follower.vy - prev_vy) / dt
                az = (self.follower.vz - prev_vz) / dt
                lin_acc = math.sqrt(ax * ax + ay * ay + az * az)
                self.assertLessEqual(lin_acc, self.follower.max_acceleration + 0.1)
            prev_vx = self.follower.vx
            prev_vy = self.follower.vy
            prev_vz = self.follower.vz

    def test_velocity_saturation_limits(self):
        self.follower.max_velocity_xy = 2.0
        self.follower.max_velocity_z = 1.0
        cmd = sim_web.FlightCommand()
        cmd.tx = 50.0
        cmd.ty = 50.0
        cmd.tz = 50.0
        cmd.en = 1

        for _ in range(100):
            self.follower.step(cmd, dt=0.05)
            h_speed = math.sqrt(self.follower.vx ** 2 + self.follower.vy ** 2)
            self.assertLessEqual(h_speed, self.follower.max_velocity_xy + 1e-4)
            self.assertLessEqual(abs(self.follower.vz), self.follower.max_velocity_z + 1e-4)

    def test_wind_drift_integration(self):
        self.follower.drift_vel_x = 0.20
        self.follower.drift_vel_y = -0.10
        cmd = sim_web.FlightCommand()
        cmd.tx = 0.0
        cmd.ty = 0.0
        cmd.tz = 1.0
        cmd.en = 1

        for _ in range(40):
            self.follower.step(cmd, dt=0.05)

        # Controller opposes drift, resulting in steady-state wind offset
        self.assertGreater(self.follower.total_dist, 0.02)
        self.assertGreater(self.follower.true_x, 0.02)
        self.assertLess(self.follower.true_y, -0.01)

    def test_tag_drift_correction(self):
        self.reset_follower(1.5, -0.7, 1.0, 0.0)
        self.follower.correct_drift(2.0, -1.0, 1.0, 0.0, confidence=0.95)

        # Innovation should shift feedback pose closer to surveyed location
        self.assertGreater(self.follower.feedback_x, 1.7)
        self.assertLess(self.follower.feedback_y, -0.8)


class TestCollisionDetection(unittest.TestCase):
    """Verify 3D hoop rim, aperture, and pylon collision detection."""

    def setUp(self):
        self.goals = [
            {"shape": 1, "x": 2.5, "y": 0.0, "z": 1.0, "yaw": 0.0},     # Circle (R=0.22)
            {"shape": 2, "x": 2.5, "y": -1.3, "z": 1.0, "yaw": 0.0},    # Triangle
            {"shape": 3, "x": 2.5, "y": 1.3, "z": 1.0, "yaw": 0.0},     # Square (L=0.1768)
        ]

    def test_aperture_pass_through_no_collision(self):
        collided, reason = sim_web.check_collision(2.5, 0.0, 1.0, 0.0, self.goals)
        self.assertFalse(collided)

    def test_circle_rim_collision(self):
        # Circle rim at radius 0.22m in gate plane (u = 0, v = 0.22)
        collided, reason = sim_web.check_collision(2.5, 0.22, 1.0, 0.0, self.goals)
        self.assertTrue(collided)
        self.assertIn("Gate 1 hoop rim", reason)

    def test_vertical_pylon_collision(self):
        # Post below the gate (z = 0.40m, directly under circle gate center at x=2.5, y=0.0)
        collided, reason = sim_web.check_collision(2.5, 0.0, 0.40, 0.0, self.goals)
        self.assertTrue(collided)
        self.assertIn("Gate 1 pylon", reason)

    def test_triangle_aperture_no_collision(self):
        collided, reason = sim_web.check_collision(2.5, -1.3, 1.0, 0.0, self.goals)
        self.assertFalse(collided)

    def test_triangle_apex_collision(self):
        # Triangle apex is at (u=0, w=+0.35) -> z = 1.35
        collided, reason = sim_web.check_collision(2.5, -1.3, 1.35, 0.0, self.goals)
        self.assertTrue(collided)
        self.assertIn("Gate 2 hoop rim", reason)

    def test_square_aperture_no_collision(self):
        collided, reason = sim_web.check_collision(2.5, 1.3, 1.0, 0.0, self.goals)
        self.assertFalse(collided)

    def test_square_edge_collision(self):
        # Square edge at w = 0.1768m -> z = 1.1768m
        collided, reason = sim_web.check_collision(2.5, 1.3, 1.1768, 0.0, self.goals)
        self.assertTrue(collided)
        self.assertIn("Gate 3 hoop rim", reason)

    def test_far_drone_no_collision(self):
        collided, reason = sim_web.check_collision(0.0, 0.0, 1.0, 0.0, self.goals)
        self.assertFalse(collided)


class TestCameraPerception(unittest.TestCase):
    """Verify camera frustum, range limits, and tag raycasting."""

    def setUp(self):
        self.goals = [
            {"shape": 1, "x": 2.5, "y": 0.0, "z": 1.0, "yaw": 0.0},
            {"shape": 2, "x": 4.5, "y": 0.0, "z": 1.0, "yaw": 0.0},
        ]
        self.tags = [
            {"id": 10, "x": 1.5, "y": 0.0, "z": 1.0, "size": 0.16},
            {"id": 11, "x": -2.0, "y": 0.0, "z": 1.0, "size": 0.16}, # Behind drone
        ]

    def test_goal_ahead_detected(self):
        # Drone at (0.5, 0.0, 1.0) facing +X (yaw = 0.0)
        detected = sim_web.evaluate_camera_vision(0.5, 0.0, 1.0, 0.0, self.goals)
        shapes = [g["shape"] for g in detected]
        self.assertIn(1, shapes)
        g1 = next(g for g in detected if g["shape"] == 1)
        self.assertGreater(g1["conf"], 0.7)

    def test_goal_behind_not_detected(self):
        # Drone facing -X (yaw = pi)
        detected = sim_web.evaluate_camera_vision(0.5, 0.0, 1.0, math.pi, self.goals)
        self.assertEqual(len(detected), 0)

    def test_tag_ahead_detected(self):
        tag_detections = sim_web.evaluate_camera_tags(0.5, 0.0, 1.0, 0.0, self.tags)
        tag_ids = [t["id"] for t in tag_detections]
        self.assertIn(10, tag_ids)
        self.assertNotIn(11, tag_ids) # Tag 11 is behind camera

    def test_default_surveyed_tags_loaded(self):
        tags = sim_web.load_default_surveyed_tags()
        self.assertGreater(len(tags), 0)
        for t in tags:
            self.assertIn("id", t)
            self.assertIn("x", t)
            self.assertIn("y", t)
            self.assertIn("z", t)


class TestRepeatedSimulations(unittest.TestCase):
    """Verify repeated simulations and run-after-run sequences for collision-free orthogonal traversal."""

    def setUp(self):
        self.goals = [
            {"id": 1, "shape": 1, "x": 2.5, "y": -1.3, "z": 1.0, "yaw": 0.0},
            {"id": 2, "shape": 2, "x": 2.5, "y":  0.0, "z": 1.0, "yaw": 0.0},
            {"id": 3, "shape": 3, "x": 2.5, "y":  1.3, "z": 1.0, "yaw": 0.0},
        ]

    def test_repeated_runs_traversal_direction_and_no_collision(self):
        """Run 5 repeated simulations, verifying order (1->2->3), right directions (+X, -X, +X), and zero collisions."""
        num_runs = 5
        for run in range(num_runs):
            # Waypoint corridor segments through default arena gates:
            # 1. Takeoff from (0, 0, 1) and approach Gate 1 entrance standoff (1.75, -1.3, 1.0)
            # 2. Traverse Gate 1 orthogonally from (1.75, -1.3, 1.0) to (3.25, -1.3, 1.0) [+X direction]
            # 3. Transit to Gate 2 entrance standoff (3.25, 0.0, 1.0)
            # 4. Traverse Gate 2 orthogonally from (3.25, 0.0, 1.0) to (1.75, 0.0, 1.0) [-X direction]
            # 5. Transit to Gate 3 entrance standoff (1.75, 1.3, 1.0)
            # 6. Traverse Gate 3 orthogonally from (1.75, 1.3, 1.0) to (3.25, 1.3, 1.0) [+X direction]
            segments = [
                ((0.0, 0.0, 1.0, 0.0), (1.75, -1.3, 1.0, 0.0)),         # transit to Gate 1 standoff
                ((1.75, -1.3, 1.0, 0.0), (3.25, -1.3, 1.0, 0.0)),       # Gate 1 traversal (+X)
                ((3.25, -1.3, 1.0, math.pi), (3.25, 0.0, 1.0, math.pi)),# transit to Gate 2 standoff
                ((3.25, 0.0, 1.0, math.pi), (1.75, 0.0, 1.0, math.pi)), # Gate 2 traversal (-X)
                ((1.75, 0.0, 1.0, 0.0), (1.75, 1.3, 1.0, 0.0)),         # transit to Gate 3 standoff
                ((1.75, 1.3, 1.0, 0.0), (3.25, 1.3, 1.0, 0.0)),         # Gate 3 traversal (+X)
            ]

            traversed_gates = []
            for (p_start, p_end) in segments:
                steps = 50
                for s in range(steps + 1):
                    alpha = s / float(steps)
                    x = p_start[0] + alpha * (p_end[0] - p_start[0])
                    y = p_start[1] + alpha * (p_end[1] - p_start[1])
                    z = p_start[2] + alpha * (p_end[2] - p_start[2])
                    yaw = p_start[3] + alpha * (p_end[3] - p_start[3])

                    collided, msg = sim_web.check_collision(x, y, z, yaw, self.goals)
                    self.assertFalse(collided, f"Run {run} collided: {msg} at ({x:.3f}, {y:.3f}, {z:.3f})")

                    # Check aperture crossing at X = 2.5
                    if abs(x - 2.5) < 0.02:
                        for g in self.goals:
                            if math.hypot(y - g["y"], z - g["z"]) < 0.25:
                                dir_str = "+X" if (p_end[0] > p_start[0]) else "-X"
                                if not traversed_gates or traversed_gates[-1][0] != g["shape"]:
                                    traversed_gates.append((g["shape"], dir_str))

            self.assertEqual(len(traversed_gates), 3, f"Run {run} traversed count != 3")
            self.assertEqual(traversed_gates[0], (1, "+X"), f"Run {run} Gate 1 not +X")
            self.assertEqual(traversed_gates[1], (2, "-X"), f"Run {run} Gate 2 not -X")
            self.assertEqual(traversed_gates[2], (3, "+X"), f"Run {run} Gate 3 not +X")

    def test_continuous_run_after_run_chaining(self):
        """Verify back-to-back mission chaining where the drone finishes a mission and continues next without resetting."""
        # Mission A: 1 -> 2 -> 3 (ends at X=3.25, Y=1.3)
        # Mission B: 3 -> 2 -> 1 (starts at X=3.25, Y=1.3, ends at X=1.75, Y=-1.3)
        # Mission C: 1 -> 2 -> 3 (starts at X=1.75, Y=-1.3, ends at X=3.25, Y=1.3)
        chained_legs = [
            # Mission A
            ((0.0, 0.0, 1.0, 0.0), (1.75, -1.3, 1.0, 0.0)),
            ((1.75, -1.3, 1.0, 0.0), (3.25, -1.3, 1.0, 0.0)),
            ((3.25, -1.3, 1.0, math.pi), (3.25, 0.0, 1.0, math.pi)),
            ((3.25, 0.0, 1.0, math.pi), (1.75, 0.0, 1.0, math.pi)),
            ((1.75, 0.0, 1.0, 0.0), (1.75, 1.3, 1.0, 0.0)),
            ((1.75, 1.3, 1.0, 0.0), (3.25, 1.3, 1.0, 0.0)),
            # Mission B (reverse)
            ((3.25, 1.3, 1.0, math.pi), (1.75, 1.3, 1.0, math.pi)),
            ((1.75, 1.3, 1.0, 0.0), (1.75, 0.0, 1.0, 0.0)),
            ((1.75, 0.0, 1.0, 0.0), (3.25, 0.0, 1.0, 0.0)),
            ((3.25, 0.0, 1.0, math.pi), (3.25, -1.3, 1.0, math.pi)),
            ((3.25, -1.3, 1.0, math.pi), (1.75, -1.3, 1.0, math.pi)),
            # Mission C (forward again)
            ((1.75, -1.3, 1.0, 0.0), (3.25, -1.3, 1.0, 0.0)),
            ((3.25, -1.3, 1.0, math.pi), (3.25, 0.0, 1.0, math.pi)),
            ((3.25, 0.0, 1.0, math.pi), (1.75, 0.0, 1.0, math.pi)),
            ((1.75, 0.0, 1.0, 0.0), (1.75, 1.3, 1.0, 0.0)),
            ((1.75, 1.3, 1.0, 0.0), (3.25, 1.3, 1.0, 0.0)),
        ]

        for (p_start, p_end) in chained_legs:
            steps = 40
            for s in range(steps + 1):
                alpha = s / float(steps)
                x = p_start[0] + alpha * (p_end[0] - p_start[0])
                y = p_start[1] + alpha * (p_end[1] - p_start[1])
                z = p_start[2] + alpha * (p_end[2] - p_start[2])
                yaw = p_start[3] + alpha * (p_end[3] - p_start[3])

                collided, msg = sim_web.check_collision(x, y, z, yaw, self.goals)
                self.assertFalse(collided, f"Chained mission collided: {msg} at ({x:.3f}, {y:.3f}, {z:.3f})")


if __name__ == '__main__':
    unittest.main()
