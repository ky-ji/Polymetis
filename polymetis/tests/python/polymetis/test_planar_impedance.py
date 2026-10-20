"""Focused offline checks; no RobotInterface or network connection."""

import torch

from torchcontrol.policies.planar_impedance import PlanarImpedanceControl


class CartesianFixture(torch.nn.Module):
    """A linear kinematic fixture isolates task-axis and watchdog behavior."""

    def forward_kinematics(self, q):
        return q[:3], torch.tensor([0.0, 0.0, 0.0, 1.0])

    def compute_jacobian(self, q):
        return torch.cat([torch.eye(6), torch.zeros((6, 1))], dim=1)

    def inverse_dynamics(self, q, dq, ddq):
        return torch.zeros_like(q)


def make_policy(**options):
    return PlanarImpedanceControl(
        torch.zeros(7), CartesianFixture(), torch.eye(3),
        [-0.1, -0.1], [0.1, 0.1],
        [300, 300, 60, 20, 20, 20], [20, 20, 5, 4, 4, 4],
        [12, 12, 3, 3, 3, 3],
        max_speed=0.2, max_acceleration=1.0,
        velocity_time_constant=0.02, max_tracking_error=0.02,
        normal_travel=0.01, preload=1.0, preload_ramp_s=0.2,
        joint_damping=[0.1] * 7,
        **options,
    )


def command(policy, sequence, velocity=(0.15, 0.0), paused=False):
    policy.update({
        "velocity_desired": torch.tensor(velocity),
        "command_sequence": torch.tensor([sequence], dtype=torch.float64),
        "command_paused": torch.tensor([float(paused)]),
    })


def state(q=None):
    return {"joint_positions": torch.zeros(7) if q is None else q, "joint_velocities": torch.zeros(7)}


def test_scripted_blocked_motion_has_bounded_error_and_no_normal_target_drift():
    policy = torch.jit.script(make_policy())
    for step in range(1000):
        if step % 10 == 0:
            command(policy, step + 1)
        output = policy(state())
        assert float(torch.norm(output["planar_reference_xy"])) <= 0.020001
        assert abs(float(output["planar_normal_wrench"][0])) <= 3.0
        assert float(policy.normal_reference) == 0.0
    assert -1.001 < float(output["planar_normal_wrench"][0]) < -0.999


def test_pause_and_resume_reanchor_without_chasing_old_xy():
    policy = torch.jit.script(make_policy())
    for step in range(100):
        command(policy, step + 1)
        policy(state())
    actual = torch.tensor([-0.04, 0.03, 0.0, 0.0, 0.0, 0.0, 0.0])
    command(policy, 101, paused=True)
    output = policy(state(actual))
    assert torch.allclose(output["planar_reference_xy"], actual[:2])
    command(policy, 102)
    output = policy(state(actual))
    assert float(torch.norm(output["planar_reference_xy"] - actual[:2])) < 2e-6


def test_timeout_requires_pause_before_new_command_can_restart():
    policy = torch.jit.script(make_policy())
    command(policy, 1)
    for _ in range(200):
        output = policy(state())
    assert torch.allclose(output["planar_reference_xy"], torch.zeros(2))
    command(policy, 2)
    output = policy(state())
    assert torch.allclose(output["planar_reference_xy"], torch.zeros(2))
    command(policy, 3, paused=True)
    policy(state())
    command(policy, 4)
    output = policy(state())
    assert float(output["planar_reference_xy"][0]) > 0


def test_leaving_normal_band_removes_surface_force_without_blocking_xy():
    policy = torch.jit.script(make_policy())
    command(policy, 1)
    policy(state())
    lifted = torch.tensor([0.0, 0.0, 0.03, 0.0, 0.0, 0.0, 0.0])
    command(policy, 2)
    output = policy(state(lifted))
    assert float(output["planar_normal_wrench"][0]) == 0
    command(policy, 3, velocity=(0.1, 0.1))
    output = policy(state(lifted))
    assert float(output["planar_reference_xy"][0]) > 0
    assert float(output["planar_reference_xy"][1]) > 0
    assert float(output["planar_normal_wrench"][0]) == 0
    command(policy, 4, paused=True)
    output = policy(state(lifted))
    assert float(output["planar_reference_z"][0]) == float(lifted[2])


def test_workspace_edge_keeps_tangential_motion_and_rejects_outward_motion():
    policy = torch.jit.script(make_policy())
    actual = torch.tensor([0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    for step in range(100):
        command(policy, step + 1, velocity=(0.1, 0.1))
        output = policy(state(actual))
    assert abs(float(output["planar_reference_xy"][0]) - 0.1) < 1e-7
    assert float(output["planar_reference_xy"][1]) > 0


def test_measured_xy_overshoot_allows_immediate_inward_and_tangential_motion():
    for outside in (0.10052, 0.13):
        policy = torch.jit.script(make_policy())
        actual = torch.tensor([outside, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        command(policy, 1, paused=True)
        policy(state(actual))
        for step in range(60):
            command(policy, step + 2, velocity=(-0.1, 0.1))
            output = policy(state(actual))
        assert float(output["planar_reference_xy"][0]) < outside - 0.0005
        assert float(output["planar_reference_xy"][1]) > 0.0005
        assert not policy.restart_required
        # Recovery is a velocity command, not a snap to the box boundary.
        assert float(torch.norm(policy.velocity_xy)) <= 0.2


def test_actual_crossing_does_not_latch_or_block_the_other_axis():
    policy = torch.jit.script(make_policy())
    for step in range(100):
        command(policy, step + 1, velocity=(0.1, 0.1))
        x = 0.099 + step * 0.00002
        actual = torch.tensor([x, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        output = policy(state(actual))
    assert policy.was_active
    assert not policy.restart_required
    assert float(policy.velocity_xy[0]) == 0
    assert float(output["planar_reference_xy"][1]) > 0.001


def test_normal_overshoot_allows_downward_recovery_and_xy_commands():
    policy = torch.jit.script(make_policy(start_in_contact=False))
    actual = torch.tensor([0.0, 0.0, 0.2006, 0.0, 0.0, 0.0, 0.0])
    command(policy, 1, paused=True)
    policy(state(actual))
    policy.update({"normal_velocity_desired": torch.tensor([-0.05])})
    for step in range(100):
        command(policy, step + 2, velocity=(0.1, 0))
        output = policy(state(actual))
    assert float(output["planar_reference_z"][0]) < float(actual[2]) - 0.001
    assert float(output["planar_reference_xy"][0]) > 0.001
    assert float(policy.normal_velocity[0]) >= -0.05
    assert not policy.restart_required


def test_braking_near_edge_preserves_inward_motion_without_latching():
    policy = torch.jit.script(make_policy())
    actual = torch.tensor([0.0995, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    for step in range(100):
        command(policy, step + 1, velocity=(0.15, 0))
        output = policy(state(actual))
        assert float(output["planar_reference_xy"][0]) <= 0.100001
        assert float(policy.velocity_xy[0]) <= (2 * 0.0005) ** 0.5 + 1e-5
    for step in range(100):
        command(policy, step + 101, velocity=(-0.15, 0))
        output = policy(state(actual))
    assert float(output["planar_reference_xy"][0]) < float(actual[0])
    assert not policy.restart_required


def test_invalid_velocity_cannot_publish_nonfinite_torques():
    policy = torch.jit.script(make_policy())
    command(policy, 1, velocity=(float("nan"), 0.0))
    output = policy(state())
    assert bool(torch.isfinite(output["joint_torques"]).all())
    assert torch.allclose(output["planar_reference_xy"], torch.zeros(2))


def test_free_translation_lifts_and_pause_discards_queued_height():
    policy = torch.jit.script(make_policy(start_in_contact=False))
    policy.update({"normal_velocity_desired": torch.tensor([0.05])})
    for step in range(200):
        command(policy, step + 1, velocity=(0, 0))
        output = policy(state())
    assert 0.004 < float(output["planar_reference_z"][0]) < 0.02
    assert 0 < float(output["planar_normal_wrench"][0]) <= 8
    command(policy, 201, paused=True)
    output = policy(state())
    assert float(output["planar_reference_z"][0]) == 0
    assert float(output["planar_normal_wrench"][0]) == 0


def test_contact_switch_anchors_here_and_ignores_vertical_input():
    policy = torch.jit.script(make_policy(start_in_contact=False))
    lifted = torch.tensor([0, 0, 0.08, 0, 0, 0, 0], dtype=torch.float32)
    command(policy, 1, paused=True)
    policy(state(lifted))
    policy.update({"command_contact": torch.ones(1), "normal_velocity_desired": torch.tensor([0.05])})
    command(policy, 2, paused=True)
    output = policy(state(lifted))
    assert float(output["planar_normal_wrench"][0]) == 0
    for step in range(100):
        command(policy, step + 3)
        output = policy(state(lifted))
    assert abs(float(output["planar_reference_z"][0]) - 0.08) < 1e-6
    assert -1 <= float(output["planar_normal_wrench"][0]) < 0
    policy.update({"command_contact": torch.zeros(1)})
    command(policy, 104, paused=True)
    output = policy(state(lifted))
    assert float(output["planar_normal_wrench"][0]) == 0


def test_stale_vertical_command_brakes_without_later_height_catchup():
    policy = torch.jit.script(make_policy(start_in_contact=False))
    policy.update({"normal_velocity_desired": torch.tensor([0.05])})
    command(policy, 1, velocity=(0, 0))
    for _ in range(200):
        output = policy(state())
    assert float(output["planar_reference_z"][0]) == 0
    command(policy, 2, velocity=(0, 0))
    output = policy(state())
    assert float(output["planar_reference_z"][0]) == 0
