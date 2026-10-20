"""Task-space planar impedance candidate; not selected by RobotInterface.

Translation mode accepts XYZ velocity while holding orientation. Contact mode
accepts only tangential velocity and captures the current normal reference.
Normal compliance and a bounded preload replace joint-position contact tracking. This is not measured force
regulation: commanded normal wrench is not a measurement of contact force.
"""

import math
from typing import Dict

import torch

import torchcontrol as toco
from torchcontrol.transform import Rotation as R


class PlanarImpedanceControl(toco.PolicyModule):
    def __init__(
        self,
        joint_pos_current,
        robot_model,
        table_rotation,
        xy_min,
        xy_max,
        stiffness,
        damping,
        wrench_limits,
        max_speed,
        max_acceleration,
        velocity_time_constant,
        max_tracking_error,
        normal_travel,
        preload,
        preload_ramp_s,
        joint_damping,
        dt=0.001,
        timeout=0.15,
        ignore_gravity=True,
        start_in_contact=True,
        normal_bounds=None,
        normal_speed=0.05,
        normal_acceleration=0.3,
        free_normal_stiffness=300.0,
        free_normal_damping=15.0,
        free_normal_force_up=8.0,
    ):
        super().__init__()

        def vector(value, size, name):
            result = torch.as_tensor(value, dtype=torch.float32).detach().clone()
            if result.shape != (size,) or not bool(torch.isfinite(result).all()):
                raise ValueError(name + " has invalid dimensions or values")
            return result

        q = vector(joint_pos_current, 7, "initial joints")
        rotation = torch.as_tensor(table_rotation, dtype=torch.float32)
        if (
            rotation.shape != (3, 3)
            or not bool(torch.isfinite(rotation).all())
            or not torch.allclose(rotation.T @ rotation, torch.eye(3), atol=1e-5)
            or abs(float(torch.det(rotation)) - 1.0) > 1e-5
        ):
            raise ValueError("table_rotation must be a proper rotation into the robot base")
        low = vector(xy_min, 2, "xy_min")
        high = vector(xy_max, 2, "xy_max")
        kp = vector(stiffness, 6, "stiffness")
        kd = vector(damping, 6, "damping")
        limits = vector(wrench_limits, 6, "wrench_limits")
        damping_q = vector(joint_damping, 7, "joint_damping")
        if (
            not bool((low < high).all())
            or not bool((kp > 0).all())
            or not bool((kd > 0).all())
            or not bool((limits > 0).all())
            or not bool((damping_q >= 0).all())
        ):
            raise ValueError("invalid workspace, gains, or wrench limits")
        for name, value in (
            ("max_speed", max_speed),
            ("max_acceleration", max_acceleration),
            ("velocity_time_constant", velocity_time_constant),
            ("max_tracking_error", max_tracking_error),
            ("normal_travel", normal_travel),
            ("preload_ramp_s", preload_ramp_s),
            ("dt", dt),
            ("timeout", timeout),
        ):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(name + " must be finite and positive")
        if timeout <= dt or not math.isfinite(preload) or not 0 <= preload < float(limits[2]):
            raise ValueError("timeout or preload is outside its declared envelope")
        self.robot_model = robot_model
        self.invdyn = toco.modules.feedforward.InverseDynamics(robot_model, ignore_gravity=ignore_gravity)
        position, orientation = robot_model.forward_kinematics(q)
        initial_table = rotation.T @ position
        normal_limits = vector(
            normal_bounds if normal_bounds is not None else
            [float(initial_table[2]) - normal_travel, float(initial_table[2]) + 0.2],
            2, "normal_bounds",
        )
        if not float(normal_limits[0]) < float(initial_table[2]) < float(normal_limits[1]):
            raise ValueError("initial TCP must be inside normal_bounds")
        for value in (normal_speed, normal_acceleration, free_normal_stiffness,
                      free_normal_damping, free_normal_force_up):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("free translation parameters must be finite and positive")
        if not bool(((low <= initial_table[:2]) & (initial_table[:2] <= high)).all()):
            raise ValueError("initial measured TCP is outside the tangential workspace")
        self.register_buffer("table_rotation", rotation.clone())
        self.register_buffer("xy_min", low)
        self.register_buffer("xy_max", high)
        self.register_buffer("stiffness", kp)
        self.register_buffer("damping", kd)
        self.register_buffer("wrench_limits", limits)
        self.register_buffer("joint_damping", damping_q)
        self.register_buffer("normal_reference", initial_table[2].clone())
        self.register_buffer("orientation_reference", orientation.detach().clone())
        self.register_buffer("reference_xy", initial_table[:2].clone())
        self.register_buffer("velocity_xy", torch.zeros(2))
        self.register_buffer("normal_bounds", normal_limits)
        self.register_buffer("normal_velocity", torch.zeros(1))
        self.velocity_desired = torch.nn.Parameter(torch.zeros(2), requires_grad=False)
        self.normal_velocity_desired = torch.nn.Parameter(torch.zeros(1), requires_grad=False)
        self.command_contact = torch.nn.Parameter(torch.tensor([float(start_in_contact)]), requires_grad=False)
        self.command_sequence = torch.nn.Parameter(torch.zeros(1, dtype=torch.float64), requires_grad=False)
        self.command_paused = torch.nn.Parameter(torch.ones(1), requires_grad=False)
        self.max_speed = float(max_speed)
        self.max_acceleration = float(max_acceleration)
        self.alpha = 1.0 - math.exp(-dt / velocity_time_constant)
        self.max_tracking_error = float(max_tracking_error)
        self.normal_travel = float(normal_travel)
        self.preload = float(preload)
        self.preload_ramp_s = float(preload_ramp_s)
        self.dt = float(dt)
        self.timeout = float(timeout)
        self.last_sequence = 0.0
        self.age = float(timeout)
        self.active_time = 0.0
        self.was_active = False
        self.restart_required = False
        self.contact = bool(start_in_contact)
        self.normal_speed = float(normal_speed)
        self.normal_acceleration = float(normal_acceleration)
        self.free_normal_stiffness = float(free_normal_stiffness)
        self.free_normal_damping = float(free_normal_damping)
        self.free_normal_force_up = float(free_normal_force_up)

    def forward(self, state_dict: Dict[str, torch.Tensor]) -> Dict[str, torch.Tensor]:
        q = state_dict["joint_positions"]
        dq = state_dict["joint_velocities"]
        position, orientation = self.robot_model.forward_kinematics(q)
        jacobian = self.robot_model.compute_jacobian(q)
        twist = jacobian @ dq
        table_position = self.table_rotation.T @ position
        table_velocity = self.table_rotation.T @ twist[:3]
        contact = float(self.command_contact[0]) == 1.0
        mode_changed = contact != self.contact
        if mode_changed:
            # A mode switch starts here, never at a remembered surface or height.
            self.normal_reference.copy_(table_position[2])
            self.normal_velocity.zero_()
            self.was_active = False
            self.restart_required = False
            self.contact = contact
        sequence = float(self.command_sequence[0])
        if math.isfinite(sequence) and sequence > self.last_sequence:
            self.last_sequence = sequence
            self.age = 0.0
        else:
            self.age += self.dt
        valid = bool(torch.isfinite(self.velocity_desired).all() and torch.isfinite(self.normal_velocity_desired).all())
        near_plane = not contact or bool(torch.abs(table_position[2] - self.normal_reference) <= self.normal_travel)
        inside_workspace = bool(
            ((self.xy_min <= table_position[:2]) & (table_position[:2] <= self.xy_max)).all()
        ) and bool(self.normal_bounds[0] <= table_position[2] <= self.normal_bounds[1])
        paused = float(self.command_paused[0]) == 1.0
        if paused and near_plane and inside_workspace:
            self.restart_required = False
        elif not near_plane or not inside_workspace or (self.was_active and (self.age >= self.timeout or not valid)):
            self.restart_required = True
        active = (
            valid
            and self.age < self.timeout
            and float(self.command_paused[0]) == 0.0
            and near_plane
            and inside_workspace
            and not self.restart_required
        )
        if not active or not self.was_active:
            # No queued displacement survives a pause, timeout, or re-engagement.
            self.reference_xy.copy_(table_position[:2])
            self.velocity_xy.zero_()
            self.normal_velocity.zero_()
            self.active_time = 0.0
            if not contact and (active or self.was_active):
                self.normal_reference.copy_(table_position[2])
        if active:
            target_velocity = self.velocity_desired.clone()
            target_velocity *= min(1.0, self.max_speed / max(float(torch.norm(target_velocity)), 1e-8))
            delta = self.alpha * (target_velocity - self.velocity_xy)
            delta *= min(1.0, self.max_acceleration * self.dt / max(float(torch.norm(delta)), 1e-8))
            self.velocity_xy.add_(delta)
            candidate = self.reference_xy + self.dt * self.velocity_xy
            # Project outward commands; keep tangential motion along an edge.
            bounded = torch.maximum(self.xy_min, torch.minimum(self.xy_max, candidate))
            self.velocity_xy.copy_((bounded - self.reference_xy) / self.dt)
            error = bounded - table_position[:2]
            error *= min(1.0, self.max_tracking_error / max(float(torch.norm(error)), 1e-8))
            self.reference_xy.copy_(table_position[:2] + error)
            self.active_time += self.dt
            if not contact:
                desired_normal = torch.clamp(self.normal_velocity_desired, -self.normal_speed, self.normal_speed)
                change = torch.clamp(self.alpha * (desired_normal - self.normal_velocity),
                                     -self.normal_acceleration * self.dt, self.normal_acceleration * self.dt)
                self.normal_velocity.add_(change)
                candidate_z = torch.clamp(self.normal_reference + self.dt * self.normal_velocity[0],
                                          self.normal_bounds[0], self.normal_bounds[1])
                self.normal_velocity.copy_(((candidate_z - self.normal_reference) / self.dt).reshape(1))
                self.normal_reference.copy_(table_position[2] + torch.clamp(
                    candidate_z - table_position[2], -self.max_tracking_error, self.max_tracking_error))
        self.was_active = active

        position_error = torch.cat([
            self.reference_xy - table_position[:2],
            (self.normal_reference - table_position[2]).reshape(1),
        ])
        desired_velocity = torch.cat([self.velocity_xy, torch.zeros_like(table_position[2:3])])
        force_table = self.stiffness[:3] * position_error + self.damping[:3] * (desired_velocity - table_velocity)
        if not contact:
            force_table[2] = (self.free_normal_stiffness * position_error[2]
                              + self.free_normal_damping * (self.normal_velocity[0] - table_velocity[2]))
        elif not near_plane:
            # Do not seek the remembered surface after leaving its contact band.
            force_table[2] = -self.damping[2] * table_velocity[2]
        if active and contact:
            force_table[2] -= self.preload * min(1.0, self.active_time / self.preload_ramp_s)
        upper_force = self.wrench_limits[:3].clone()
        if not contact:
            upper_force[2] = self.free_normal_force_up
        force_table = torch.maximum(-self.wrench_limits[:3], torch.minimum(upper_force, force_table))

        quaternion_error = R.functional.quaternion_multiply(
            R.functional.invert_quaternion(orientation), self.orientation_reference
        )
        # Pick the shortest quaternion branch instead of unwinding near ±q.
        if float(quaternion_error[3]) < 0:
            quaternion_error = -quaternion_error
        orientation_error = R.functional.quat2matrix(orientation) @ quaternion_error[:3]
        moment = self.stiffness[3:] * orientation_error - self.damping[3:] * twist[3:]
        moment = torch.maximum(-self.wrench_limits[3:], torch.minimum(self.wrench_limits[3:], moment))
        wrench = torch.cat([self.table_rotation @ force_table, moment])
        torque = jacobian.T @ wrench - self.joint_damping * dq
        torque += self.invdyn(q, dq, torch.zeros_like(q))
        return {
            "joint_torques": torque,
            "planar_reference_xy": self.reference_xy.clone(),
            "planar_normal_wrench": force_table[2:3],
            "planar_reference_z": self.normal_reference.reshape(1).clone(),
        }
