"""Explicit joint-space servo with a native jerk-limited reference and watchdog."""

from typing import Dict

import torch
import torchcontrol as toco


class JointServoControl(toco.PolicyModule):
    """Requires libpolymetis_joint_servo in BOTH uploader and native server.

    Gains are joint-space gains; no implicit J.T Kx J term is added. Acceleration
    feedforward is deliberately absent until the payload/model is validated.
    The reference watchdog latches and requires a new session after a fault.
    """

    def __init__(self, q, dq, Kp, Kd, lower, upper, max_velocity,
                 max_acceleration, max_jerk, robot_model, dt=0.001, timeout=0.15):
        super().__init__()
        arrays = [torch.as_tensor(v, dtype=torch.float32).reshape(7)
                  for v in (q, dq, Kp, Kd, lower, upper, max_velocity, max_acceleration, max_jerk)]
        if not all(torch.isfinite(v).all() for v in arrays):
            raise ValueError("servo configuration must be finite")
        q, dq, kp, kd, low, high, vmax, amax, jmax = arrays
        if torch.any(kp < 0) or torch.any(kd <= 0):
            raise ValueError("servo gains require Kp >= 0 and Kd > 0")
        config = torch.cat([q.double(), dq.double(), low.double(), high.double(),
                            vmax.double(), amax.double(), jmax.double(),
                            torch.tensor([dt, timeout], dtype=torch.float64)])
        self.reference = torch.classes.polymetis_servo.JointReference(config)
        self.joint_pos_desired = torch.nn.Parameter(q.clone())
        self.command_paused = torch.nn.Parameter(torch.ones(1))
        self.command_sequence = torch.nn.Parameter(torch.zeros(1, dtype=torch.float64))
        self.joint_pd = toco.modules.feedback.JointSpacePD(kp, kd)
        self.invdyn = toco.modules.feedforward.InverseDynamics(robot_model, ignore_gravity=True)
        self.zero_acceleration = torch.zeros(7)

    def forward(self, state_dict: Dict[str, torch.Tensor]) -> Dict[str, torch.Tensor]:
        reference = self.reference.step(self.joint_pos_desired, int(self.command_sequence.item()),
                                        bool(self.command_paused.item() > 0.5))
        if bool(reference[3][0] > 0) and bool(torch.max(torch.abs(reference[1])) < 1e-6):
            self._terminated = True
        q = state_dict["joint_positions"]
        dq = state_dict["joint_velocities"]
        feedback = self.joint_pd(q, dq, reference[0], reference[1])
        coriolis = self.invdyn(q, dq, self.zero_acceleration)
        return {"joint_torques": feedback + coriolis,
                "joint_reference": reference[0], "joint_velocity_reference": reference[1],
                "joint_acceleration_reference": reference[2], "servo_status": reference[3]}
