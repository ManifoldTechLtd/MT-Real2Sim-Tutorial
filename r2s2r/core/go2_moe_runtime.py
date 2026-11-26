"""Simulator-independent contract for the bundled Go2 MoE-CTS and velocity PI."""
from pathlib import Path
import numpy as np
import torch
import yaml
from r2s2r.core.velocity_pi import PISettings, VelocityPI

ASSETS = Path(__file__).parent / "assets" / "go2" / "moe_cts"
JOINT_ORDER = tuple(f"{leg}_{joint}_joint" for leg in ("FL", "FR", "RL", "RR") for joint in ("hip", "thigh", "calf"))
DEFAULT_POS = np.array([.1,.8,-1.5,-.1,.8,-1.5,.1,1.,-1.5,-.1,1.,-1.5], np.float32)
POLICY_DT = .02
PHYSICS_DT = .005  # Isaac Lab training dt; policy decimation=4
KP, KD, ACTION_SCALE = 20., .5, .25
TORQUE_LIMIT = np.tile(np.array([23.7,23.7,35.55], np.float32), 4)

def rotation_matrix(quaternion):
    q = np.asarray(quaternion, dtype=np.float64)
    norm = np.linalg.norm(q)
    if q.shape != (4,) or not np.isfinite(q).all() or norm < 1e-9:
        raise ValueError("Expected a finite wxyz orientation quaternion")
    w,x,y,z = q / norm
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

class Go2MoeRuntime:
    def __init__(self, pi_enabled=True, policy_path=None):
        torch.set_num_threads(1)
        self.policy = torch.jit.load(str(policy_path or ASSETS/"policy.pt"), map_location="cpu").eval()
        if self.policy.num_single_obs != 45 or self.policy.num_actions != 12 or self.policy.history_len != 10:
            raise ValueError("Go2 policy contract must be 45 current observations, 12 actions, internal 10-frame history")
        self.pi_enabled = pi_enabled
        self.settings = PISettings.from_dict(yaml.safe_load((ASSETS/"pi_outer.yaml").read_text()))
        self.pi = VelocityPI(self.settings, np.ones(3))
        self.reset()

    def reset(self):
        self.policy.reset()
        self.pi.reset()
        self.action = np.zeros(12, np.float32)
        self.target = DEFAULT_POS.copy()
        self.last_observation = np.zeros(45, np.float32)
        self.last_reference = np.zeros(3)
        self.last_command = np.zeros(3)
        self.last_feedback = np.zeros(3)

    def infer(self, q, dq, quaternion, linear_world, angular_world, command, elapsed):
        rotation = rotation_matrix(quaternion)
        velocity = rotation.T @ np.asarray(linear_world)
        angular = rotation.T @ np.asarray(angular_world)
        reference = np.clip(np.asarray(command, dtype=np.float64), -1., 1.)
        feedback = np.array([velocity[0],velocity[1],angular[2]])
        corrected = reference if not self.pi_enabled else self.pi.update(
            reference, feedback, POLICY_DT,
            enabled=elapsed >= self.settings.warmup_seconds and rotation[2,2] >= .5)
        obs = np.concatenate((angular*.25, -rotation[2], corrected,
                              np.asarray(q)-DEFAULT_POS, np.asarray(dq)*.05, self.action)).astype(np.float32)
        if obs.shape != (45,) or not np.isfinite(obs).all():
            raise ValueError("Invalid Go2 observation")
        with torch.inference_mode():
            action = self.policy(torch.from_numpy(obs).unsqueeze(0)).numpy().reshape(-1).copy()
        if action.shape != (12,) or not np.isfinite(action).all():
            raise ValueError("Invalid Go2 policy action")
        self.last_observation = obs
        self.last_reference, self.last_command, self.last_feedback = reference.copy(), corrected.copy(), feedback.copy()
        self.action = action
        self.target = DEFAULT_POS + ACTION_SCALE*action
        return self.target

    def effort(self, q, dq):
        return np.clip(KP*(self.target-q)-KD*dq, -TORQUE_LIMIT, TORQUE_LIMIT)
