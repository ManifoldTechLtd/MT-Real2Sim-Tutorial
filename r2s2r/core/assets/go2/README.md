# Go2 MoE-CTS locomotion

Go2 uses Go2MoeCTSPolicy by default, with a body-frame velocity PI outer loop.
Start with ./start.sh --robot go2; there is no separate --pi flag.
The default robot remains kinematic_car.

Policy: wertyuilife2/go2_rl_robotlab,
commit c85e51c70df0f25e9f865654be978a2c2d53cccd,
deploy/pre_train/go2/go2_moe_cts_176k_0.6984.pt.
Source: https://github.com/wertyuilife2/go2_rl_robotlab
Bundled policy SHA256:
c602e749ac292921e3d6f5b2ab1749c4e4eaa6dbe51f751cce5b669b102d10a2.
The source Apache-2.0 license is in moe_cts/LICENSE.
moe_cts/provenance.json records the deployment contract.
NVIDIA USD resources retain their upstream layout; their original URLs and
checksums are in moe_cts/usd/manifest.json.

| Contract | Value |
|---|---|
| Observation | 45 current-frame values; 10-frame history maintained inside TorchScript |
| Order | body angular velocity ×0.25, projected gravity, physical velocity command, q-default, dq×0.05, previous raw action |
| Model joint order | FL, FR, RL, RR; hip/thigh/calf per leg, mapped by actual DOF names |
| Physics / policy | 5 ms / 20 ms, matching the source Isaac training step |
| PD | Explicit 20/0.5 every physics step; implicit USD drives disabled |
| Action | default position + 0.25 × action |
| Torque limits | Hip/thigh ±23.7, calf ±35.55 N·m |
| PI | Body vx/vy/wz; parameters in moe_cts/pi_outer.yaml |
| Reset | Clears policy history, previous action, PI/filter state and step count |

Commands are clipped to ±1 per axis. Begin with 0.4 m/s and 0.4 rad/s.
The PI corrects velocity error; it does not hold world heading or position.
ROS cmd_vel and existing keyboard/reset controls remain available.
The original experimental go2_policy.py remains in the tree.

Previous Isaac Sim 5.1 verification (2026-11-09): 20-second flat tests at
vx=0.4, wz=+0.4 and wz=-0.4 averaged 0.3978 m/s, +0.3983 rad/s and
-0.3984 rad/s, with no falls. Main-program TCP stop/reset/resume passed.
The 10×19 cm staircase did not pass: the policy stalled partway up with PI
and earlier without PI. MuJoCo staircase results do not establish Isaac success.
