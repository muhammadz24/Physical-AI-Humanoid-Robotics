---
sidebar_position: 3
title: Training Environment
---

# Training in Isaac Gym

## Parallel Environment Setup (256 Unitree Go2)

```python
from isaacgym import gymapi, gymtorch
import torch

# Create gym instance
gym = gymapi.acquire_gym()

# Simulation parameters
sim_params = gymapi.SimParams()
sim_params.dt = 0.005  # 200 Hz
sim_params.use_gpu_pipeline = True
sim_params.physx.use_gpu = True
sim_params.physx.num_position_iterations = 4
sim_params.physx.num_velocity_iterations = 1

sim = gym.create_sim(0, 0, gymapi.SIM_PHYSX, sim_params)

# Load Unitree Go2 asset
asset_root = "assets/"
asset_file = "go2/urdf/go2.urdf"
asset = gym.load_asset(sim, asset_root, asset_file)

# Create 256 environments (16x16 grid)
num_envs = 256
spacing = 2.0

for i in range(num_envs):
    env = gym.create_env(sim, gymapi.Vec3(-spacing, -spacing, 0), gymapi.Vec3(spacing, spacing, spacing), int(np.sqrt(num_envs)))

    # Spawn Go2
    pose = gymapi.Transform()
    pose.p = gymapi.Vec3(0, 0, 0.4)
    actor = gym.create_actor(env, asset, pose, f"go2_{i}", i, 1)

    # Enable GPU tensors
    gym.prepare_sim(sim)
```

## PPO Training Loop

```python
import torch
import torch.nn as nn

class PolicyNetwork(nn.Module):
    def __init__(self, num_obs, num_actions):
        super().__init__()
        self.fc1 = nn.Linear(num_obs, 256)
        self.fc2 = nn.Linear(256, 128)
        self.fc3 = nn.Linear(128, num_actions)

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        return torch.tanh(self.fc3(x))  # Actions in [-1, 1]

policy = PolicyNetwork(num_obs=48, num_actions=12).cuda()  # 12 joints
optimizer = torch.optim.Adam(policy.parameters(), lr=3e-4)

# Training loop
for iteration in range(10000):
    # Step simulation
    obs = get_observations()  # [256, 48] tensor
    actions = policy(obs)
    gym.set_dof_position_target_tensor(sim, actions)
    gym.simulate(sim)
    gym.fetch_results(sim, True)

    # Compute rewards
    rewards = compute_rewards(obs)

    # Update policy every 16 steps
    if iteration % 16 == 0:
        loss = ppo_update(policy, obs_buffer, actions_buffer, rewards_buffer)
        print(f"Iteration {iteration}, Loss: {loss:.4f}, Mean Reward: {rewards.mean():.2f}")

    # Save checkpoint every 1000 iterations
    if iteration % 1000 == 0:
        torch.save(policy.state_dict(), f"policy_iter_{iteration}.pth")
```

**Training Time**: ~2 hours on RTX 4070 Ti to learn stable walking.

---

**Next**: [Sim-to-Real Transfer](./sim-to-real)
