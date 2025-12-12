---
sidebar_position: 2
title: RL Basics
---

# Reinforcement Learning Fundamentals

## Markov Decision Process (MDP)

RL formulates robot control as an MDP:
- **State** (s): Robot joint positions, velocities, orientation, contact forces
- **Action** (a): Joint torques or position commands
- **Reward** (r): +1 for forward progress, -0.1 for energy, -10 for falling
- **Policy** (π): Neural network mapping states to actions

**Goal**: Learn policy π* that maximizes cumulative reward: Σ γ^t * r_t

## Proximal Policy Optimization (PPO)

PPO is the gold standard for robotics RL:

**Advantages**:
- Stable (constrains policy updates)
- Sample-efficient (reuses data via importance sampling)
- Parallelizable (scales to 1000+ environments)

**Algorithm** (simplified):

```python
for iteration in range(num_iterations):
    # Collect rollouts from current policy
    states, actions, rewards = collect_rollouts(policy, num_envs=256)

    # Compute advantages
    advantages = compute_gae(rewards)

    # Update policy (multiple epochs)
    for epoch in range(10):
        policy_loss = ppo_loss(states, actions, advantages)
        optimizer.step(policy_loss)
```

**Hyperparameters** (humanoid walking):
- Learning rate: 3e-4
- Clip epsilon: 0.2
- Discount (γ): 0.99
- GAE lambda: 0.95
- Batch size: 4096 (256 envs × 16 steps)

## Reward Shaping for Locomotion

**Walking Reward Function**:

```python
def compute_reward(state):
    # Forward velocity (primary objective)
    reward = state.base_lin_vel[0] * 1.0

    # Penalties
    reward -= torch.sum(torch.square(state.joint_torques)) * 0.0001  # Energy
    reward -= torch.sum(torch.square(state.joint_acc)) * 0.001  # Smoothness
    reward -= torch.abs(state.base_ang_vel[2]) * 0.05  # Yaw stability

    # Terminal penalty
    if state.base_height < 0.3:
        reward -= 10.0  # Fallen

    return reward
```

---

**Next**: [Training Environment](./training-environment) in Isaac Gym
