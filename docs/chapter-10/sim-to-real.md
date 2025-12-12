---
sidebar_position: 4
title: Sim-to-Real Transfer
---

# Sim-to-Real Transfer

## Domain Randomization

Randomize simulation parameters to make policy robust to real-world variations:

```python
# Randomize physics parameters
mass_scale = torch.rand(num_envs) * 0.4 + 0.8  # ±20% mass
friction = torch.rand(num_envs) * 0.5 + 0.5    # 0.5-1.0 friction
damping = torch.rand(num_envs) * 2.0 + 1.0     # 1.0-3.0 damping

# Randomize external forces (wind, pushes)
external_force = torch.randn(num_envs, 3) * 10.0  # Random 10N force

# Randomize observations (sensor noise)
obs_noise = torch.randn_like(obs) * 0.05

# Apply during training
obs_noisy = obs + obs_noise
```

**Result**: Policy learns to handle uncertainty → better real-world performance.

## Deploying to Jetson Orin Nano

### Step 1: Export to ONNX

```python
# Export trained policy
dummy_input = torch.randn(1, 48).cuda()
torch.onnx.export(
    policy,
    dummy_input,
    "go2_policy.onnx",
    input_names=["observations"],
    output_names=["actions"],
    dynamic_axes={'observations': {0: 'batch'}}
)
```

### Step 2: Optimize with TensorRT (Jetson)

```bash
# On Jetson Orin Nano
/usr/src/tensorrt/bin/trtexec \
  --onnx=go2_policy.onnx \
  --saveEngine=go2_policy_fp16.trt \
  --fp16  # Use Tensor Cores for speed
```

**Performance**: 5ms inference (200 Hz control loop).

### Step 3: ROS 2 Inference Node

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import tensorrt as trt
import numpy as np

class PolicyInferenceNode(Node):
    def __init__(self):
        super().__init__('policy_inference')

        # Load TensorRT engine
        self.engine = self.load_engine("go2_policy_fp16.trt")
        self.context = self.engine.create_execution_context()

        # Subscribe to joint states
        self.create_subscription(JointState, '/joint_states', self.state_callback, 10)

        # Publish actions
        self.action_pub = self.create_publisher(Float32MultiArray, '/joint_commands', 10)

    def load_engine(self, engine_path):
        with open(engine_path, 'rb') as f, trt.Runtime(trt.Logger()) as runtime:
            return runtime.deserialize_cuda_engine(f.read())

    def state_callback(self, msg):
        # Convert joint states to observation
        obs = np.array(msg.position + msg.velocity, dtype=np.float32)

        # Run inference
        actions = self.infer(obs)

        # Publish
        action_msg = Float32MultiArray()
        action_msg.data = actions.tolist()
        self.action_pub.publish(action_msg)

    def infer(self, obs):
        # TensorRT inference (GPU)
        # ... (omitted for brevity, see TensorRT Python API docs)
        return actions

def main():
    rclpy.init()
    node = PolicyInferenceNode()
    rclpy.spin(node)
```

**Deployment**: Upload to Jetson → Run ROS 2 node → Robot executes learned policy!

## Fine-Tuning on Hardware

**Online Adaptation** (optional):

```python
# Collect real-world data
real_obs, real_rewards = collect_hardware_rollouts(num_steps=1000)

# Fine-tune policy (small learning rate)
optimizer = torch.optim.Adam(policy.parameters(), lr=1e-5)

for epoch in range(10):
    loss = ppo_loss(real_obs, policy(real_obs), real_rewards)
    optimizer.step(loss)
```

**Result**: Policy adapts to hardware-specific dynamics (motor delays, sensor noise).

---

✅ **CHAPTER 10 COMPLETE**: RL training, sim-to-real transfer, Jetson deployment!

🎉 **TEXTBOOK CHAPTERS 7-10 COMPLETE**!
