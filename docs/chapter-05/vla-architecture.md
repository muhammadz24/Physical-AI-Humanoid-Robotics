---
sidebar_position: 2
title: VLA Architecture
---

# Vision-Language-Action Architecture

## Introduction

Vision-Language-Action (VLA) models represent a breakthrough in Physical AI, enabling robots to understand natural language commands and execute corresponding physical actions by grounding language in visual perception and motor control. Unlike traditional robots that require task-specific programming, VLA systems can perform novel tasks described in natural language by leveraging large-scale pre-trained models. This section explores how VLA models integrate computer vision, language understanding, and robotic control into unified architectures like RT-1 and RT-2.

## Conceptual Foundation

### The VLA Paradigm

Traditional robot programming: Engineer writes code for each specific task (pick red block, place in bin).

VLA approach: Robot understands "Put the apple in the bowl" and figures out how to do it—combining:
- **Vision**: Identify apple and bowl in camera image
- **Language**: Understand command semantics
- **Action**: Generate motor commands to grasp and place

### Components of VLA Models

**1. Vision Encoder**: Processes camera images to extract visual features
- CNN or Vision Transformer (ViT)
- Outputs: Object locations, scene understanding

**2. Language Encoder**: Processes natural language instructions
- BERT, T5, or GPT-style transformers
- Outputs: Semantic representation of task

**3. Action Decoder**: Generates robot control commands
- Outputs: Joint positions, gripper state, or end-effector poses
- Trained via imitation learning or reinforcement learning

**4. Fusion Module**: Combines vision and language representations
- Cross-attention mechanisms
- Grounds language tokens in visual features

### RT-1: Robotics Transformer 1

Google's RT-1 (2022) demonstrated VLA at scale:

**Architecture**:
```
Image (RGB) → Vision Encoder (EfficientNet) →
Language ("pick up cup") → Language Encoder (BERT) →
    → Fusion (Transformer) → Action Decoder → Robot Commands
```

**Training**: 130,000 demonstrations from 13 robots over 17 months

**Capabilities**:
- 97% success on trained tasks
- 76% success on novel objects
- Generalizes across different robots

### RT-2: Grounding Language Models

RT-2 (2023) goes further by using web-scale vision-language models (PaLI, PaLM-E):

**Key Insight**: Pre-training on internet images and text provides semantic knowledge that transfers to robotics.

**Example**:
- Saw "trash" in millions of web images
- Can identify trash in robot workspace
- Executes "throw this away" without specific trash-throwing training

## Technical Details

### Vision Encoding

```python
import torch
import torchvision.models as models
from torchvision import transforms

class VisionEncoder:
    """Extract visual features from robot camera."""

    def __init__(self):
        # Pre-trained vision model
        self.model = models.efficientnet_b0(pretrained=True)
        self.model.eval()

        # Remove classification head, keep features
        self.feature_extractor = torch.nn.Sequential(*list(self.model.children())[:-1])

        # Image preprocessing
        self.preprocess = transforms.Compose([
            transforms.Resize(256),
            transforms.CenterCrop(224),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

    def encode(self, image):
        """
        Args:
            image: PIL Image from robot camera
        Returns:
            Feature vector (1280-dim for EfficientNet-B0)
        """
        img_tensor = self.preprocess(image).unsqueeze(0)

        with torch.no_grad():
            features = self.feature_extractor(img_tensor)

        return features.squeeze()

# Example usage
# from PIL import Image
# encoder = VisionEncoder()
# image = Image.open("robot_view.jpg")
# visual_features = encoder.encode(image)
# print(f"Visual features shape: {visual_features.shape}")
```

### Language Encoding

```python
from transformers import BertTokenizer, BertModel

class LanguageEncoder:
    """Encode natural language commands."""

    def __init__(self):
        self.tokenizer = BertTokenizer.from_pretrained('bert-base-uncased')
        self.model = BertModel.from_pretrained('bert-base-uncased')
        self.model.eval()

    def encode(self, text_command):
        """
        Args:
            text_command: String like "pick up the red cup"
        Returns:
            Language embeddings (768-dim for BERT-base)
        """
        inputs = self.tokenizer(text_command, return_tensors='pt', padding=True, truncation=True)

        with torch.no_grad():
            outputs = self.model(**inputs)

        # Use [CLS] token embedding as sentence representation
        sentence_embedding = outputs.last_hidden_state[:, 0, :]

        return sentence_embedding.squeeze()

# Example
# lang_encoder = LanguageEncoder()
# command = "pick up the blue block"
# language_features = lang_encoder.encode(command)
# print(f"Language features shape: {language_features.shape}")
```

### Action Decoding

```python
class ActionDecoder(torch.nn.Module):
    """Map fused vision-language features to robot actions."""

    def __init__(self, input_dim=2048, num_joints=7):
        super().__init__()

        self.fc1 = torch.nn.Linear(input_dim, 512)
        self.fc2 = torch.nn.Linear(512, 256)
        self.fc3 = torch.nn.Linear(256, num_joints + 1)  # joints + gripper

        self.relu = torch.nn.ReLU()

    def forward(self, fused_features):
        """
        Args:
            fused_features: Combined vision-language representation
        Returns:
            Robot actions: [joint1, joint2, ..., joint7, gripper_open]
        """
        x = self.relu(self.fc1(fused_features))
        x = self.relu(self.fc2(x))
        actions = self.fc3(x)

        return actions

# Example
# decoder = ActionDecoder(input_dim=1280+768, num_joints=7)
# fused = torch.cat([visual_features, language_features])
# actions = decoder(fused)
# print(f"Predicted actions: {actions}")
```

### Complete VLA Pipeline

```python
class SimpleVLAModel:
    """Minimal VLA model combining vision, language, action."""

    def __init__(self):
        self.vision_encoder = VisionEncoder()
        self.language_encoder = LanguageEncoder()
        self.action_decoder = ActionDecoder(input_dim=1280+768)

    def predict_action(self, image, text_command):
        """
        Main inference function.

        Args:
            image: PIL Image from robot camera
            text_command: Natural language instruction
        Returns:
            Robot action commands
        """
        # Encode inputs
        visual_features = self.vision_encoder.encode(image)
        language_features = self.language_encoder.encode(text_command)

        # Fuse (simple concatenation; RT-1/RT-2 use transformer attention)
        fused = torch.cat([visual_features, language_features])

        # Decode to actions
        actions = self.action_decoder(fused)

        return actions.detach().numpy()

# Complete pipeline example
# vla = SimpleVLAModel()
# image = Image.open("workspace.jpg")
# actions = vla.predict_action(image, "pick up the red cup")
# print(f"Actions: {actions}")
```

## Hands-On Examples

### Simulation: Object Manipulation from Language

```python
import numpy as np

class SimulatedVLARobot:
    """Simulated robot executing VLA commands."""

    def __init__(self):
        self.objects = {
            'red_cup': {'position': np.array([0.3, 0.2, 0.0]), 'grasped': False},
            'blue_block': {'position': np.array([0.4, -0.1, 0.0]), 'grasped': False}
        }
        self.gripper_position = np.array([0.0, 0.0, 0.3])
        self.gripper_open = True

    def parse_command(self, command):
        """Simple command parsing (real VLA uses language model)."""
        if "pick up" in command or "pick" in command:
            action = "pick"
            if "red" in command:
                target = "red_cup"
            elif "blue" in command:
                target = "blue_block"
            else:
                target = None
            return action, target
        return None, None

    def execute(self, command):
        """Execute natural language command."""
        action, target = self.parse_command(command)

        if action == "pick" and target:
            obj = self.objects[target]

            # Move gripper to object
            print(f"Moving to {target} at {obj['position']}")
            self.gripper_position = obj['position'].copy()
            self.gripper_position[2] = 0.1  # Above object

            # Descend
            print("Descending...")
            self.gripper_position[2] = obj['position'][2]

            # Close gripper
            print("Closing gripper")
            self.gripper_open = False
            obj['grasped'] = True

            # Lift
            print("Lifting")
            self.gripper_position[2] = 0.2

            print(f"✓ Successfully picked up {target}")
            return True

        return False

# Demo
robot = SimulatedVLARobot()
robot.execute("pick up the red cup")
robot.execute("pick up the blue block")
```

## Further Resources

### Research Papers
- RT-1: Brohan et al. (2022). "RT-1: Robotics Transformer for Real-World Control at Scale"
- RT-2: Brohan et al. (2023). "RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control"
- PaLM-E: Driess et al. (2023). "PaLM-E: An Embodied Multimodal Language Model"

### Documentation
- Hugging Face Transformers: https://huggingface.co/docs/transformers
- PyTorch Vision Models: https://pytorch.org/vision/stable/models.html

### Code Repositories
- Google Research RT-1: https://github.com/google-research/robotics_transformer
- Open-VLA: Open-source VLA implementations

---

**Next**: [Integration with ROS 2](./integration) | [Self-Assessment](./self-assessment)
