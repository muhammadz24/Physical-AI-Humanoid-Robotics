---
sidebar_position: 3
title: Unity Integration with ROS 2
---

# Unity Integration with ROS 2

## Introduction

Unity, the world's leading real-time 3D development platform, is transforming robotics workflows beyond gaming and VR. When integrated with ROS 2, Unity becomes a powerful tool for photorealistic visualization, synthetic data generation, VR teleoperation, and AR-enhanced debugging. While Gazebo excels at physics simulation, Unity offers superior graphics, intuitive UI development, and cross-platform deployment (PC, mobile, HoloLens, VR headsets). This section teaches you to bridge ROS 2 and Unity using ROS#, enabling hybrid pipelines where Gazebo handles dynamics and Unity handles visualization and human interaction.

Modern robotics companies like Nvidia (Isaac Sim), Boston Dynamics, and autonomous vehicle teams use game engines for simulation. Unity's Universal Render Pipeline (URP) generates photorealistic sensor data for training perception models, while its XR Toolkit enables immersive teleoperation interfaces. By mastering Unity-ROS integration, you unlock capabilities impossible in traditional robotics tools: procedural scene generation for training data, mixed-reality debugging overlays, and publish-ready demonstration videos—all while keeping ROS 2 as your control backbone.

## Conceptual Foundation

### Why Unity for Robotics?

**Visualization Superiority**:
- Photorealistic rendering (ray tracing, global illumination)
- Post-processing effects (bloom, ambient occlusion, motion blur)
- Real-time shadows and reflections
- 60 FPS+ even with complex scenes

**Synthetic Data Generation**:
- Domain randomization (lighting, textures, object poses)
- Semantic segmentation labels (pixel-perfect, no manual annotation)
- Bounding box automation
- Export in COCO, YOLO, or custom formats

**Cross-Platform Deployment**:
- Build for Windows, Linux, macOS, Android, iOS
- VR headsets (Quest, Vive, PSVR)
- AR devices (HoloLens, Magic Leap, ARKit/ARCore)

**UI Development**:
- Intuitive robot control dashboards
- Real-time telemetry visualization (graphs, 3D overlays)
- Touch/gesture interfaces for tablets

### ROS# (ROS-Sharp) Architecture

**ROS#** is the bridge between Unity (C#) and ROS 2 (C++/Python):

```
ROS 2 Nodes (C++/Python) → ROS 2 Topics/Services/Actions
         ↕ (TCP/WebSockets)
ROS# Bridge (rosbridge_server)
         ↕ (JSON messages)
Unity (C#) Scripts → GameObjects/Renderers
```

**Components**:

1. **rosbridge_suite**: ROS 2 package providing WebSocket server
   - Converts ROS messages to JSON
   - Handles topic pub/sub, service calls, action feedback

2. **ROS# Library**: Unity C# scripts for ROS communication
   - RosConnector: Manages WebSocket connection
   - Subscribers/Publishers: C# wrappers for ROS topics
   - Message types: C# equivalents of ROS msgs

3. **Unity Robotics Hub**: Unity packages for URDF import, TF visualization
   - URDF Importer: Converts URDF to Unity prefabs
   - TF Visualizer: Displays coordinate frames
   - Controller scripts: Joint control from Unity UI

### Data Flow Example: Visualizing Robot in Unity

```
Gazebo (Physics Simulation)
  ↓ /joint_states (sensor_msgs/JointState)
  ↓ /camera/image_raw (sensor_msgs/Image)
ROS 2 Topics
  ↓
rosbridge_server (JSON conversion)
  ↓ WebSocket (ws://localhost:9090)
Unity ROS# Scripts
  → JointStateSubscriber.cs updates robot joints
  → ImageSubscriber.cs renders camera feed on Texture2D
Unity Scene
  → 3D robot model moves in real-time
  → Camera image displayed on UI canvas
```

## Technical Details

### Installing Unity and ROS#

**Step 1: Install Unity**

```bash
# Download Unity Hub
wget -O UnityHub.AppImage https://public-cdn.cloud.unity3d.com/hub/prod/UnityHub.AppImage
chmod +x UnityHub.AppImage
./UnityHub.AppImage

# Install Unity 2022.3 LTS (via Unity Hub GUI)
# Choose modules: Linux Build Support, Android Build Support (optional)
```

**Step 2: Install ROS# Bridge**

```bash
# Install rosbridge_suite
sudo apt install ros-humble-rosbridge-suite

# Launch rosbridge server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

**Expected Output**:
```
[INFO] [rosbridge_websocket]: Rosbridge WebSocket server started on port 9090
```

**Step 3: Unity Robotics Hub Setup**

1. Open Unity Hub → Create New Project (3D URP template)
2. Window → Package Manager → Add package from git URL:
   ```
   https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
   ```
3. Install "URDF Importer":
   ```
   https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer
   ```

**Step 4: Configure ROS Connection**

Unity Menu: Robotics → ROS Settings
- ROS IP Address: `127.0.0.1` (localhost, or remote IP)
- ROS Port: `9090` (default rosbridge port)
- Protocol: `ROS 2`

Click "Connect" → Should show "Connected to ROS"

### Importing URDF into Unity

**Convert URDF to Unity Prefab**:

1. Copy robot URDF and meshes to Unity project:
   ```
   UnityProject/Assets/URDF/
     ├── my_robot.urdf
     └── meshes/
         ├── base_link.dae
         ├── arm_link.dae
         └── ...
   ```

2. Unity Menu: **Assets → Import Robot from URDF**
   - Select `my_robot.urdf`
   - Choose import settings:
     - **Axis Convention**: Y-up (Unity) or Z-up (ROS)
     - **Mesh Decomposer**: Use Colliders (for physics)
     - **Scale Factor**: 1.0 (if URDF in meters)
   - Click Import

3. Unity generates:
   - GameObject hierarchy matching URDF links/joints
   - MeshRenderers for visuals
   - ArticulationBody components for joints (Unity physics)

**Result**: Drag the generated prefab into scene → Robot appears in 3D view

### Subscribing to ROS 2 Topics in Unity

**Example: Visualize Joint States**

**C# Script** (`JointStateSubscriber.cs`):

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class JointStateSubscriber : MonoBehaviour
{
    public string topicName = "/joint_states";
    public ArticulationBody[] articulationBodies;  // Assign robot joints in Inspector

    private ROSConnection ros;

    void Start()
    {
        // Connect to ROS
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<JointStateMsg>(topicName, UpdateJointStates);

        Debug.Log($"Subscribed to {topicName}");
    }

    void UpdateJointStates(JointStateMsg jointStateMsg)
    {
        // Update Unity robot joints from ROS message
        for (int i = 0; i < jointStateMsg.position.Length && i < articulationBodies.Length; i++)
        {
            ArticulationBody joint = articulationBodies[i];

            if (joint != null && joint.jointType == ArticulationJointType.RevoluteJoint)
            {
                // Convert radians to degrees
                float targetAngle = (float)jointStateMsg.position[i] * Mathf.Rad2Deg;

                // Set joint target (Unity drives to this position)
                var drive = joint.xDrive;
                drive.target = targetAngle;
                joint.xDrive = drive;
            }
        }
    }
}
```

**Attach Script to Robot**:
1. Select robot GameObject in Hierarchy
2. Add Component → JointStateSubscriber
3. In Inspector, expand "Articulation Bodies" array
4. Drag each joint (ArticulationBody component) into array slots

**Launch Gazebo with Robot**:
```bash
ros2 launch my_robot gazebo_robot.launch.py
```

**Play in Unity** → Robot in Unity mirrors Gazebo robot movements in real-time!

### Displaying Camera Feed in Unity

**C# Script** (`CameraImageSubscriber.cs`):

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using UnityEngine.UI;

public class CameraImageSubscriber : MonoBehaviour
{
    public string topicName = "/camera/image_raw";
    public RawImage uiImage;  // Assign UI RawImage in Inspector

    private ROSConnection ros;
    private Texture2D texture;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<ImageMsg>(topicName, UpdateCameraImage);

        Debug.Log($"Subscribed to {topicName}");
    }

    void UpdateCameraImage(ImageMsg imageMsg)
    {
        int width = (int)imageMsg.width;
        int height = (int)imageMsg.height;

        // Create texture if needed
        if (texture == null || texture.width != width || texture.height != height)
        {
            texture = new Texture2D(width, height, TextureFormat.RGB24, false);
            uiImage.texture = texture;
        }

        // Decode image data (assuming RGB8 encoding)
        if (imageMsg.encoding == "rgb8")
        {
            texture.LoadRawTextureData(imageMsg.data);
            texture.Apply();
        }
        else if (imageMsg.encoding == "bgr8")
        {
            // Convert BGR to RGB
            byte[] rgbData = new byte[imageMsg.data.Length];
            for (int i = 0; i < imageMsg.data.Length; i += 3)
            {
                rgbData[i] = imageMsg.data[i + 2];     // R
                rgbData[i + 1] = imageMsg.data[i + 1]; // G
                rgbData[i + 2] = imageMsg.data[i];     // B
            }
            texture.LoadRawTextureData(rgbData);
            texture.Apply();
        }
    }
}
```

**Unity UI Setup**:
1. Create UI → Canvas → RawImage
2. Resize RawImage to desired viewport size
3. Attach `CameraImageSubscriber` script to any GameObject
4. Assign RawImage to script's "Ui Image" field
5. Set topic name: `/camera/image_raw`

**Result**: Camera feed from Gazebo/RealSense displays in Unity UI panel.

## Hands-On Examples

### Example 1: Synthetic Data Generation for YOLO Training

**Scenario**: Generate 10,000 labeled images of objects in randomized scenes for training object detection.

**Unity Scene Setup**:

```csharp
// ObjectRandomizer.cs - Spawns random objects with varied poses and lighting

using UnityEngine;
using System.IO;

public class ObjectRandomizer : MonoBehaviour
{
    public GameObject[] objectPrefabs;  // Assign cube, cylinder, sphere prefabs
    public Material[] materials;        // Different colors/textures
    public Light mainLight;
    public Camera captureCamera;

    public int imagesToGenerate = 10000;
    public string outputPath = "Assets/SyntheticData/";

    private int currentImage = 0;

    void Start()
    {
        Directory.CreateDirectory(outputPath + "images/");
        Directory.CreateDirectory(outputPath + "labels/");

        InvokeRepeating("GenerateScene", 0f, 0.1f);  // 10 scenes/second
    }

    void GenerateScene()
    {
        if (currentImage >= imagesToGenerate)
        {
            Debug.Log("Dataset generation complete!");
            return;
        }

        // Clear previous objects
        foreach (Transform child in transform)
        {
            Destroy(child.gameObject);
        }

        // Randomize lighting
        mainLight.intensity = Random.Range(0.5f, 2.0f);
        mainLight.color = new Color(
            Random.Range(0.8f, 1.0f),
            Random.Range(0.8f, 1.0f),
            Random.Range(0.8f, 1.0f)
        );

        // Spawn random objects
        int numObjects = Random.Range(3, 8);
        for (int i = 0; i < numObjects; i++)
        {
            GameObject prefab = objectPrefabs[Random.Range(0, objectPrefabs.Length)];
            GameObject obj = Instantiate(prefab, transform);

            // Random position
            obj.transform.position = new Vector3(
                Random.Range(-2f, 2f),
                Random.Range(0f, 1f),
                Random.Range(2f, 5f)  // In front of camera
            );

            // Random rotation
            obj.transform.rotation = Random.rotation;

            // Random material
            obj.GetComponent<Renderer>().material = materials[Random.Range(0, materials.Length)];
        }

        // Capture image and labels
        StartCoroutine(CaptureAndSave());

        currentImage++;
    }

    System.Collections.IEnumerator CaptureAndSave()
    {
        yield return new WaitForEndOfFrame();

        // Capture image
        RenderTexture renderTexture = captureCamera.targetTexture;
        Texture2D image = new Texture2D(renderTexture.width, renderTexture.height, TextureFormat.RGB24, false);
        RenderTexture.active = renderTexture;
        image.ReadPixels(new Rect(0, 0, renderTexture.width, renderTexture.height), 0, 0);
        image.Apply();

        // Save image
        string imagePath = $"{outputPath}images/img_{currentImage:D6}.jpg";
        File.WriteAllBytes(imagePath, image.EncodeToJPG());

        // Generate YOLO labels (bounding boxes)
        string labelPath = $"{outputPath}labels/img_{currentImage:D6}.txt";
        using (StreamWriter writer = new StreamWriter(labelPath))
        {
            foreach (Transform child in transform)
            {
                // Convert 3D position to 2D screen space
                Vector3 screenPos = captureCamera.WorldToScreenPoint(child.position);

                if (screenPos.z > 0)  // Object in front of camera
                {
                    // Normalize coordinates (YOLO format: class x_center y_center width height)
                    float x_norm = screenPos.x / captureCamera.pixelWidth;
                    float y_norm = 1.0f - (screenPos.y / captureCamera.pixelHeight);  // Unity Y is bottom-up

                    // Estimate bounding box size (simplified)
                    float width_norm = 0.1f;
                    float height_norm = 0.1f;

                    int classId = GetClassId(child.name);

                    writer.WriteLine($"{classId} {x_norm} {y_norm} {width_norm} {height_norm}");
                }
            }
        }

        Destroy(image);
    }

    int GetClassId(string objectName)
    {
        if (objectName.Contains("Cube")) return 0;
        if (objectName.Contains("Cylinder")) return 1;
        if (objectName.Contains("Sphere")) return 2;
        return 3;  // Unknown
    }
}
```

**Usage**:
1. Create Empty GameObject → Attach `ObjectRandomizer` script
2. Assign prefabs (Cube, Cylinder, Sphere) and materials
3. Play → Generates 10,000 images in `Assets/SyntheticData/`

**Train YOLO**:
```bash
# Install Ultralytics YOLO
pip install ultralytics

# Create dataset.yaml
cat > dataset.yaml <<EOF
train: /path/to/UnityProject/Assets/SyntheticData/images/
val: /path/to/UnityProject/Assets/SyntheticData/images/

nc: 4  # Number of classes
names: ['cube', 'cylinder', 'sphere', 'unknown']
EOF

# Train YOLOv8
yolo train data=dataset.yaml model=yolov8n.pt epochs=50 imgsz=640
```

**Result**: Trained object detector ready for real robot deployment!

### Example 2: VR Teleoperation Interface

**Scenario**: Control a simulated robot arm using Quest 2 VR controllers.

**Unity XR Setup**:

1. Window → Package Manager → Install "XR Plugin Management"
2. Edit → Project Settings → XR Plugin Management → Enable "Oculus"
3. Install "XR Interaction Toolkit" from Package Manager

**C# Script** (`VRRobotController.cs`):

```csharp
using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Trajectory;

public class VRRobotController : MonoBehaviour
{
    public XRController rightController;  // Assign in Inspector
    public Transform robotEndEffector;     // Assign robot's end-effector

    private ROSConnection ros;
    private string trajectoryTopic = "/arm_controller/trajectory";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JointTrajectoryMsg>(trajectoryTopic);
    }

    void Update()
    {
        // Read controller position/rotation
        Vector3 controllerPos = rightController.transform.position;
        Quaternion controllerRot = rightController.transform.rotation;

        // Map to robot workspace (scale and offset)
        Vector3 targetPos = controllerPos * 2.0f + new Vector3(0, -1, 0.5f);

        // When trigger pressed, send command to robot
        if (rightController.inputDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.triggerButton, out bool triggerPressed))
        {
            if (triggerPressed)
            {
                SendRobotCommand(targetPos, controllerRot);

                // Haptic feedback
                rightController.inputDevice.SendHapticImpulse(0, 0.5f, 0.1f);
            }
        }

        // Visualize target in Unity
        robotEndEffector.position = Vector3.Lerp(robotEndEffector.position, targetPos, Time.deltaTime * 5f);
    }

    void SendRobotCommand(Vector3 position, Quaternion rotation)
    {
        // Create ROS trajectory message (simplified)
        JointTrajectoryMsg msg = new JointTrajectoryMsg();
        // ... populate with IK-solved joint angles ...

        ros.Publish(trajectoryTopic, msg);

        Debug.Log($"Sent robot command: {position}");
    }
}
```

**Result**: Wearing Quest 2, user sees robot in VR, moves hand to control end-effector, feels haptic feedback on contact.

### Example 3: AR Debugging Overlay (HoloLens)

**Scenario**: Overlay planned path and sensor FOV on real robot using HoloLens AR glasses.

**Unity AR Setup**:
1. Install "AR Foundation" and "Windows XR Plugin"
2. Build → Universal Windows Platform (UWP)

**C# Script** (`ARPathVisualizer.cs`):

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;

public class ARPathVisualizer : MonoBehaviour
{
    public LineRenderer pathRenderer;
    public string pathTopic = "/planned_path";

    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<PathMsg>(pathTopic, UpdatePath);
    }

    void UpdatePath(PathMsg pathMsg)
    {
        // Convert ROS path to Unity LineRenderer
        pathRenderer.positionCount = pathMsg.poses.Length;

        for (int i = 0; i < pathMsg.poses.Length; i++)
        {
            var pose = pathMsg.poses[i].pose.position;
            pathRenderer.SetPosition(i, new Vector3((float)pose.x, (float)pose.z, (float)pose.y));
        }

        Debug.Log($"Updated path with {pathMsg.poses.Length} waypoints");
    }
}
```

**Result**: Engineer wears HoloLens, sees holographic green line showing robot's planned path overlaid on physical workspace.

### Common Pitfalls

1. **WebSocket Connection Failures**: Ensure rosbridge_server is running **before** pressing Play in Unity. Check firewall allows port 9090.

2. **Coordinate Frame Mismatches**: Unity uses Y-up, ROS uses Z-up. Always apply rotation when importing URDFs or converting poses.

3. **Message Type Mismatches**: ROS# message definitions must match ROS 2 package versions. Regenerate message C# files if mismatches occur.

4. **Performance Issues**: Disable V-Sync (Edit → Project Settings → Quality) and reduce physics timestep for smooth 60+ FPS.

5. **Missing Meshes**: Unity can't find URDF mesh files if paths are absolute (`/home/user/...`). Use relative paths or copy meshes to Unity project.

## Further Resources

### Official Documentation
- **Unity Robotics Hub**: [https://github.com/Unity-Technologies/Unity-Robotics-Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- **ROS# (ROS-Sharp)**: [https://github.com/siemens/ros-sharp](https://github.com/siemens/ros-sharp)
- **Unity XR Toolkit**: [https://docs.unity3d.com/Packages/com.unity.xr.interaction.toolkit@latest](https://docs.unity3d.com/Packages/com.unity.xr.interaction.toolkit@latest)

### Video Tutorials
- Unity Robotics Tutorial Series (Unity Learn)
- VR Robot Teleoperation Demo (YouTube)

### Research Applications
- *Synthetic Data for Robot Learning* (Google Research, 2023)
- *Domain Randomization for Sim-to-Real Transfer* (OpenAI, 2022)

---

**Chapter 7 Complete!** You now have the tools to simulate advanced sensors and integrate Unity for visualization, synthetic data generation, and immersive interfaces.

**Next Chapter**: [Chapter 8: NVIDIA Isaac Platform](/docs/chapter-08/) for GPU-accelerated simulation and Isaac ROS.
