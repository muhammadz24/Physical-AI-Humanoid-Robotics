---
sidebar_position: 4
title: Self-Assessment
---

# Chapter 2: Self-Assessment

Test your understanding of humanoid robotics fundamentals.

---

## Question 1: Degrees of Freedom

**A humanoid robot has 6 DOF per leg (3 hip, 1 knee, 2 ankle) and 7 DOF per arm. How many total DOF does it have, and why might more DOF be useful?**

<details>
<summary>Answer</summary>

**Total DOF**: 6×2 (legs) + 7×2 (arms) = **26 DOF** (excluding torso/head)

**Benefits of More DOF**:
- **Redundancy**: Multiple joint configurations can achieve the same end-effector position, allowing optimization for obstacles, joint limits, or energy efficiency
- **Dexterity**: More DOF enables complex manipulations (e.g., reaching around obstacles)
- **Human-Like Motion**: Matches human joint structure for natural appearance

**Example**: A 7-DOF arm can keep its elbow raised while reaching forward (useful for avoiding table edges), while a 6-DOF arm has a unique configuration for each end position.

</details>

---

## Question 2: Actuation Trade-offs

**What are the primary advantages and disadvantages of hydraulic actuation compared to electric motors for humanoid robots?**

<details>
<summary>Answer</summary>

**Hydraulic Actuation**:
- ✅ **Advantages**:
  - Very high power-to-weight ratio (10x higher than electric)
  - Massive torque output (enables dynamic motions like jumping, running)
  - Natural compliance (forgiving under impacts)
- ❌ **Disadvantages**:
  - Complex system (pumps, valves, hoses, fluid reservoirs)
  - Noisy operation (pump noise)
  - Potential fluid leaks (maintenance issue)
  - Higher cost and complexity

**Electric Motors**:
- ✅ **Advantages**:
  - Precise position control
  - Energy-efficient
  - Quiet, clean operation
  - Easier to program and control
- ❌ **Disadvantages**:
  - Lower power-to-weight ratio
  - Limited torque (requires heavy gear boxes for high torque)

**Choice**: Research robots (NAO, Pepper) use electric for ease of use. Dynamic robots (Atlas) use hydraulics for athletic performance.

</details>

---

## Question 3: Center of Pressure

**Explain why monitoring the Center of Pressure (CoP) in the robot's feet is critical for balance.**

<details>
<summary>Answer</summary>

**Center of Pressure (CoP)**: The point on the ground where the resultant of all ground reaction forces acts.

**Why It's Critical**:
- **Stability Requirement**: For the robot to remain balanced (not tip over), the CoP must stay within the support polygon (the area enclosed by foot/feet in contact with ground)
- **If CoP Moves Outside**: The robot will begin to rotate (tip) around the edge of the support polygon, leading to a fall
- **Dynamic Control**: By tracking CoP, the controller can:
  - Adjust body lean to keep CoP centered
  - Shift weight between feet during walking
  - Trigger emergency stepping to prevent falls

**Example Code**: The `get_center_of_pressure()` function in the chapter computes CoP from force sensor readings. If CoP approaches the boundary of the support polygon, the controller must take corrective action (lean opposite direction or take a step).

</details>

---

## Question 4: Servo Motor Control

**In the ServoMotor class example, what would happen if the proportional gain was set too high (e.g., 100 instead of 0.5)?**

<details>
<summary>Answer</summary>

**High Proportional Gain** (e.g., kp = 100):
- **Initial Response**: Very aggressive correction—large torque applied for small errors
- **Likely Result**: **Oscillation** or **Instability**
  - Motor overshoots target
  - Error becomes negative (target < current)
  - Motor applies large torque in opposite direction
  - Overshoots again in other direction
  - Cycle repeats → oscillation around target

**Physical Consequences**:
- Mechanical stress on gears and joints
- Excessive energy consumption
- Potential damage if oscillations are violent
- Poor precision (never settles to target)

**Solution**: **Tuning PID gains**:
- Lower proportional gain (reduce overshoot)
- Add derivative term (dampen oscillations)
- Add integral term (eliminate steady-state error)

**Proper Gain Values**: Depend on motor inertia, load, and desired response time. Typically tuned experimentally or via control theory methods (Ziegler-Nichols, pole placement).

</details>

---

## Question 5: Forward vs. Inverse Kinematics

**Why is inverse kinematics generally more challenging to solve than forward kinematics?**

<details>
<summary>Answer</summary>

**Forward Kinematics** (FK):
- **Input**: Joint angles → **Output**: End-effector position
- **Solution**: **Straightforward**—apply transformation matrices sequentially
- **Uniqueness**: Given joint angles, there is exactly one end-effector position
- **Computation**: Fast (matrix multiplications)

**Inverse Kinematics** (IK):
- **Input**: End-effector position → **Output**: Joint angles
- **Solution**: **Complex**—solve system of nonlinear equations
- **Challenges**:
  1. **Multiple Solutions**: Many joint configurations can achieve the same end position (redundancy)
     - Example: 7-DOF arm has infinite solutions for most positions
  2. **No Solution**: Target may be out of reach or violate joint limits
  3. **Computational Cost**: Analytical solutions exist only for simple geometries. Complex robots require numerical methods (Jacobian-based iteration, optimization)

**Example**: 2-link arm IK has 2 solutions (elbow up/down). A 6-DOF arm may have 8+ solutions. A humanoid arm (7-DOF) has infinite solutions.

**Methods**:
- **Analytical**: Closed-form equations (only for simple robots)
- **Numerical**: Jacobian pseudo-inverse, Newton-Raphson iteration
- **Optimization**: Minimize error while satisfying constraints

</details>

---

## Question 6: Jacobian Matrix Application

**The Jacobian matrix relates joint velocities to end-effector velocity. How would you use the Jacobian to make a robot's hand move in a straight line?**

<details>
<summary>Answer</summary>

**Steps for Straight-Line Motion Using Jacobian**:

1. **Define Desired Velocity**: Specify desired end-effector linear velocity vector (e.g., ẋ_desired = [0.1, 0, 0] m/s for moving right)

2. **Compute Current Jacobian**: Evaluate Jacobian matrix J at current joint configuration θ:
   ```
   ẋ = J(θ) · θ̇
   ```

3. **Solve for Joint Velocities**: Use pseudo-inverse to find required joint velocities:
   ```
   θ̇ = J⁺(θ) · ẋ_desired
   ```
   Where J⁺ is the Moore-Penrose pseudo-inverse.

4. **Command Joint Velocities**: Send θ̇ to motor controllers

5. **Repeat**: At each control cycle (e.g., 100 Hz):
   - Update current joint angles from encoders
   - Recompute Jacobian at new configuration
   - Compute new joint velocities for desired end-effector velocity
   - Send to motors

**Result**: End-effector follows a straight-line path because velocity is continuously directed along the desired direction.

**Code Example** (from chapter):
```python
J = compute_jacobian_2d(arm, theta1, theta2)
desired_velocity = np.array([0.1, 0.0])  # Move right
joint_velocities = np.linalg.pinv(J) @ desired_velocity
```

**Advantages**: Smooth motion, real-time adaptation to changing targets.

</details>

---

## Question 7: Zero Moment Point (ZMP)

**A humanoid robot is standing on both feet. It shifts its weight to the left foot to prepare to lift the right foot. Describe what happens to the ZMP during this weight shift.**

<details>
<summary>Answer</summary>

**Initial State** (standing on both feet):
- **Support Polygon**: Rectangle formed by both feet
- **ZMP Position**: Centered between the two feet (assuming symmetric weight distribution)

**Weight Shift to Left Foot**:
1. **Robot Leans Left**: Center of mass (CoM) shifts left
2. **ZMP Moves Left**: As more weight transfers to left foot, ZMP moves leftward within the support polygon
3. **Right Foot Force Decreases**: Force sensors in right foot read lower values as weight shifts off it

**Preparation to Lift Right Foot**:
- **ZMP Must Be Inside Left Foot**: Before right foot can lift, ZMP must be entirely within the left foot's support polygon
- **Safety Margin**: Typically, ZMP should be well inside (not near edges) for stability

**Code Verification** (from chapter):
```python
controller.is_stable(zmp, left_foot_polygon)  # Returns True when safe to lift right foot
```

**Physical Observation**: You can feel this when standing—shifting weight left causes increased pressure on left foot and lighter pressure on right.

**Walking**: This ZMP shift is the first phase of every step—transfer weight to support foot, then swing the other leg.

</details>

---

## Question 8: Trajectory Planning

**What is the purpose of generating smooth trajectories (like the linear_trajectory function) instead of just commanding the robot to move directly from start to end position?**

<details>
<summary>Answer</summary>

**Problems with Direct Position Jumps**:
- **Discontinuous Velocity**: Instantaneous change from stationary to moving causes:
  - **Infinite Acceleration** (physically impossible)
  - **Motor Saturation**: Motors cannot produce infinite torque
  - **Mechanical Stress**: Sudden jerks damage gears and joints
  - **Imprecise Motion**: Robot oscillates around target

**Benefits of Smooth Trajectories**:
1. **Continuous Velocity**: Gradual acceleration and deceleration
2. **Finite Acceleration**: Motors can track the command
3. **Reduced Wear**: Gentle motions extend hardware lifetime
4. **Better Accuracy**: Controlled motions reach target precisely
5. **Safety**: Predictable motion avoids sudden collisions with humans or objects

**Trajectory Types**:
- **Linear**: Constant velocity (what the chapter example uses)
- **Trapezoidal**: Acceleration phase, constant velocity, deceleration phase
- **Polynomial**: Smooth acceleration profiles (minimum jerk trajectories)
- **Splines**: Complex paths through multiple waypoints

**Example from Code**:
```python
linear_trajectory([0.3, 0.2], [0.5, 0.4], duration=2.0, dt=0.5)
```
Generates 5 waypoints over 2 seconds instead of jumping directly—smooth, trackable motion.

</details>

---

## Question 9: Reachability and Joint Limits

**The inverse kinematics function returns `None` if the target is unreachable. List three reasons why a target position might be unreachable.**

<details>
<summary>Answer</summary>

**Three Reasons for Unreachability**:

1. **Beyond Workspace** (Too Far):
   - Target distance exceeds sum of link lengths
   - Example: 2-link arm with L1=0.3m, L2=0.25m cannot reach beyond 0.55m radius
   - Code Check:
     ```python
     if d > (self.L1 + self.L2):
         return None  # Too far
     ```

2. **Inside Dead Zone** (Too Close):
   - Target is closer than minimum reach (difference of link lengths)
   - Example: Same arm cannot reach closer than |0.3 - 0.25| = 0.05m
   - Physically: Links cannot fold back on themselves completely
   - Code Check:
     ```python
     if d < abs(self.L1 - self.L2):
         return None  # Too close
     ```

3. **Joint Limit Violations**:
   - Analytical solution produces joint angles outside physical limits
   - Example: Shoulder joint limited to -90° to +180°, but IK solution requires 200°
   - Must Check: `if theta1 > MAX_ANGLE or theta1 < MIN_ANGLE: return None`

**Additional Reasons** (not in simplified example):
- **Singularities**: Jacobian becomes non-invertible (loss of one or more DOF)
- **Obstacle Collisions**: Path to target intersects robot body or environment
- **Orientation Constraints**: For 6-DOF arms, target orientation may be unreachable even if position is valid

**Practical Handling**: Return nearest reachable position or suggest alternative target to user.

</details>

---

## Question 10: Energy Management

**Why do humanoid robots consume power even when standing still, and how might this be minimized?**

<details>
<summary>Answer</summary>

**Why Power Consumed While Stationary**:
- **Gravity Compensation**: Electric motors must continuously output torque to hold joints against gravity
  - Example: Holding arm horizontal requires constant shoulder torque
- **Active Balance**: Standing on two feet requires continuous micro-adjustments to maintain balance (ankle/hip torques)
- **No Passive Locking**: Unlike humans (who lock knee joints when standing), most robots rely on active motor control

**Power Consumption Breakdown**:
- **Standing**: 50-100W (mostly gravity compensation in legs, torso, arms)
- **Walking**: 100-300W (add locomotion dynamics)
- **Running**: 500-1000W (high acceleration, impact forces)

**Minimization Strategies**:

1. **Mechanical Locks**:
   - Deploy locking mechanisms at joints when stationary
   - Motors can turn off, lock holds position
   - Example: Some industrial robots have brake mechanisms

2. **Passive Dynamics**:
   - Design joints with natural equilibrium positions
   - Springs to counteract gravity (e.g., spring-loaded hip)
   - Reduces motor effort

3. **Efficient Postures**:
   - Stand with straight legs (reduces knee torque)
   - Keep arms low (reduces shoulder torque)
   - Minimize moment arms between CoM and support

4. **Smart Power Management**:
   - Enter low-power mode when idle
   - Reduce control frequency (from 1kHz to 10Hz when stationary)
   - Sleep non-critical sensors

5. **Lightweight Design**:
   - Carbon fiber/magnesium instead of steel
   - Less mass → less gravity load → less holding torque

**Human Comparison**: Humans are very energy-efficient at standing (~5-10W) because:
- Knee joint mechanically locks
- Passive ligaments/tendons support posture
- Minimal muscle activation required

**Result**: Battery-powered humanoids typically operate 30min-2hrs before recharging.

</details>

---

## Scoring Guide

- **8-10 correct**: Excellent! Strong understanding of humanoid robotics.
- **6-7 correct**: Good! Review kinematics and control concepts.
- **4-5 correct**: Fair. Revisit technical sections and code examples.
- **0-3 correct**: Re-read chapter focusing on mathematical concepts.

---

## Next Steps

- **Hands-On**: Implement the kinematics code examples, visualize arm motion
- **Further Study**: Explore ROS MoveIt for practical IK solving
- **Proceed**: [Chapter 3: ROS 2 Fundamentals](/chapter-03/) to learn robot software frameworks

**Great work completing Chapter 2!**
