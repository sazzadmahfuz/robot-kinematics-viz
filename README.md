# Robot Arm Kinematics Visualizer

<div align="center">

[![Live Demo](https://img.shields.io/badge/▶_Live_Demo-7B61FF?style=for-the-badge)](https://sazzadmahfuz.github.io/robot-kinematics-viz)
[![JavaScript](https://img.shields.io/badge/JavaScript-F7DF1E?style=for-the-badge&logo=javascript&logoColor=black)](.)
[![Canvas API](https://img.shields.io/badge/Canvas_API-FF6B6B?style=for-the-badge)](.)
[![Robot Kinematics](https://img.shields.io/badge/Robotics-Kinematics-00FFD1?style=for-the-badge)](.)
[![Inverse Kinematics](https://img.shields.io/badge/IK-Numerical_Solver-orange?style=for-the-badge)](.)
[![Course](https://img.shields.io/badge/HAMK-Robot_Kinematics-003087?style=for-the-badge)](.)

### Advanced Interactive Robot Kinematics Simulator & Visualization System

Real-time 3-DOF planar robot arm simulation developed using:

**JavaScript · HTML5 Canvas · Forward Kinematics · Inverse Kinematics · Workspace Analysis · Motion Animation**

---

### Developed by

# Sazzad Ibna Mahfuz

Bachelor of Engineering — ICT Robotics  
Häme University of Applied Sciences (HAMK), Finland

</div>

---

# Project Overview

This project demonstrates a complete interactive **Robot Kinematics Visualization and Simulation System** designed for robotics education, motion analysis, and robot control understanding.

The simulator was developed as part of the:

### Robot Kinematics Course — HAMK University of Applied Sciences

The application provides a fully interactive environment for studying:

- Forward Kinematics (FK)
- Inverse Kinematics (IK)
- Joint-space control
- Task-space control
- Workspace analysis
- Motion interpolation
- End-effector positioning
- Planar robotic manipulators
- Geometric robot modeling
- Real-time robot animation

The simulator allows users to:

✅ Control robot joints interactively  
✅ Visualize link transformations in real time  
✅ Solve inverse kinematics numerically  
✅ Click anywhere to move the end-effector  
✅ Analyze workspace boundaries  
✅ Simulate robotic trajectories  
✅ Study arm singularities and reachability  
✅ Animate robot motion continuously  
✅ Observe end-effector orientation and reach

The entire project is implemented using:

- Vanilla JavaScript
- HTML5 Canvas API
- Real-time mathematical computation
- Dynamic graphical rendering

without any external libraries or robotics engines.

---

# Project Objectives

The main objectives of this project were:

- Understand robotic manipulator mathematics
- Implement forward kinematics algorithms
- Develop inverse kinematics solvers
- Study planar robot arm geometry
- Visualize robot coordinate transformations
- Simulate robot joint motion
- Implement workspace analysis
- Create real-time robot interaction systems
- Develop animation-based robotics simulations
- Understand task-space and joint-space control
- Build robotics visualization interfaces
- Apply robotics mathematics to interactive systems

---

# Robot System Architecture

```text
User Input
    │
    ▼
Joint Control / Target Position
    │
    ▼
Forward or Inverse Kinematics
    │
    ▼
Joint Angle Computation
    │
    ▼
Transformation Calculations
    │
    ▼
Canvas Rendering Engine
    │
    ▼
Robot Arm Visualization
    │
    ▼
End-Effector Metrics & HUD
```

---

# Core Features

| Feature | Description |
|---|---|
| **Forward Kinematics (FK)** | Direct control of θ₁, θ₂, θ₃ joint angles |
| **Inverse Kinematics (IK)** | Automatic target solving via geometric IK |
| **3-DOF Planar Manipulator** | Shoulder–Elbow–Wrist robotic arm |
| **Interactive Canvas** | Click-to-target robot movement |
| **Workspace Visualization** | Reachable workspace boundaries |
| **Auto-Animation System** | Continuous robotic motion generation |
| **Live End-Effector Metrics** | Real-time position and orientation |
| **Joint Angle Visualization** | Dynamic angle arcs at each joint |
| **Dynamic Link Lengths** | Adjustable manipulator dimensions |
| **Target Reachability Detection** | Workspace clamping for unreachable points |
| **Smooth Rendering Engine** | High-frequency canvas updates |
| **Motion Interpolation** | Continuous movement transitions |

---

# Robot Manipulator Model

The simulator models a:

### 3-DOF Planar Robot Arm

consisting of:

| Joint | Function |
|---|---|
| θ₁ | Shoulder rotation |
| θ₂ | Elbow rotation |
| θ₃ | Wrist orientation |

---

## Link Configuration

| Link | Description |
|---|---|
| L₁ | Upper arm |
| L₂ | Forearm |
| L₃ | Wrist / End-effector |

---

## 🔄 Forward Kinematics (FK)

The simulator implements full forward kinematics calculations using trigonometric coordinate transformations.

---

## FK Principle

Forward kinematics computes the:

```text
Joint Angles → End-Effector Position
```

mapping.

---

## Mathematical Model

Given:

- Joint angles θ₁, θ₂, θ₃
- Link lengths L₁, L₂, L₃

the end-effector position is calculated through cumulative transformations.

---

## Forward Kinematics Equations

```math
\alpha_1=\theta_1,\quad \alpha_2=\theta_1+\theta_2,\quad \alpha_3=\theta_1+\theta_2+\theta_3
```

```math
P_1=P_0+L_1[\cos(\alpha_1),\sin(\alpha_1)]
```

```math
P_2=P_1+L_2[\cos(\alpha_2),\sin(\alpha_2)]
```

```math
P_3=P_2+L_3[\cos(\alpha_3),\sin(\alpha_3)]
```

---

## FK JavaScript Implementation

```JavaScript
function fk(j1, j2, j3) {
    const [L1, L2, L3] = getLinks();

    const a1 = j1;
    const a2 = j1 + j2;
    const a3 = j1 + j2 + j3;

    const p1 = [
        OX + L1 * Math.cos(a1),
        OY - L1 * Math.sin(a1)
    ];

    const p2 = [
        p1[0] + L2 * Math.cos(a2),
        p1[1] - L2 * Math.sin(a2)
    ];

    const p3 = [
        p2[0] + L3 * Math.cos(a3),
        p2[1] - L3 * Math.sin(a3)
    ];

    return { points: [origin, p1, p2, p3] };
}
```

---

# Inverse Kinematics (IK)

The system also implements an analytical/geometric inverse kinematics solver.

Inverse kinematics computes:

```text
Target Position → Joint Angles
```

---

# IK Solver Strategy

The IK solver works in four major steps:

```text
1. Compute wrist position
2. Apply law of cosines
3. Solve shoulder angle
4. Compute wrist orientation
```

---

## Geometric IK Equation

```math
\cos(\theta_2)=\frac{|P_w|^2-L_1^2-L_2^2}{2L_1L_2}
```

---

## Shoulder Angle

```math
\theta_1=\operatorname{atan2}(w_y,w_x)-\operatorname{atan2}(L_2\sin(\theta_2),L_1+L_2\cos(\theta_2))
```

---

## Wrist Orientation

```math
\theta_3=\theta_{target}-\theta_1-\theta_2
```

---

# IK JavaScript Implementation

```javascript
function solveIK(tx, ty) {

    const [L1, L2, L3] = getLinks();

    const ang = Math.atan2(OY - ty, tx - OX);

    const wx = (tx - OX) - L3 * Math.cos(ang);
    const wy = (OY - ty) - L3 * Math.sin(ang);

    const cosA2 =
        (wx*wx + wy*wy - L1*L1 - L2*L2) /
        (2 * L1 * L2);

    const a2 = Math.acos(
        Math.max(-1, Math.min(1, cosA2))
    );

    const a1 =
        Math.atan2(wy, wx) -
        Math.atan2(
            L2 * Math.sin(a2),
            L1 + L2 * Math.cos(a2)
        );

    const a3 = ang - a1 - a2;

    return {
        j1: a1,
        j2: a2,
        j3: a3
    };
}
```

---

# Workspace Analysis

The simulator dynamically computes the robot workspace.

---

## Workspace Boundaries

### Maximum Reach

```math
R_{max}=L_1+L_2+L_3
```

---

### Minimum Reach

```math
R_{min}=|L_1-L_2-L_3|
```

---

The workspace is visualized using dashed boundary circles rendered directly on the canvas.

---

### Workspace Clamping

One advanced feature implemented in the IK solver is:

### Reachability Clamping

When a target is outside the reachable workspace:

- The solver does not fail
- The arm extends toward the target
- Joint angles remain stable
- Numerical instability is avoided

This creates smoother robotic behavior similar to industrial manipulators.

---

## Interactive Visualization Engine

The rendering engine uses:

## HTML5 Canvas API

to create a real-time robotics visualization environment.

---

## Rendering Features

✅ Real-time arm rendering  
✅ Dynamic joint glow effects  
✅ Angle arc visualization  
✅ Workspace overlays  
✅ IK target crosshairs  
✅ Motion trails  
✅ Smooth animations  
✅ Responsive canvas scaling  
✅ Grid visualization  
✅ Real-time HUD updates

---

# Simulation Interface

| Component | Function |
|---|---|
| Joint Sliders | Direct FK manipulation |
| IK Target Mode | Mouse-based target solving |
| HUD Metrics | End-effector feedback |
| Workspace Circles | Reach visualization |
| Auto-Animation | Continuous robot movement |
| Angle Indicators | Joint rotation visualization |

---

# Motion Animation System

The simulator supports automated robot motion generation.

---

## FK Animation

Joint trajectories are generated using sinusoidal functions:

```math
\theta_1(t)=80\sin(t)
```

```math
\theta_2(t)=60\sin(1.3t+1)+30
```

```math
\theta_3(t)=50\sin(1.7t+2)-15
```

---

## IK Animation

The target follows a circular trajectory:

```math
x(t)=r\cos(t),\quad y(t)=r\sin(t)
```

---

This creates smooth robotic movement demonstrations for both FK and IK modes.

---

# Real-Time Metrics

The simulator continuously computes:

- End-effector X position
- End-effector Y position
- Arm reach distance
- End-effector orientation
- Joint rotations
- Workspace distance
- IK solution validity

---

# Robotics Concepts Implemented

The project combines multiple robotics concepts including:

- Robot Kinematics
- Coordinate Transformations
- Trigonometric Modeling
- Workspace Analysis
- Motion Planning
- Joint-space Control
- Task-space Control
- Analytical IK Solving
- Numerical Stability
- Robot Simulation
- Interactive Visualization
- Real-time Rendering

---

# Engineering Concepts Applied

| Concept | Application |
|---|---|
| Linear Algebra | Coordinate transformations |
| Trigonometry | Joint rotations |
| Geometry | IK solving |
| Numerical Methods | Reachability handling |
| Real-Time Systems | Interactive updates |
| Graphics Programming | Canvas rendering |
| Human–Machine Interaction | Interactive robot control |

---

# Relation to Industrial Robotics

This simulator directly reflects concepts used in real industrial robots such as:

- Universal Robots (UR)
- ABB Robotics
- FANUC
- KUKA
- Dobot
- Yaskawa

Industrial robotic systems use the same mathematical foundations implemented in this project.

---

### Relation to Universal Robots Certification

This project strongly relates to concepts learned during:

### Universal Robots e-Series Certifications

including:

- Core Training
- Application Training
- Programming Training

The simulator demonstrates the same FK/IK concepts exposed in industrial robot teach pendants.

---

# Topics Covered

- Forward Kinematics
- Inverse Kinematics
- Denavit–Hartenberg Concepts
- Workspace Analysis
- Robot Motion
- Coordinate Systems
- Task-Space Control
- Joint-Space Control
- Interactive Robotics
- Manipulator Mathematics
- Robot Visualization

---

# Key Learning Outcomes

This project helped develop practical robotics engineering skills in:

✅ Robot kinematics mathematics  
✅ FK & IK implementation  
✅ Real-time robot simulation  
✅ Interactive robotics interfaces  
✅ Workspace analysis  
✅ Coordinate transformations  
✅ Robot rendering systems  
✅ Motion animation  
✅ Human–robot interaction systems  
✅ Industrial robotics concepts  
✅ Numerical robotics methods

---

# Future Improvements

Potential future developments include:

- 6-DOF robotic arm simulation
- Full Denavit–Hartenberg matrix support
- 3D robot visualization
- WebGL rendering
- ROS2 integration
- Collision detection
- Trajectory planning
- Path optimization
- Jacobian-based IK solvers
- Dynamics simulation
- Torque analysis
- Physics engine integration
- Multi-robot systems
- AI-assisted motion planning

---

# 👨‍💻 Author

# Sazzad Ibna Mahfuz

Bachelor of Engineering — ICT Robotics  
Häme University of Applied Sciences (HAMK)  
Finland  
[Portfolio](https://sazzadmahfuz.github.io) · [LinkedIn](https://www.linkedin.com/in/sazzad-mahfuz) · [GitHub](https://github.com/sazzadmahfuz)


---

<div align="center">

# ⭐ If you found this robotics project interesting, consider starring the repository!

</div>
