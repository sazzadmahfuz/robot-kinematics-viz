# 🦾 Robot Arm Kinematics Visualizer

<div align="center">

[![Live Demo](https://img.shields.io/badge/▶_Live_Demo-7B61FF?style=for-the-badge)](https://sazzadmahfuz.github.io/robot-kinematics-viz)
[![JavaScript](https://img.shields.io/badge/JavaScript-F7DF1E?style=for-the-badge&logo=javascript&logoColor=black)](.)
[![Canvas API](https://img.shields.io/badge/Canvas_API-FF6B6B?style=for-the-badge)](.)
[![Course](https://img.shields.io/badge/HAMK-Robot_Kinematics-003087?style=for-the-badge)](.)

> **Course project for Robot Kinematics — HAMK University of Applied Sciences**  
> Interactive 3-DOF planar robot arm simulator with forward kinematics (FK) and numerical inverse kinematics (IK), real-time joint control, and animated path planning.

</div>

---

## 📸 Demo

```
┌───────────────────────────────────────┬────────────────────────┐
│  // 3-DOF PLANAR ROBOT ARM  [FK MODE] │  Control Mode          │
│                                       │  [FK — Forward] [IK]   │
│         ◉ Joint 1 (Shoulder)          ├────────────────────────┤
│         │                             │  Joint Angles          │
│         ◉ Joint 2 (Elbow)             │  θ₁ Shoulder  ——●——  45°│
│         │                             │  θ₂ Elbow     ——●——  30°│
│         ✛ End-Effector                │  θ₃ Wrist    ——●—— -20°│
│                                       ├────────────────────────┤
│  [Workspace circle — dashed]          │  Link Lengths (px)     │
│                                       │  L1  [130]  L2 [100]   │
│  EE: (142, 198) px | reach: 242       │  L3  [60]              │
│                                       ├────────────────────────┤
│                                       │  X: 142px  Y: 198px    │
│                                       │  Reach: 242px  Ang:55° │
│                                       │  [▶ Auto-animate]      │
└───────────────────────────────────────┴────────────────────────┘
```

---

## ✨ Features

| Feature | Description |
|---|---|
| **Forward Kinematics (FK)** | Control θ₁, θ₂, θ₃ directly via sliders; end-effector follows |
| **Inverse Kinematics (IK)** | Click canvas to set target; arm solves configuration automatically |
| **3-DOF Planar Arm** | Shoulder, Elbow, Wrist — configurable link lengths |
| **Live Angle Arcs** | Visual angle indicator drawn at each joint |
| **Workspace Boundary** | Dashed circle showing full reachable workspace |
| **Auto-Animate** | Smooth continuous motion demo for FK and IK (circular path) |
| **End-Effector HUD** | Real-time X, Y, reach distance, and orientation readout |
| **Adjustable Links** | Change L1/L2/L3 lengths live and watch kinematics update |

---

## 🧠 Mathematical Foundation

### Forward Kinematics
Given joint angles θ₁, θ₂, θ₃ and link lengths L₁, L₂, L₃, the end-effector position is computed by successive rotations using the **Denavit–Hartenberg** convention for planar arms:

```
α₁ = θ₁
α₂ = θ₁ + θ₂
α₃ = θ₁ + θ₂ + θ₃

P₀ = (base)
P₁ = P₀ + L₁·[cos(α₁), sin(α₁)]
P₂ = P₁ + L₂·[cos(α₂), sin(α₂)]
P₃ = P₂ + L₃·[cos(α₃), sin(α₃)]   ← End-Effector
```

```javascript
function fk(j1, j2, j3) {
    const [L1, L2, L3] = getLinks();
    const a1 = j1, a2 = j1 + j2, a3 = j1 + j2 + j3;
    const p1 = [OX + L1*Math.cos(a1), OY - L1*Math.sin(a1)];
    const p2 = [p1[0] + L2*Math.cos(a2), p1[1] - L2*Math.sin(a2)];
    const p3 = [p2[0] + L3*Math.cos(a3), p2[1] - L3*Math.sin(a3)];
    return { points: [origin, p1, p2, p3], angles: [a1, a2, a3] };
}
```

### Inverse Kinematics
The IK solver uses the **geometric/analytical method** (closed-form solution for 2-link sub-chain + wrist compensation):

```
Step 1: Compute wrist position (subtract L3 contribution in direction of target)
Step 2: Apply law of cosines to find elbow angle θ₂:
        cos(θ₂) = (|Pw|² - L1² - L2²) / (2·L1·L2)
Step 3: Solve shoulder angle θ₁ using atan2
Step 4: Wrist angle θ₃ = target_angle - θ₁ - θ₂
```

```javascript
function solveIK(tx, ty) {
    const [L1, L2, L3] = getLinks();
    const ang = Math.atan2(OY - ty, tx - OX);
    const wx  = (tx - OX) - L3 * Math.cos(ang);
    const wy  = (OY - ty) - L3 * Math.sin(ang);
    const cosA2 = (wx*wx + wy*wy - L1*L1 - L2*L2) / (2*L1*L2);
    const a2  = Math.acos(Math.max(-1, Math.min(1, cosA2)));  // clamp for workspace limit
    const a1  = Math.atan2(wy, wx) - Math.atan2(L2*Math.sin(a2), L1 + L2*Math.cos(a2));
    const a3  = ang - a1 - a2;
    return { j1: a1, j2: a2, j3: a3 };
}
```

> **Workspace clamping**: When the target is outside the reachable workspace, the solver clamps the wrist distance to `L1 + L2`, so the arm always reaches toward the target at maximum extension rather than failing.

---

## 🗂️ File Structure

```
robot-kinematics-viz/
└── index.html        # Self-contained — Canvas API, all math inline, no dependencies
```

---

## 🔗 Relation to Universal Robots (UR) Experience

This project directly reflects concepts from my **Universal Robots e-Series Core, Application and Pro certifications**. UR cobots use the same FK/IK mathematical principles — the UR teach pendant exposes joint angles (FK mode) and target poses (IK mode) matching this visualiser's two modes exactly. The 6-DOF UR arm extends this 3-DOF planar model into 3D space with additional joints for orientation control.

---

## 🎓 Academic Context

**Course**: Robot Kinematics  
**Programme**: BEng ICT & Robotics, HAMK University of Applied Sciences  
**Topics covered**: Denavit–Hartenberg parameters, forward kinematics, inverse kinematics (geometric method), workspace analysis, joint-space vs. task-space control.

---

## 👤 Author

**Sazzad Ibna Mahfuz** — BEng ICT & Robotics, HAMK · Universal Robots Certified  
[Portfolio](https://sazzadmahfuz.github.io) · [LinkedIn](https://www.linkedin.com/in/sazzad-mahfuz) · [GitHub](https://github.com/sazzadmahfuz)
