
# Realistic Table Tennis Physics Simulation: Core Equations from "Flight and bounce of spinning sports balls"

This markdown summarizes the key equations from the paper and explains how to implement them in PyBullet. Each formula is tagged with the section where it appears in the original paper.

---

## 🧮 1. Flight Phase – Motion in Air

### Newton’s Second Law with Drag and Magnus Force  
**Equation:**  
\[
\vec{a} = \vec{g} + \frac{1}{m}(\vec{F}_{\text{drag}} + \vec{F}_{\text{Magnus}})
\]  
**Paper location:** Section II – p.2, paragraph 2

---

### Drag Force  
**Equation:**  
\[
\vec{F}_{\text{drag}} = -\frac{1}{2} \rho C_D A \|\vec{v}\| \vec{v}
\]  
- \( \rho \): air density (≈ 1.225 kg/m³)  
- \( C_D \): drag coefficient (~0.4–0.5)  
- \( A = \pi r^2 \): cross-sectional area  
- \( \vec{v} \): velocity vector  

**Paper location:** Section II – Eq. (2), p.2

---

### Magnus Force  
**Equation:**  
\[
\vec{F}_{\text{Magnus}} = \frac{1}{2} \rho C_L A \|\vec{v}\| (\vec{\omega} \times \vec{v})
\]  
- \( C_L \): lift coefficient (~0.2–0.3)  
- \( \vec{\omega} \): angular velocity (spin)

**Paper location:** Section III – p.4, paragraph 1

---

## 💥 2. Bounce Phase – Collision with Table or Racket

### Normal Velocity After Bounce  
**Equation:**  
\[
v_{n}' = -e_n v_n
\]  
- \( e_n \): normal coefficient of restitution

**Paper location:** Section IV – p.6, paragraph 2

---

### Tangential Velocity After Bounce  
**Equation:**  
\[
v_{t}' = v_{t} - \mu (1 + e_n) v_n
\]  
**Paper location:** Section IV – p.6, paragraph 2

---

### Angular Velocity Update  
**Equation:**  
\[
\omega' = \omega + \frac{5}{2r} \mu (1 + e_n) v_n
\]  
**Paper location:** Section IV – p.6, paragraph 2

---

## ✅ Simplification Guidelines

| Effect              | Implement? | Reason |
|---------------------|------------|--------|
| Air Drag            | Yes        | Major impact on light balls |
| Magnus Effect       | Yes        | Crucial for spin behavior |
| Ball deformation    | No         | Minor effect for rigid balls |
| Turbulent wake      | No         | Requires complex CFD |

---

## 🧪 Python Code Template

```python
def apply_aerodynamics(ball_id, velocity, omega):
    rho = 1.225
    r = 0.02
    A = np.pi * r ** 2
    Cd = 0.47
    Cl = 0.2
    mass = 0.0027

    v = np.array(velocity)
    w = np.array(omega)

    F_drag = -0.5 * rho * Cd * A * np.linalg.norm(v) * v
    F_magnus = 0.5 * rho * Cl * A * np.linalg.norm(v) * np.cross(w, v)

    total_force = F_drag + F_magnus
    p.applyExternalForce(ball_id, -1, total_force.tolist(), [0,0,0], p.LINK_FRAME)
```

---

> Reference: Mencke et al., *Flight and bounce of spinning sports balls*, American Journal of Physics, 2020.
