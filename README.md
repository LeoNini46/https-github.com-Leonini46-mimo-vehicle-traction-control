# https-github.com-Leonini46-mimo-vehicle-traction-control
# MIMO Dynamic Systems Modeling & Traction Control Architecture

## 📌 Project Overview
This repository contains the analytical modeling, state-space control design, and stability analysis for a high-performance motorcycle (**Ducati Panigale V2**) undergoing limit-handling cornering maneuvers. 

The core objective of the project is to develop a Multi-Input Multi-Output (MIMO) regulator capable of stabilizing the vehicle's coupled roll and yaw dynamics, ensuring system performance and safety under perturbed track conditions.

---

## 📐 Vehicle Parameters & Dataset
The non-linear model incorporates the mass properties of the vehicle combined with the rider's positioning constraints:
*   **Total Mass ($M$):** $290\text{ kg}$ (Bike: $220\text{ kg}$, Rider: $70\text{ kg}$)
*   **Wheelbase ($l$):** $1.436\text{ m}$
*   **Center of Gravity Height ($h$):** $0.6\text{ m}$ (in upright condition)
*   **CoG Longitudinal Position ($a_1$):** $0.5 \cdot l$
*   **Corner Radius ($R_c$):** $67.5\text{ m}$ (Correntaio Curve simulation layout)

---

## ⚙️ Mathematical & Dynamic Modeling

The vehicle-rider system is modeled via continuous coupled differential equations tracking longitudinal speed ($v$), roll angle ($\theta$), and yaw deviation ($\psi$) relative to the ideal racing line:

### 1. Longitudinal Dynamics
$$M\dot{v} = T - F_a$$
*Where $T$ is the drive thrust and $F_a = \frac{1}{2}\rho A C_x v^2$ represents aerodynamic drag applied directly at the system's center of gravity.*

### 2. Roll Dynamics
$$I_{\theta}\ddot{\theta} = \tau_c - Mg(h\sin\theta + s)$$
*Where $\tau_c = \frac{M v^2}{R_c} h \sin\theta$ is the centrifugal moment, $s$ is the lateral displacement of the CoG, and the moment of inertia is computed as $I_{\theta} = M(h^2 + s^2\cos^2\theta)$.*

### 3. Yaw Dynamics
$$I_{\psi}\ddot{\psi} = -F_a(h\cos\theta + s) + \mu Z_1 l - F_c\sqrt{a_1^2 + h^2\cos^2\theta}$$
*Where $I_{\psi} = M((h\cos\theta)^2 + a_1^2)$ and the vertical load on the tire is evaluated. Tire-road interaction incorporates a non-linear slip dependency via a localized friction sensitivity coefficient: $\mu = 1.2 - k\psi$.*

---

## 🛠️ Control System Architecture

### 1. State-Space Representation & Linearization
The system is framed as a MIMO state-space model defined by:
*   **State Vector:** $\mathbf{x} = \begin{bmatrix} \theta & \dot{\theta} & \psi & \dot{\psi} & v \end{bmatrix}^T$
*   **Input Vector:** $\mathbf{u} = \begin{bmatrix} T & s \end{bmatrix}^T$ (Thrust, Lateral CoG shift)
*   **Output Vector:** $\mathbf{y} = \begin{bmatrix} \theta & \psi \end{bmatrix}^T$

Linearization was performed around the open-loop unstable cornering equilibrium point:
$$\mathbf{x}_{eq} = \begin{bmatrix} 0.72\text{ rad} & 0 & 0.1178\text{ rad} & 0 & 30\text{ m/s} \end{bmatrix}^T$$

### 2. Stability Analysis
Evaluating the eigenvalues of the plant's system matrix ($A$):
$$\text{eig}(A) = \begin{bmatrix} \pm 5.21i & -5.42 & \mathbf{+5.42} & -0.06 \end{bmatrix}$$
Due to the presence of a strictly positive real eigenvalue ($\lambda = +5.42$), the open-loop system is **unstable** via Lyapunov's indirect method. A full-state feedback regulator is mandatory to achieve asymptotic stability.

### 3. Structural Properties
*   **Reachability:** Checked via $\text{rank}(\mathcal{R}) = 5 \rightarrow$ Fully Reachable.
*   **Observability:** Checked via $\text{rank}(\mathcal{O}) = 5 \rightarrow$ Fully Observable.

### 4. Full-State Observer-Regulator Design
Poles were reallocated using a **trial-and-error method / Pole Placement** (`place` and `reg` routines in MATLAB) combined with series integrators to reject steady-state tracking errors. Three control architectures were built and compared in **Simulink**:
1.  Feedback Loop Configuration (Catena di retroazione)
2.  Direct Feedforward Configuration (Catena diretta)
3.  Combined Observer-Regulator Architecture (Osservatore-Regolatore)

---

## 📊 Performance & Stability Mapping (RAS)

### Linear vs. Non-Linear Step Responses
The system responses ($\theta, \psi$) were validated under step inputs and initial condition perturbations. The Observer-Regulator architecture demonstrated robust tracking performance, ensuring seamless convergence for the non-linear plant.

### Region of Asymptotic Stability (RAS) Estimation
Since non-linear systems only guarantee *local* asymptotic stability, a numerical iterative script was executed in MATLAB to estimate a conservative lower bound of the **Region of Asymptotic Stability (RAS)**.
*   **Methodology:** Evaluated via an iterative stochastic loop using $100$ hyper-ellipsoids ($N_e$) and $1000$ random boundary points ($N_p$) verified across specific Lyapunov level surfaces ($V(x) > 0, \dot{V}(x) < 0$).
*   The boundary limits confirm the robust perturbation margins ($[-0.1, 0.1]$ bounding box) inside which the controller successfully prevents vehicle low-side or high-side crash dynamics.

---

## 💻 Repository Structure
```text
mimo-vehicle-traction-control/
│
├── scripts/
│   ├── linearization_and_poles.m  # System matrices, stability check, pole placement
│   └── ras_estimation.m           # Stochastic RAS ellipsoid evaluation loop
│
├── models/
│   ├── linear_control_sim.slx     # Simulink model with linearized plant architectures
│   └── nonlinear_control_sim.slx  # Simulink model with interpreted non-linear plant
│
└── docs/
    └── traction_control_report.pdf # Complete control system presentation and plots
