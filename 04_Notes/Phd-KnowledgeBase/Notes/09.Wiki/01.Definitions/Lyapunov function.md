Lyapunov functions are a fundamental tool in stability analysis, particularly for nonlinear systems. Their application in analyzing the stability of Nonlinear Model Predictive Control (NMPC) systems is crucial. Here's how Lyapunov functions are typically used for this purpose:

### Basic Concept of Lyapunov Stability Analysis

1. **Lyapunov's Direct Method**:
   - This method involves finding a Lyapunov function, which is a scalar function $V(x)$ that provides a measure of the system's energy or distance from a stable point (like an equilibrium).
   - The key criterion is that $V(x)$ must be positive definite (i.e., $V(x) > 0$ for all $x \neq 0$ and $V(0) = 0$) and its time derivative $\dot{V}(x)$ along the system's trajectories must be negative definite or negative semi-definite.

2. **Stability Inference**:
   - If such a function can be found, it can be concluded that the system is stable. Specifically, if $\dot{V}(x) < 0$, the system is asymptotically stable.

### Application in Nonlinear MPC

1. **MPC Stability Criterion**:
   - In NMPC, the goal is to optimize a cost function subject to the system dynamics and various constraints. A key aspect of NMPC is ensuring that this optimization leads to a stable control system.
   - A Lyapunov-based approach to NMPC stability involves showing that the NMPC control law decreases a candidate Lyapunov function at each step.

2. **Designing Lyapunov Functions**:
   - The design of the Lyapunov function is crucial and can be challenging. It must be tailored to the specific dynamics of the system being controlled.
   - For NMPC, the Lyapunov function is often constructed based on the cost function used in the optimization, but it must also incorporate system dynamics.

3. **Lyapunov Function as a Constraint**:
   - In some NMPC formulations, the Lyapunov function's decrease is enforced as a constraint in the optimization problem. This approach ensures that the resulting control law improves stability.

4. **Predictive Stability Analysis**:
   - The Lyapunov function can be used predictively, analyzing how the function will evolve over the prediction horizon of the NMPC. This predictive stability analysis helps in ensuring long-term stability of the controlled system.

### Challenges and Practical Considerations

- **Function Construction**: Finding an appropriate Lyapunov function for complex nonlinear systems can be non-trivial.
- **Computational Complexity**: Incorporating Lyapunov stability analysis into NMPC can increase computational complexity, which is a critical factor in real-time control applications.
- **Global vs Local Stability**: In many cases, the Lyapunov function may only guarantee local stability. Ensuring global stability can be more challenging and requires careful design.

### Conclusion

Using Lyapunov functions in NMPC stability analysis provides a systematic and rigorous way to ensure that the control strategy will not lead to unstable behavior. It's a powerful approach, especially for complex nonlinear systems where intuitive stability assessments might not be feasible. However, the effectiveness of this approach hinges on the appropriate design and implementation of the Lyapunov function within the NMPC framework.


To provide a mathematical formulation of a Nonlinear Model Predictive Control (NMPC) system for trajectory planning in the context of a vehicle using a bicycle model, we'll first define the bicycle model and then formulate the NMPC problem.

### 1. Bicycle Model for Vehicle Dynamics

The bicycle model is a simplified representation of a vehicle's dynamics that amalgamates the two front wheels and the two rear wheels into single wheels (one at the front and one at the rear). This model is often used for lateral dynamics studies in vehicle control systems.

**State Variables:**
- $x, y$: Position of the vehicle (center of gravity) in the 2D plane.
- $\psi$: Heading angle of the vehicle.
- $v$: Velocity of the vehicle.
- $\delta$: Steering angle of the front wheel.

**Dynamics Equations:**
- $\dot{x} = v \cos(\psi)$
- $\dot{y} = v \sin(\psi)$
- $\dot{\psi} = \frac{v}{L} \tan(\delta)$
- $\dot{v} = a$

where $L$ is the wheelbase of the vehicle and $a$ is the linear acceleration, which along with $\delta$, serves as the control inputs.

### 2. NMPC Formulation for Trajectory Planning

**Objective:**
To plan a trajectory that follows a desired path while minimizing deviations and ensuring smooth control inputs.

**Optimization Problem:**
Minimize the following cost function over a prediction horizon $N$:

$J = \sum_{k=0}^{N-1} \left( w_1 (x_{k} - x_{k}^{ref})^2 + w_2 (y_{k} - y_{k}^{ref})^2 + w_3 (\psi_{k} - \psi_{k}^{ref})^2 + w_4 \delta_{k}^2 + w_5 a_{k}^2 \right)$

subject to the vehicle dynamics (bicycle model equations) and constraints:

- $( x_{k+1} = x_{k} + T v_{k} \cos(\psi_{k})$
- $( y_{k+1} = y_{k} + T v_{k} \sin(\psi_{k})$
- $\psi_{k+1} = \psi_{k} + T \frac{v_{k}}{L} \tan(\delta_{k})$
- $v_{k+1} = v_{k} + T a_{k}$


where $T$ is the sampling time, $w_i$ are the weighting factors for each term in the cost function, and $(x_{k}^{ref}, y_{k}^{ref}, \psi_{k}^{ref})$ are the reference (desired) states.

**Constraints:**
- Physical constraints on the vehicle, such as limits on $\delta$ and a.
- Road boundaries or other environmental constraints.
- Safety constraints like collision avoidance.

### Conclusion

This NMPC formulation aims to compute the optimal control inputs $\delta_{k}$ and $a_{k}$ at each time step to follow the desired trajectory while satisfying the vehicle dynamics and constraints. The optimization problem is typically solved at each time step using an appropriate nonlinear solver, and only the first set of control inputs is applied before the problem is solved again at the next timestep (receding horizon strategy).


Applying Lyapunov analysis to assess the stability of the NMPC-controlled system, especially for a vehicle following a trajectory based on the bicycle model, involves several steps. We need to construct a Lyapunov function and demonstrate that this function decreases over time under the NMPC control law. Here’s a conceptual framework for this analysis:

### 1. Constructing a Lyapunov Function

The Lyapunov function $V(x)$ should be a positive definite function of the system state $x$ (which includes position, orientation, and velocity in our case). A common choice for trajectory tracking problems is a quadratic function representing the deviation from the desired trajectory:

$V(x) = (x - x_{ref})^T P (x - x_{ref})$

where $x$ is the state vector $[x, y, \psi, v]^T$, $x_{ref}$ is the reference state, and $P$ is a positive definite matrix, often chosen based on the system's dynamics and the desired performance characteristics.

### 2. Demonstrating Decrease of the Lyapunov Function

For the system to be stable, the time derivative of the Lyapunov function $\dot{V}(x)$ should be negative definite or at least negative semi-definite. This derivative, given the system dynamics under the NMPC control law, is:

$\dot{V}(x) = \frac{\partial V}{\partial x} \cdot \dot{x}$

Substitute the system dynamics equations (from the bicycle model) into $\dot{x}$ and then calculate $\dot{V}(x)$. 

### 3. Stability Analysis

- **If $\dot{V}(x) < 0$ for all $x \neq x_{ref}$**: This implies that the system is asymptotically stable, and the trajectory will converge to the reference trajectory.
- **If $\dot{V}(x) \leq 0$**: This suggests that the system is stable in the sense of Lyapunov, meaning that the trajectory will not diverge from the reference, but convergence is not guaranteed.

### 4. Practical Considerations

- **Model Accuracy**: The accuracy of the bicycle model in representing the true vehicle dynamics is crucial for the validity of the Lyapunov analysis.
- **Controller Design**: The NMPC controller design, including the cost function and constraints, affects the system dynamics under control and hence the stability analysis.
- **Computational Aspects**: For real-time applications, like vehicle trajectory tracking, ensure that the Lyapunov function and its derivative can be computed efficiently.

### Conclusion

Lyapunov stability analysis for NMPC in the context of the bicycle model provides a theoretical assurance of stability. However, the practical effectiveness of this analysis depends on the accuracy of the model, the appropriateness of the Lyapunov function chosen, and the NMPC controller's design. In practice, this analysis often accompanies simulations and empirical testing to validate the stability of the controlled system.


Certainly! Let's consider an example for constructing a Lyapunov function $V(x)$ and the matrix $P$ in the context of a vehicle following a trajectory, controlled by NMPC, and described by the bicycle model. The Lyapunov function is designed to assess the stability of the system.

### Example of Lyapunov Function and Matrix $P$

Suppose the state vector $x$ for the bicycle model is given as $x = [x, y, \psi, v]^T$, where:
- $x, y$ are the vehicle's position coordinates.
- $\psi$ is the heading angle.
- $v$ is the velocity.

Let $x_{ref} = [x_{ref}, y_{ref}, \psi_{ref}, v_{ref}]^T$ be the reference state vector that the vehicle aims to track.

#### Lyapunov Function $V(x)$:
A common choice for the Lyapunov function in trajectory tracking problems is a quadratic form:

$V(x) = (x - x_{ref})^T P (x - x_{ref})$

#### Matrix $P$:
The matrix $P$ should be symmetric and positive definite. A simple example could be:

$P = \begin{bmatrix} p_1 & 0 & 0 & 0 \\ 0 & p_2 & 0 & 0 \\ 0 & 0 & p_3 & 0 \\ 0 & 0 & 0 & p_4 \end{bmatrix}$

where $p_1, p_2, p_3,$ and $p_4$ are positive constants. The choice of these constants depends on how much weight you want to give to each state variable's deviation from its reference value. For instance, if tracking the position ($x, y$) accurately is more critical than matching the velocity and heading angle, $p_1$ and $p_2$ might be chosen to be larger than $p_3$ and $p_4$.

#### Example Values:
Let's say we assign the following values:
- $p_1 = p_2 = 2$ (higher weight for position tracking)
- $p_3 = p_4 = 1$ (lower weight for heading and velocity)

Then, $P$ becomes:

$P = \begin{bmatrix} 2 & 0 & 0 & 0 \\ 0 & 2 & 0 & 0 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$

### Interpretation:
In this setup, the Lyapunov function $V(x)$ represents the weighted squared error between the current state of the vehicle and its desired state. The choice of $P$ reflects the importance of accurately following the trajectory in terms of position, heading, and velocity.

### Note:
- The actual selection of $P$ should be based on the specific requirements of your control problem and might require tuning through simulation and testing.
- The positive definiteness of $P$ is crucial for ensuring that $V(x)$ is a valid Lyapunov function, i.e., $V(x) > 0$ for all $x \neq x_{ref}$ and $V(x_{ref}) = 0$.