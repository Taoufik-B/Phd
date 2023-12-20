

**Unified Decision-making Framework for Autonomous Intersection Navigation**

- [R] [[System Architecture Review]], Link research paper with the proposed architecture

## System Architecture

**1. Sensing Layer**
    - Sensors Array: Comprising LiDAR, radars, cameras, ultrasonic sensors, GPS, and IMU to capture a detailed understanding of the environment.
    - Data Preprocessing: Raw data from the sensors is filtered, calibrated, and processed to generate a unified data stream. Noise reduction algorithms are applied, especially focusing on probabilistic sensor models.
**2. State Estimation & Probabilistic Robotics Layer**
    - SLAM (Simultaneous Localization And Mapping): To generate and update the map of the environment in real-time while concurrently tracking the vehicle's position.
    - Probabilistic State Estimation: Leverage particle filters or Kalman filters to estimate the vehicle's state considering the uncertainties in observations.
**3. Decision-making Layer**
    - POMDP Solver: Given the probabilistic nature of the environment and the vehicle's current state estimate, the POMDP solver generates a policy that maximizes expected rewards over time, considering the uncertainties.
    - MPC Module: Based on the POMDP policy and current state, the MPC calculates the optimal control actions over a finite time horizon to follow the desired trajectory. It continuously re-evaluates these actions in real-time to adjust to the changing environment.
**4. Control Layer**
    - Actuation: Convert the control commands derived from the MPC to actual motor torques, steering angles, and brake forces.
    - Safety Protocols: Implement real-time monitoring of vehicle states to quickly identify hazardous situations. If a potential danger is detected, fail-safe mechanisms intervene, ensuring safety.
**5. Learning & Adaptation Layer**
    - Reinforcement Learning Agent: This agent learns from the decisions made and their outcomes, helping in fine-tuning the POMDP model and decision-making process over time.
    - Data Storage & Analysis: Store data from each trip for offline analysis and learning. This data can also be used to refine and retrain the decision-making models.
**6. Simulation & Validation**
    - Virtual Environment (e.g., CARLA): Before real-world deployment, test the framework in simulated multi-agent scenarios like urban intersections to ensure robustness and efficacy.
**7. Communication Layer**
    - Vehicle-to-Vehicle (V2V): Enables our autonomous vehicle to communicate with other vehicles, especially helpful in multi-agent scenarios like intersections.
    - Vehicle-to-Infrastructure (V2I): Get updates from traffic signals, sensors embedded in roads, or other infrastructure components for better-informed decisions.

**Link to [[Use Case and Implementation Steps]]**
