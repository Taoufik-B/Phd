---
Reference ID: A unique identifier for each entry.
Title: Title of the paper/article/book
Authors: Authors of the work.
Year: Year of publication.
Source: Journal name, conference, or publisher.
Keywords:
  - Keywords relevant to your research.
  - this is another row
Methodology: The methodology used in the paper (experimental, review, simulation, etc.).
Summary: Your brief summary of the paper.
Link/DOI: Direct link or DOI for quick access.
Cited By: How many times it has been cited (you can get this from Google Scholar, for instance). It helps in identifying influential papers.
Relevance: High/Medium/Low
tags:
  - "#research/paper"
  - research/communication
File Path/Link: If you have a digital copy, link to its location on your drive for quick access.
Date Reviewed: 
Status: Open/In Progress/Done
---


## Reflections/Notes

**Key Objectives/Questions:**
Example: To explore the challenges and opportunities of implementing POMDPs in real-world multi-agent driving scenarios.

**Main Findings/Results:**
1. POMDPs present challenges in multi-agent environments due to X, Y, Z.
2. Proposed solution A improved computational efficiency by 20%.  

**Methodologies Used:**
Example: The authors used a hybrid approach combining traditional POMDPs with a new algorithm for faster computation.

**Relevance to My Research:**
Example: This paper's approach to improving computational efficiency in POMDPs can be beneficial for my own work on real-time decision making in autonomous vehicles.

**Key Quotations:**
Example: "Our findings suggest that while traditional POMDPs struggle in complex environments, hybrid solutions present a promising direction for future research."

**Personal Reflections/Thoughts:**
Example: While the paper presented a novel approach, there are still some questions about the scalability of their proposed solution. It would be interesting to experiment with their algorithm in a more diverse set of scenarios.
  

## Actionable Next Steps:

Task list:
1. [ ] Replicate the hybrid algorithm discussed in the paper.
2. [ ] Compare its efficiency with the current model I'm working on.

**Reference Link :** [Trajectory based lateral control: A Reinforcement Learning case study](Trajectory%20based%20lateral%20control%20A%20Reinforcement%20Learning%20case%20study.md)

  
  

**TODOs:**

- [ ] Prepare the environment for in loop simulation

  
  
  
![Exported image](Exported%20image%2020231118114033-0.png)  

  
![Exported image](Exported%20image%2020231118114033-1.png)3. State space
4. Action space
5. Proposed reward function:
![Exported image](Exported%20image%2020231118114033-2.png)7. Termination state

  
  

Challenges:

- Define the parameters:
    
    - ThetaMax
    - Dmax
    - Vmax
    - Yaw acceleration threshold
    - The constant C of the reward function

  
  

Application note:

- [ ] **What is the state space**

Represents the quantified observation from the environment, in this case it is waypoints of the given trajectory

- [ ] **What is the action space**

- Action 1: Break [0, 1]
- Action 2: Throttle [0, 1]
- Action 3: Steer [-1.22, 1.22]

Iterative reward function definition

- [ ] V1:

- [ ] **Termination state**

Define a longitudinal and lateral distance to terminate the learning

Dmax and Thetamax