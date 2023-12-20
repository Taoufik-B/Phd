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


**Reference Link :** [Learning Model Predictive Controllers with Real-Time Attention for Real-World Navigation](Learning%20Model%20Predictive%20Controllers%20with%20Real-Time%20Attention%20for%20Real)

  
  

Calude.ai summary:

Introduction & Motivation:

  

Discusses challenges of real-world robot navigation in complex spaces and around humans. Limitations of classical vs learning-based methods.

Approach: Performer-MPC

  

Proposes a learnable MPC using Performers to embed visual context into cost function. Retains MPC's advantages while learning from demonstrations.

Formulates parametric MPC with learned residual cost based on Performer embeddings. Keeps real-time performance with linear attention.

Uses bilevel optimization and implicit diff to train cost function params on expert demonstrations. Handles distribution shift.

Experiments:

  

Tested on wheeled robot with LiDAR/depth sensors in cluttered spaces, blind corners, and pedestrian interactions.

Compared to MPC and explicit policy baselines. Evaluates goal reaching, navigation metrics, and social scores.

Results show Performer-MPC learns behaviors that improve over baselines, especially in social metrics and generalization.

Limitations & Future Work:

  

Currently only spatial attention, separate training by scenario. Suggests additions like temporal modeling, multimodal inputs.

User studies limited in scale and protocol. Plans for expanded studies and refined evaluation.

  
  

Cluttered env remains unsolved problem,

Unwritten social norms around human

Classical approach can already move robots from point A to point B safely and reliably

  

Enable adaptive behaviour, in contrast to classical approach, machine learning help to learn these behaviours directly from data.

Then research focused on global and local planner, also other navigation components including cost representation, kinodynamic models and planner parameter to enable:

- Better navigation performance
- Off road
- Social navigation

MPC enables synthesis of real time feedback controller for robots operating in real world environments that satisfy given safety constraints optimality criteria and kinodynamic model

  

Therefore they design a learnable MPC that could be considered as class of implicit Behavior cloning policies

  
  

Introduction

The research paper is about control policy for a differential wheeled robot. Robot moving in a cluttered environment in respect with human social norms is still an unsolved problem. The objective of the paper is to introduce a learnable Model Predictive Controller that make use of expert demonstration in order to optimize the cost function. Also to compare the performance of the approach against two main studies, RMPC (Regular MPC) and Explicit Policy (EP)

  

Methodology

The paper leverages the outstanding results shown in Transformers architecture, These results are subject of many advances in generative AI, such as Large MultiModals (ChatGPT as an example), image and sound generation… Although the Transformers present a challenge in embedding the solution as an implicit controller, as the output is a quadratic space and time complexity of the input. Therefore, they adopt the same Transformer architecture that has been used in EP (Explicit Policy) called Performers, a low rank implicit Transformer with linear space and time complexity, then additionally combined with MPC. The cost function of the MPC is learnable through expert demonstration based on behaviour cloning.

The main input to the solution is an occupancy grid, a result of sensors fusion of 3DLidar, Camera and depth sensors.

  
  
38. - [x] ==Write a concise introduction: Start the resume by briefly introducing the research topic, the problem being addressed, and the objectives of the study.==
39. - [x] ==Summarize the methodology: Provide a brief overview of the methods and techniques used in the research, including the study design, data collection methods, and any statistical analyses.==
40. - [x] ==Present the main results: Summarize the most significant findings and outcomes of the research. Include key numerical results and statistical significance, if applicable.==
41. - [ ] ==Highlight the discussion points: Mention any important interpretations, implications, or limitations discussed by the authors in the original article.==
42. - [ ] ==Conclude the resume: Finish the resume with a brief concluding statement that highlights the overall significance of the research and its potential impact.==

  
  
  
  
  
  
  
  

Abstract:

The research is about improving implicit controller policies by combining imitation learning and robust handling of system control by Model Predictive Controller.

The approach is called Performer-MPC using a learnable cost function parametrized by vision context embeddings provided by ==Performers==- a low rank implicit-attention Transformer.

Results as claimed shows 40% improvement in reaching goal compared to standard MPC in cluttered environment and 65% better on social metrics when navigating around human.

  

Introduction