---
keywords: flight Profiles
---

----
### Motion planning and discrete graph search

Graph search algorithm are very useful for planning motion and trajectories for autonomous vehicles

But robot do not move on a graph ?
- Discretize (spatial and temporal)
- Use graph search as a component in a continuous planner, for example in so called lattice planners [[Lecture 4 - Motion Planning with Diff Constraints]]

Labs : hand-in 1, hand-in 2

Scope of the lecture
1. Formalization of a planning problem as a graph search problem
2. Main algorithms for graph search
	1. Djikstra's algorithm
	2. A*
3. Properties of heuristics in A* to ensure optimality and efficiency
4. Introduction to any-time planning using A* -- ARA*