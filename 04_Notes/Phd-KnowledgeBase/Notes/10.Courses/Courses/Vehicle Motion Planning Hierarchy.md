![Exported image](Exported%20image%2020231118114137-0.png)  

Path and trajectory

  

Motion primitives

  

Classification of motion planning algorithm

- Goal oriented motion planning algorithms
- Progress oriented motion planning algorithms

  

Motion planning classes:

1. Variational methods: known as curve fitting, the main idea to express the path or the trajectory as a parametrized curve.
2. Incremental exploration methods: construct a reachability graph based on motion primitives. Based on random sampling and guided by heuristic function.
3. Graph search methods: discretize the search graph into a grid of cells that are associated with the nodes of a graph. Two node of a graph are connected if it exists a collision free motion primitive between the corresponding nodes.