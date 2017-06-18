
# Longitudinal trajectory planning with a continuous method

In this project I show how to implement a trajectory planner that solves problems such as the two depicted below.

![](img/scenarios.png)

In both scenarios the green car represents a self driving car. Its path is predefined, however its velocity decomposition is yet to be computed. In the [Math-Basics-Notebook](https://github.com/MarcosSobrinho/trajectory_planning/blob/master/math_basics.ipynb) I explain the mathematical background. It is a simplification of the method introduced in the [Trajectory Planning for BERTHA - a Local, Continuous Method](https://pdfs.semanticscholar.org/bdca/7fe83f8444bb4e75402a417053519758d36b.pdf) - paper. The paper is about a 2D trajectory planning, inlcuding longitudinal and lateral control. 

In the [Discretization-Notebook](https://github.com/MarcosSobrinho/trajectory_planning/blob/master/Discretization.ipynb) I transform the described problem into a numerically computable form. The testing.py contains an example and is ready to run. 
