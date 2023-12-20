### What is Mathematical Optimization?

==Mathematical optimization is the process of maximizing or minimizing an objective function by finding the best available values across a set of inputs. Some variation of optimization is required for all== ==deep learning== ==models to function, whether using supervised or== ==unsupervised learning====. There are many specific techniques to choose from, but all optimization algorithms require a minimum of:==

==An objective function f(x), to define the output that’s being maximized or minimized. This function can be deterministic (with specific effects) or stochastic (achieving a== ==probability== ==threshold).==

==Inputs that are controllable, in the form of variables like x1, x2, etc… Each can be either discrete or continuous.==

==Constraints to place limits on how large or small variables can be. Equality constraints are usually noted hn (x) and inequality constraints are noted gn (x). Equations without constraints are known as “unlimited” optimization problems.==

  
> From <[https://deepai.org/machine-learning-glossary-and-terms/mathematical-optimization](https://deepai.org/machine-learning-glossary-and-terms/mathematical-optimization)>  
  
  
  
  
  

- Stochastic optimization
- Robust Optimization

Robust Optimization

Robust optimization is a more recent approach to uncertainty representation in mathematical programming than the stochastic optimization [13].

  
> From <[https://www.sciencedirect.com/topics/earth-and-planetary-sciences/mathematical-programming](https://www.sciencedirect.com/topics/earth-and-planetary-sciences/mathematical-programming)>  
  
  
  

Resources:

- [https://web.stanford.edu/group/sisl/k12/optimization/#!index.md](https://web.stanford.edu/group/sisl/k12/optimization/#!index.md)

  
> From <[https://www.sciencedirect.com/topics/earth-and-planetary-sciences/mathematical-programming](https://www.sciencedirect.com/topics/earth-and-planetary-sciences/mathematical-programming)>



### Why Optimization?
Optimization algorithms are used in many applications from diverse areas.

- Business: Allocation of resources in logistics, investment, etc...
- Science: Estimation and fitting of models to measurement data, design of experiments
- Engineering: Design and operation of technical systems, e.g. bridges, cars, aircraft, digital devices, etc.

### What characterize an Optimization Problem?
An optimization problem consists of the following three ingredients.

- An objective function, $\Phi(\pmb{w})$, that shall be minimized or maximized.
- decision variables, $\pmb{w}$, that can be chosen, and
- constraints that shall be respected, e.g. of the form $\pmb{g}_1(\pmb{w})=0$ (equality constraints) or  $\pmb{g}_2(\pmb{w})\geq0$ 


### Mathematical Formulation in Standard Form
**Nonlinear Programming Problem (NLP):** A standard problem formulation in numerical optimization.

$$\min_{\pmb{w}} \Phi(\pmb{w}) \; \text{ Objective function}$$
$$\text{s.t } \pmb{g}_1(\pmb{w})\leq0  \; \text{ Inequality constraints}$$
$$\pmb{g}_2(\pmb{w})=0  \; \text{ Equality constraints}$$
$\Phi(.)$, $\pmb{g}_1(.)$ and $\pmb{g}_2(.)$ are usually assumed to be differentiable.

**Special cases of NLP include:**
- **Linear Programming (LP)** (when $\Phi(.)$, $\pmb{g}_1(.)$ and $\pmb{g}_2(.)$ are  ==affine==, i.e. these functions can be expressed as linear combination of the elements of $\pmb{w}$ ),
- **Quadratic Programming (QP)** (when $\pmb{g}_1(.)$ and $\pmb{g}_2(.)$ are  ==affine==, but the objective  $\Phi(.)$ is a ==linear-quadratic== function),

**Min or Max**
For a given objective function, maximization can be treated as a minimization of the negative objective.

**Local Vs. Global minimum**
![[Pasted image 20231125230511.png]]

**Solution of the optimization problem**
Normally we are looking at the value of $\pmb{w}$ that minimizes our objective $\pmb{w}^*={\arg \min}_{\pmb{w}}\Phi(\pmb{w})$
By direct substitution we can get the corresponding value of the objective 
$\Phi(\pmb{w}^*):=\Phi(\pmb{w})|_{\pmb{w}^*}$
$\quad\qquad:=\min_{\pmb{w}}\Phi(\pmb{w})$


**Ipopt** ([[Interior Point Optimizer]])* is an open source software package for large-scale nonlinear optimization. It can be used to solve general nonlinear programming problems (NLPs)


> [!Note]- Steps in Casadi
> Example of fitting data
> $y=m.x+c$ 
> The objective function to minimize is:
> $$\min_{m,c} \sum_{i=1}^{n_{data}}(y(i)-(m.x(i)+c))²$$
> 1. Define decision variables
> ```python
> import casadi.*
> # Decision variables
> m = SX.sym('m') #Decision variable (slope)
> c = SX.sym('c') #Decision variable (y-intersection)
> ```
> 2. Define the objective function
> ```python
> obj = 0
> for i in len(x):
> 	obj = obj + (y[i] - (m*x[i]+c))**2
> ```
> 3. Declare optimization constraints and problem parameters
> ```python
> g = [] #Optimization constraints – empty (unconstrained)
> P = [] Optimization problem parameters – empty (no parameters used here)
> ```
> 4. Declare the optimization variables
> ```python
> OPT_variables = [m,c]
> ```
> 5. Define the solver
> ```python
> nlp_prob = {'f':obj, 'x':OPT_variables, 'g':g, 'p':P}
> opts = {
> 	'ipopt': # interior point optimizer
> 	{
> 		'max_iter':100,
> 		'print_level':0,
> 		'acceptable_tol':1e-0,
> 		acceptable_obj_change_tol':1e-6
> 	},
> 	'print_time':0,
> }
solver = nlpsol('solver','ipopt', nlp_prob, opts)
> ```
> 6. Solve with args
> ```python
> args = {
> 	'lbx' : -inf,
> 	'ubx' : inf,
> 	'lbg' : -inf,
> 	'ubg' : inf,
> 	'p' : [],
> 	'x0' : -0.5 # initialization of the optimization variable
> }
> sol = solver(**args)
> x_sol = sol['x']
> min_value = sol['f']
> ```





![[animation1612211110.9228647.gif]]
#### Footnote tests
This is a simple footnote[^3]. 
This is another simple note[^2]
This is the 3 note [^note]

[^1]: This is the referenced text. 
[^2]: Add 2 spaces at the start of each new line. This lets you write footnotes that span multiple lines. 
[^note]: Named footnotes still appear as numbers, but can make it easier to identify and link references.
[^3]: trying for 3
