

- [R] Reflextion on Learning to control to review
from : [Introduction Vehicle Systems](file:///home/taoufik/projects/learning/tsfs12/Lecture_notes/lecture_01_introduction.pdf)


### Youtube video link:

![](https://www.youtube.com/watch?v=IEZFwh8sw8s)


-- Useful links
- https://www.classcentral.com/classroom/youtube-2019-adsi-summer-workshop-algorithmic-foundations-of-learning-and-control-csaba-szepesvar-136289/6408199f8322a
- https://l4dc.web.ox.ac.uk/


Key point:
What is ML?
- using past **data** to **learn** about and/or **act** upon the world
![[Pasted image 20231119110202.png]]
What is Control?
- using **feedback** to **mitigate** the effects of **dynamic uncertainty**.
![[Pasted image 20231119110302.png]]


How to get the best of both?

In practice ...
Control: are PID
ML: nonparametric supervised prediction

How do we merge ML and control?
![[Pasted image 20231119110544.png]]

Optimal control:
![[Pasted image 20231119110851.png]]

Let's take the case when dynamics are unknown.

==Challenge== : How to perform optimal control when the system is unknown.
-> How well we must understand a system in order to it

### RL Methods
![[Pasted image 20231119111226.png]]

- Model-Based RL
![[Pasted image 20231119111527.png]]
Motivating example: LQR (Linear Quadratic Regulator) -> most classic optimal control problem
![[Pasted image 20231119111657.png]]

Simplest Example: LQR

- The matrices A and B are unknown
- We can generate trajectories of length with arbitrary input sequence u
![[Pasted image 20231119111828.png]]
The naive idea 
- Obvious strategy: certainty equivalent control
![[Pasted image 20231119112508.png]]


ADP : Approximate dynamic programming
LSTD : Least Square Temporal Difference Method

![[Pasted image 20231119112932.png]]

Direct Policy Search
![[Pasted image 20231119113113.png]]
System Identification
Sample exploitation 
tend to be more efficient as a policy gradient
![[Pasted image 20231119113133.png]]

![[Pasted image 20231119113625.png]]

![[Pasted image 20231119113732.png]]
Considering unknown dynamics
![[Pasted image 20231119114627.png]]
Either ignore the estimation errors
or
Include them to solve the robust LQR
![[Pasted image 20231119114729.png]]![[Pasted image 20231119115031.png]]
CE is asymptotically more efficient that Robust LQR

![[Pasted image 20231119115134.png]]
![[Pasted image 20231119115244.png]]
![[Pasted image 20231119115515.png]]

![[Pasted image 20231119115542.png]]
![[Pasted image 20231119115721.png]]
![[Pasted image 20231119115951.png]]

![[Pasted image 20231119194447.png]]

![[Pasted image 20231119120612.png]]

![[Pasted image 20231119120703.png]]
