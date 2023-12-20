|   |
|---|
|Chapter 1 of Introduction to Probability by Joseph K. Blitzstein and Jessica Hwang|

  

Strategies to avoid potential pitfalls in probability:

- Simulation : Study problems through simulation
- Biohazards: Studying common mistakes is important for gaining stronger understanding of what is and is not valid reasoning in probability.
- Sanity checks: After solving a problem one way, try to love the same problem in different manner to examine the results.

  

Definition _(Sample space and event)_

The sample space S of an experiment is the set of all possible outcomes of the experiment. An event A is a subset of the sample space S, and we say that A occurred if the actual outcome is in A.

  
![Exported image](Exported%20image%2020231118114140-0.png)  
  

Definition _(Naïve definition of probability)_

Let A be an event for an experiment with a finite sample space S. The naïve probability of A is:

![Exported image](Exported%20image%2020231118114140-1.png)  

Theorem _(Multiplication rule)_

Consider a compound experiment consisting of two sub-experiments, Experiment A and Experiment B. Suppose that experiment A has a possible outcomes, and for each of those outcomes Experiment B has b possible outcomes. Then the compound experiment has ab possible outcomes

  
![Exported image](Exported%20image%2020231118114140-2.png)  

Theorem _(Sampling with replacement)_

Consider n objects and making k choices from them, one at a time with replacement. Then there are n^k possible outcomes.

  

Theorem _(Sampling without replacement)_

Consider n objects and making k choices from them, one at a time without replacement. Then there are n(n-1)…(n-k+1) possible outcomes, for k <=n and 0 otherwise.

  

The birthday problem:

There are k people in a room. Assume each person's birthday is equally likely to be any of the 365 days of the year, and that people's birthdays are independent. What is the probability that two or more people in the group have the same birthday.