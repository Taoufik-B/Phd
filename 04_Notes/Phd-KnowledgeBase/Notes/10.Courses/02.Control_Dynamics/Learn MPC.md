MPC, model predictive controller, becomes widely used in automotive industry. It features MIMO Multiple inputs and Multiple Outputs and knows for goal optimization every single time step.

### About MPC
• Can be generally applied to nonlinear MIMO systems.
• Natural consideration of both states and control constraints.
• Approximately optimal control.
• Used in industrial applications since the mid of 1970’s.
• Requires online optimization

**Central Issues related to MPC**
• When does MPC stabilize the system?,
• How good is the performance of the MPC feedback law?,
• How long does the optimization horizon N need to be?,
• How to Implement it numerically? (The main scope of this TALK!).


### Workshop CasADI
Link:: [[Learning CasADi#MPC and MHE Implementation in Matlab using CasADi]]
**Model Predictive Control(MPC)** aka Receding/Moving Horizon Control

-- Example of single input, single output

$$x(k+1)=f(x(k),u(k))$$
![[Pasted image 20231126082237.png]]

**MPC Mathematical Formulation**
#mpc/running_cost #mpc/cost_function #mpc/ocp #mpc/value_function
![[Pasted image 20231126082509.png]]

### MPC Implementation to Mobile Robots control

![[Pasted image 20231126084139.png]]

![[Pasted image 20231126084250.png]]
![[Pasted image 20231126084345.png]]
![[Pasted image 20231126084624.png]]

![[Pasted image 20231126085049.png]]
![[Pasted image 20231126085202.png]]
![[Pasted image 20231126085254.png]]

The objective is to transfer or to migrate from Optimal control problem to Nonlinear problem.
### Several shooting Methods exist:
- [[Learn MPC.Methods#1. Single Shooting]]
- [[Learn MPC.Methods#2. Multiple Shooting]]
- [[Learn MPC.Methods#3. Other Methods in MPC]]
[[Learn MPC.Methods#Recommendations]]
