Aerial Navigation
=================
Model-based control of quadrotor

Objectives
----------
1. Learn about quadrotor dynamics
2. Practice designing full-state feedback controllers

Model Development
-----------------

Linearization
-------------

Plain State-Feedback Design (First Attempt)
-------------------------------------------
### State-Space Representation
Before designing a plain state-feedback controller, we need to represent our linearized model in state-space form.

This is done in the ```gen_qr_ss.m``` MATLAB script by simply filling out the ```A``` and ```B``` matrices.

```matlab
A =
 
[0, 0, 0, 0,                         0,                         0, 1, 0, 0, 0, 0, 0]
[0, 0, 0, 0,                         0,                         0, 0, 1, 0, 0, 0, 0]
[0, 0, 0, 0,                         0,                         0, 0, 0, 1, 0, 0, 0]
[0, 0, 0, 0,                         0,                         0, 0, 0, 0, 1, 0, 0]
[0, 0, 0, 0,                         0,                         0, 0, 0, 0, 0, 1, 0]
[0, 0, 0, 0,                         0,                         0, 0, 0, 0, 0, 0, 1]
[0, 0, 0, 0, (4*g*(M/4 + m))/(M + 4*m),                         0, 0, 0, 0, 0, 0, 0]
[0, 0, 0, 0,                         0, (4*g*(M/4 + m))/(M + 4*m), 0, 0, 0, 0, 0, 0]
[0, 0, 0, 0,                         0,                         0, 0, 0, 0, 0, 0, 0]
[0, 0, 0, 0,                         0,                         0, 0, 0, 0, 0, 0, 0]
[0, 0, 0, 0,                         0,                         0, 0, 0, 0, 0, 0, 0]
[0, 0, 0, 0,                         0,                         0, 0, 0, 0, 0, 0, 0]
```

```matlab
B = 

[                                0,                                  0,                                 0,                                  0]
[                                0,                                  0,                                 0,                                  0]
[                                0,                                  0,                                 0,                                  0]
[                                0,                                  0,                                 0,                                  0]
[                                0,                                  0,                                 0,                                  0]
[                                0,                                  0,                                 0,                                  0]
[                                0,                                  0,                                 0,                                  0]
[                                0,                                  0,                                 0,                                  0]
[(2*(g*(M/4 + m))^(1/2))/(M + 4*m), -(2*(g*(M/4 + m))^(1/2))/(M + 4*m), (2*(g*(M/4 + m))^(1/2))/(M + 4*m), -(2*(g*(M/4 + m))^(1/2))/(M + 4*m)]
[              (l*m)/(L*(M + 4*m)),                (l*m)/(L*(M + 4*m)),               (l*m)/(L*(M + 4*m)),                (l*m)/(L*(M + 4*m))]
[        (2*(g*(M/4 + m))^(1/2))/m,                                  0,        -(2*(g*(M/4 + m))^(1/2))/m,                                  0]
[                                0,          (2*(g*(M/4 + m))^(1/2))/m,                                 0,         -(2*(g*(M/4 + m))^(1/2))/m]
```

### Open-Loop Poles and Controllability
Now that the model is in state-space form, we can determine the poles and controllability of 
the system. The poles of the system are simply the eigenvalues of the ```A``` matrix.

```matlab
>> eig(A)

ans =

     0
     0
     0
     0
     0
     0
     0
     0
     0
     0
     0
     0
```

Oddly enough, for this equilibrium, all the poles are at the origin, making the system unstable. 

Note that since we haven't made any determinations about the ```C``` or ```D``` matrices yet, since we don't have a 
measurement model, we can't make any determinations about system zeros.

To determine whether the system is stabilizable and consequently controllable (since all the poles are unstable), we 
need to perform a controllability test.  

One way of doing this is to compute the rank of the controllability matrix. If the rank is equal to the number of state 
variables, then the system is controllable. The number of state variables of this system is ```12```.

```matlab
>> Mc = simplify([B A*B A*A*B A*A*A*B A*A*A*A*B A*A*A*A*A*B A*A*A*A*A*A*B A*A*A*A*A*A*A*B A*A*A*A*A*A*A*A*B A*A*A*A*A*A*A*A*A*B A*A*A*A*A*A*A*A*A*A*B A*A*A*A*A*A*A*A*A*A*A*B A*A*A*A*A*A*A*A*A*A*A*A*B]);
rank(double(simplify(subs(Mc, [M m L l g], [1 1 0.2 0.15 9.81]))))

ans =

    12
```

Therefore, the system is controllable and therefore stabilizable. So we can arbitrarily place all the poles of the 
system anywhere in the s-plane. I acknowledge that this is pretty hacky. But it's good for now.


### Controller Design with ```place(...)``` Command
For purposes of simplicity, I have chosen to place all the poles roughly around ```s=-1```. Because the system is 
controllable, we may arbitrarily place the poles of the system and placing the poles at ```s=-1``` stabilizes the system
and shouldn't produce super oscillatory responses (since no imag part).

```matlab
K = place(A,B,[-1 -1.1 -1.2 -1.3 -1.4 -1.5 -1.6 -1.7 -1.8 -1.9 -1.91 -1.92])
```
- note the ```place(...)``` function in MATLAB doesn't allow the algebraic multiplicity of any pole to exceed 
```rank(B)```. So calls to the place function often look like this if one is seeking to place all the poles roughly near 
the other poles.

```matlab
>> eig(A - B * K)

ans =

  -1.000000000000465
  -1.100000000000321
  -1.199999999999832
  -1.299999999998955
  -1.399999999999417
  -1.499999999997619
  -1.600000000002091
  -1.700000000006318
  -1.899999999999082
  -1.919999999999211
  -1.909999999998876
  -1.799999999998659
```
- Clearly, the eigenvalues of the closed-loop system have been placed roughly where we want them.

The controller brings the quadrotor back to the (0,0,0,0,0,...,0) position regardless of the starting configuration if 
we are using the linear model. 
- despite the (x,y,z) not showing up in the linearization, somehow, we are always brought back to (x=0,y=0,z=0).
- this warrants further investigation to see how we might change the controller to move the quadrotor to any arbitrary 
position

Despite working for the linear system, the plain state feedback controller breaks down almost immediately for any minor 
perturbation in the nonlinear system case.
- the reasoning is unclear. Though, based on the few simulations I have run, I would say that the roll-pitch 
dynamics are involved somehow. My hypothesis is that small perturbations of the roll and pitch state variables in the 
non-linear system deviate too much from what we expect in the linear case.
- Potential Fix #1: One simple fix might be to try an LQR approach for controller design. See how the nominal controller
behaves. Then, increase the cost on the state variables having to do with the roll and pitch. 
- Potential Fix #2: A more complicated fix to this problem might be to perform something like gain scheduling or just
parameter variation controller. I'm not sure how the analysis or the design of such a controller would work, so I'd have
to play around and read around a little to find out. Essentially, we linearize the system about another equilibrium 
about a small perturbation in the roll and the pitch. Then, we design another stabilizing controller and switch between 
those controllers depending on our current state. Perhaps we really just interpolate between those controller gains to 
avoid something of a switch statement in the feedback loop.

#### Bugs + Features
- Feature: plotting the control input... how can we do that?
- Feature: exporting the animation and running the animation in real-time regardless of the simulation length.

### Controller Design with ```lqr(...)``` Command
LQR controller design is pretty straightforward, especially with MATLAB. Essentially, rather than choosing where to
place the poles directly, we place the poles in such a way as to minimize a scalar cost index (cost function) by finding
the unique symmetric PSD solution ```P``` to the so-called "algebraic Riccati equation" (ARE). 

This transforms the decision from choosing the pole position to choosing how tightly we want to control different state 
variables and how much control effort we want to expend by choosing the ```Q``` and ```R``` matrices in the third and
fourth argument of the ```lqr(...)``` command in MATLAB, respectively.

Increasing the ```ith``` diagonal element of ```Q``` will increase the weight corresponding to the ```ith``` state
variable in the cost function of not bringing that variable back to the setpoint. Essentially we want to tighten the 
feedback loops corresponding to the control of that state variable.

Increasing the ```ith``` diagonal element of ```R``` will increase the weight corresponding to the ```ith``` control
variable in the cost function. Essentially we want to make using that actuator more expensive.

```matlab
>> K = lqr(A, B, eye(12,12), eye(4,4));
>> eig(A - B * K)

ans =

  -1.0001 + 0.0000i
  -2.1593 + 2.2690i
  -2.1593 - 2.2690i
  -9.9031 + 0.0000i
  -9.9031 + 0.0000i
  -2.1593 + 2.2690i
  -2.1593 - 2.2690i
  -2.5830 + 0.0000i
  -1.0001 + 0.0000i
  -1.0846 + 0.0000i
  -0.4153 + 0.3571i
  -0.4153 - 0.3571i
```
- Some notes: interestingly, the controller gains of the LQR controller are much smaller than those for the pole 
placement controller. There are many more zero entries in the controller as well. All of the closed-loop system poles 
are in the OLHP.

The controller still works for the linear system, and now the controller works for various small perturbations in the 
state variables for the non-linear system as well. This probably has to do with improving the robustness of the 
closed-loop system. The LQR controller guarantees a 60 degree phase-margin, for example.

Plain State-Feedback with Integral Action Design (Second Attempt)
-----------------------------------------------------------------
We are currently capable of bringing the system back to the equilibrium position. This is good if we want to reject 
disturbances and noise, but insufficient if we want to have the quadrotor follow some trajectory. 

I *think* one way of incorporating that functionality would be to augment the current state-feedback controller with 
integral action. Then we can apply references to the system and see if the controller will bring the system to those 
references. It's also possible that my choice of equilibrium is wrong or doesn't entirely capture what we want, but 
let's start with this.



Todos
-----
1. [DONE] Flesh out the animation and plotting capabilities of the non-linear simulation
    - acceptance criterion: show a GIF of any simulation of the quadrotor.
    - acceptance criterion: show the plots of each state variable of the quadrotor
2. [DONE] Flesh out the linear model of the quadrotor for nominal level and stable equilibrium
    - acceptance criterion: matches the dynamics of the non-linear simulation at the equilibrium
    - acceptance criterion: nearly matches the dynamics of the non-linear simulation near the equilibrium
3. Flesh out how the linear model changes with different (x,y,z) equilibria.
    - question: how does the linear model change if the equilibrium position is changed from (0,0,0) to (10,0,0) or 
   (-10, 10, 0) or (0, 0, 10)?
    - hypothesis: because x,y,z don't come up in the dynamics at all, the equilibrium position has no effect on the 
   linear model dynamics
4. Flesh out how the linear model changes with different (alpha, beta, gamma) equilibria.
    - question: how does the linear model change if the equilibrium position is changed from (0, 0, 0) to (90, 0, 0)
    - hypothesis: the attitude dynamics never change, the z-direction dynamics don't change w.r.t. yaw but do w.r.t. 
   everything else. the x and y direction dynamics change with everything in a complex way.
5. [DONE] Convert the linear model into state-space representation for the (x,y,z,al,be,ga)->(X,X,X,0,0,0)
    - how do we really do this?
6. [DONE] Perform analysis of the state-space representation
    - poles (all at 0), zeros, observability (no measurement model yet, so whatever we want it to be), controllability 
   (controllable), stabilizability, detectability
7. [DONE] Design a plain full-state feedback controller for the system to stabilize the drone about the equilibrium position. 
See how a change of reference from (x,y,z):(0,0,0)->(0, 0, 1) behaves... Circle back on the analysis after this

Medium-term goals: have a single LQR controller that stabilizes the drones and can move the drone to any arbitrary 
position... See what kinds of problems there are with this kind of technique... see what can be improved...

Longer-term goals: how many state-feedback controllers do we really need to get good performance? can we design 
different feedback controllers for different configurations of the drone or for different modes of operation? How do we
switch between those modes of operation. What is an H-infinity controller? what are its benefits and detractions? How 
can we design one for the system? What are the affects of such a contorller on the system? Observer + full-state
feedback controller design? sensor models? simulating sensor noise and disturbances? Simulating faults? following a 
trajectory. switching between controllers? voronoi diagrams? sequential loop closure around LQR/Hinfinity controllers 
with MPC outer loop for devising paths? static obstacle avoidance. dynamic obstacle avoidance. Completing missions... 
What kinds of things do we want to do and why? How can we measure performance?
