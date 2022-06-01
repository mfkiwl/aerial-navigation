Aerial Navigation
=================
Model-based control of quadrotor

Objectives
----------
1. Learn about quadrotor dynamics
2. Practice designing full-state feedback controllers

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
6. Perform analysis of the state-space representation
    - poles (all at 0), zeros, observability (no measurement model yet, so whatever we want it to be), controllability 
   (controllable), stabilizability, detectability
7. Design a plain full-state feedback controller for the system to stabilize the drone about the equilibrium position. 
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
