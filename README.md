# Planning Throwing Motions for Mobile Manipulator
The aim of this project is to plan a throwing motion for a planar mobile manipulator (MM), in an attempt to increase its workspace or reduce the time needed for a pick and place task. The motion planning problem is formulated as an Optimal Control Problem (OCP) and solved using numerical optimization via the Optimization Toolbox of MATLAB.
Robot balance is guaranteed along the whole planned trajectory using an appropriate nonlinear constraint based on the MM full dynamics, ensuring non-negative moments around the edges of the support polygon.
## Proposed methods
When planning a throwing trajectory for a robot, the motion is divided into two main phases: the _throwing phase_ that takes place in the time interval $[t_{init},t_{rel}]$ and the _stopping phase_ in $(t_{rel},t_{final}]$. To solve the motion planning problem we take different approaches:
    * _Approach 0_ : solve an OCP to find a kinematically feasible throwing state $\textbf{x}_{rel}$. Then, we solve the two phases separately: one OCP gives an optimal trajectory from $\textbf{x}_{init}$ to $\textbf{x}_{rel}$, and another OCP from $\textbf{x}_{rel}$ to $\textbf{x}_{final}$.
    * _Approach 1_ : optimization of the throwing state is incorporated in the same OCP of the throwing phase trajectory, while the stopping phase is solved separately with a second OCP.
    * _Approach 2_ : the throwing phase, the throwing state and the stopping phase are incorporated in the same OCP.
The following video --- also in `media/video_approaches.mp4` file --- shows a comparison among the different approaches
![Approaches](media/video_approaches.mp4)

Then, considering only the _Approach 2_ additional simulations are carried out to show more aggressive and dynamic motions, to better appreciate the importance of the balance constraint. The simulations involve three different initial configurations for the Mobile Manipulator:
    * stretched down manipulator arm;
    * stretched forward manipulator arm;
    * stretched upward manipulator arm.
Moreover, the influence of a base motion penalization term in the Cost Function of the NLP is investigated.
The following video --- also in `media/video_aggressive_manoeuvres.mp4` file --- shows the obtained results
![Aggressive manoeuvres](media/video_aggressive_manoeuvres.mp4)

Further details are available in the report.

## Main source files
In the `source` folder it is possible to find three different subdirectory, one for each approach. The main files in each subdirectory are:
    * `dyn_model_complete.m` derives the dynamic model of the robot. It generates 3 functions:
        - `get_balance_terms.m` used to formulate the balance constraint
        - `get_dyn_terms.m` used for the equations of motion in `ct_dynamics.m`
        - `get_gen_forces.m` that computes the reaction forces used to find the ZMP
    * `optimization.m` solves the Non Linear Program
    * (`dt_dynamics.m`) discretizes the dynamics with 4th-order Runge-Kutta
    * (`weighted_matrix.m`) used for checking the validity of velocity weighting matrix
	* (`dyn_model_PRR.m`) contains the simplified dynamic model considering the MM as a PRR robot (not used)

A sketch of the mobile Manipulator is shown in `media/MM_sketch.PNG`.

Project realized by me (gianba.gravina7@gmail.com) and Nunziante Luca (luca.nunziante1999@gmail.com)
