# Planning_Throwing_Motions_Mobile_Manipulator
The aim of this project is to plan a throwing motion for a planar mobile manipulator (MM), in an attempt to increase its workspace or reduce the time needed for a pick and place task. The motion planning problem is formulated as an Optimal Control Problem (OCP) and solved using numerical optimization via the Optimization Toolbox of MATLAB.
Robot balance is guaranteed along the whole planned trajectory using an appropriate nonlinear constraint based on the MM full dynamics, ensuring non-negative moments around the edges of the support polygon.

Project realized by me (gianba.gravina7@gmail.com) and Nunziante Luca (luca.nunziante1999@gmail.com)
