Each folder contains:
	dyn_model_complete.m derives the dynamic model of the robot. It generates 3 functions:
		get_balance_terms.m used to formulate the balance constraint
		get_dyn_terms.m used for the equations of motion in ct_dynamics.m
		get_gen_forces.m that computes the reaction forces used to find the ZMP
	optimization.m solves the various NLPs
	dt_dynamics.m discretizes the dynamics with 4th-order Runge-Kutta
	weighted_matrix.m used for checking the validity of velocity weighting matrix
	dyn_model_PRR.m contains the simplified dynamic model considering the MM as a PRR robot (not used)
