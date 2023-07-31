clear all
syms qw q1 q2 dqw dq1 dq2 ddqw ddq1 ddq2 real
syms hb ht h1 h2 Ib It I1 I2 mb mt m1 m2 mball rw rt1 rt2 rt3 rt4 r1 r2 real
syms fw tau1 tau2 real

%% find the direct kinematics and Jacobian
q=[qw q1 q2]';
f = [qw-rt1+rt4+h1*cos(q1)+h2*cos(q1+q2);
     rw+hb+ht+h1*sin(q1)+h2*sin(q1+q2)];
J=jacobian(f,q)
rank(J)
simplify(J*J')
simplify(det(J*J'))

%% considering an example in two different unit measures
% [m/s]
h1=0.32; h2=0.34; rt1=0.062; ht=0.597; rt4=0.155; rw=0.0985; hb=0.193;
q1=pi/3; q2=pi/6;
dp=[-1 1]'; % [m/s]
dq=pinv(eval(J))*dp

% [cm/s]
h1=32; h2=34; rt1=6.2; ht=59.7; rt4=15.5; rw=9.85; hb=19.3;
q1=pi/3; q2=pi/6;
dp=[-100 100]'; % [cm/s]
dq=pinv(eval(J))*dp

%same values in different unit measures result in different solution using
%the pinv
%% find a proper weighting matrix
syms w real
syms hb ht h1 h2 Ib It I1 I2 mb mt m1 m2 mball rw rt1 rt2 rt3 rt4 r1 r2 real
W=diag([w 1 1]');
simplify(det(J*inv(W)*J'))
w=1/(h1*h2);

simplify(det(J*inv(eval(W))*J'))
J_pinv_w=inv(W)*J'*inv(J*inv(eval(W))*J')
%% reconsider previous example, now we expect same result considering weighted pseudoninverse
% [m/s]
h1=0.32; h2=0.34; rt1=0.062; ht=0.597; rt4=0.155; rw=0.0985; hb=0.193;
q1=pi/3; q2=pi/6;
dp=[-1 1]'; % [m/s]
w=eval(w);
dq=vpa(eval(J_pinv_w)*dp)

% [cm/s]
h1=32; h2=34; rt1=6.2; ht=59.7; rt4=15.5; rw=9.85; hb=19.3;
q1=pi/3; q2=pi/6;
dp=[-100 100]'; % [cm/s]
dq=vpa(eval(J_pinv_w)*dp)