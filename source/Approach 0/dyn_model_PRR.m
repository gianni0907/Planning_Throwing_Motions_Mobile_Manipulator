clear variables 
clc

syms qw q1 q2 dqw dq1 dq2 ddqw ddq1 ddq2 real
syms fw tau1 tau2 real

%% kinematics and dynamics parameters
hb=0.193; ht=0.797; h1=0.32; h2=0.59; r1=0.06; r2=0.06; d1=h1/2; d2=h2/2; 
rt1=0.062; rt2=0.159; rt3=0.02435; rt4=0.155;
rw=0.0985; db=0.54; g0=9.81;
mb=30; mt=27; m1=6.15; m2=3.96; mball=3;
%with w we refer to base + vertical fixed link bodies (b+t)
mw=mb+mt;
I1=(1/12)*m1*(3*r1^2+h1^2);
I2=(1/12)*m2*(3*r2^2+h2^2);

%for now we dont consider the inertia Ib and It being in the hp of prismatic
%joint
%% Kinetic energy computation
%position of the center of mass of b+t
pcw=(1/mw)*[qw*mb + (qw-rt1+rt3)*mt;
           rw*mb + (rw+hb+rt2)*mt];

vcw = simplify(diff(pcw,qw)*dqw);

Tw = 0.5*mw*vcw'*vcw;
%equal to T1 = (1/2)*mw*dqw'*dqw since prismatic joint

pc1 = [qw-rt1+rt4+d1*cos(q1);
       rw+hb+ht+d1*sin(q1)];

vc1 = simplify(diff(pc1,qw)*dqw+diff(pc1,q1)*dq1);

T1a = 0.5*I1*dq1^2;
T1l = 0.5*m1*vc1'*vc1;
T1 = simplify(T1a + T1l);

%compute the new parameters of link 2 
%influenced by the ball payload
m2p = m2 + mball;
r2p = (m2*d2 + mball*h2)/m2p;
I2p = I2 + m2*(r2p-d2)^2 + mball*(h2-r2p)^2;

pc2p = [qw-rt1+rt4+h1*cos(q1)+r2p*cos(q1+q2);
        rw+hb+ht+h1*sin(q1)+r2p*sin(q1+q2)];

vc2p = simplify(diff(pc2p,qw)*dqw+diff(pc2p,q1)*dq1+diff(pc2p,q2)*dq2);

T2a = 0.5*I2p*(dq1+dq2)^2;
T2l = 0.5*m2p*vc2p'*vc2p;
T2 = simplify(T2l + T2a);

T = simplify(Tw+T1+T2);
T = collect(T,dqw^2);
T = collect(T,dq1^2);
T = collect(T,dq2^2);

%% Inertia matrix

M(1,1)=diff(T,dqw,2);
M(2,2)=diff(T,dq1,2);
M(3,3)=diff(T,dq2,2);

M(1,2)=diff(diff(T,dqw),dq1);
M(2,1)=M(1,2);

M(1,3)=diff(diff(T,dqw),dq2);
M(3,1)=M(1,3);

M(2,3)=diff(diff(T,dq1),dq2);
M(3,2)=M(2,3);
 
M=simplify(M);

%% Centrifugal and Coriolis terms
q=[qw; q1; q2];

M1=M(:,1);
C1=0.5*(jacobian(M1,q)+jacobian(M1,q)'-diff(M,qw));

M2=M(:,2);
C2=0.5*(jacobian(M2,q)+jacobian(M2,q)'-diff(M,q1));

M3=M(:,3);
C3=0.5*(jacobian(M3,q)+jacobian(M3,q)'-diff(M,q2));
 
dq=[dqw; dq1; dq2];
c1=dq'*C1*dq;
c2=dq'*C2*dq;
c3=dq'*C3*dq;
c=[c1;c2;c3];

c=simplify(c);

%% Gravitational terms
g=[0 -g0]';

U1=-mw*g'*pcw;
U2=-m1*g'*pc1;
U3=-m2p*g'*pc2p;
U=simplify(U1+U2+U3);

gradU=simplify(jacobian(U,q)');

ddq=[ddqw; ddq1; ddq2];
tau=[fw; tau1; tau2];

%% Dynamics model
dyn_mod = M*ddq+c+gradU==tau;