function f = cost_function(x)
% return cost function based on the reduction of control effort and damping
% terms

global h1 h2;
f=0;
R=eye(3);
L=eye(2);
P=diag([1/(h1*h2) 1 1]);
for k=1:9:size(x,1)-6
    v_i=x(k+3:k+5);
    u_i=x(k+6:k+8);
    f=f+u_i'*R*u_i+v_i'*P*v_i;
end
v_i=x(end-2:end);
f=f+v_i'*P*v_i;
end