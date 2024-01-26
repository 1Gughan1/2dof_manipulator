
clear all; close all; clc; % clear memory, figures, and window

to = 0; qo = 0;  qdoto = 0;
% final time, joint value and joint velocity
tf = 3; qf = 75; qdotf = 0; 
% Coefficient matrix for cubic trajectory and its derivative
% at initial and final joint values.
A = [1,  to,  to^2, to^3;   ...
     0,  1,   2*to, 3*to^2; ...
     1,  tf,  tf^2, tf^3;   ...
     0,  1,   2*tf, 3*tf^2];
% Vector of intial and final joint positions and velocities
b = [qo; qdoto; qf; qdotf];
% Compute coefficients of trajectory polynomial using
% notion of a = inv(A)*b, but using Gaussian Elimination
a = A\b;
% Evaluate cubic polynomial at times
t = to:(tf-to)/1000:tf;
q     = a(1) + a(2)*t + a(3)*t.^2 + a(4)*t.^3;
qdot  = a(2) + 2*a(3)*t + 3*a(4)*t.^2;
qddot = 2*a(3) + 6*a(4)*t;
qt = transpose(q);
tt = transpose(t);
data_matrix = [tt ,qt];
% Plot trajectories
figure(1);
plot(t,q,'b-','LineWidth',2);
legend('q');
xlabel('time (sec)'); ylabel('Joint Trajectory');
title('Trajectory using Cubic Polynomial - position');
figure(2);
plot(t, qdot, 'g--','LineWidth',2);
legend('dq/dt');
xlabel('time (sec)'); ylabel('Joint Trajectory');
title('Trajectory using Cubic Polynomial - velocity');
figure(3);
plot(t, qddot, 'r-.','LineWidth', 2);
legend('d^2q/dt^2');
xlabel('time (sec)'); ylabel('Joint Trajectory');
title('Trajectory using Cubic Polynomial - acceleration');