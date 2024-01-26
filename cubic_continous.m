clear all; close all; clc; 

to = 0; qo = 0; qdoto = 0;
tf = 10; qf = 30; qdotf = 0; 

A = [1,  to,  to^2, to^3;   ...
     0,  1,   2*to, 3*to^2; ...
     1,  tf,  tf^2, tf^3;   ...
     0,  1,   2*tf, 3*tf^2];

b = [qo; qdoto; qf; qdotf];
a = A\b;
t = to:(tf-to)/500:tf;
q     = a(1) + a(2)*t + a(3)*t.^2 + a(4)*t.^3;
qdot  = a(2) + 2*a(3)*t + 3*a(4)*t.^2;
qddot = 2*a(3) + 6*a(4)*t;

%second
qo = qf;
qdoto = qdotf;

tf2 = 20; qf2 = 0; qdotf2 = 0; 

A2 = [1,  tf,  tf^2, tf^3;   ...
      0,  1,   2*tf, 3*tf^2; ...
      1,  tf2,  tf2^2, tf2^3;   ...
      0,  1,   2*tf2, 3*tf2^2];

b2 = [qo; qdoto; qf2; qdotf2];

a2 = A2\b2;

t2 = tf:(tf2-tf)/500:tf2;
q2     = a2(1) + a2(2)*t2 + a2(3)*t2.^2 + a2(4)*t2.^3;
qdot2  = a2(2) + 2*a2(3)*t2 + 3*a2(4)*t2.^2;
qddot2 = 2*a2(3) + 6*a2(4)*t2;


%third
% qo = qf2;
% qdoto = qdotf2;
% tf3 = 8; qf3 = 30; qdotf3 = 3; 
% A3 = [1,  tf2,  tf2^2, tf2^3;   ...
%       0,  1,   2*tf2, 3*tf2^2; ...
%       1,  tf3,  tf3^2, tf3^3;   ...
%       0,  1,   2*tf3, 3*tf3^2];
% b3 = [qo; qdoto; qf3; qdotf3];
% a3 = A3\b3;
% t3 = tf2:(tf3-tf2)/500:tf3;
% q3     = a3(1) + a3(2)*t3 + a3(3)*t3.^2 + a3(4)*t3.^3;
% qdot3  = a3(2) + 2*a3(3)*t3 + 3*a3(4)*t3.^2;
% qddot3 = 2*a3(3) + 6*a3(4)*t3;
% %fourth
% qo = qf3;
% qdoto = qdotf3;
% tf4 = 10; qf4 = 40; qdotf4 = 0; 
% A4 = [1,  tf3,  tf3^2, tf3^3;   ...
%       0,  1,   2*tf3, 3*tf3^2; ...
%       1,  tf4,  tf4^2, tf4^3;   ...
%       0,  1,   2*tf4, 3*tf4^2];
% b4 = [qo; qdoto; qf4; qdotf4];
% a4 = A4\b4;
% t4 = tf3:(tf3-tf2)/500:tf4;
% q4     = a4(1) + a4(2)*t4 + a4(3)*t4.^2 + a4(4)*t4.^3;
% qdot4  = a4(2) + 2*a4(3)*t4 + 3*a4(4)*t4.^2;
% qddot4 = 2*a4(3) + 6*a4(4)*t4;
figure(1);
plot(t,q,'b-','LineWidth',2);
hold on;
plot(t2, q2, 'r-', 'LineWidth', 2);
hold off;
% plot(t3, q3, 'black-', 'LineWidth', 2);
% hold on;
% plot(t4, q4, 'g-', 'LineWidth', 2);
legend('Trajectory 1', 'Trajectory 2','Trajectory 3','Trajectory 4');
xlabel('Time (sec)'); ylabel('Joint Trajectory');
title('Trajectory using Cubic Polynomial - Position');
% 
figure(2);
plot(t, qdot, 'g--','LineWidth',2);
hold on;
plot(t2, qdot2, 'm--', 'LineWidth', 2);
hold off;
% plot(t3, qdot3, 'm--', 'LineWidth', 2);
% hold on;
% plot(t4, qdot4, 'm--', 'LineWidth', 2);
% legend('Velocity - Trajectory 1', 'Velocity - Trajectory 2');
% hold off;
% xlabel('Time (sec)'); ylabel('Joint Trajectory');
% title('Trajectory using Cubic Polynomial - Velocity');
% 
figure(3);
plot(t, qddot, 'r-.','LineWidth', 2);
hold on;
plot(t2, qddot2, 'c-.', 'LineWidth', 2);
hold off;
% plot(t3, qddot3, 'c-.', 'LineWidth', 2);
% hold on;
% plot(t4, qddot4, 'c-.', 'LineWidth', 2);
% legend('Acceleration - Trajectory 1', 'Acceleration - Trajectory 2');

xlabel('Time (sec)'); ylabel('Joint Trajectory');
title('Trajectory using Cubic Polynomial - Acceleration');
