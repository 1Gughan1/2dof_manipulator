close all;
clear all;
clc;

l1 = 200;
l2 = 200;
m1 = 1; 
m2 = 1;  
g = 9.81;


pos1 = [];
pos2 = [];

xvalue = [];
yvalue = [];

theta2 = [];
theta1 = [];

tt = [];

n = input('Enter the number of intermediate points you require: ');
tt = input('Enter the total time: ');
ti = [];
ct = 0;

while ct <= tt
    ti = [ti, ct];
    ct = ct + (tt / (n+1));
end

xvalue(1) = input('Enter the initial x-point: ');
yvalue(1) = input('Enter the initial y-point: ');

for i = 2:n+1
    xvalue(i) = input('Enter the intermediate x-point: ');
    yvalue(i) = input('Enter the intermediate y-point: ');
end

xvalue(n+2) = input('Enter the final x-point: ');
yvalue(n+2) = input('Enter the final y-point: ');


for i = 1:n+2
    theta2_i = -rad2deg(2 * atan2(sqrt((l1 + l2)^2 - (xvalue(i)^2 + yvalue(i)^2)), sqrt(xvalue(i)^2 + yvalue(i)^2 - (l1 - l2)^2)));
    theta1_i = rad2deg(atan2((-xvalue(i) .* l2 .* sin(deg2rad(theta2_i)) + yvalue(i) * (l1 + l2 * cos(deg2rad(theta2_i)))), (yvalue(i) * l2 * sin(deg2rad(theta2_i)) + xvalue(i) .* (l1 + l2 * cos(deg2rad(theta2_i))))));
    theta2 = [theta2, theta2_i];
    theta1 = [theta1, theta1_i];
end

torque1 = [];
torque2 = [];

for i = 1:n+1
    to = ti(i);
    q1 = theta1(i);  
    qdoto = 0;
    tf = ti(i+1);
    q2 = theta1(i+1);  
    qdotf = 0;
    A = [1, to, to^2, to^3; ...
         0, 1, 2*to, 3*to^2; ...
         1, tf, tf^2, tf^3; ...
         0, 1, 2*tf, 3*tf^2];
    b = [q1; qdoto; q2; qdotf];
    a = A \ b;
    t = linspace(ti(i), ti(i+1), 100);
    q1_interp = a(1) + a(2)*t + a(3)*t.^2 + a(4)*t.^3;
    pos1 = [pos1, q1_interp];

    too = ti(i);
    q11 = theta2(i);
    qdoto2 = 0;
    tff = ti(i+1);
    q22 = theta2(i+1);
    qdotf2 = 0;
    A2 = [1, too, too^2, too^3; ...
          0, 1, 2*too, 3*too^2; ...
          1, tff, tff^2, tff^3; ...
          0, 1, 2*tff, 3*tff^2];
    b2 = [q11; qdoto2; q22; qdotf2];
    a2 = A2 \ b2;
    t1 = linspace(ti(i), ti(i+1), 100);
    q2_interp = a2(1) + a2(2)*t1 + a2(3)*t1.^2 + a2(4)*t1.^3;
    pos2 = [pos2, q2_interp];

    q1dot = a(2) + 2*a(3)*t + 3*a(4)*t.^2;
    q1ddot = 2*a(3) + 6*a(4)*t;
    
    q1dotf = q1dot(end);

    tou1 = m2*l2^2*q1ddot + m2*l1*l2*cos(deg2rad(q2_interp)).*(2*q1ddot + q1ddot) + (m1 + m2)*l1^2*q1ddot - m2*l1*l2*sin(deg2rad(q2_interp)).*q1dot.^2 - 2*m2*l1*l2*sin(deg2rad(q2_interp)).*q1dot.*q1dotf + m2*l2*g*cos(deg2rad(q1_interp) + deg2rad(q2_interp)) + (m1 + m2)*l1*g*cos(deg2rad(q1_interp));
    torque1 = [torque1, tou1];

    q2dot = a2(2) + 2*a2(3)*t1 + 3*a2(4)*t1.^2;
    q2ddot = 2*a2(3) + 6*a2(4)*t1;
    tou2 = m2*l1*l2*cos(deg2rad(q2_interp)).*q2ddot + m2*l1*l2*sin(deg2rad(q2_interp)).*q2dot.^2 + m2*l2*g*cos(deg2rad(q1_interp) + deg2rad(q2_interp)) + m2*l2^2*(q2ddot + q2ddot);
    torque2 = [torque2, tou2];
end
time = linspace(0,tt,length(pos1));
aa = transpose(pos1);
bb = transpose(pos2);
cc = transpose(time);
d = [cc,aa];
e = [cc,bb];
figure(1);
subplot(2,2,1)
plot(linspace(0, tt, length(pos1)), pos1)
title('Theta1 Interpolation')

subplot(2,2,2)
plot(linspace(0, tt, length(pos2)), pos2)
title('Theta2 Interpolation')

subplot(2,2,3)
plot(linspace(0, tt, length(torque1)), torque1)
title('Torque 1')

subplot(2,2,4)
plot(linspace(0, tt, length(torque2)), torque2)
title('Torque 2')
