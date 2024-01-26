clear all; close all; clc;

p = [];
v = [];
a = [];
n = input('Enter no. of points you need between initial to final: ');
p(1) = input('Enter the initial theta value: ');
p(n+2) = input('Enter the final theta values: ');
tt = [];

for i = 2:n+1
    p(i) = input('Enter the via points theta values ');
end

v(1) = input('Enter the initial velocity value: ');
v(n+2) = input('Enter the final velocity values: ');
for i = 2:n+1
    v(i) = input('Enter the via point velocity: ');
end

combined_position = [];
combined_velocity = [];
combined_acceleration = [];

for i = 1:n+1
    qo = p(i);
    qf = p(i+1);

    ti = (i-1) * 5;
    tf = i * 5;
    
    vi = v(i);
    vf = v(i+1);
    t = linspace(ti, tf, 400);

    A = [1, ti, ti^2, ti^3, ti^4, ti^5;
         0, 1, 2*ti, 3*ti^2, 4*ti^3, 5*ti^4;
         0, 0, 2, 6*ti, 12*ti^2, 20*ti^3;
         1, tf, tf^2, tf^3, tf^4, tf^5;
         0, 1, 2*tf, 3*tf^2, 4*tf^3, 5*tf^4;
         0, 0, 2, 6*tf, 12*tf^2, 20*tf^3];

    b = [qo; vi; 0; qf; vf; 0];

    coefficients = A \ b;

    position = coefficients(1) + coefficients(2) * t + coefficients(3) * t.^2 + coefficients(4) * t.^3 + coefficients(5) * t.^4 + coefficients(6) * t.^5;
    velocity = coefficients(2) + 2 * coefficients(3) * t + 3 * coefficients(4) * t.^2 + 4 * coefficients(5) * t.^3 + 5 * coefficients(6) * t.^4;
    acceleration = 2 * coefficients(3) + 6 * coefficients(4) * t + 12 * coefficients(5) * t.^2 + 20 * coefficients(6) * t.^3;

    combined_position = [combined_position, position];
    combined_velocity = [combined_velocity, velocity];
    combined_acceleration = [combined_acceleration, acceleration];
    tt = [tt,t];
end
cp = transpose(combined_position);
ttt = transpose(tt);
data_matrix = [ttt ,cp];
data_matrix2 = [cp, ttt];
% Plotting
figure(1);
plot(linspace(0, (n+1)*5, length(combined_position)), rad2deg(combined_position));
title('Joint Position');
xlabel('Time (s)');
ylabel('Position (degrees)');

figure(2);
plot(linspace(0, (n+1)*5, length(combined_velocity)), rad2deg(combined_velocity));
title('Joint Velocity');
xlabel('Time (s)');
ylabel('Velocity (degrees/s)');

figure(3);
plot(linspace(0, (n+1)*5, length(combined_acceleration)), rad2deg(combined_acceleration));
title('Joint Acceleration');
xlabel('Time (s)');
ylabel('Acceleration (degrees/s^2)');
