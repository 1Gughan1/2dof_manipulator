clear all; close all; clc;

a = [];
n = input('Enter no. of points you need between initial to final: ');
a(1) = input('Enter the initial theta value: ');
a(n+2) = input('Enter the final theta values: ');

for i = 2:n+1
    a(i) = input('Enter the between values: ');
end

disp(a)

tt = [];

combined_position = [];
combined_velocity = [];
combined_acceleration = [];

for i = 1:n+1
    qo = a(i);
    qf = a(i+1);

    ti = (i-1) * 5;
    tf = i * 5;

    t = linspace(ti, tf, 1600);

    A = [1, ti, ti^2, ti^3, ti^4, ti^5;
         0, 1, 2*ti, 3*ti^2, 4*ti^3, 5*ti^4;
         0, 0, 2, 6*ti, 12*ti^2, 20*ti^3;
         1, tf, tf^2, tf^3, tf^4, tf^5;
         0, 1, 2*tf, 3*tf^2, 4*tf^3, 5*tf^4;
         0, 0, 2, 6*tf, 12*tf^2, 20*tf^3];

    b = [qo; 0; 0; qf; 0; 0];

    co = A \ b;

    position = co(1) + co(2) * t + co(3) * t.^2 + co(4) * t.^3 + co(5) * t.^4 + co(6) * t.^5;
    velocity = co(2) + 2 * co(3) * t + 3 * co(4) * t.^2 + 4 * co(5) * t.^3 + 5 * co(6) * t.^4;
    acceleration = 2 * co(3) + 6 * co(4) * t + 12 * co(5) * t.^2 + 20 * co(6) * t.^3;

    combined_position = [combined_position, position];
    combined_velocity = [combined_velocity, velocity];
    combined_acceleration = [combined_acceleration, acceleration];
    tt = [tt,t];
end

cp = transpose(combined_position);
ttt = transpose(tt);
data_matrix = [ttt ,cp];
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
