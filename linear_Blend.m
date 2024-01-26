clear all; close all; clc;
n = input('no of cycle: ');
ti = input('time interval: ');
tii = [];
q = [];
tb = [];

for i = 1:n + 2
    tii(i) = (i - 1) * ti;
end

disp(tii)
q(1) = input('initial theta: ');
q(n + 2) = input('final theta: ');

for i = 2:n + 1
    q(i) = input('enter the via points: ');
end

figure;

for i = 1:n + 1
    V = 25;
    tb(i) = (q(i) - q(i + 1) + V * tii(i + 1)) / V;
    Vmin = (q(i + 1) - q(i)) / tii(i + 1);
    disp(Vmin)
    a(1) = q(i);
    a(2) = 0;
    a(3) = V / (2 * tb(i));
    b(1) = q(i + 1) - (V * tii(i + 1)^2) / (2 * tb(i));
    b(2) = V * tii(i + 1) / tb(i);
    b(3) = -V / (2 * tb(i));
    
    t = linspace(tii(i), tii(i + 1), 500);
    
    q_i = (a(1) + a(2) * t + a(3) * t.^2).*(t <= tb(i)) + ...
        ((q(i + 1) + q(i) - V * tii(i + 1)) / 2 + V * t).*((t > tb(i)) - (t >= (tii(i + 1) - tb(i)))) + ...
        (b(1) + b(2) * t + b(3) * t.^2).*(t > (tii(i + 1) - tb(i)));
    qdot = (a(2) + 2 * a(3) * t).*(t <= tb(i)) + ...
           V.*((t > tb(i)) - (t >= (tii(i + 1) - tb(i)))) + ...
           (b(2) + 2 * b(3) * t).*(t > (tii(i + 1) - tb(i)));
    qddot = 2 * a(3) * (t <= tb(i)) + ...
            0 * ((t > tb(i)) - (t >= (tii(i + 1) - tb(i)))) + ...
            2 * b(3) * (t > (tii(i + 1) - tb(i)));
    plot(t, q_i, 'b-', t, qdot, 'g--', t, qddot, 'r-.', 'LineWidth', 2);
    hold on;
end

legend('q','dq/dt','d^2q/dt^2');
xlabel('time (sec)'); ylabel('Joint Trajectory');
title('Trajectory using LSPB');
