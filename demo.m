clc;
clear all;
close all;

numEndpoints = input('Enter the number of endpoints: ');

endpoints = zeros(numEndpoints, 2);

for i = 1:numEndpoints
    fprintf('Enter coordinates for endpoint %d:\n', i);
    endpoints(i, 1) = input('Enter x-coordinate: ');
    endpoints(i, 2) = input('Enter y-coordinate: ');
end

allIntermediatePointsX = [];
allIntermediatePointsY = [];

for i = 1:numEndpoints - 1
    x1 = endpoints(i, 1);
    y1 = endpoints(i, 2);
    x2 = endpoints(i + 1, 1);
    y2 = endpoints(i + 1, 2);
    
    numIntermediatePoints = 1000; 

   
    xIntermediate = linspace(x1, x2, numIntermediatePoints);

    yIntermediate = linspace(y1, y2, numIntermediatePoints);

    allIntermediatePointsX = [allIntermediatePointsX, xIntermediate];
    allIntermediatePointsY = [allIntermediatePointsY, yIntermediate];
end

l1 = 200;
l2 = 200;

pos1 = [];
pos2 = [];
theta2 = [];
theta1 = [];

tt = [];

tt = input('Enter the total time: ');
ti = [];
ct = 0;

while ct <= tt
    ti = [ti, ct];
    ct = ct + (tt / (numEndpoints+1));
end
for i = 1:length(xIntermediate)
    theta2_i = rad2deg(2 * atan2(sqrt((l1 + l2)^2 - (xIntermediate(i)^2 + yIntermediate(i)^2)), sqrt(xIntermediate(i)^2 + yIntermediate(i)^2 - (l1 - l2)^2)));
    theta1_i = rad2deg(atan2((-xIntermediate(i) .* l2 .* sin(deg2rad(theta2_i)) + yIntermediate(i) * (l1 + l2 * cos(deg2rad(theta2_i)))), (yIntermediate(i) * l2 * sin(deg2rad(theta2_i)) + xIntermediate(i) .* (l1 + l2 * cos(deg2rad(theta2_i))))));
    theta2 = [theta2, theta2_i];
    theta1 = [theta1, theta1_i];
end
for i = 1:numEndpoints+1
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
end


figure(1);
subplot(2,1,1)
plot(linspace(0, tt, length(pos1)), pos1)
title('Theta1 Interpolation')

subplot(2,1,2)
plot(linspace(0, tt, length(pos2)), pos2)
title('Theta2 Interpolation')


time = linspace(0,15,length(pos1));
aa = transpose(pos1);
bb = transpose(pos2);
cc = transpose(time);
d = [cc,aa];
e = [cc,bb];
