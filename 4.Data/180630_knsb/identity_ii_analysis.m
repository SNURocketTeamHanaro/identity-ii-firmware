%% Clear
clear;
close all;
clc;

%% Retrieve data
raw = csvread('raw.csv',1,0)';
t = raw(1,:);

% Retreive only meaningful data
idx = [0 1];
for i = t
    if (i > 130000)
        idx = idx+[0 1];
    else
        idx = idx+[1 1];
    end
    if (i > 160000)
        break;
    end
end

A = raw(2:4,:);     % Accelerometer
G = raw(5:7,:);     % Gyroscope
M = raw(8:10,:);    % Magnetometer
alt_p = raw(11,:);  % Pressure altimeter
alt_M = raw(12,:);  % Maximum altitude
state = raw(14,:);  % Rocket state

%% Evaluate the trajectory
a_abs = sqrt(A(1,:).*A(1,:)+A(2,:).*A(2,:)+A(3,:).*A(3,:));
figure(1); plot(t(idx(1):idx(2)),a_abs(idx(1):idx(2))); grid on;
hold on; plot(t(idx(1):idx(2)),state(idx(1):idx(2)));
line([143113 143113], [0 18], 'Color', 'red', 'Linewidth', 1);