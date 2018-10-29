%% Clear
clear;
close all;
clc;

%% Data retrieval
M = csvread('data.csv',1);

% field names
f_dev_time = 1;
f_A = 2:4;
f_G = 5:7;
f_temp = 8;
f_press = 9;
f_alt_press = 10;
f_alt_max = 11;
f_count = 12;
f_state = 13;

% retrieve fields
dev_time = M(:,f_dev_time);
A = M(:,f_A);
G = M(:,f_G);
temp = M(:,f_temp);
press = M(:,f_press);
alt_press = M(:,f_alt_press);
alt_max = M(:,f_alt_max);
count = M(:,f_count);
state = uint8(M(:,f_state));

clear M

% post-processing of altitude
alt_offset = mean(alt_press(1:50));
alt_press = alt_press-alt_offset;
alt_max = alt_max-alt_offset;

% post-processing of pressure
min_current = .004;     % A
max_current = .020;     % A
min_gauge = 0;          % bar
max_gauge = 100;        % bar
atm_press = 1.01325;    % bar

press = (min_current/mean(press(1:20)))*press;
gauge = (press-min_current)*(max_gauge-min_gauge)/(max_current-min_current)+min_gauge;
press = gauge+atm_press;

%% Plot
index_min = find(state>0,1) - 10;

figure(1);
% subplot(3,1,1);
    [ax,h1,h2]=plotyy(dev_time(index_min:end), press(index_min:end), ...
        dev_time(index_min:end), state(index_min:end), 'plot'); grid on;
    set(ax(1),'ylim',[0,13]);
    title('pressure');
    
figure(2);
subplot(2,1,1);
    plotyy(dev_time(index_min:end), temp(index_min:end), ...
        dev_time(index_min:end), state(index_min:end), 'plot'); grid on;
subplot(2,1,2);
    plotyy(dev_time, temp, ...
        dev_time, state, 'plot'); grid on;
    title('temperature');
    
figure(3);
% subplot(3,1,3);
    plot(dev_time(index_min:end), alt_press(index_min:end)); hold on;
    plot(dev_time(index_min:end), alt_max(index_min:end)); grid on;
    title('altitude');
