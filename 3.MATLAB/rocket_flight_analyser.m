%% Rocket flight Analyser
%
% Refine data and retrieve trajectory and rotation in time domain of the
% rocket system after recovery from the flight mission.
%
% Author: Jaerin Lee
%   SNU Rocket Team HANARO
%   School of Engineering
%   Seoul National University
%
% Last Modified: 10/13/2018

% Clear code
clear;
close all;
clc;

%% Mode of operation
mode_plot = true;
mode_rot_anim = false;
mode_save_data = false;

state_dp = 6;
state_mp = 7;

%% Data retrieval
M = csvread('data.csv',1);

% Field names
f_dev_time = 1;
f_A = 2:4;
f_G = 5:7;
% f_temp = 8;
% f_press = 9;
f_alt_press = 8;
f_alt_max = 9;
f_state = 10;

% Retrieve fields
dev_time = M(:,f_dev_time);
A = M(:,f_A);
G = M(:,f_G);
% temp = M(:,f_temp);
% press = M(:,f_press);
alt_press = M(:,f_alt_press);
alt_max = M(:,f_alt_max);
state = uint8(M(:,f_state));

n = length(dev_time);
clear f_dev_time f_A f_G f_alt_press f_alt_max f_state
clear M

%% Pre-process
ignition_start = find(state>0,1);
calib_ref = 1:ignition_start-100; % throw out 100 samples before ignition

%--- Specify tilt angle of the rocket mount ---%
tilt = 84 * pi/180;

Aoff = mean(A(calib_ref,:));
Aoff_ref = [cos(tilt) 0 sin(tilt)];
A = A-repmat(Aoff-Aoff_ref,[size(A,1) 1]);
Goff = mean(G(calib_ref,:));
G = G-repmat(Goff,[size(G,1) 1]);

alt_offset = mean(alt_press(calib_ref));
alt_press = alt_press-alt_offset;
alt_max = alt_max-alt_offset;

% Pre-processing of pressure
% min_current = .004;     % A
% max_current = .020;     % A
% min_gauge = 0;          % bar
% max_gauge = 100;        % bar
% atm_press = 1.01325;    % bar

% press = (min_current/mean(press(1:20)))*press;
% gauge = (press-min_current)*(max_gauge-min_gauge)/(max_current-min_current)+min_gauge;
% press = gauge+atm_press;

%% Post-process
% Post-process of device time
dev_time = (dev_time-dev_time(1))/1000;

% Initialize state variable
% g
g = [0;0;-1];
g0 = 9.80665;
% Sensor to rocket body
C = [0 0 -1; 0 1 0; 1 0 0];
% Inertial frame to Earth frame
E = [0 0 -1; 0 1 0; -1 0 0];
% Eulerian angle and its variation
Phi = zeros(n, 1);      Theta = zeros(n, 1);    Psi = zeros(n, 1);
dPhi = zeros(n-1, 1);   dTheta = zeros(n-1, 1); dPsi = zeros(n-1, 1);
% Quaternion and its variation
a = zeros(n, 1);    b = zeros(n, 1);    c = zeros(n, 1);    d = zeros(n, 1);
da = zeros(n-1, 1); db = zeros(n-1, 1); dc = zeros(n-1, 1); dd = zeros(n-1, 1);
% Inertial frame
ri = zeros(n, 3); vi = zeros(n, 3); ai = zeros(n-1, 3);
% Direction cosine matrix from body frame to inertial frame
C_Q = zeros(3,3,n-1);

% Initial state
Phi(1)  = 0;        Theta(1)  = -pi/2+tilt; Psi(1)  = 0;
EPhi(1) = Phi(1);   ETheta(1) = Theta(1);   EPsi(1) = Psi(1);
a(1) = cos(Phi(1)/2)*cos(Theta(1)/2)*cos(Psi(1)/2) + sin(Phi(1)/2)*sin(Theta(1)/2)*sin(Psi(1)/2);
b(1) = sin(Phi(1)/2)*cos(Theta(1)/2)*cos(Psi(1)/2) + cos(Phi(1)/2)*sin(Theta(1)/2)*sin(Psi(1)/2);
c(1) = cos(Phi(1)/2)*sin(Theta(1)/2)*cos(Psi(1)/2) + sin(Phi(1)/2)*cos(Theta(1)/2)*sin(Psi(1)/2);
d(1) = cos(Phi(1)/2)*cos(Theta(1)/2)*sin(Psi(1)/2) + sin(Phi(1)/2)*sin(Theta(1)/2)*cos(Psi(1)/2);

% Acceleration body frame
ab = (C*A')';

% Angular velocity body frame
w = (C*G')';
wx = w(:,1)*pi/180;
wy = w(:,2)*pi/180;
wz = w(:,3)*pi/180;

% Headings update
for i = 1:n-1
	dt = dev_time(i+1)-dev_time(i);

	% Update quaternion from angular velocity
    da(i) = -0.5*(b(i)*wx(i) + c(i)*wy(i) + d(i)*wz(i));
    db(i) =  0.5*(a(i)*wx(i) - d(i)*wy(i) + c(i)*wz(i));
    dc(i) =  0.5*(d(i)*wx(i) + a(i)*wy(i) - b(i)*wz(i));
    dd(i) = -0.5*(c(i)*wx(i) - b(i)*wy(i) - a(i)*wz(i));
    
    a(i+1) = a(i) + dt*da(i);
    b(i+1) = b(i) + dt*db(i);
    c(i+1) = c(i) + dt*dc(i);
    d(i+1) = d(i) + dt*dd(i);
    
    % Update direction cosine matrix
    C_Q(:,:,i) = [(a(i+1)^2+b(i+1)^2-c(i+1)^2-d(i+1)^2) 2*(b(i+1)*c(i+1)-a(i+1)*d(i+1))         2*(b(i+1)*d(i+1)+a(i+1)*c(i+1))
                  2*(b(i+1)*c(i+1)+a(i+1)*d(i+1))       (a(i+1)^2-b(i+1)^2+c(i+1)^2-d(i+1)^2)   2*(c(i+1)*d(i+1)-a(i+1)*b(i+1))
                  2*(b(i+1)*d(i+1)-a(i+1)*c(i+1))       2*(c(i+1)*d(i+1)+a(i+1)*b(i+1))         (a(i+1)^2-b(i+1)^2-c(i+1)^2+d(i+1)^2)];
              
    % Update Euler angle from angular velocity
    dPhi(i)     = (wy(i)*sin(Phi(i)) + wz(i)*cos(Phi(i)))*tan(Theta(i)) + wx(i);
    dTheta(i)   =  wy(i)*cos(Phi(i)) - wz(i)*sin(Phi(i));
    dPsi(i)     = (wy(i)*sin(Phi(i)) + wz(i)*cos(Phi(i)))/cos(Theta(i));
    
    EPhi(i+1)   = EPhi(i)   + dPhi(i)*dt;
    ETheta(i+1) = ETheta(i) + dTheta(i)*dt;
    EPsi(i+1)   = EPsi(i)   + dPsi(i)*dt;
    
    Phi(i+1)    = atan2(C_Q(3,2,i), C_Q(3,3,i));
    Theta(i+1)  = asin(-C_Q(3,1,i));
    Psi(i+1)    = atan2(C_Q(2,1,i), C_Q(1,1,i));
    
    % Update acceleration from inertial frame
    ai(i,:) = (E*C_Q(:,:,i)*ab(i,:)'+g)*g0;
end

% Update velocity and position
for i = 1:n-1
	dt = dev_time(i+1)-dev_time(i);

    vi(i+1,1) = vi(i,1) + dt*ai(i,1);
    vi(i+1,2) = vi(i,2) + dt*ai(i,2);
    vi(i+1,3) = vi(i,3) + dt*ai(i,3);
    
    ri(i+1,1) = ri(i,1) + dt*vi(i,1);
    ri(i+1,2) = ri(i,2) + dt*vi(i,2);
    ri(i+1,3) = ri(i,3) + dt*vi(i,3);
end

%% Plot
if mode_plot
index_min = ignition_start-100;
index_apo = find(state==state_dp,1);
index_para = find(state==state_mp,1);

index_max = index_apo+150;

% figure;
% subplot(3,1,1);
%     [ax,h1,h2]=plotyy(dev_time(index_min:end), press(index_min:end), ...
%         dev_time(index_min:end), state(index_min:end), 'plot'); grid on;
%     set(ax(1),'ylim',[0,13]);
%     title('pressure');
    
% figure;
% subplot(2,1,1);
%     plotyy(dev_time(index_min:end), temp(index_min:end), ...
%         dev_time(index_min:end), state(index_min:end), 'plot'); grid on;
% subplot(2,1,2);
%     plotyy(dev_time, temp, ...
%         dev_time, state, 'plot'); grid on;
%     title('temperature');

figure;
    plot(dev_time(index_min:end), alt_press(index_min:end)); hold on;
    plot(dev_time(index_min:end), alt_max(index_min:end));
    grid on; grid minor;
    title('altitude'); hold off;

figure;
subplot(3,1,1);
    plot(dev_time(index_min:index_max), ab(index_min:index_max,1)*g0); hold on;
    grid on; grid minor;
    xlim([dev_time(index_min) dev_time(index_max)]);
    title('a_{body}[x]'); hold off;
subplot(3,1,2);
    plot(dev_time(index_min:index_max), ab(index_min:index_max,2)*g0); hold on;
    grid on; grid minor;
    xlim([dev_time(index_min) dev_time(index_max)]);
    title('a_{body}[y]'); hold off;
subplot(3,1,3);
    plot(dev_time(index_min:index_max), ab(index_min:index_max,3)*g0); hold on;
    grid on; grid minor;
    xlim([dev_time(index_min) dev_time(index_max)]);
    title('a_{body}[z]'); hold off;

figure;
subplot(3,1,1);
    plot(dev_time(index_min:index_max), ai(index_min:index_max,1)); hold on;
    grid on; grid minor;
    xlim([dev_time(index_min) dev_time(index_max)]);
    title('a_{iner}[x]'); hold off;
subplot(3,1,2)
    plot(dev_time(index_min:index_max), ai(index_min:index_max,2)); hold on;
    grid on; grid minor;
    xlim([dev_time(index_min) dev_time(index_max)]);
    title('a_{iner}[y]'); hold off;
subplot(3,1,3)
    plot(dev_time(index_min:index_max), ai(index_min:index_max,3)); hold on;
    grid on; grid minor;
    xlim([dev_time(index_min) dev_time(index_max)]);
    title('a_{iner}[z]'); hold off;

figure;
subplot(3,1,1);
    plot(dev_time(index_min:index_max), vi(index_min:index_max,1)); hold on;
    grid on; grid minor;
    xlim([dev_time(index_min) dev_time(index_max)]);
    title('v_{iner}[x]'); hold off;
subplot(3,1,2);
    plot(dev_time(index_min:index_max), vi(index_min:index_max,2)); hold on;
    grid on; grid minor;
    xlim([dev_time(index_min) dev_time(index_max)]);
    title('v_{iner}[y]'); hold off;
subplot(3,1,3);
    plot(dev_time(index_min:index_max), vi(index_min:index_max,3)); hold on;
    grid on; grid minor;
    xlim([dev_time(index_min) dev_time(index_max)]);
    title('v_{iner}[z]'); hold off;

figure;
subplot(3,1,1);
    plot(dev_time(index_min:index_max), ri(index_min:index_max,1)); hold on;
    grid on; grid minor;
    xlim([dev_time(index_min) dev_time(index_max)]);
    title('r_{iner}[x]'); hold off;
subplot(3,1,2);
    plot(dev_time(index_min:index_max), ri(index_min:index_max,2)); hold on;
    grid on; grid minor;
    xlim([dev_time(index_min) dev_time(index_max)]);
    title('r_{iner}[y]'); hold off;
subplot(3,1,3);
    plot(dev_time(index_min:index_max), ri(index_min:index_max,3)); hold on;
    plot(dev_time(index_min:index_max), alt_press(index_min:index_max));
    grid on; grid minor;
    xlim([dev_time(index_min) dev_time(index_max)]);
    legend('Accelerometer', 'Altimeter');
    title('r_{iner}[z]'); hold off;

[apogee_val, apogee_idx] = max(ri(index_min:index_max,3));
apogee_idx = apogee_idx + index_min;
apogee = ri(apogee_idx,:);
para = ri(index_para,:);
figure;
    plot3(ri(index_min:index_max,1), ri(index_min:index_max,2), ri(index_min:index_max,3)); hold on;
    grid on; grid minor;
    zlim([0 apogee_val+20]);
    plot3(apogee(1), apogee(2), apogee(3),'r*','MarkerSize',10);
    text(apogee(1), apogee(2), apogee(3), ...
        [{'APOGEE'}, strcat({'@('},num2str(apogee(1)),{', '},num2str(apogee(2)),{', '},num2str(apogee(3)),{')'})], ...
        'HorizontalAlignment','left','FontSize',10,'FontName','FixedWidth');
    %plot3(para(1), para(2), para(3),'b*','MarkerSize',10);
    %text(para(1), para(2), para(3), ...
    %    [{'MAIN PARACHUTE ON'}, strcat({'@('},num2str(para(1)),{', '},num2str(para(2)),{', '},num2str(para(3)),{')'})], ...
    %    'HorizontalAlignment','left','FontSize',10,'FontName','FixedWidth');
    xlabel('X'); ylabel('Y'); zlabel('Z');
    daspect([1 1 1]);
    title('trajectory'); hold off;

ababs = sqrt(ab(:,1).^2+ab(:,2).^2+ab(:,3).^2)*g0;
figure;
    plot(dev_time, ababs*4.5); hold on;
    plot([dev_time(index_apo), dev_time(index_apo)], ylim, 'r');
    plot([dev_time(index_para), dev_time(index_para)], ylim, 'r');
    grid on; grid minor;
    xlabel('time(s)'); ylabel('net force on the body(N)');
    title('Net Force on the Rocket Body');
end

near_apo = index_apo-10:index_apo+300;
near_para = index_para-10:index_para+300;
figure;
subplot(2,1,1);
    plot(dev_time(near_apo), ababs(near_apo)); hold on;
    scatter(dev_time(near_apo), ababs(near_apo));
    plot([dev_time(index_apo), dev_time(index_apo)], ylim, 'r');
    grid on; grid minor;
    xlabel('time(s)'); ylabel('acceleration(m/s^2)');
    title('near apo');
subplot(2,1,2);
    plot(dev_time(near_para), ababs(near_para)); hold on;
    scatter(dev_time(near_para), ababs(near_para));
    plot([dev_time(index_para), dev_time(index_para)], ylim, 'r');
    grid on; grid minor;
    xlabel('time(s)'); ylabel('acceleration(m/s^2)');
    title('near para');

%% Rotation animation
if mode_rot_anim
figure('pos', [0 0 1400 1000]);
XYZE = [1 0 0; 0 1 0; 0 0 1];
time_offset = dev_time(index_min);
for i=index_min:index_max
    psi = Psi(i);
    phi = Phi(i);
    theta = Theta(i);

    xyzb =  -E*C_Q(:,:,i)*XYZE;
    
    xb = xyzb(:,1);
    yb = xyzb(:,2);
    zb = xyzb(:,3);
    origin = [0; 0; 0];

    quiver3(0,0,0, xb(1),xb(2),xb(3), 'r'); hold on;
    quiver3(0,0,0, yb(1),yb(2),yb(3), 'g');
    quiver3(0,0,0, zb(1),zb(2),zb(3), 'b');
    
    text_pos = [strcat({'TIME : '},num2str(dev_time(i+1)-time_offset),{'s'}), ...
        strcat({'ALT  : '},num2str(alt_press(i+1)),{'m'})];
    switch state(i)
        case 0
            text_state = {'READY ...'};
        case 1
            text_state = {'DROGUE OFF', 'MAIN   OFF', 'IN FLIGHT'};
        case 2
            text_state = {'DROGUE ON', 'MAIN   OFF'};
        case 3
            text_state = {'DROGUE ON', 'MAIN   ON'};
        otherwise
            text_state = {'DROGUE OFF', 'MAIN   OFF'};
    end     
    text(0,0,-1,[text_pos, text_state], ...
        'HorizontalAlignment','left','FontSize',12,'FontName','FixedWidth');
    axis([-1 1 -1 1 -1 1])
    legend('X(Roll)', 'Y(Pitch)', 'Z(Yaw)')
    hold off;
    pause(0.001);
end
end

%% Save refined data
if mode_save_data
leg = {'time(s)', ...
    'abody_x', 'abody_y', 'abody_z', ...
    'ainer_x', 'ainer_y', 'ainer_z', ...
    'viner_x', 'viner_y', 'viner_z', ...
    'riner_x', 'riner_y', 'riner_z', ...
    'w_x(dps)', 'w_y(dps)', 'w_z(dps)', ...
    'quar_a', 'quar_b', 'quar_c', 'quar_d', ...
    'ephi(rad)', 'etheta(rad)', 'epsi(rad)', ...
    'alt_press', 'alt_max', 'state'};
data = zeros(n, 26);
data(:,1) = dev_time(1:n);
data(:,2:4) = ab;
data(2:end,5:7) = ai;
data(:,8:10) = vi;
data(:,11:13) = ri;
data(:,14:16) = w;
data(:,17:20) = [a b c d];
data(:,21:23) = [EPhi' ETheta' EPsi'];
data(:,24:26) = [alt_press alt_max double(state)];
xlswrite('result.xlsx',leg,'Data')
xlswrite('result.xlsx',data,'Data',strcat('A2:AA',num2str(n+1)))
end