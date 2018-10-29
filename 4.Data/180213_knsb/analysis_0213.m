data = MainAvionics;

close all;

len = length(data);

time = zeros(len,1);

time(:,1) = (data(:,1)-data(1,1))/1000;

% sensor to rocket body

C = [0 -1 0; 1 0 0; 0 0 1];

takeoff = 21767;
apogee = 22941 - takeoff ;

ax_bias = mean(data(1:takeoff,2));
ay_bias = mean(data(1:takeoff,3))+1;
az_bias = mean(data(1:takeoff,4));
a_bias = [ax_bias ay_bias az_bias];
wx_bias = mean(data(1:takeoff,5));
wy_bias = mean(data(1:takeoff,6));
wz_bias = mean(data(1:takeoff,7));
w_bias = [wx_bias wy_bias wz_bias];

data = data(takeoff+1:len,:);

g0 = 9.81;
n = length(data);

apaogee = 90;

% Eulerian angle

Phi = zeros(n, 1);
Theta = zeros(n, 1);
Psi = zeros(n, 1);

% Quaternion variables

a = zeros(n, 1);
b = zeros(n, 1);
c = zeros(n, 1);
d = zeros(n, 1);

% position from inertial frame

rx = zeros(n, 1);
ry = zeros(n, 1);
rz = zeros(n, 1);

% velocity from inertial frame

vx = zeros(n, 1);
vy = zeros(n, 1);
vz = zeros(n, 1);

% acceleration from body frame

ab = data(2:n, 2:4);
a_bias_mat = ones(n-1, 3);
a_bias_mat(:,1) = a_bias(1) * ones(n-1,1);
a_bias_mat(:,2) = a_bias(2) * ones(n-1,1);
a_bias_mat(:,3) = a_bias(3) * ones(n-1,1);
ab = C*(ab-a_bias_mat)';
ab = ab';

% acceleration from inertial frame

ai = zeros(n-1, 3);
ax = zeros(n-1, 1);
ay = zeros(n-1, 1);
az = zeros(n-1, 1);

% angular velocity from body frame

w = data(2:n, 5:7);
w_bias_mat = ones(n-1, 3);
w_bias_mat(:,1) = w_bias(1) * ones(n-1,1);
w_bias_mat(:,2) = w_bias(2) * ones(n-1,1);
w_bias_mat(:,3) = w_bias(3) * ones(n-1,1);
w = C*(w-w_bias_mat)';
w = w';
wx = w(:,1)*pi/180;
wy = w(:,2)*pi/180;
wz = w(:,3)*pi/180;

% Direction Cosine Matrix from body frame to inertial frame
C_Q = zeros(3,3,n-1);

% g- vector
g = [0 0 -1]';

for i = 1:n-1
dt(i) = (data(i+1,1) - data(i,1))*0.001;
end
time = (data(:,1)-data(1,1))/1000;
alt = data(:,11)- data(1,11);

% initial position, velocity, attitude
rx(1) = 0; ry(1) = 0; rz(1) = 0;
vx(1) = 0; vy(1) = 0; vz(1) = 0;

Phi(1) = 0;
Theta(1) = -pi/2;
Psi(1) = 0;
EPhi(1) = Phi(1);
ETheta(1) = -pi/2;
EPsi(1) = 0;
a(1) = cos(Phi(1)/2)*cos(Theta(1)/2)*cos(Psi(1)/2) + sin(Phi(1)/2)*sin(Theta(1)/2)*sin(Psi(1)/2);
b(1) = sin(Phi(1)/2)*cos(Theta(1)/2)*cos(Psi(1)/2) + cos(Phi(1)/2)*sin(Theta(1)/2)*sin(Psi(1)/2);
c(1) = cos(Phi(1)/2)*sin(Theta(1)/2)*cos(Psi(1)/2) + sin(Phi(1)/2)*cos(Theta(1)/2)*sin(Psi(1)/2);
d(1) = cos(Phi(1)/2)*cos(Theta(1)/2)*sin(Psi(1)/2) + sin(Phi(1)/2)*sin(Theta(1)/2)*cos(Psi(1)/2);

% heading update
for i = 1:n-1

    time(i+1) = time(i) + dt(i);

    da(i) = -0.5*(b(i)*wx(i) + c(i)*wy(i) + d(i)*wz(i));
    db(i) = 0.5*(a(i)*wx(i) - d(i)*wy(i) + c(i)*wz(i));
    dc(i) = 0.5*(d(i)*wx(i) + a(i)*wy(i) - b(i)*wz(i));
    dd(i) = -0.5*(c(i)*wx(i) - b(i)*wy(i) -a(i)*wz(i));
    
    a(i+1) = a(i) + dt(i)*da(i);
    b(i+1) = b(i) + dt(i)*db(i);
    c(i+1) = c(i) + dt(i)*dc(i);
    d(i+1) = d(i) + dt(i)*dd(i);
    
    C_Q(:,:,i) = [(a(i+1)^2+b(i+1)^2-c(i+1)^2-d(i+1)^2) 2*(b(i+1)*c(i+1)-a(i+1)*d(i+1)) 2*(b(i+1)*d(i+1)+a(i+1)*c(i+1))
                    2*(b(i+1)*c(i+1)+a(i+1)*d(i+1)) (a(i+1)^2-b(i+1)^2+c(i+1)^2-d(i+1)^2) 2*(c(i+1)*d(i+1)-a(i+1)*b(i+1))
                    2*(b(i+1)*d(i+1)-a(i+1)*c(i+1)) 2*(c(i+1)*d(i+1)+a(i+1)*b(i+1)) (a(i+1)^2-b(i+1)^2-c(i+1)^2+d(i+1)^2)];
    
    dPhi(i) = (wy(i)*sin(Phi(i))+wz(i)*cos(Phi(i)))*tan(Theta(i))+wx(i);
    dTheta(i) = wy(i)*cos(Phi(i)) - wz(i)*sin(Phi(i));
    dPsi(i) = (wy(i)*sin(Phi(i))+wz(i)*cos(Phi(i)))/cos(Theta(i));            
    EPhi(i+1) = EPhi(i) + dPhi(i)*dt(i);
    ETheta(i+1) = ETheta(i) + dTheta(i)*dt(i);
    EPsi(i+1) = EPsi(i) + dPsi(i)*dt(i);
    
    Phi(i+1) = atan2(C_Q(3,2,i),C_Q(3,3,i));
    Theta(i+1) = asin(-C_Q(3,1,i));
    Psi(i+1) = atan2(C_Q(2,1,i),C_Q(1,1,i));
    
    ai(i,:) = (C_Q(:,:,i)*ab(i,:)' + g)*g0;
    ax(i) = ai(i,1);
    ay(i) = ai(i,2);
    az(i) = ai(i,3);
                
end

% velocity/ position update
for i = 1:n-1
    vx(i+1) = vx(i) + dt(i)*ax(i);
    vy(i+1) = vy(i) + dt(i)*ay(i);
    vz(i+1) = vz(i) + dt(i)*az(i);
    
    rx(i+1) = rx(i) + dt(i)*vx(i);
    ry(i+1) = ry(i) + dt(i)*vy(i);
    rz(i+1) = rz(i) + dt(i)*vz(i);
end
r = [rx, ry, rz];

% figure(7)
% subplot(1,3,1)
% plot(time(1:apogee), ab(1:apogee,1)*g0)
% title('a_x')
% subplot(1,3,2)
% plot(time(1:apogee), ab(1:apogee,2)*g0)
% title('a_y')
% subplot(1,3,3)
% plot(time(1:apogee), ab(1:apogee,3)*g0)
% title('a_z')
% 
% figure(3)
% subplot(1,3,1)
% plot(time(1:apogee), ax(1:apogee))
% hold on
% title('a_x')
% 
% subplot(1,3,2)
% plot(time(1:apogee), ay(1:apogee))
% hold on
% title('a_y')
% 
% subplot(1,3,3)
% plot(time(1:apogee), az(1:apogee))
% hold on
% title('a_z')
% title('a_z')
% 
% figure(4)
% subplot(1,3,1)
% plot(time(1:apogee), vx(1:apogee))
% title('v_x')
% subplot(1,3,2)
% plot(time(1:apogee), vy(1:apogee))
% title('v_y')
% subplot(1,3,3)
% plot(time(1:apogee), vz(1:apogee))
% title('v_z')
% 
% figure(5)
% subplot(1,3,1)
% plot(time(1:apogee), rx(1:apogee))
% title('r_x')
% subplot(1,3,2)
% plot(time(1:apogee), ry(1:apogee))
% title('r_y')
% subplot(1,3,3)
% plot(time(1:apogee), rz(1:apogee))
% hold on 
% plot(time(1:apogee), data(1:apogee,11)-data(1,11))
% title('r_z')
% legend('accelerometer', 'altimeter')
% 
% figure(6)
% plot3(rx(1:apogee),ry(1:apogee),rz(1:apogee))
% title('trajectory')
% grid on
% xlabel('X')
% ylabel('Y')
% zlabel('Z')

XYZE = [1 0 0; 0 1 0; 0 0 1];

for i=1:n-1
    psi = Psi(i);
    phi = Phi(i);
    theta = Theta(i);

    xyzb =  C_Q(:,:,i) * XYZE;
    
    xb = xyzb(:,1);
    yb = xyzb(:,2);
    zb = xyzb(:,3);
    origin = [0; 0; 0];

    quiver3(0,0,0, xb(1),xb(2), xb(3),'r')
    hold on;
    quiver3(0,0,0, yb(1),yb(2), yb(3),'g')
    quiver3(0,0,0, zb(1),zb(2), zb(3),'b')
    text(0,0,-1,strcat(num2str(time(i+1)),{'s, '},num2str(alt(i+1)),{'m'}),'HorizontalAlignment','left','FontSize',12);
    axis([-1 1 -1 1 -1 1])
    legend('X(Roll)', 'Y(Pitch)', 'Z(Yaw)')
    hold off;
    pause(0.001);
end
    