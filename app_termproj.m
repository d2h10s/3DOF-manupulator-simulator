clc, clear, close all
%% 상수 설정
l1 = 0.4; l2 = 0.4; l3 = 0.4; l4 = 0; l5 = 0.2+0.15; % 각 arm의 길이이다.
h1 = 0.05; h2 = 0.05; h3 = 0.05; h4 = 0.05; h5 = 0.05; % 각 arm의 두께이다.
m1 = 1.35; m2 = 1.35+1.3; m3 = 1.35+1.3; m4 = 1.3; m5 = 0.73+0.7; % 각 arm의 질량이다.
g = 9.806;

lc2 = l2/2; lc3 = l3/2; lc4 = l5/2; % 무게중심으로 arm의 중간지점으로 한다.
I2 = 1/12/2*m2*(h2^2+l2^2); I3 = 1/12/2*m3*(h3^2+l3^2); I4 = 1/12/2*m4*(h4^2+l5^2); % 각 arm의 관성모멘트이다.

%% 경로 계획

x0 =  0.15;   y0 = 0.40;                    dy_dx_0  =  0;  t0 = 0;
thd0 = -pi;
% ↑
% path1
% ↓
x1 =  0.45;   y1 = 0.03;   dy_dx_1f = 0;   dy_dx_1i = 1;   t1 = 1;
thd1 = -pi;
% ↑
% path2
% ↓
x2 =  0.60;   y2 = 0.40;   dy_dx_2f =  0;   dy_dx_2i = 0;  t2 = 2;
thd2 = -pi/2;
% ↑
% path3
% ↓
x3 = 1.065;   y3 = 0.20;   dy_dx_3f = -1;                  t3 = 3;
thd3 = -pi/8*5;
% ↑
% path4
% ↓
t4 = 4;


del_t = 0.01;
tspan = t0:del_t:t4;
tspan1 = t0:del_t:t1;
tspan2 = t1:del_t:t2;
tspan3 = t2:del_t:t3;

len = size(tspan);
len1 = size(tspan1);
len2 = size(tspan2);
len3 = size(tspan3);
len4 = len - len1(2) - len2(2) - len3(2);

px = zeros(len);      py = zeros(len);      thd = zeros(len);       R = cell(len);

T1 = make_trajectory(x0, y0, dy_dx_0,  x1, y1, dy_dx_1f);
T2 = make_trajectory(x1, y1, dy_dx_1i, x2, y2, dy_dx_2f);
T3 = make_trajectory(x2, y2, dy_dx_2i, x3, y3, dy_dx_3f);

for i = 0:len1(2)-1
    px(i+1) = (x1-x0)*0.5*(1-cos(pi/(len1(2)-1)*i))+x0;
    py(i+1) = T1'*[1 px(i+1) px(i+1)^2 px(i+1)^3]';
    thd(i+1) = thd0*(1-i/(len1(2)-1))+thd1*i/(len1(2)-1);
    R{i+1} = RotMat('z', 0);
end
for i = len1(2):len1(2)+len2(2)-1
    px(i+1) = (x2-x1)*0.5*(1-cos(pi/(len2(2)-1)*(i-len1(2))))+x1;
    py(i+1) = T2'*[1 px(i+1) px(i+1)^2 px(i+1)^3]';
    thd(i+1) = (thd2 - thd1)*0.5*(1-cos(pi/(len2(2)-1)*(i-len1(2))*2))+thd1;
    if thd(i+1)-thd(i) < 0
        thd(i+1) = thd2;
    end
    R{i+1} = RotMat('z', 0);
end
for i = len1(2)+len2(2):len1(2)+len2(2)+len3(2)-1
    px(i+1) = (x3-x2)*0.5*(1-cos(pi/(len3(2)-1)*(i-len1(2)-len2(2))))+x2;
    py(i+1) = T3'*[1 px(i+1) px(i+1)^2 px(i+1)^3]';
    thd(i+1) = (thd3-thd2)*0.5*(1-cos(pi/(len3(2)-1)*(i-len1(2)-len2(2))))+thd2;
    R{i+1} = RotMat('z', 0);
end
for i = len1(2)+len2(2)+len3(2):len(2)-1
    px(i+1) = px(len1(2)+len2(2)+len3(2)-1);
    py(i+1) = py(len1(2)+len2(2)+len3(2)-1);
    thd(i+1) = thd3;
    R{i+1} = RotMat('z',pi*(i-len1(2)-len2(2)-len3(2))/(len4(2)-1));
end

subplot(3,3,4)
plot(tspan, thd)
xlabel('Time [sec]'); ylabel('Angle [rad]')
title('Orientation of End Effector')

subplot(3,3,1)
plot(tspan, px,'o', 'MarkerSize', 1)
xlabel('Time [sec]'); ylabel('Position of X [m]');
title('X Position')

subplot(3,3,2)
plot(tspan,py, 'o', 'MarkerSize', 1)
title('Y Position')
xlabel('Time [sec]'); ylabel('Position of Y [m]');

subplot(3,3,3)
plot(px, py)
title('X-Y Position')
xlabel('Position of X'); ylabel('Position of Y [m] ');
%% 역기구학
x3_ = px - (l5).*cos(thd+pi/2);
y3_ = py - (l5).*sin(thd+pi/2);

c3 = (x3_.^2 + (y3_-l1).^2 - l2.^2 - l3.^2)./(2*l2*l3);
s3 = -sqrt(1-c3.^2);

th3 = atan2(s3, c3);
th2 = atan2(y3_ - l1, x3_) - atan2(l3.*sin(th3),l2+l3.*cos(th3));
th4 = thd - th3 - th2;
th5 = zeros(len);
for i = 1:len(2)
    th5(i) = atan2(R{i}(2,1), R{i}(2,2));
end
%% Dynamics

subplot(3,3,5)
plot(tspan, th2, tspan, th3, tspan, th4);
title('Joint Angle')
grid on
legend('\theta_2', '\theta_3', '\theta_4')
xlabel('Time [sec]'); ylabel('Angle [rad]')

M11 = m2*lc2^2 + I2 +m3*l2^2 + m3*lc3^2 + 2*m3*l2*lc3*cos(th3) + I3+m4*l2^2+m4*l3^2+m4*lc4^2 + m4*l2*l3*2*cos(th3) + m4*l3*lc4*2*cos(th4)+ m4*l2*lc4*2*cos(th3+th4) + I4;
M12 = m3*lc3^2 + m3*l2*lc3*cos(th3) + I3 + m4*l3^2 + m4*lc4^2 + m4*l2*l3*cos(th3) + m4*l3*lc4*2*cos(th4) + m4*l2*lc4*cos(th3+th4) + I4;
M13 = m4*lc4^2 + m4*l3*lc4*cos(th4) + m4*l2*lc4*cos(th3+th4) + I4;
M21 = m3*lc3^2 + I4 + m3*l2*lc3*cos(th3) + I3 + m4*l3^2 + m4*lc4^2 + m4*l2*l3*cos(th3) + 2*m4*l3*lc4*cos(th4) + m4*l2*lc4*cos(th3+th4);
M22 = m3*lc3^2 + I4 + I3 + m4*l3^2 + m4*lc4^2 +2*m4*l3*lc4*cos(th4);
M23 = m4*lc4^2 + m4*l3*lc4*cos(th4) + I4;
M31 = m4*lc4^2 + m4*l3*lc4*cos(th4) + m4*l2*lc4*cos(th3+th4) + I4;
M32 = m4*lc4^2 + m4*l3*lc4*cos(th4) +I4;
M33 = ones(len)*(m4*lc4^2 + I4);

A112 = -m3*l2*lc3*2*sin(th3) - m4*l2*l3*2*sin(th3) - 2*m4*l2*lc4*sin(th3+th4);
A123 = -m4*l3*lc4*2*sin(th4) + -2*m4*l2*lc4*sin(th3+th4);
A113 = -2*m4*l3*lc4*sin(th4) - 2*m4*l2*lc4*sin(th3+th4);
A122 = -m3*l2*lc4*sin(th4) - m4*l2*l3*sin(th3) - m4*l2*lc4*sin(th3+th4) ;
A133 = -m4*l2*lc4*sin(th3+th4) -m4*l3*lc4*sin(th4);
A211 = m3*l2*lc3*sin(th3) + m4*l2*l3*sin(th3) + m4*l2*lc4*sin(th3+th4);
A233 = -m4*l3*lc4*sin(th4);
A212 = zeros(len);
A223 = -m4*l3*lc4*2*sin(th4);
A213 = -2*m4*l3*lc4*sin(th4);
A311 = m4*l3*lc4*sin(th4) + m4*l2*lc4*sin(th3+th4);
A322 = m4*l3*lc4*sin(th4);
A312 = 2*m4*l3*lc4*sin(th4);
A323 = zeros(len);
A313 = zeros(len);

dth2 = zeros(len);   dth3 = zeros(len);  dth4 = zeros(len);
ddth2 = zeros(len); ddth3 = zeros(len); ddth4 = zeros(len);

tau = zeros(len(2), 3);     power = zeros(len(2), 3);

for i = 1:len(2)
    dth2(i) = (th2(i) - th2(max(1,i-1)))/(del_t);
    dth3(i) = (th3(i) - th3(max(1,i-1)))/(del_t);
    dth4(i) = (th4(i) - th4(max(1,i-1)))/(del_t);
    
    ddth2(i) = (dth2(i) - dth2(max(1,i-1)))/(del_t);
    ddth3(i) = (dth3(i) - dth3(max(1,i-1)))/(del_t);
    ddth4(i) = (dth4(i) - dth4(max(1,i-1)))/(del_t);
    
    M = [M11(i) M12(i) M13(i) ; M21(i) M22(i) M23(i) ; M31(i) M32(i) M33(i)];
    C = [A112(i)*dth2(i)*dth3(i) + A123(i)*dth3(i)*dth4(i) + A113(i)*dth2(i)*dth4(i) + A133(i)*dth4(i).^2 + A122(i)*dth3(i).^2;
        A211(i)*dth2(i).^2 + A233(i)*dth3(i).^2 + A212(i)*dth2(i)*dth3(i) + A223(i)*dth3(i)*dth4(i) + A213(i)*dth2(i)*dth4(i);
        A311(i)*dth2(i).^2 + A322(i)*dth3(i).^2 + A312(i)*dth2(i)*dth3(i) + A323(i)*dth3(i)*dth4(i) + A313(i)*dth2(i)*dth4(i)];
    G = [m2*g*lc2*cos(th2(i)) + m3*g*(l2*cos(th2(i))+ lc3*cos(th2(i)+th3(i))) + m4*g*(l2*cos(th2(i)) + l3*cos(th2(i)+th3(i)) + lc4*cos(th2(i)+th3(i)+th4(i)));
        m3*g*lc3*cos(th3(i)+th2(i)) + m4*g*(l3*cos(th2(i)+th3(i)) + lc4*cos(th2(i)+th3(i)+th4(i)));
        m4*g*lc4*cos(th2(i)+cos(th3(i) + th4(i)))];
    
    tau(i,1:3) = M*[ddth2(i) ddth3(i) ddth4(i)]' + C + G;
    power(i,1:3) = abs([dth2(i) dth3(i) dth4(i)]'.*tau(i,1:3)');
end

subplot(3,3,6)
plot(tspan, dth2, tspan, dth3, tspan, dth4)
title('Joint Anglular Velocity')
legend('\omega_2', '\omega_3', '\omega_4')
xlabel('Time [sec]'); ylabel('Angular Velocity [rad/sec]')
grid on

subplot(3,3,7)
plot(tspan, ddth2, tspan, ddth3, tspan, ddth4)
title('Joint Anglular Acceleration')
legend('\alpha_2', '\alpha_3', '\alpha_4')
xlabel('Time [sec]'); ylabel('Angular Acceleration [rad/sec^2]')
grid on

subplot(3,3,8)
plot(tspan, tau)
title('Joint Torgue')
legend('\tau_2', '\tau_3', '\tau_4')
xlabel('Time [sec]'); ylabel('torque [Nm]')
grid on

subplot(3,3,9)
plot(tspan, power)
title('Joint Power')
legend('\itp_2', '\itp_3', '\itp_4')
xlabel('Time [sec]'); ylabel('power [W]')
grid on

fprintf('max angular velocity(rad/s) is [%f, %f, %f]\n', max(abs(dth2)), max(abs(dth3)), max(abs(dth4)))
fprintf('max angular velocity(rpm) is [%f, %f, %f]\n', max(abs(dth2))/2/pi*60, max(abs(dth3))/2/pi*60, max(abs(dth4))/2/pi*60)
fprintf('mean angular velocity(rpm) is [%f, %f, %f]\n', mean(abs(dth2))/2/pi*60, mean(abs(dth3))/2/pi*60, mean(abs(dth4))/2/pi*60)
fprintf('max angular acceleration is [%f, %f, %f]\n', max(abs(ddth2)), max(abs(ddth3)), max(abs(ddth4)))
fprintf('max torque is [%f, %f, %f]\n', max(tau))
fprintf('median torque is [%f, %f, %f]\n', median(tau))
fprintf('max power is [%f, %f, %f]\n', max(power))
%% DH-Parameter 시뮬레이션

%{
%           a    alpha    d    theta
DHparams = [  0,  pi/2, l1,   0;
             l2,     0,  0, th2;
             l3,     0,  0, th3;
              0, -pi/2,  0, th4;
              0,     0, l5, th5]
%}
%  L = Link([th  d   a  alpha])
L(1) = Link([0, l1,  0,  pi/2]);
L(2) = Link([0,  0, l2,     0], 'R');
L(3) = Link([0,  0, l3,     0], 'R');
L(4) = Link([0,  0, l4, -pi/2], 'R');
L(5) = Link([0, l5,  0,     0], 'R');
robot = SerialLink(L, 'name', 'bobper');
%v = VideoWriter('motion.avi');
%{
%open(v)
figure
for i =1:len(2)
    robot.plot([0 th2(i) th3(i) th4(i) th5(i)], 'floorlevel', 0, 'jvec', 'fps', 1/del_t, 'base', 'trail',{'r','LineWidth', 1})
 %   frame = getframe(gcf);
  %  writeVideo(v,frame);
end
%close(v)
%}

%% function
function R = RotMat(axis, rad)
    axis = find(['x', 'y', 'z'] == axis);
    R = ones(3); V = [cos(rad) -sin(rad); sin(rad) cos(rad)];
    R(axis,:) = 0; R(:,axis) = 0; R(logical(R)) = V; R(axis,axis) = 1;
end

function p = make_trajectory(xi, yi, dy_dx_i, xf, yf, dy_dx_f)
    p = [1 xi xi^2   xi^3;
         0  1 2*xi 3*xi^2;
         1 xf xf^2   xf^3;
         0  1 2*xf 3*xf^2] \ [yi dy_dx_i yf dy_dx_f]';
end