clc, clear, close all

a = [1 1 1]; % 각 arm의 길이이다.
h = [0 0 0]; % arm의 두께이다 slender rod로 가정하고 계산한다.
m = [10 5 3]; % 각 arm의 무게이다.
g = 9.806;

lc = a./2; % 무게중심으로 arm의 중간지점으로 한다.
I = 1/12.*m.*(h.^h+a.^2); % 각 arm의 관성모멘트이다.

del_t = 0.01; % 타임 업데이트 간격

t = 0:del_t:1;

px = 1.5+0.6*0.5*(1-cos(pi*t/1.));
py = 1.0+0.4*0.5*(1-cos(pi*t/1.));


C2 = ((px-a(3)).^2+py.^2-a(1).^2-a(2).^2)/(2*a(1)*a(2));
S2 = sqrt(1-C2.^2);
theta2 = atan2(S2,C2);
theta1 = atan2(py, px-a(3)) - atan2(a(2)*S2, a(1)+a(2)*C2);
theta3 = 2*pi - theta1 - theta2;

subplot(2,3,1)
plot(0,0, 'ko')
title('moving motion')
Ax = [0 a(1)*cos(0) a(1)*cos(0)+a(2)*cos(0)+a(3) a(1)*cos(0)+a(2)*cos(0)+a(3)*cos(0)];
Ay = [0 a(1)*sin(0) a(1)*sin(0)+a(2)*sin(0)+a(3) a(1)*sin(0)+a(2)*sin(0)+a(3)*sin(0)];
p = line(Ax,Ay, 'EraseMode','xor','LineWidth',4);
grid on
axis([0 2.5 0 2.5])

subplot(2,3,2)
plot(t, theta1, t, theta2, t, theta3);
title('joint angle')
grid on
legend('\theta_1', '\theta_2', '\theta_3')

M11 = m(1)*lc(1)^2 + I(1) +m(2)*a(1)^2 + m(2)*lc(2)^2 + 2*m(2)*a(1)*lc(2)*cos(theta2) + I(2)+m(3)*a(1)^2+m(3)*a(2)^2+m(3)*lc(3)^2 + m(3)*a(1)*a(2)*2*cos(theta2) + m(3)*a(2)*lc(3)*2*cos(theta3)+ m(3)*a(1)*lc(3)*2*cos(theta2+theta3) + I(3);
M12 = m(2)*lc(2)^2 + m(2)*a(1)*lc(2)*cos(theta2) + I(2) + m(3)*a(2)^2 + m(3)*lc(3)^2 + m(3)*a(1)*a(2)*cos(theta2) + m(3)*a(2)*lc(3)*2*cos(theta3) + m(3)*a(1)*lc(3)*cos(theta2+theta3) + I(3);
M13 = m(3)*lc(3)^2 + m(3)*a(2)*lc(3)*cos(theta3) + m(3)*a(1)*lc(3)*cos(theta2+theta3) + I(3);
M21 = m(2)*lc(2)^2 + I(3) + m(2)*a(1)*lc(2)*cos(theta2) + I(2) + m(3)*a(2)^2 + m(3)*lc(3)^2 + m(3)*a(1)*a(2)*cos(theta2) + 2*m(3)*a(2)*lc(3)*cos(theta3) + m(3)*a(1)*lc(3)*cos(theta2+theta3);
M22 = m(2)*lc(2)^2 + I(3) + I(2) + m(3)*a(2)^2 + m(3)*lc(3)^2 +2*m(3)*a(2)*lc(3)*cos(theta3);
M23 = m(3)*lc(3)^2 + m(3)*a(2)*lc(3)*cos(theta3) + I(3);
M31 = m(3)*lc(3)^2 + m(3)*a(2)*lc(3)*cos(theta3) + m(3)*a(1)*lc(3)*cos(theta2+theta3) + I(3);
M32 = m(3)*lc(3)^2 + m(3)*a(2)*lc(3)*cos(theta3) +I(3);
M33 = ones(size(M32))*(m(3)*lc(3)^2 + I(3));

A112 = -m(2)*a(1)*lc(2)*2*sin(theta2) - m(3)*a(1)*a(2)*2*sin(theta2) - 2*m(3)*a(1)*lc(3)*sin(theta2+theta3);
A123 = -m(3)*a(2)*lc(3)*2*sin(theta3) + -2*m(3)*a(1)*lc(3)*sin(theta2+theta3);
A113 = -2*m(3)*a(2)*lc(3)*sin(theta3) - 2*m(3)*a(1)*lc(3)*sin(theta2+theta3);
A122 = -m(2)*a(1)*lc(3)*sin(theta3) - m(3)*a(1)*a(2)*sin(theta2) - m(3)*a(1)*lc(3)*sin(theta2+theta3) ;
A133 = -m(3)*a(1)*lc(3)*sin(theta2+theta3) -m(3)*a(2)*lc(3)*sin(theta3);
A211 = m(2)*a(1)*lc(2)*sin(theta2) + m(3)*a(1)*a(2)*sin(theta2) + m(3)*a(1)*lc(3)*sin(theta2+theta3);
A233 = -m(3)*a(2)*lc(3)*sin(theta3);
A212 = zeros(size(A233));
A223 = -m(3)*a(2)*lc(3)*2*sin(theta3);
A213 = -2*m(3)*a(2)*lc(3)*sin(theta3);
A311 = m(3)*a(2)*lc(3)*sin(theta3) + m(3)*a(1)*lc(3)*sin(theta2+theta3);
A322 = m(3)*a(2)*lc(3)*sin(theta3);
A312 = 2*m(3)*a(2)*lc(3)*sin(theta3);
A323 = zeros(size(A312));
A313 = zeros(size(A312));
dtheta1 = zeros(size(A312)); 
dtheta2 = zeros(size(A312));
dtheta3 = zeros(size(A312));
ddtheta1 = zeros(size(A312));
ddtheta2 = zeros(size(A312));
ddtheta3 = zeros(size(A312));
tau = zeros(1/del_t+1, 3);
power = zeros(1/del_t+1, 3);
for i = 2:1/del_t
    dtheta1(i) = (theta1(i) - theta1(i-1))/(del_t);
    dtheta2(i) = (theta2(i) - theta2(i-1))/(del_t);
    dtheta3(i) = (theta3(i) - theta3(i-1))/(del_t);
    
    ddtheta1(i) = (dtheta1(i) - dtheta1(i-1))/(del_t);
    ddtheta2(i) = (dtheta2(i) - dtheta2(i-1))/(del_t);
    ddtheta3(i) = (dtheta3(i) - dtheta3(i-1))/(del_t);
    
    M = [M11(i) M12(i) M13(i) ; M21(i) M22(i) M23(i) ; M31(i) M32(i) M33(i)];
    C = [A112(i)*dtheta1(i)*dtheta2(i) + A123(i)*dtheta2(i)*dtheta3(i) + A113(i)*dtheta1(i)*dtheta3(i) + A133(i)*dtheta3(i).^2 + A122(i)*dtheta2(i).^2;
        A211(i)*dtheta1(i).^2 + A233(i)*dtheta2(i).^2 + A212(i)*dtheta1(i)*dtheta2(i) + A223(i)*dtheta2(i)*dtheta3(i) + A213(i)*dtheta1(i)*dtheta3(i);
        A311(i)*dtheta1(i).^2 + A322(i)*dtheta2(i).^2 + A312(i)*dtheta1(i)*dtheta2(i) + A323(i)*dtheta2(i)*dtheta3(i) + A313(i)*dtheta1(i)*dtheta3(i)];
    G = [m(1)*g*lc(1)*cos(theta1(i)) + m(2)*g*(a(1)*cos(theta1(i)) + lc(2)*cos(theta1(i)+theta2(i))) + m(3)*g*(a(1)*cos(theta1(i)) + a(2)*cos(theta1(i)+theta2(i)) + lc(3)*cos(theta1(i)+theta2(i)+theta3(i)));
        m(2)*g*lc(2)*cos(theta2(i)+theta1(i)) + m(3)*g*(a(2)*cos(theta1(i)+theta2(i)) + lc(3)*cos(theta1(i)+theta2(i)+theta3(i)));
        m(3)*g*lc(3)*cos(theta1(i)+cos(theta2(i) + theta3(i)))];
    
    tau(i,1:3) = M*[ddtheta1(i) ddtheta2(i) ddtheta3(i)]' + C + G;
    power(i,1:3) = abs([dtheta1(i) dtheta2(i) dtheta3(i)]'.*tau(i,1:3)');
    
    Ax = [0 a(1)*cos(theta1(i)) a(1)*cos(theta1(i))+a(2)*cos(theta1(i)+theta2(i)) a(1)*cos(theta1(i))+a(2)*cos(theta1(i)+theta2(i))+a(3)*cos(theta1(i)+theta2(i)+theta3(i))];
    Ay = [0 a(1)*sin(theta1(i)) a(1)*sin(theta1(i))+a(2)*sin(theta1(i)+theta2(i)) a(1)*sin(theta1(i))+a(2)*sin(theta1(i)+theta2(i))+a(3)*sin(theta1(i)+theta2(i)+theta3(i))];
    
    set(p, 'XData',Ax,'YData',Ay);
    drawnow;
end
subplot(2,3,3)
plot(t(2:end-2), dtheta1(2:end-2), t(2:end-2), dtheta2(2:end-2), t(2:end-2), dtheta3(2:end-2))
title('joint angle velocity')
legend('\omega_1', '\omega_2', '\omega_3')
grid on

subplot(2,3,4)
plot(t(2:end-2), ddtheta1(2:end-2), t(2:end-2), ddtheta2(2:end-2), t(2:end-2), ddtheta3(2:end-2))
title('joint angle acceleration')
legend('\alpha_1', '\alpha_2', '\alpha_3')
grid on

subplot(2,3,5)
plot(t(2:end-2), tau(2:end-2,:,:))
title('joint torgue')
legend('\tau_1', '\tau_2', '\tau_3')
grid on

subplot(2,3,6)
plot(t(2:end-2), power(2:end-2,:,:))
title('joint power')
legend('\itp_1', '\itp_2', '\itp_3')
grid on