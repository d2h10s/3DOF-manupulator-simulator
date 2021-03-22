clc, clear, close all

l2 = 1; l3 = 1; l4 = 1; % 각 arm의 길이이다.
m2 = 10; m3 = 5; m4 = 3; % 각 arm의 무게이다.
g = 9.806;

lc2 = l2/2; lc3 = l3/2; lc4 = l4/2; % 무게중심으로 arm의 중간지점으로 한다.
I2 = 1/12*m2*l2^2; I3 = 1/12*m3*l3^2; I4 = 1/12*m4*l4^2;% 각 arm의 관성모멘트이다.

del_t = 0.01; % 타임 업데이트 간격

t = 0:del_t:1;

px = 1.5+0.6*0.5*(1-cos(pi*t/1.));
py = 1.0+0.4*0.5*(1-cos(pi*t/1.));


C2 = ((px-l4).^2+py.^2-l2.^2-l3.^2)/(2*l2*l3);
S2 = sqrt(1-C2.^2);
th2 = atan2(S2,C2);
th1 = atan2(py, px-l4) - atan2(l3*S2, l2+l3*C2);
th3 = 2*pi - th1 - th2;

subplot(2,3,2)
plot(t, th1, t, th2, t, th3);
title('joint angle')
grid on
legend('\theta_1', '\theta_2', '\theta_3')

len = size(t);

M11 = m2*lc2^2 + I2 +m3*l2^2 + m3*lc3^2 + 2*m3*l2*lc3*cos(th2) + I3+m4*l2^2+m4*l3^2+m4*lc4^2 + m4*l2*l3*2*cos(th2) + m4*l3*lc4*2*cos(th3)+ m4*l2*lc4*2*cos(th2+th3) + I4;
M12 = m3*lc3^2 + m3*l2*lc3*cos(th2) + I3 + m4*l3^2 + m4*lc4^2 + m4*l2*l3*cos(th2) + m4*l3*lc4*2*cos(th3) + m4*l2*lc4*cos(th2+th3) + I4;
M13 = m4*lc4^2 + m4*l3*lc4*cos(th3) + m4*l2*lc4*cos(th2+th3) + I4;
M21 = m3*lc3^2 + I4 + m3*l2*lc3*cos(th2) + I3 + m4*l3^2 + m4*lc4^2 + m4*l2*l3*cos(th2) + 2*m4*l3*lc4*cos(th3) + m4*l2*lc4*cos(th2+th3);
M22 = m3*lc3^2 + I4 + I3 + m4*l3^2 + m4*lc4^2 +2*m4*l3*lc4*cos(th3);
M23 = m4*lc4^2 + m4*l3*lc4*cos(th3) + I4;
M31 = m4*lc4^2 + m4*l3*lc4*cos(th3) + m4*l2*lc4*cos(th2+th3) + I4;
M32 = m4*lc4^2 + m4*l3*lc4*cos(th3) +I4;
M33 = ones(len)*(m4*lc4^2 + I4);

A112 = -m3*l2*lc3*2*sin(th2) - m4*l2*l3*2*sin(th2) - 2*m4*l2*lc4*sin(th2+th3);
A123 = -m4*l3*lc4*2*sin(th3) + -2*m4*l2*lc4*sin(th2+th3);
A113 = -2*m4*l3*lc4*sin(th3) - 2*m4*l2*lc4*sin(th2+th3);
A122 = -m3*l2*lc4*sin(th3) - m4*l2*l3*sin(th2) - m4*l2*lc4*sin(th2+th3) ;
A133 = -m4*l2*lc4*sin(th2+th3) -m4*l3*lc4*sin(th3);
A211 = m3*l2*lc3*sin(th2) + m4*l2*l3*sin(th2) + m4*l2*lc4*sin(th2+th3);
A233 = -m4*l3*lc4*sin(th3);
A212 = zeros(len);
A223 = -m4*l3*lc4*2*sin(th3);
A213 = -2*m4*l3*lc4*sin(th3);
A311 = m4*l3*lc4*sin(th3) + m4*l2*lc4*sin(th2+th3);
A322 = m4*l3*lc4*sin(th3);
A312 = 2*m4*l3*lc4*sin(th3);
A323 = zeros(len);
A313 = zeros(len);
dth1 = zeros(len); 
dth2 = zeros(len);
dth3 = zeros(len);
ddth1 = zeros(len);
ddth2 = zeros(len);
ddth3 = zeros(len);
tau = zeros(len(2), 3);
power = zeros(len(2), 3);
for i = 1:len(2)
    dth1(i) = (th1(i) - th1(max(1,i-1)))/(del_t);
    dth2(i) = (th2(i) - th2(max(1,i-1)))/(del_t);
    dth3(i) = (th3(i) - th3(max(1,i-1)))/(del_t);
    
    ddth1(i) = (dth1(i) - dth1(max(1,i-1)))/(del_t);
    ddth2(i) = (dth2(i) - dth2(max(1,i-1)))/(del_t);
    ddth3(i) = (dth3(i) - dth3(max(1,i-1)))/(del_t);
    
    M = [M11(i) M12(i) M13(i) ; M21(i) M22(i) M23(i) ; M31(i) M32(i) M33(i)];
    C = [A112(i)*dth1(i)*dth2(i) + A123(i)*dth2(i)*dth3(i) + A113(i)*dth1(i)*dth3(i) + A133(i)*dth3(i).^2 + A122(i)*dth2(i).^2;
        A211(i)*dth1(i).^2 + A233(i)*dth2(i).^2 + A212(i)*dth1(i)*dth2(i) + A223(i)*dth2(i)*dth3(i) + A213(i)*dth1(i)*dth3(i);
        A311(i)*dth1(i).^2 + A322(i)*dth2(i).^2 + A312(i)*dth1(i)*dth2(i) + A323(i)*dth2(i)*dth3(i) + A313(i)*dth1(i)*dth3(i)];
    G = [m2*g*lc2*cos(th1(i)) + m3*g*(l2*cos(th1(i))+ lc3*cos(th1(i)+th2(i))) + m4*g*(l2*cos(th1(i)) + l3*cos(th1(i)+th2(i)) + lc4*cos(th1(i)+th2(i)+th3(i)));
        m3*g*lc3*cos(th2(i)+th1(i)) + m4*g*(l3*cos(th1(i)+th2(i)) + lc4*cos(th1(i)+th2(i)+th3(i)));
        m4*g*lc4*cos(th1(i)+cos(th2(i) + th3(i)))];
    
    tau(i,1:3) = M*[ddth1(i) ddth2(i) ddth3(i)]' + C + G;
    power(i,1:3) = abs([dth1(i) dth2(i) dth3(i)]'.*tau(i,1:3)');
end
subplot(2,3,3)
plot(t, dth1, t, dth2, t, dth3)
title('joint angle velocity')
legend('\omega_1', '\omega_2', '\omega_3')
grid on

subplot(2,3,4)
plot(t, ddth1, t, ddth2, t, ddth3)
title('joint angle acceleration')
legend('\alpha_1', '\alpha_2', '\alpha_3')
grid on

subplot(2,3,5)
plot(t, tau)
title('joint torgue')
legend('\tau_1', '\tau_2', '\tau_3')
grid on

subplot(2,3,6)
plot(t, power)
title('joint power')
legend('\itp_1', '\itp_2', '\itp_3')
grid on