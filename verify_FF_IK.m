%% initial value
l1 = 0.4;
l2 = 0.4;
l3 = 0.4;
l4 = 0.2+0.15;

px = 0.5;
py = 0.3;
thd = 0;
fprintf("initial value :: px: %f, py: %f, thd: %f\n", px, py, thd)
%% IK calculation
x3 = px - l4*cos(thd+pi/2);
y3 = py - l4*sin(thd+pi/2);

C2 = (x3^2 + (y3-l1)^2 - l2^2 - l3^2)./(2*l2*l3);
S2 = -sqrt(1-C2^2);

th2 = atan2(S2, C2);
th1 = atan2(y3 - l1, x3) - atan2(l3*sin(th2),l2+l3*cos(th2));
th3 = thd - th2 - th1;
fprintf("IK :: th1: %f, th2: %f, th3: %f\n", th1, th2, th3);

%% FK calculation
Px = 0.4*(cos(th1+th2)+cos(th1))-0.35*sin(th1+th2+th3);
Py = 0.4*(sin(th1+th2)+sin(th1)+1)+0.35*cos(th1+th2+th3);
fprintf("FK :: px: %f, py: %f\n", Px, Py);