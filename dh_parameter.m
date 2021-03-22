clear
syms th2 th3 th4 th5
syms r11 r12 r13 r21 r22 r23 r31 r32 r33
l1 = 0.4; l2 = 0.4; l3 = 0.4; l4 = 0; l5 = 0.2+0.15;

DHparams = [  0,  pi/2, l1,    0;
             l2,     0,  0  th2;
             l3,     0,  0, th3;
             l4, -pi/2,  0, th4;
              0,     0, l5, th5 ];

R = [r11 r12 r13;
     r21 r22 r23;
     r31 r32 r33 ];

R1 = dhr(DHparams(1,:));
R2 = dhr(DHparams(2,:));
R3 = dhr(DHparams(3,:));
R4 = dhr(DHparams(4,:));
R5 = dhr(DHparams(5,:));


R03 = simplify(R1*R2*R3)
R35 = simplify(R4*R5)
R35_ = simplify((R03.')*R)

function R = dhr(arr)
    a = arr(1);  al = arr(2); d = arr(3); th = arr(4);
    R = [cos(th) -sin(th)*cos(al)  sin(th)*sin(al);
         sin(th)  cos(th)*cos(al) -cos(th)*sin(al);
               0          sin(al)          cos(al); ];
end
