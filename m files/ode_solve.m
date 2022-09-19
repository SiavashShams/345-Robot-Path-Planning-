clc
tspan = 0 : 0.0001: 1;
x0 = 0.38;
y0=0;
z0=0.077;
XX=cos(theta1_num).*(0.126*cos(theta2_num+theta3_num+theta4_num)+0.124*cos(theta2_num+theta3_num )+0.13*cos(theta2_num));
YY=sin(theta1_num).*(0.126*cos(theta2_num+theta3_num+theta4_num)+0.124*cos(theta2_num+theta3_num )+0.13*cos(theta2_num));
ZZ=0.077+0.126*sin(theta2_num+theta3_num+theta4_num)+0.124*sin(theta2_num+theta3_num )+0.13*sin(theta2_num);
plot(tspan,ZZ)
A=0.126*cos(theta2_num+theta3_num+theta4_num)+0.124*cos(theta2_num+theta3_num )+0.13*cos(theta2_num);
B=-0.126*sin(theta2_num+theta3_num+theta4_num)-0.124*sin(theta2_num+theta3_num )-0.13*sin(theta2_num);
C=-0.126*sin(theta2_num+theta3_num+theta4_num)-0.124*sin(theta2_num+theta3_num);
D=-0.126*sin(theta2_num+theta3_num+theta4_num);
E=0.126*cos(theta2_num+theta3_num+theta4_num)+0.124*cos(theta2_num+theta3_num );
F=0.126*cos(theta2_num+theta3_num+theta4_num);
J=[zeros(1,10001) ones(1,10001) ones(1,10001) ones(1,10001); -sin(theta1_num).*A cos(theta1_num).*B cos(theta1_num).*C cos(theta1_num).*D ;cos(theta1_num).*A sin(theta1_num).*B sin(theta1_num).*C sin(theta1_num).*D; zeros(1,10001) A E F]; 
theta_dot=[theta1_dot_num; theta2_dot_num; theta3_dot_num; theta4_dot_num];
theta=[theta1_num theta2_num theta3_num theta4_num];
xee=J(2,1:10001).*theta1_dot_num+J(2,10002:20002).*theta2_dot_num+J(2,20003:30003).*theta3_dot_num+J(2,30004:40004).*theta4_dot_num;
yee=J(3,1:10001).*theta1_dot_num+J(3,10002:20002).*theta2_dot_num+J(3,20003:30003).*theta3_dot_num+J(3,30004:40004).*theta4_dot_num;
zee=J(4,1:10001).*theta1_dot_num+J(4,10002:20002).*theta2_dot_num+J(4,20003:30003).*theta3_dot_num+J(4,30004:40004).*theta4_dot_num;
 [tt,xx] = ode23s(@(t,x)   interp1(tspan, xee, t) ,tspan, x0);
 [tt,yy] = ode23s(@(t,y)  interp1(tspan, yee, t),tspan, y0);
 [tt,zz] = ode23s(@(t,z)  interp1(tspan, zee, t),tspan, z0);
figure (1)
 plot(tt,xx)
hold on
plot(tspan,XX)
title('X end effector')
legend('Solved by ode','Solved by FKP')
hold off

figure (2)
plot(tt,yy)
hold on
plot(tspan,YY)
title('Y end effector')
legend('Solved by ode','Solved by FKP')

hold off

figure(3)
plot(tt,zz)
hold on
plot(tspan,ZZ)
title('Z end effector')
legend('Solved by ode','Solved by FKP')

hold off

 X_f=cos(-pi/2)*(0.126*cos(0.3*pi)+0.124*cos(pi/2)+0.13*cos(pi/3))
 Y_f=sin(-pi/2)*(0.126*cos(0.3*pi)+0.124*cos(pi/2)+0.13*cos(pi/3))
 Z_f=0.077+0.126*sin(0.3*pi)+0.124*sin(pi/2)+0.13*sin(pi/3)



