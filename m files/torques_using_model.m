%%

tav = dynamic_model()

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%plynomianl 3 4 5 
Ts_M = 0.01;
beta = atan(128/24);
bias = [ 0 ; beta ; -beta; 0 ] ;
theta_i_m = theta_i + bias;
theta_f_m = theta_f+ bias;
p = 0;
TAV = zeros(4, 1 + tf / Ts_M);
for j = 0 : Ts_M : tf
    p = p + 1;
      s = a * j ^ 5 + b * j ^ 4 + c * j ^ 3;
    s_prime = 5*a*j^4 + 4*b*j^3 + 3*c*j^2;
    s_second = 20*a*j^3 + 12*b*j^2 + 6*c*j^1;
    
    
    theta = theta_i_m + (theta_f_m - theta_i_m) * s;
    theta = theta - round( theta /2 / pi ) *2*pi;

    theta_dot = (theta_f_m - theta_i_m) * s_prime / tf;
    theta_ddot = (theta_f_m - theta_i_m) * s_second / tf^2;
    
    
    
    
    theta1 = theta(1);
    theta2 = theta(2);
    theta3 = theta(3);
    theta4 = theta(4);

    
    theta1_dot = theta_dot(1);
    theta2_dot = theta_dot(2);
    theta3_dot = theta_dot(3);
    theta4_dot = theta_dot(4);

    theta1_ddot = theta_ddot(1);
    theta2_ddot = theta_ddot(2);
    theta3_ddot = theta_ddot(3);
    theta4_ddot = theta_ddot(4);

    TAV(:,p) = eval(tav);
    
end
%%
close all
figure
plot(0:Ts_M:tf , TAV','--')
hold on
plot( torque.time ,[torque.Data])
ylim([-0.6 1])
legend('tav_1' , 'tav_2','tav_3','tav_4','tav1_{sim}' , 'tav2_{sim}','tav3_{sim}','tav4_{sim}')
print('dynamic_model_validation','-depsc')
title('joints torque using simscape and dynamic model')
xlabel('t')
ylabel('\tau (N.m)')
