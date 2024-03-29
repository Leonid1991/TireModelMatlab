wfunction Force = ForcePress(in1,in2,in3,in4,press)
%ForcePress
%    Force = ForcePress(IN1,IN2,IN3,IN4,PRESS)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    22-Jan-2024 15:14:02

P_E1 = in3(1,:);
P_E2 = in3(2,:);
P_E3 = in3(3,:);
P_N1 = in1(1,:);
P_N2 = in1(2,:);
P_N3 = in1(3,:);
P_S1 = in2(1,:);
P_S2 = in2(2,:);
P_S3 = in2(3,:);
P_W1 = in4(1,:);
P_W2 = in4(2,:);
P_W3 = in4(3,:);
t2 = -P_S1;
t3 = -P_S2;
t4 = -P_S3;
t5 = -P_W1;
t6 = -P_W2;
t7 = -P_W3;
t8 = P_E1+t5;
t9 = P_E2+t6;
t10 = P_E3+t7;
t11 = P_N1+t2;
t12 = P_N2+t3;
t13 = P_N3+t4;
Force = [press.*(t9.*t13-t10.*t12).*(-1.0./2.0);(press.*(t8.*t13-t10.*t11))./2.0;press.*(t8.*t12-t9.*t11).*(-1.0./2.0)];
end
