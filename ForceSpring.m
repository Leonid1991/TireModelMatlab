function Force = ForceSpring(in1,in2,in3,in4,cp,l0,eta)
%ForceSpring
%    Force = ForceSpring(IN1,IN2,IN3,IN4,CP,L0,ETA)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    22-Jan-2024 15:14:01

e_c1 = in1(1,:);
e_c2 = in1(2,:);
e_c3 = in1(3,:);
e_s1 = in3(1,:);
e_s2 = in3(2,:);
e_s3 = in3(3,:);
v_c1 = in2(1,:);
v_c2 = in2(2,:);
v_c3 = in2(3,:);
v_s1 = in4(1,:);
v_s2 = in4(2,:);
v_s3 = in4(3,:);
t2 = -e_s1;
t3 = -e_s2;
t4 = -e_s3;
t5 = -v_s1;
t6 = -v_s2;
t7 = -v_s3;
t8 = e_c1+t2;
t9 = e_c2+t3;
t10 = e_c3+t4;
t11 = t5+v_c1;
t12 = t6+v_c2;
t13 = t7+v_c3;
t14 = abs(t8);
t15 = abs(t9);
t16 = abs(t10);
t17 = t14.^2;
t18 = t15.^2;
t19 = t16.^2;
t20 = t17+t18+t19;
t21 = sqrt(t20);
t22 = 1.0./t21;
t23 = -t21;
t24 = l0+t23;
t26 = t8.*t11.*t22;
t27 = t9.*t12.*t22;
t28 = t10.*t13.*t22;
t25 = cp.*t24;
t29 = t26+t27+t28;
t30 = eta.*t29;
t31 = -t30;
t32 = t25+t31;
Force = [-t8.*t22.*t32;-t9.*t22.*t32;-t10.*t22.*t32];
end
