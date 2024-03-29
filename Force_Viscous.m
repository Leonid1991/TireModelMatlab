function Force_vi = Force_Viscous(in1,in2,in3,in4,eta)
%Force_Viscous
%    Force_vi = Force_Viscous(IN1,IN2,IN3,IN4,ETA)

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
t21 = 1.0./sqrt(t20);
t22 = t8.*t11.*t21;
t23 = t9.*t12.*t21;
t24 = t10.*t13.*t21;
t25 = t22+t23+t24;
Force_vi = [eta.*t8.*t21.*t25;eta.*t9.*t21.*t25;eta.*t10.*t21.*t25];
end
