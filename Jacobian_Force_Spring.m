function tt1 = Jacobian_Force_Spring(in1,in2,cp,l0)
%Jacobian_Force_Spring
%    TT1 = Jacobian_Force_Spring(IN1,IN2,CP,L0)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    22-Jan-2024 19:20:02

e_c1 = in1(1,:);
e_c2 = in1(2,:);
e_c3 = in1(3,:);
e_s1 = in2(1,:);
e_s2 = in2(2,:);
e_s3 = in2(3,:);
t2 = e_c1.^2;
t3 = e_c2.^2;
t4 = e_c3.^2;
t5 = e_s1.^2;
t6 = e_s2.^2;
t7 = e_s3.^2;
t8 = -e_s1;
t9 = -e_s2;
t10 = -e_s3;
t17 = e_c1.*e_s1.*l0.*2.0;
t18 = e_c2.*e_s2.*l0.*2.0;
t19 = e_c3.*e_s3.*l0.*2.0;
t11 = l0.*t2;
t12 = l0.*t3;
t13 = l0.*t4;
t14 = l0.*t5;
t15 = l0.*t6;
t16 = l0.*t7;
t20 = e_c1+t8;
t21 = e_c2+t9;
t22 = e_c3+t10;
t23 = -t17;
t24 = -t18;
t25 = -t19;
t26 = t20.^2;
t27 = t21.^2;
t28 = t22.^2;
t29 = t26+t27+t28;
t30 = t29.^(3.0./2.0);
t31 = 1.0./t30;
t32 = -t30;
t33 = cp.*l0.*t20.*t21.*t31;
t34 = cp.*l0.*t20.*t22.*t31;
t35 = cp.*l0.*t21.*t22.*t31;
t39 = t11+t12+t14+t15+t23+t24+t32;
t40 = t11+t13+t14+t16+t23+t25+t32;
t41 = t12+t13+t15+t16+t24+t25+t32;
t36 = -t33;
t37 = -t34;
t38 = -t35;
t42 = cp.*t31.*t39;
t43 = cp.*t31.*t40;
t44 = cp.*t31.*t41;
tt1 = reshape([-t44,t33,t34,t33,-t43,t35,t34,t35,-t42,t44,t36,t37,t36,t43,t38,t37,t38,t42],[3,6]);
end