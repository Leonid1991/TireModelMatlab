function tt2 = Jacobian_Force_Viscous(in1,in2,in3,in4,eta)
%Jacobian_Force_Viscous
%    TT2 = Jacobian_Force_Viscous(IN1,IN2,IN3,IN4,ETA)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    22-Jan-2024 19:20:03

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
t2 = e_c1.*v_c1;
t3 = e_c2.*v_c2;
t4 = e_c3.*v_c3;
t5 = e_c1.*v_s1;
t6 = e_s1.*v_c1;
t7 = e_c2.*v_s2;
t8 = e_s2.*v_c2;
t9 = e_c3.*v_s3;
t10 = e_s3.*v_c3;
t11 = e_s1.*v_s1;
t12 = e_s2.*v_s2;
t13 = e_s3.*v_s3;
t14 = e_c1.*2.0;
t15 = e_c2.*2.0;
t16 = e_c3.*2.0;
t17 = e_s1.*2.0;
t18 = e_s2.*2.0;
t19 = e_s3.*2.0;
t20 = -e_s1;
t22 = -e_s2;
t24 = -e_s3;
t26 = -v_s1;
t27 = -v_s2;
t28 = -v_s3;
t21 = -t17;
t23 = -t18;
t25 = -t19;
t29 = -t5;
t30 = -t6;
t31 = -t7;
t32 = -t8;
t33 = -t9;
t34 = -t10;
t35 = e_c1+t20;
t36 = e_c2+t22;
t37 = e_c3+t24;
t38 = t26+v_c1;
t39 = t27+v_c2;
t40 = t28+v_c3;
t41 = t14+t21;
t42 = t15+t23;
t43 = t16+t25;
t44 = t35.^2;
t45 = t36.^2;
t46 = t37.^2;
t68 = t2+t3+t4+t11+t12+t13+t29+t30+t31+t32+t33+t34;
t47 = t44+t45+t46;
t48 = 1.0./t47;
t49 = t48.^2;
t50 = eta.*t44.*t48;
t51 = eta.*t45.*t48;
t52 = eta.*t46.*t48;
t53 = eta.*t35.*t36.*t48;
t54 = eta.*t35.*t37.*t48;
t55 = eta.*t36.*t37.*t48;
t56 = eta.*t35.*t38.*t48;
t57 = eta.*t35.*t39.*t48;
t58 = eta.*t36.*t38.*t48;
t59 = eta.*t35.*t40.*t48;
t60 = eta.*t36.*t39.*t48;
t61 = eta.*t37.*t38.*t48;
t62 = eta.*t36.*t40.*t48;
t63 = eta.*t37.*t39.*t48;
t64 = eta.*t37.*t40.*t48;
t69 = eta.*t48.*t68;
t65 = -t53;
t66 = -t54;
t67 = -t55;
t70 = -t69;
t71 = eta.*t35.*t41.*t49.*t68;
t72 = eta.*t35.*t42.*t49.*t68;
t73 = eta.*t36.*t41.*t49.*t68;
t74 = eta.*t35.*t43.*t49.*t68;
t75 = eta.*t36.*t42.*t49.*t68;
t76 = eta.*t37.*t41.*t49.*t68;
t77 = eta.*t36.*t43.*t49.*t68;
t78 = eta.*t37.*t42.*t49.*t68;
t79 = eta.*t37.*t43.*t49.*t68;
tt2 = reshape([t56+t69-t71,t58-t73,t61-t76,t57-t72,t60+t69-t75,t63-t78,t59-t74,t62-t77,t64+t69-t79,t50,t53,t54,t53,t51,t55,t54,t55,t52,-t56+t70+t71,-t58+t73,-t61+t76,-t57+t72,-t60+t70+t75,-t63+t78,-t59+t74,-t62+t77,-t64+t70+t79,-t50,t65,t66,t65,-t51,t67,t66,t67,-t52],[3,12]);
end
