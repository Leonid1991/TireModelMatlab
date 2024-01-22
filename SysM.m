function sysM = SysM(Ix_,Iy_,Iz_,e0,e1,e2,e3,m)
%SYSM
%    SYSM = SYSM(IX_,IY_,IZ_,E0,E1,E2,E3,M)

%    This function was generated by the Symbolic Math Toolbox version 8.3.
%    11-Sep-2023 07:35:45

t2 = e0.^2;
t3 = e1.^2;
t4 = e2.^2;
t5 = e3.^2;
t6 = Ix_.*e0.*e1.*4.0;
t7 = Ix_.*e0.*e2.*4.0;
t8 = Ix_.*e0.*e3.*4.0;
t9 = Ix_.*e1.*e2.*4.0;
t10 = Ix_.*e1.*e3.*4.0;
t11 = Ix_.*e2.*e3.*4.0;
t12 = Iy_.*e0.*e1.*4.0;
t13 = Iy_.*e0.*e2.*4.0;
t14 = Iy_.*e0.*e3.*4.0;
t15 = Iy_.*e1.*e2.*4.0;
t16 = Iy_.*e1.*e3.*4.0;
t17 = Iy_.*e2.*e3.*4.0;
t18 = Iz_.*e0.*e1.*4.0;
t19 = Iz_.*e0.*e2.*4.0;
t20 = Iz_.*e0.*e3.*4.0;
t21 = Iz_.*e1.*e2.*4.0;
t22 = Iz_.*e1.*e3.*4.0;
t23 = Iz_.*e2.*e3.*4.0;
t24 = -t6;
t25 = -t7;
t26 = -t10;
t27 = -t11;
t28 = -t13;
t29 = -t14;
t30 = -t15;
t31 = -t16;
t32 = -t18;
t33 = -t20;
t34 = -t21;
t35 = -t23;
t36 = t12+t27+t32;
t37 = t19+t25+t31;
t38 = t8+t29+t34;
t39 = t9+t30+t33;
t40 = t22+t26+t28;
t41 = t17+t24+t35;
sysM = reshape([m,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,m,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,m,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Ix_.*t3.*4.0+Iy_.*t4.*4.0+Iz_.*t5.*4.0,t41,t40,t39,e0,0.0,0.0,0.0,t41,Ix_.*t2.*4.0+Iy_.*t5.*4.0+Iz_.*t4.*4.0,t38,t37,e1,0.0,0.0,0.0,t40,t38,Ix_.*t5.*4.0+Iy_.*t2.*4.0+Iz_.*t3.*4.0,t36,e2,0.0,0.0,0.0,t39,t37,t36,Ix_.*t4.*4.0+Iy_.*t3.*4.0+Iz_.*t2.*4.0,e3,0.0,0.0,0.0,e0,e1,e2,e3,0.0],[8,8]);
