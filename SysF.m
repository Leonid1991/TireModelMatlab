function sysF = SysF(F1,F2,F3,Ix_,Iy_,Iz_,T1,T2,T3,de0,de1,de2,de3,e0,e1,e2,e3)
%SYSF
%    SYSF = SYSF(F1,F2,F3,IX_,IY_,IZ_,T1,T2,T3,DE0,DE1,DE2,DE3,E0,E1,E2,E3)

%    This function was generated by the Symbolic Math Toolbox version 8.3.
%    11-Sep-2023 07:35:46

t2 = de0.*e1.*2.0;
t3 = de1.*e0.*2.0;
t4 = de0.*e2.*2.0;
t5 = de2.*e0.*2.0;
t6 = de0.*e3.*2.0;
t7 = de1.*e2.*2.0;
t8 = de2.*e1.*2.0;
t9 = de3.*e0.*2.0;
t10 = de1.*e3.*2.0;
t11 = de3.*e1.*2.0;
t12 = de2.*e3.*2.0;
t13 = de3.*e2.*2.0;
t14 = -t3;
t15 = -t5;
t16 = -t7;
t17 = -t9;
t18 = -t11;
t19 = -t12;
t20 = t2+t13+t14+t19;
t21 = t4+t10+t15+t18;
t22 = t6+t8+t16+t17;
sysF = [F1;F2;F3;T1.*e1.*-2.0-T2.*e2.*2.0-T3.*e3.*2.0-Ix_.*de1.*t20.*4.0-Iy_.*de2.*t21.*4.0-Iz_.*de3.*t22.*4.0;T1.*e0.*2.0+T2.*e3.*2.0-T3.*e2.*2.0+Ix_.*de0.*t20.*4.0-Iy_.*de3.*t21.*4.0+Iz_.*de2.*t22.*4.0;T2.*e0.*2.0-T1.*e3.*2.0+T3.*e1.*2.0+Ix_.*de3.*t20.*4.0+Iy_.*de0.*t21.*4.0-Iz_.*de1.*t22.*4.0;T1.*e2.*2.0-T2.*e1.*2.0+T3.*e0.*2.0-Ix_.*de2.*t20.*4.0+Iy_.*de1.*t21.*4.0+Iz_.*de0.*t22.*4.0;-de0.^2-de1.^2-de2.^2-de3.^2];
