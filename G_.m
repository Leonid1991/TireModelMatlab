function G_ = G_(e0,e1,e2,e3)
%G_
%    G_ = G_(E0,E1,E2,E3)

%    This function was generated by the Symbolic Math Toolbox version 8.3.
%    11-Sep-2023 07:35:46

t2 = e0.*2.0;
t3 = e1.*2.0;
t4 = e2.*2.0;
t5 = e3.*2.0;
t6 = -t3;
t7 = -t4;
t8 = -t5;
G_ = reshape([t6,t7,t8,t2,t8,t4,t5,t2,t6,t7,t3,t2],[3,4]);