function dydt = eomTire4(t,y,parameters)
t

e0 = y(4);
e1 = y(5);
e2 = y(6);
e3 = y(7);

de0 = y(11);
de1 = y(12);
de2 = y(13);
de3 = y(14);

m = parameters{1}(1);
Ix_ = parameters{1}(3);
Iy_ = parameters{1}(4);
Iz_ = parameters{1}(5);
Node= parameters{1}(6);
level= parameters{1}(7);
tst=parameters{1}(13);
Fz= parameters{1}(14);

Tir_M=parameters{4};
%============================ solution ====================================
if t<=tst
    [M,F,Node_F]=Syst(y,parameters);
    Rim_M = SysM(Ix_,Iy_,Iz_,e0,e1,e2,e3,m);
    Rim_F = SysF(F(1),F(2),F(3),Ix_,Iy_,Iz_,M(1),M(2),M(3),de0,de1,de2,de3,e0,e1,e2,e3);
    
    conts=[0 0 1 0 0 0 0 0];
    sys_M1=[Rim_M conts';
            conts     0];
    EOM1 = sys_M1\[Rim_F; 0];
    EOM2 = Tir_M*Node_F;
else
    [M,F,Node_F]=Syst(y,parameters);
    F = F + Fz*[0 0 1]';
    Rim_M = SysM(Ix_,Iy_,Iz_,e0,e1,e2,e3,m);
    Rim_F = SysF(F(1),F(2),F(3),Ix_,Iy_,Iz_,M(1),M(2),M(3),de0,de1,de2,de3,e0,e1,e2,e3);
   
    sys_M1=Rim_M;        
    EOM1 = sys_M1\Rim_F;
    EOM2 = Tir_M*Node_F; 
end    
%============================ solution ====================================
dydt=zeros(14+2*3*Node*level,1);
dydt(1:7)=y(8:14); 
dydt(8:14) = EOM1(1:7);
dydt(14+1:14+3*Node*level)=y(14+3*Node*level+1:14+3*Node*level+3*Node*level); 
dydt(14+3*Node*level+1:14+3*Node*level+3*Node*level)=EOM2;
dydt = dydt(:);
end