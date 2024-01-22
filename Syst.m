function [M,F,Node_F]=Syst(y,parameters)
e0 = y(4);
e1 = y(5);
e2 = y(6);
e3 = y(7);

de0 = y(11);
de1 = y(12);
de2 = y(13);
de3 = y(14);

A = Af(e0,e1,e2,e3);
dA=dAf(de0,de1,de2,de3,e0,e1,e2,e3);

m = parameters{1}(1);
g = parameters{1}(2);

Node= parameters{1}(6);
level= parameters{1}(7);
mu= parameters{1}(8);
pn= parameters{1}(9);
torsion=parameters{1}(10);
R=parameters{1}(11);
p=parameters{1}(12);
angle_c=parameters{1}(15);

wheel_Rim=parameters{2};
F_mass_R =parameters{3};

kx= parameters{5};
eta=parameters{6};
Poind_data_arr=parameters{7};
angle_l=parameters{8};
t_lat = parameters{9};
t_cir = parameters{10};

F=[0 0 -m*g]';
M=[0 torsion 0]';
[F_struc_R,F_rim1,M_rim1]=Force_struc(A,dA,Node,level,Poind_data_arr,wheel_Rim,y,kx(1:3),eta(1:3)); 
[F_bend_R,F_rim2,M_rim2]=Force_bending(A,dA,Node,level,wheel_Rim,y,kx(4),eta(4),angle_c,t_cir,angle_l,t_lat); 
F_pressure = Force_pres_fun(A,Node,level,wheel_Rim,y,p); % Farroni2019 copy
%% Contact forces
% forces applied to the rim from ground
% if y(3)<R
%    for jj=1:level
%        for kk=1:Node
%            [~,~,index]=position_from_y(jj,kk,Node,level,y);
% 
%            rr=wheel_Rim(index);% point of the rim          
%            pos= y(1:3)+  A*rr; % position
%            vel= y(8:10)+dA*rr; % velocity 
% 
%            F_cont=F_cont_point(pos,vel,mu,pn);
%            M=M+skew(A*rr)*F_cont;
%            F=F+F_cont;
%        end
%    end    
% end
F_cont_R=zeros(3*Node*level,1);
% for jj=1:level
%     for kk=1:Node      
%         [pos,vel,index] = position_from_y(jj,kk,Node,level,y);
%         F_cont_R(index) = F_cont_point(pos,vel,mu,pn);              
%     end    
% end   
%% Assemblance of all forces

Node_F=F_struc_R  + F_cont_R + F_mass_R + F_pressure  + F_bend_R;
% norm(F_bend_cir)

F=F+F_rim1 + F_rim2;
M=M+M_rim1 + M_rim2;