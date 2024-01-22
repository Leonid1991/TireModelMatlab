function [Poind_data_arr,Tir_M,F_mass_R] = generate_mass(y0,Node,level,g,m_p,wheel_Rim,Rt)

A = Af(y0(4),y0(5),y0(6),y0(7)); % rotational matrix
F_mass_R = zeros(3*Node*level,1);
M_mass=zeros(3*Node*level,3*Node*level);

Poind_data_arr=zeros(level,2);   % (1 - l_cir, 2 - l_left) % for each point of the level
Poind_data_arr(:,1) = 2*Rt'*sin(pi/Node);
for jj=1:level
    jl=jj-1;
    [P_kk_jj,~,index]= position_from_y(jj,1,Node,level,y0); % current point                
    if jl~=0   % if we have layer from the left (leftside jj - jl)            
       [P_kk_jl,~,~]= position_from_y(jl,1,Node,level,y0); % point data
    else % the left wall node / right is the same       
       P_kk_jl= y0(1:3) + A*wheel_Rim(index); % point on the rim
    end        
    Poind_data_arr(jj,2)=norm(P_kk_jj-P_kk_jl);
end 
% assamblance of mass vector and matrix
for jj=1:level
    for kk=1:Node        
        [~,~,index] = position_from_y(jj,kk,Node,level,y0);
        F_mass_R(index,1)=m_p * [0 0 -g]';  
        M_mass(index,index)=m_p * eye(3);
    end
end
Tir_M=inv(M_mass); % inverse to mass matrix