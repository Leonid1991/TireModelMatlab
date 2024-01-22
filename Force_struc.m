function [Force_node,F,M]=Force_struc(A,dA,Node,level,Poind_data_arr,wheel_Rim,y,k_ex,eta_ex)

Force_node = zeros(3*Node*level,1);
F=zeros(3,1); % applied force  to rim
M=zeros(3,1); % applied moment to rim
%% Create circumferential springs
for jj=1:level
    l_cir = Poind_data_arr(jj,1);  % springs initial length in circumferential direction
    for kk=1:Node

        kl = kk + 1;          
        if kl==Node+1
               kl=1;
        end   

        [P_no1,v_no1,index_nod1] = position_from_y(jj,kk,Node,level,y); % current node   
        [P_no2,v_no2,index_nod2] = position_from_y(jj,kl,Node,level,y); % next node   
     
        force = Force_Spring(P_no1,P_no2,k_ex(3),l_cir) + Force_Viscous(P_no1,v_no1,P_no2,v_no2,eta_ex(3));
       
        Force_node(index_nod1) = Force_node(index_nod1) - force;
        Force_node(index_nod2) = Force_node(index_nod2) + force;         
    end
end
%% Create transversal spring
for kk=1:Node

    % Radial spring connected to the first ring
    [P_no1,v_no1,index_nod1] = position_from_y(1,kk,Node,level,y); % current node
    P_no2= y(1:3) + A*wheel_Rim(index_nod1); % rim corresponded point
    v_no2= y(8:10) + dA*wheel_Rim(index_nod1); % velocity
    l_left = Poind_data_arr(1,2);    
    force = Force_Spring(P_no1,P_no2,k_ex(1),l_left) + Force_Viscous(P_no1,v_no1,P_no2,v_no2,eta_ex(1));   % kx(1) is a redial springs     

    Force_node(index_nod1) = Force_node(index_nod1) - force;   

    F=F + force;           
    M=M+skew( A*wheel_Rim(index_nod1) ) * force;  

    % node-node springs    
    for jj=2:level  % jl - jj 
        l_lat = Poind_data_arr(jj,2); 
        [P_no1,v_no1,index_nod1] = position_from_y(jj  ,kk,Node,level,y); % current node
        [P_no2,v_no2,index_nod2] = position_from_y(jj-1,kk,Node,level,y); % current node
        force = Force_Spring(P_no1,P_no2,k_ex(2),l_lat) + Force_Viscous(P_no1,v_no1,P_no2,v_no2,eta_ex(2));            

        Force_node(index_nod1) = Force_node(index_nod1) - force; 
        Force_node(index_nod2) = Force_node(index_nod2) + force; 
    end

    % Radial spring connected to the first ring
    [P_no1,v_no1,index_nod1] = position_from_y(level,kk,Node,level,y); % current node
    P_no2= y(1:3)+  A*wheel_Rim(index_nod1); % rim corresponded point
    v_no2= y(8:10)+dA*wheel_Rim(index_nod1); % velocity
    l_right = Poind_data_arr(1,2);    
    force = Force_Spring(P_no1,P_no2,k_ex(1),l_right) + Force_Viscous(P_no1,v_no1,P_no2,v_no2,eta_ex(1));        

    Force_node(index_nod1) = Force_node(index_nod1) - force;    
    F=F + force;           
    M=M+skew( A*wheel_Rim(index_nod1) ) * force;  
end    
       

