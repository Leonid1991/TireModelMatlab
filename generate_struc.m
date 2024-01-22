function generate_struc(Node,level,nam_l,Poind_data_arr,k_eta_d,wheel_Rim)
%% what we need to calculate
syms ns nd real;
Force_node = sym('Force_node', [3*Node*level 1], 'real'); % structural forces for nodes
F=zeros(3,1); % applied force  to rim
M=zeros(3,1); % applied moment to rim
%% what we have 
y = sym('y', [14+3*Node*level+3*Node*level 1], 'real'); % arbirtrary position of the system
%y=y0;
eta_ex = sym('eta_ex', [3 1], 'real'); % visco properties
k_ex   = sym('k_ex', [3 1], 'real');   % elastic properties
A = Af(y(4),y(5),y(6),y(7)); % rotation matrix
dA=dAf(y(11),y(12),y(13),y(14),y(4),y(5),y(6),y(7)); % rotation velocity matrix
%% derivation
for jj=1:level
    l_cir = Poind_data_arr(jj,1);  % springs initial length in circumferential direction 
    l_left = Poind_data_arr(jj,2); % springs initial length in left direction
    l_right = Poind_data_arr(jj,3);% springs initial length in right direction 

    k_cir = k_ex( nam_l(jj) );            % elastic prop. in circumferential direction
    eta_cir = eta_ex( nam_l(jj) );        % visco prop. in circumferential direction
    
    k_left =   k_ex( k_eta_d(jj,1) );
    eta_left = eta_ex( k_eta_d(jj,1) );
    k_right =   k_ex( k_eta_d(jj,2) );
    eta_right = eta_ex( k_eta_d(jj,2) );
    
    jl=jj-1; 
    jr=jj+1;

    for kk=1:Node
        [jj,kk]
        [P_kk_jj,v_kk_jj,index_kk_jj] = position_from_y(jj,kk,Node,level,y); % current node         
        kl=kk-1;    % points "before (leftside of kk - kl)" the current one within the same layer         
        if kl == 0  % such as it is a wheel, we have a cylce  
           kl=Node;       
        end         
        F_kl_jj = force_spring2(jj,kk,jj,kl,y,Node,level,k_cir,l_cir,eta_cir,ns,nd);
        
        kr=kk+1;    % points "after (rightside of kk - kr)" the current one within the same layer        
        if kr==Node+1
           kr=1;
        end          
        F_kr_jj = force_spring2(jj,kk,jj,kr,y,Node,level,k_cir,l_cir,eta_cir,ns,nd);
        if jl~=0   % if we have layer from the left (leftside jj - jl)               
           F_kk_jl=force_spring2(jj,kk,jl,kk,y,Node,level,k_left,l_left,eta_left,ns,nd);           
        else % jj==1
           [~,~,index]= position_from_y(1,kk,Node,level,y); % point data                       
           P_kk_jl= y(1:3)+  A*wheel_Rim(index); % rim corresponded point
           v_kk_jl= y(8:10)+dA*wheel_Rim(index); % velocity           
           F_kk_jl=ForceSpring(P_kk_jj,v_kk_jj,P_kk_jl,v_kk_jl,k_left,l_left,eta_left,ns,nd);
           %F_kk_jl=ForceSpringExpan(P_kk_jj,v_kk_jj,P_kk_jl,v_kk_jl,k_left,l_left,eta_left,ns,nd);
           %F_kk_jl=ForceSpringExpanPower(P_kk_jj,v_kk_jj,P_kk_jl,v_kk_jl,k_left,l_left,eta_left,ns,nd);
           F=F-F_kk_jl;           
           M=M+skew( A*wheel_Rim(index) )*(-F_kk_jl);           
        end                 

        if jr~=level+1 % if we have layer from the right (rightside jj - jr)             
            F_kk_jr=force_spring2(jj,kk,jr,kk,y,Node,level,k_right,l_right,eta_right,ns,nd);  
            %[P_kk_jr,~,~] = position_from_y(jr,kk,Node,level,y); % current node 
        else % if jr==level+1 
           [~,~,index]= position_from_y(level,kk,Node,level,y); % point data 
           
           P_kk_jr= y(1:3)+  A*wheel_Rim(index); 
           v_kk_jr= y(8:10)+dA*wheel_Rim(index); % velocity
           
           
           F_kk_jr=ForceSpring(P_kk_jj,v_kk_jj,P_kk_jr,v_kk_jr,k_right,l_right,eta_right,ns,nd);
           %F_kk_jr=ForceSpringExpan(P_kk_jj,v_kk_jj,P_kk_jr,v_kk_jr,k_right,l_right,eta_right,ns,nd);
           %F_kk_jr=ForceSpringExpanPower(P_kk_jj,v_kk_jj,P_kk_jr,v_kk_jr,k_right,l_right,eta_right,ns,nd);
           F=F-F_kk_jr;   % negative, such as it is force applied to the rim                   
           M=M+skew( A*wheel_Rim(index) )*(-F_kk_jr);        
        end      
    
        %[norm(P_kk_jr-P_kk_jj) l_right Poind_data_arr(jj,3)]
        Force_node(index_kk_jj) =  F_kl_jj + F_kr_jj + F_kk_jl + F_kk_jr; % forces applied to the node
    end
end    
%% Function derivation
matlabFunction(Force_node, 'file', 'Force_node_gen4','vars',{y,k_ex,eta_ex,ns,nd});
matlabFunction(F, 'file', 'F_rim_gen4','vars',{y,k_ex,eta_ex,ns,nd});
matlabFunction(M, 'file', 'M_rim_gen4','vars',{y,k_ex,eta_ex,ns,nd});

