function Force=Force_pres_fun(A,Node,level,wheel_Rim,y,press)

Force=zeros(3*Node*level,1);

for jj=1:level
    for kk=1:Node
        
        [~,~,index]= position_from_y(jj,kk,Node,level,y); % current point
        
        kl=kk-1;    % points "before (leftside of kk - kl)" the current one within the same layer         
        if kl == 0  % such as it is a wheel, we have a cylce  
           kl=Node;       
        end   
        [P_kl_jj,~,~]        = position_from_y(jj,kl,Node,level,y); % back point

        kr=kk+1;    % points "after (rightside of kk - kr)" the current one within the same layer        
        if kr==Node+1
           kr=1;
        end  
        [P_kr_jj,~,~]        = position_from_y(jj,kr,Node,level,y); % forward point 

        jl=jj-1;
        if jl~=0   % if we have layer from the left (leftside jj - jl)            
           [P_kk_jl,~,~]= position_from_y(jl,kk,Node,level,y); % point data
        else
            P_kk_jl= y(1:3) +  A*wheel_Rim(index); 
        end 

        jr=jj+1;
        if jr<level+1 % if we have layer from the right (rightside jj - jr)       
           [P_kk_jr,~,~]= position_from_y(jr,kk,Node,level,y); % point data
        else % if jr==level+1
            P_kk_jr= y(1:3)+  A*wheel_Rim(index); 
        end                

        Force(index) = - ForcePress(P_kr_jj,P_kl_jj,P_kk_jr,P_kk_jl,press);
    end
end    

