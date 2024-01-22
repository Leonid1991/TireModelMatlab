function Force_node = Force_benging_cir(A,Node,level,y,k_cir,a0,t_cir)
Force_node = zeros(3*Node*level,1);
%% Bending forces in circumferential direction (node-node-node)
for jj=1:level 
    for kk=1:Node  % it goes conterclockwise   
        
        kp = kk - 1;          
        if kp==0
           kp=Node;
        end
        
        kn = kk + 1;          
        if kn==Node+1
           kn=1;
        end
        
        [node_p,~,index_p] = position_from_y(jj,kp,Node,level,y); % current node        
        [node_c,~,index_c] = position_from_y(jj,kk,Node,level,y); % current node
        [node_n,~,index_n] = position_from_y(jj,kn,Node,level,y); % current node
        
        [F_p,F_n] = Force_bend(A,t_cir,node_p,node_c,node_n,a0,k_cir);

        Force_node(index_p) = Force_node(index_p) + F_p;
        Force_node(index_c) = Force_node(index_c) - (F_p + F_n);
        Force_node(index_n) = Force_node(index_n) + F_n;
    end     
end    
