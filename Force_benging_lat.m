function [Force_node,F_rim,M_rim] = Force_benging_lat(A,Node,level,y,k_lat,wheel_Rim,angle_l,t_lat)
%% what we need to calculate
Force_node = zeros(3*Node*level,1);
F_rim = zeros(3,1);
M_rim = zeros(3,1);
%% Bending forces in lateral direction 
for kk=1:Node
    t = t_lat(:,kk);
    % spring rim - node -node 
    [node_c,~,index_c] = position_from_y(1,kk,Node,level,y); % current node
    [node_n,~,index_n] = position_from_y(2,kk,Node,level,y); % current node        
     node_p = y(1:3) +  A*wheel_Rim(index_c); % rim corresponded point

    [F_p,F_n] = Force_bend(A,t,node_p,node_c,node_n,angle_l(1),k_lat);

    F_rim = F_rim + F_p ; % rim obtains forces, there is no tire points
    M_rim=M_rim+skew( node_p - y(1:3) )*(F_p);

    Force_node(index_c) = Force_node(index_c) - (F_p + F_n);
    Force_node(index_n) = Force_node(index_n) + F_n;

    % spring node-node-node
    for jj=2:level-1 % position of cenral node
        [node_p,~,index_p] = position_from_y(jj-1,kk,Node,level,y); % current node     
        [node_c,~,index_c] = position_from_y(  jj,kk,Node,level,y); % current node
        [node_n,~,index_n] = position_from_y(jj+1,kk,Node,level,y); % current node

        [F_p,F_n] = Force_bend(A,t,node_p,node_c,node_n,angle_l(jj),k_lat);

        Force_node(index_p) = Force_node(index_p) + F_p;
        Force_node(index_c) = Force_node(index_c) - (F_p + F_n);
        Force_node(index_n) = Force_node(index_n) + F_n;
    end

    % spring node-node-rim
    [node_n,~,index_n] = position_from_y(level-1,kk,Node,level,y); % current node 
    [node_c,~,index_c] = position_from_y(level,kk,Node,level,y); % current node            
     node_p = y(1:3)+  A*wheel_Rim(index_c); % rim corresponded point

    [F_p,F_n] = Force_bend(A,-t,node_p,node_c,node_n,angle_l(1),k_lat);


    Force_node(index_n) = Force_node(index_n) + F_n;
    Force_node(index_c) = Force_node(index_c) - (F_p + F_n);
    F_rim = F_rim + F_p ; % rim obtains forces, there is no tire points
    M_rim=M_rim+skew(node_p - y(1:3))*(F_p);
end    