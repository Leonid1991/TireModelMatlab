function [Force_node,F_rim,M_rim]=Force_bending(A,dA,Node,level,wheel_Rim,y,kx,eta,angle_c,t_cir,angle_l,t_lat) 


F_bend_cir = Force_benging_cir(A,Node,level,y,kx,angle_c,t_cir);
[F_bend_lat,F_rim,M_rim] = Force_benging_lat(A,Node,level,y,kx,wheel_Rim,angle_l,t_lat); 


Force_node = F_bend_cir + F_bend_lat;
