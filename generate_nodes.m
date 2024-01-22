function [y0,wheel_Rim,level,angle_l,angle_c,t_cir,t_lat] = generate_nodes(pos,phi_,vel,w_,Node,R,Rt,bt)
level = length(bt);   % total number of layers/levels
%% Rim
y0 = zeros(14,1);            % rim DoFs
y0(1) =  pos(1);
y0(2) =  pos(2);
y0(3) =  pos(3);        
A=A_fun(phi_);              % rotational matrix
% recalculation of rotational parameters              
y0(4)=sqrt(1+trace(A))/2;
y0(5)=sqrt(1+2*A(1,1)-trace(A))/2;
y0(6)=sqrt(1+2*A(2,2)-trace(A))/2;
y0(7)=sqrt(1+2*A(3,3)-trace(A))/2;

y0(8:10)=vel;
y0(11:14)=pinv(G_(y0(4),y0(5),y0(6),y0(7)))*w_;
%% Nodes DoFs
y0=[y0;zeros(3*Node*level,1)]; % adding position
y0=[y0;zeros(3*Node*level,1)]; % adding velocities, necessary to start position_from_y function
%% "nodes" of the Rim
wheel_Rim=zeros(3*Node*2,1);
for jj=1:level
    for kk=1:Node 
        %% Attention here 
        tetta=2*pi*(kk-1)/Node;  % node circumferential distribution
        %%
        [~,~,index]=position_from_y(jj,kk,Node,level,y0);
        wheel_Rim(index)=[R*cos(tetta) bt(jj) R*sin(tetta)]'; % it is local coordinates
        y0(14+index)=y0(1:3)+A*[Rt(jj)*cos(tetta) bt(jj) Rt(jj)*sin(tetta)]';  % it is in global 
    end    
end
%% Bending angles in lateral direction    
angle_l = zeros(level,1);
for jj=1:level % tire lateral nodes
    [Point_c,~,index] = position_from_y(jj,1,Node,level,y0); % current node 
    
    jl=jj-1;     
    if jl~=0   % if we have layer from the left (leftside jj - jl)               
       [Point_l,~,~] = position_from_y(jl,1,Node,level,y0); % current node 
    else % jj==1
       Point_l= y0(1:3) + A*wheel_Rim(index); % rim corresponded point          
    end

    jr=jj+1;
    if jr~=level+1 % if we have layer from the right (rightside jj - jr)             
       [Point_r,~,~] = position_from_y(jr,1,Node,level,y0); % current node 
    else % if jr==level+1 
       Point_r= y0(1:3) + A*wheel_Rim(index); 
    end 
    angle_l(jj) = Angle_cos(Point_l,Point_c,Point_r);
end
%% Bending angles in circumferential direction    
angle_c = pi - 2*pi/Node;
%% vectors t0
t_cir = [0 -1 0]';
t_lat = zeros(3,Node);
for kk=1 : Node % tire lateral nodes   
    tetta=2*pi*(kk-1)/Node;  % node circumferential distribution
    t_lat(:,kk) = [-sin(tetta) 0 cos(tetta)]';
end