clc, clear, close all;
clear variables
syms l0 real; % initial length 
syms k c real; % stiffness and damping of the spring
% i --> j
% Node under consideration
pos1 = sym('pos1', [3 1], 'real'); % position
vel1 = sym('vel1', [3 1], 'real'); % speed 
% "Side" node, which is connected to the considered node  
pos2 = sym('pos2', [3 1], 'real'); % position
vel2 = sym('vel2', [3 1], 'real'); % speed
%% Relations
direction   = pos2 - pos1;  % direction vector from the considered node to the "side node"
l=norm(direction);      % current distance between nodes
n = direction/l;        % normalize direction vector  
v_relative  = vel2-vel1;  % relative velocity of the side node (where it goes from considered node)
v_n=dot(v_relative, n); % relative velocity in the chosen direction
%% Standard
Force_el=  simplify(- k * (l-l0) * n); % elastic force calculation 
Force_vi= simplify(- c * v_n * n );   % viscous force calculation
%%  Functions
jacobianSpringStiffness = simplify(jacobian(Force_el,[pos1;pos2]));
jacobianSpringDamping   = simplify(jacobian(Force_vi,[pos1;pos2;vel1;vel2]));

jacobianSpringStiffness = ccode(jacobianSpringStiffness);
jacobianSpringDamping = ccode(jacobianSpringDamping);

create_File_for_Chrono(jacobianSpringStiffness,strcat('jacobianSpringStiffness', '.txt'))
create_File_for_Chrono(jacobianSpringDamping,strcat('jacobianSpringDamping', '.txt'))
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% clc, clear, close all;
% clear variables
% %% Nodes under consideration
% %       P_N
% %        A
% %        | 
% %  P_W---|--->P_E 
% %        |
% %        |
% %       P_S
% P_N = sym('P_N', [3 1], 'real'); 
% P_S = sym('P_S', [3 1], 'real'); 
% P_W = sym('P_W', [3 1], 'real'); 
% P_E = sym('P_E', [3 1], 'real'); 
% %% Air pressure
% syms press real; % pressure
% %% derivation
% n = cross(P_N - P_S,P_E - P_W);
% Force = simplify(0.5 * press * n);
% % matlabFunction(Force,'file','ForcePress','vars',{P_N,P_S,P_E,P_W,press}); % forming function
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% clc, clear, close all;
% clear variables
% %% Nodes under consideration
% % n <-- c <-- p
% pos_p = sym('pos_p', [3 1], 'real'); 
% pos_c = sym('pos_c', [3 1], 'real'); 
% pos_n = sym('pos_n', [3 1], 'real'); 
% 
% t = sym('t', [3 1], 'real'); 
% %% Bending spring
% syms a0 a real;  % initial angle
% syms k_b real; % bending stiffness 
% %% derivation
% dir_p = pos_c - pos_p;   
% dir_n = pos_n - pos_c;
% 
% F_p = simplify( k_b * (a - a0) / norm(dir_p)^2 * cross(t,dir_p) ); % square , because dir_p isn't normed
% F_n = simplify( k_b * (a - a0) / norm(dir_p)^2 * cross(t,dir_n) );
% 
% % matlabFunction(F_p,'file','F_p_bend','vars',{a,t,pos_p,pos_c,pos_n,a0,k_b}); % forming function
% % matlabFunction(F_n,'file','F_n_bend','vars',{a,t,pos_p,pos_c,pos_n,a0,k_b}); % forming function
