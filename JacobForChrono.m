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
Force_el= simplify(- k * (l-l0) * n); % elastic force calculation 
Force_vi= simplify(- c * v_n * n );   % viscous force calculation

% Force = Force_el + Force_vi;






%%  Functions
jacobianSpringStiffness = simplify(jacobian(Force_el,[pos1;pos2]));
jacobianSpringDamping   = simplify(jacobian(Force_vi,[pos1;pos2;vel1;vel2]));
jacobianSpringStiffness = ccode(jacobianSpringStiffness);
jacobianSpringDamping   = ccode(jacobianSpringDamping);
% create_File_for_Chrono(jacobianSpringStiffness,strcat('jacobianSpringStiffness', '.txt'))
% create_File_for_Chrono(jacobianSpringDamping,strcat('jacobianSpringDamping', '.txt'))
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Air pressure
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
% pos_N = sym('pos_N', [3 1], 'real'); 
% pos_S = sym('pos_S', [3 1], 'real'); 
% pos_W = sym('pos_W', [3 1], 'real'); 
% pos_E = sym('pos_E', [3 1], 'real'); 
% syms press real; % pressure
% %% derivation (not to forget to * by 0.5)
% Force = simplify(press * cross(pos_N - pos_S,pos_E - pos_W) );
% jacobianAirPressue = simplify(jacobian(Force,[pos_N; pos_S; pos_E; pos_W]));
% create_File_for_Chrono(ccode(jacobianAirPressue),strcat('jacobianAirPressue', '.txt'))
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc, clear, close all;
clear variables
%% Nodes under consideration
% n <-- c <-- p
pos_p = sym('pos_p', [3 1], 'real'); 
pos_c = sym('pos_c', [3 1], 'real'); 
pos_n = sym('pos_n', [3 1], 'real');
% 
%% Bending spring
syms a0 real;  % initial angle
syms k_b real; % bending stiffness 
%% derivation
dir_p = pos_c - pos_p;   
dir_n = pos_n - pos_c;
t = cross(dir_p , dir_n) / norm( cross(dir_p , dir_n) );
F_p = simplify( k_b * (a - a0) / norm(dir_p)^2 * cross(t,dir_p) ); % square , because dir_p isn't normed
F_n = simplify( k_b * (a - a0) / norm(dir_p)^2 * cross(t,dir_n) );
% 
% jacobianBendingNodeNode_Fp = simplify(jacobian(F_p,[pos_p; pos_c; pos_n]));
% % create_File_for_Chrono(ccode(jacobianAirPressue),strcat('jacobianAirPressue', '.txt'))


n_p = [1  2 0]';
n_n = [1 -2 0]';
acosd(n_n'*n_p)