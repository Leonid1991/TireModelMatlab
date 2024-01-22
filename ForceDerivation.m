clc, clear, close all;
clear variables
%% Linear spring
syms l0 real; % initial length 
syms cp eta real; % stiffness and damping of the spring
% i --> j
%% Node under consideration
e_c = sym('e_c', [3 1], 'real'); % position
v_c = sym('v_c', [3 1], 'real'); % speed 
%% "Side" node, which is connected to the considered node  
e_s = sym('e_s', [3 1], 'real'); % position
v_s = sym('v_s', [3 1], 'real'); % speed
%% Relations
direction   = e_s-e_c;  % direction vector from the considered node to the "side node"
l=norm(direction);      % current distance between nodes
n = direction/l;        % normalize direction vector  
v_relative  = v_s-v_c;  % relative velocity of the side node (where it goes from considered node)
v_n=dot(v_relative, n); % relative velocity in the chosen direction
%% Standard
Force_el=  - cp * (l-l0) * n; % elastic force calculation 
Force_vi= - eta * v_n * n ;   % viscous force calculation 
%%  Functions
matlabFunction(Force_el,'file','Force_Spring','vars',{e_c,e_s,cp,l0}); % forming function
matlabFunction(Force_vi,'file','Force_Viscous','vars',{e_c,v_c,e_s,v_s,eta}); % forming function
Force= - ( cp * (l-l0) + eta * v_n ) * n; % force calculation "linear standart way"
matlabFunction(Force,'file','ForceSpring','vars',{e_c,v_c,e_s,v_s,cp,l0,eta}); % forming function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc, clear, close all;
clear variables
%% Nodes under consideration
%       P_N
%        A
%        | 
%  P_W---|--->P_E 
%        |
%        |
%       P_S
P_N = sym('P_N', [3 1], 'real'); 
P_S = sym('P_S', [3 1], 'real'); 
P_W = sym('P_W', [3 1], 'real'); 
P_E = sym('P_E', [3 1], 'real'); 
%% Air pressure
syms press real; % pressure
%% derivation
n = cross(P_N - P_S,P_E - P_W);
Force = simplify(0.5 * press * n);
matlabFunction(Force,'file','ForcePress','vars',{P_N,P_S,P_E,P_W,press}); % forming function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc, clear, close all;
clear variables
%% Nodes under consideration
% n <-- c <-- p
pos_p = sym('pos_p', [3 1], 'real'); 
pos_c = sym('pos_c', [3 1], 'real'); 
pos_n = sym('pos_n', [3 1], 'real'); 

t = sym('t', [3 1], 'real'); 
%% Bending spring
syms a0 a real;  % initial angle
syms k_b real; % bending stiffness 
%% derivation
dir_p = pos_c - pos_p;   
dir_n = pos_n - pos_c;

F_p = simplify( k_b * (a - a0) / norm(dir_p)^2 * cross(t,dir_p) ); % square , because dir_p isn't normed
F_n = simplify( k_b * (a - a0) / norm(dir_p)^2 * cross(t,dir_n) );

matlabFunction(F_p,'file','F_p_bend','vars',{a,t,pos_p,pos_c,pos_n,a0,k_b}); % forming function
matlabFunction(F_n,'file','F_n_bend','vars',{a,t,pos_p,pos_c,pos_n,a0,k_b}); % forming function
