clc, clear, close all
format short 
clear variables
% global variables
p = 1e3;
g = 0*9.81;
pn = 0;
mu = 0;
% ===================== Set editable parameters ===========================
% Tire parameters 
mass = 20;           % rim mass  (unknown)
m_tire = 37.6;    % total tire mass
R = 0.268;        % radius of rim
% Discritization
Node=20;                             % number of nodes
Rt = [0.388 0.465 0.465 0.465 0.388];% tire's raudii  
bt = [-0.15 -0.1 0 0.1 0.15];        % tire's thickness
m_point = m_tire/(length(Rt)*Node);  % mass of the one point
% ================ Define physical properties =============================
kk = 1.0e3;
kR = kk;  % -1
cR = 1.0e2;  % -1
kC = kk;
cC = 1.0e2;  
kT = kk;  % -2
cT = 1.0e2;  % -2
kB = kk;
cB = 0;
kx  = [kR, kT, kC, kB]; % this order is important, cause in k_eta_d only (1,2)
eta = [cC, cT, cC, cB];    
% ======================= Generate bodies =================================
% mass moments of inertia, _ = in local coordinate system
b = bt(end) - bt(1);
Ix_ = mass*(3*R^2 + b^2)/12;       % rotational inertia about x
Iy_ = 0.5*mass*R^2;                % rotational inertia about y
Iz_ = Ix_;                         % rotational inertia about z
pos  = [0; 0; R + max(Rt) + 0.2];  % 0.2 to up the tire 
phi_ = [0; 0; 0];                  % put tire straight 
vel  = [0; 0; 0];                  % no velocity
rot_ = [0; 0; 0];                  % remove rotation
% generate2(); % symbolical formulation
[y0,wheel_Rim,level,angle_l,angle_c,t_cir,t_lat] = generate_nodes(pos,phi_,vel,rot_,Node,R,Rt,bt);
[Poind_data_arr,Tir_M,F_mass_R] = generate_mass(y0,Node,level,g,m_point,wheel_Rim,Rt); 
% present(y0,level,Node,wheel_Rim,nam_l) % presentation of the initial positions
% ======================= System parameters =================================
torsion=0;        % applied torion
Fz=0;             % applied after time = tst 
dt=0.001;
tst=0.1;
tspan = 0:dt:tst;
parameters = [mass, g, Ix_, Iy_, Iz_, Node, level, mu, pn, torsion, R, p, tst, Fz, angle_c];
parameters = [parameters,{wheel_Rim},{F_mass_R},{Tir_M},{kx},{eta},{Poind_data_arr},{angle_l},{t_lat},{t_cir},{y0}];
% ========================= Simulation settings ===========================
% profile on
tic
[t,y]=SolMex(tspan, y0, parameters);
toc
% profile viewer
% %========================== Postprocessing ================================
postprocess2(t, y, y0, parameters, dt,tst,1,0,0,0);