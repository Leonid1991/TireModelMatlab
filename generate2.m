function generate2( )

% Declaration of symbolic variables 
syms R11 R21 R31 e0 e1 e2 e3 real               % Generalized coordinates
syms dR11 dR21 dR31 de0 de1 de2 de3 real        % Generalized velocities
syms m g h r t real                             % mass, gravity, height,  radius, time, 
syms F1 F2 F3 T1 T2 T3 real                     % force, torque
syms Ix_ Iy_ Iz_ real                           % Inertias at mass center                  

 q = [R11, R21, R31, e0 e1 e2 e3].';
dq = [dR11, dR21, dR31, de0 de1 de2 de3].';

 e = [e1; e2; e3];
 p = [e0; e];
de = [de1; de2; de3];
dp = [de0; de];
A = (2*e0^2-1)*eye(3)+2*(e*e'+e0*skew(e)); % good formula,(see also Johannes LUT lectures (2017)  2.40)
dA = diff(A,q(4))*dq(4)+diff(A,q(5))*dq(5)+diff(A,q(6))*dq(6)+diff(A,q(7))*dq(7);
% Johannes LUT lectures (2017) eq (2.36)
E =[-e1,  e0, -e3,  e2;
    -e2,  e3,  e0, -e1;
    -e3, -e2,  e1,  e0]; 

E_=[-e1,  e0,  e3, -e2;
    -e2, -e3,  e0,  e1;
    -e3,  e2, -e1,  e0];  
% Johannes LUT lectures (2017) eq (2.70)
G =2 * E;
G_=2 * E_;
% Angular velocity of the system
w_ = G_* dp; % Johannes LUT lectures (2017) eq (2.60)


% Mass matrix of the system. 
Mrr  = m * eye(3);
Ixx_ = Ix_;   % _ = in local coordinate system
Iyy_ = Iy_;
Izz_ = Iz_;

Itt_ = [Ixx_,     0,    0;
            0, Iyy_,    0;
            0,    0, Izz_];

% Angular velocity of the system
Mtt = G_' * Itt_ * G_; %(see 2.90)

M(1:3,1:3) = Mrr;
M(4:7,4:7) = Mtt; %if the origin of the body axes is attached to the center of mass of the body, then Mrt is the null matrix.
% Quadratic velocity vector
dG_ =        diff(G_, q(4))*dq(4)...
           + diff(G_, q(5))*dq(5)...
           + diff(G_, q(6))*dq(6)...
           + diff(G_, q(7))*dq(7);   

Qvt = -2 * dG_' * Itt_ * w_; % (see 2.107) Q_v

Qv(1:3,1) = sym(zeros(3,1));
Qv(4:7,1) = Qvt;

% Vector of external forces
Q_trans_  = [F1; F2; F3]; % Q_r (see 2.106)
Q_torque_ = [T1; T2; T3];  
Qe = [Q_trans_; G'*Q_torque_];  % the second part is Q_theta (see 2.113) Johannes LUT lectures

c = dp.'*dp; % (Cq*q_dot)q*q_dot

P = [zeros(1,3), p.']; % p is constrains
sysM = simplify([M, P.';
                 P, 0]);
sysF = simplify([Qe + Qv; % 2.106
    -c]);

matlabFunction(sysM, 'file', 'SysM');
matlabFunction(sysF, 'file', 'SysF');
matlabFunction(A,'file','Af');
matlabFunction(dA,'file','dAf');
matlabFunction(G_,'file','G_');
end

