function [F_p,F_n]=Force_bend(A,t0,pos_p,pos_c,pos_n,a0,kx)
        %% derivation
        F_p = zeros(3,1);
        F_n = zeros(3,1);
        dir_p = pos_c - pos_p;   
        dir_n = pos_n - pos_c;
        length_p = norm(dir_p);
        length_n= norm(dir_n);

        dir_p = dir_p / length_p;
        dir_n = dir_n / length_n;
        
        t = cross(dir_p, dir_n);
        a = Angle_cos(pos_p,pos_c,pos_n);

        if (abs(a - a0) > 1.0e-3)
            if norm(t) < 1.0e-3
                t = A * t0;
            end    
            F_p = kx * (a - a0) / length_p * cross(t,dir_p);
            F_n = kx * (a - a0) / length_n * cross(t,dir_n);
        end


   