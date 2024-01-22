function F_cont=F_cont_point(pos,vel,mu,pn)

n=[0 0 1]';
F_cont=[0 0 0]';
if pos(3)<0
   F_n=pn*abs( pos(3) )*n;
   v_t=[vel(1:2);0];% velocity in tangential direction        
   F_t=[0 0 0]';
   if norm(v_t)>1e-4
      tau=v_t/norm(v_t);          % tangental vector
      F_t=-mu*norm(F_n)*tau;            % tangental force
   end
   F_cont=F_n+F_t;      
end