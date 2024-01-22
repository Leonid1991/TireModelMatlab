function [t,y]=SolMex(tspan, y0, parameters)
       options = odeset('RelTol',1e-3,'AbsTol',1e-4,'Stats','off');
       [t,y] = ode45(@eomTire4, tspan, y0, options, parameters);
end
