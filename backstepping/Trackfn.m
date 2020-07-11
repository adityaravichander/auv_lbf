function dpdt = Trackfn(t,p)
    
    % Global variables
    global Fin  % Force input
    global d    % linear drag coefficient
    global m    % combined inertia and added mass terms
    global onebym 
    
    % ODE solver 
    dpdt = zeros(6,1);   
 
    dpdt(4) = ((m(2)*p(5)*p(6))*onebym(1)) + ((-d(1)*p(4))*onebym(1)) + (Fin(1)*onebym(1));        % to find u

    dpdt(5) = (-(m(1)*p(4)*p(6))*onebym(2)) + ((-d(2)*p(5))*onebym(2));                            % to find v

    dpdt(6) = (((m(1)-m(2))*p(4)*p(5))*onebym(3)) + ((-d(3)*p(6))*onebym(3)) + (Fin(2)*onebym(3)); % to find r

    dpdt(3) = (p(6));                                                                              % to find psi

    dpdt(1) = (p(4)*cos(p(3))) - (p(5)*sin(p(3)));                                                 % to find x

    dpdt(2) = (p(4)*sin(p(3))) + (p(5)*cos(p(3)));                                                 % to find y
    
    