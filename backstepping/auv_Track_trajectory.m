function dqdt = auv_Track_trajectory(t2,q)

    %disp('reqd');
    %r = [q(1),q(2),q(3),q(4),q(5),q(6)];
    %disp(r);
    
    a=10;
    b=0.01;
    
    dqdt = zeros(6,1); 
    dqdt(1) =      (a*b*cos(b*t2));   %to find xreq
    dqdt(2) =     -(a*b*sin(b*t2));	  %to find yreq
    dqdt(3) =   -(a*b*b*sin(b*t2));   %to find x_dotreq
    dqdt(4) =   -(a*b*b*cos(b*t2));   %to find y_dotreq
    dpdt(5) = -(a*b*b*b*cos(b*t2));   %to find X_doubledotreq
    dpdt(6) = +(a*b*b*b*sin(b*t2));   %to find Y_doubledotreq
    
    
    
    % dqdt(1) = 5;       %to find xreq
    % dqdt(2) = 10;      %to find yreq
    % dqdt(3) = 0;       %to find x_dotreq
    % dqdt(4) = 0;       %to find y_dotreq
    %xr = 10cos(0.5t)
    %yr = 10sin(0.5t)
    %xr = 5t