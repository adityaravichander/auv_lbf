
% Trajectory planning and Tracking control of underactuated AUV

% Constants
global m
global onebym 
global d
mass= 185.0;                    % Mass kg
Iz = 50.0;                      % Rotational inertia kg-m^2
Xu = -30.0;                     % added mass kg
Yv = -90.0;                     % added mass kg
Nr = -30.0;                     % added mass kg
m = [(mass-Xu), (mass-Yv), (Iz-Nr)]; % combined inertia and added mass terms
onebym = [1.0/(mass-Xu), 1.0/(mass-Yv), 1.0/(Iz-Nr)]; % one by combined inertia and added mass terms
d = [70.0, 100.0, 50.0];            % linear drag [surge, sway, yaw]
j = 300;

% AUV values 
auv = zeros(6, j);   % [ x, y, psi, u, v, r ] Auv values in global frame
auv(:, 1) = [10.5, 5.0, 0, 0, 0, 0];
%disp('Auv');
%disp(auv(:, 1)); 
global Fin             
Fin = [0; 0];          % Force input

% PATH values
Reqd = zeros(10, j);  % [ x_req, y_req, psi_req, u_req, v_req, r_req, xdot_req, ydot_req, X_doubledotreq, Y_doubledotreq ] Required values in global frame
Reqd(:,1) = [ 0, 10, 0, 0.001, 0, 0, 0.1, 0, 0, -0.001 ];

nu = 0; n = 0;
vp = sqrt((Reqd(7,1)^2) + (Reqd(8,1)^2));
vp_dot = ((Reqd(7,1)*Reqd(9,1)) + (Reqd(8,1)*Reqd(10,1)))/vp;  
beta = angwrapfn(atan2(Reqd(8,1),Reqd(7,1)));  % beta in range ( 0 to 6.28 )        '
beta_dot = ((Reqd(7,1)*Reqd(10,1)) - (Reqd(8,1)*Reqd(9,1)))/(vp*vp);   
global ureq_dot;
ureq_dot = 0;
[t3,ur] = ode45('auv_surge',[nu nu+0.0059],Reqd(4,1));    
Reqd(4,1) = ur(end,1);
nu = nu + 0.0059;
Error = zeros(6, j); % Error values [ x_error, y_error, psi_error, u_error, v_error, r_error ] 

%% Path Tracking

for i = 1:j
    disp(i);

    % Required Velocity
    
    if(beta_dot<=0)    
         Reqd(5,i) = sqrt( abs((vp^2) - (Reqd(4,i)^2)) );
         Reqd(3,i) = beta - angwrapfn(atan2(Reqd(5,i),Reqd(4,i)));
         Reqd(6,i) = beta_dot - ((Reqd(4,i)*vp_dot - ureq_dot*vp)/(vp*Reqd(5,i)));
    else
         Reqd(5,i) = - sqrt( abs((vp^2) - (Reqd(4,i)^2)) );
         Reqd(3,i) = beta + angwrapfn(atan2(Reqd(5,i),Reqd(4,i)));
         Reqd(6,i) = beta_dot + ( Reqd(4,i)*vp_dot - ureq_dot*vp ) / ( vp*Reqd(5,i) ) ;
    end

    % Required Force and Torque
    ts = 1;                              % sample time
    if(i==1)
        u_dotreq = (Reqd(4,i)/ts);        % u_dot required 
        r_dotreq = (Reqd(6,i)/ts);        % r_dot required
    else
        u_dotreq = ((Reqd(4,i) - Reqd(4,i-1))/ts); % u_dot required
        r_dotreq = ((Reqd(6,i) - Reqd(6,i-1))/ts); % r_dot required
    end
    Freq = [ (m(1)*u_dotreq)  + (d(1)*Reqd(4,i)) - (m(2)*Reqd(5,i)*Reqd(6,i));
             (m(3)*r_dotreq)  + (d(3)*Reqd(6,i)) - ((m(2)-m(1))*Reqd(4,i)*Reqd(5,i)) ];    
    
    Error(1:6,i) = auv(1:6,i) - Reqd(1:6,i); % Error
    
    % Display
    disp('reqd');
    disp(Reqd(:,i));
    %disp('error');
    %disp(Error(:,i))

    % Tuning Parameters
    Zu = 0;
    Zv = 0;
    Zr = 0;
    
    k = 0.5;
    cu = 0.5;
    
    k1 = 0.7;
    ku = 0.7;
    
    cr = 0.1;
    c = 0.6;
    kc = 0.6;
    
    % Backstepping Control action 
    Tu = -(k+k1)*Error(4,i) - cu*Zu - ku*Zu*(k+k1)*(k+k1);    
    Tr = -Zv - Error(3,i) - cr*(((m(1)*auv(4,i)*onebym(2))^2)+1)*Error(6,i) - (c + kc*(cr*m(1)*auv(4,i)*onebym(2))*(cr*m(1)*auv(4,i)*onebym(2)))*Zr;
    
    % Updating Input of AUV ( Force and Torque )
    Fin = [ Freq(1) + m(1)*Tu + (d(1)*Error(4,i)) - ( m(2)*(Error(5,i)*(Error(6,i)+Reqd(6,i)) + Reqd(5,i)*Error(6,i)) );      
            Freq(2) + m(3)*Tr + (d(3)*Error(6,i)) + ( (m(2)-m(1)) * ( Error(4,i)*(Error(5,i)+Reqd(5,i)) + Reqd(4,i)*Error(5,i) ) ) ];
    
    % ODE solver to update auv values
    V0 = auv(:,i);                             % input to ODE45
    [t1, p] = ode45('Trackfn', [0 ts], V0);    % ODE45 
    auv(:,i+1) = p(end,:);                     % update auv parameters            
    auv(3,i+1) = angwrapfn(auv(3,i+1));        % psi in ( 0 to 6.28 )

    % Plots
    figure(1)
    plot(auv(1,1:i), auv(2,1:i), 'b');   % plots auv's path
    hold on
    plot(Reqd(1,1:i),Reqd(2,1:i), 'r');  % plots required trajectory
    
    % Display 
    disp('Fin');
    disp(Fin);
    disp('Auv');
    disp(auv(1:6,i+1));
    
    % Update Trajectory Values
    V1 = [ Reqd(1,i), Reqd(2,i), Reqd(7,i), Reqd(8,i), Reqd(9,i), Reqd(10,i) ];
    [t2,q] = ode45('auv_Track_trajectory',[n n+0.15],V1);    
    Reqd(1:2,i+1) = real(q(end,1:2));
    Reqd(7:10,i+1) = real(q(end,3:6));
    n = n + 0.15;  
    
    % Update Required Surge Velocity
    beta = angwrapfn(atan2(Reqd(8,i+1),Reqd(7,i+1)));  % beta in range ( 0 to 6.28 )        '
    beta_dot = ((Reqd(7,i+1)*Reqd(10,i+1)) - (Reqd(8,i+1)*Reqd(9,i+1)))/(vp*vp); 
    ureq_dot = ( (d(2)*Reqd(4,i)*Reqd(4,i)*vp) - d(2)*vp*vp*vp - (m(1)*beta_dot*Reqd(4,i)*vp*Reqd(5,i)) + (m(1)*Reqd(4,i)*Reqd(4,i)*vp_dot) - (m(2)*vp_dot*vp*vp) )/ (Reqd(4,i)*vp*(m(1)-m(2))); 
    [t3,ur] = ode45('auv_surge',[nu nu+0.0059],Reqd(4,i));    
    Reqd(4,i+1) = ur(end,1);
    nu = nu + 0.0059;

end
