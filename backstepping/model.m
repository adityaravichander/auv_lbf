%% Trajectory planning and Tracking control of underactuated AUV

% Constants
global m
global onebym 
global d

mass = 185.0;                     
Iz   =  50.0;                     
Xu   = -30.0;                     
Yv   = -90.0;                    
Nr   = -30.0;                     

m      = [(mass-Xu), (mass-Yv), (Iz-Nr)]; 
onebym = [1.0/(mass-Xu), 1.0/(mass-Yv), 1.0/(Iz-Nr)];
d      = [70.0, 100.0, 50.0];           
j = 100;

% AUV values 
auv = zeros(6, j);  
auv(:, 1) = [10, 5.0, 0, 0, 0, 0];
disp('Auv');
disp(auv(:, 1)); 

global Fin             
Fin = [0; 0];

%% Path Tracking

for i = 1:j
    ts = 1;
    disp(i);

    % Control input
    Fin(1) = 7.004;    
    Fin(2) = -0.488;
    
    % q(2) + m(3)*Tr + (d(3)*Error(6,i)) + ( (m(2)-m(1)) * ( Error(4,i)*(Error(5,i)+Reqd(5,i)) + Reqd(4,i)*Error(5,i) ) ) ];
    
    % ODE solver to update auv values
    V0 = auv(:,i);                             % input to ODE45
    [t1, p] = ode45('Trackfn', [0 ts], V0);    % ODE45 
    auv(:,i+1) = p(end,:);                     % update auv parameters            
    auv(3,i+1) = angwrapfn(auv(3,i+1));        % psi in ( 0 to 6.28 )

    % Plots
    figure(3)
    plot(auv(1,1:i), auv(2,1:i), 'b');   % plots auv's path
    
    % Display 
    disp('Fin');
    disp(Fin);
    disp('Auv');
    disp(auv(1:6,i+1));
    

end
