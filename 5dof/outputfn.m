
function [ x_dot, y_dot, z_dot, theta_dot, psi_dot, u_dot, v_dot, w_dot, q_dot, r_dot ] = outputfn(Tu, Tq, Tr, theta, psi, u, v, w, q, r)

global m
global onebym

onebym = 1/m;

fu = ((m(2)*v*r*onebym(1));

fv = (-(m(1)*u*r*onebym(2)); 

fr = ((m(1)-m(2))*u*v*onebym(3)); 

x_dot = u*cos(theta)*cos(psi) - v*sin(psi) + w*sin(theta)*cos(psi);
    
y_dot = u*cos(theta)*sin(psi) + v*cos(psi) + w*sin(theta)*sin(psi);

z_dot = -u*sin(theta) + w*cos(theta);

theta_dot = q;

psi_dot = r*sec(theta);

u_dot = fu + bu*Tu + du;

v_dot = fv + dv;

w_dot = fw + dw;

q_dot = fq + bq*Tq + dq;

r_dot = fr + br*Tr + dr; 

end
