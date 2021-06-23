function [ ul, theta_l, psi_l ] = sphericalfn(w,v,u,psi,theta)

    ul = sqrt(u^2 + v^2 + w^2);
    theta_l = asin((sqrt(u^2+w^2)/ul)*(sin(theta + atan(-w/u))));
    psi_l = psi + atan(u/u*cos(theta));
end 