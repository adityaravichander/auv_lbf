function [Tu, Tq, Tr] = controlfn(xe, ye, ze, uld, theta_ld, psi_ld, ul, theta_l, psi_l, theta, psi, u, v, w, q, r)

   
    
    re = sqrt(xe^2 + ye^2 + ze^2);

    xe_dot = diff(xe)
    ye_dot = diff(ye);
    ze_dot = diff(ze);

    theta_l_D_dot = ( (xe^2 + ye^2)*ze_dot - ze*( xe*xe_dot + ye*ye_dot) ) / ( re^2* ((xe^2 + ye^2)^0.5) );
    psi_l_D_dot = ( xe*ye_dot - ye*xe_dot ) / ( xe^2 + ye^2 );
    theta_a = -atan( -w/ sqrt(u^2 + v^2));
    psi_a = atan(v/u);

    theta_b = atan( ze / sqrt(xe^2 + ye^2) );
    theta_le = theta_b - theta_l;
    alpha = theta + atan(-w/u);
    B = sqrt(u^2 + w^2) / ul; 
    b_theta = (B*cos(alpha))/(sqrt(1 - (B*sin(alpha)^2)));
    F_theta_l = ( B_dot*sin(alpha) + B*cos(alpha)*(u_dot*w - u*w_dot)*(u^2 + w^2) ) / (sqrt(1 - (B*sin(alpha)^2)));

    alpha_q = b_theta^(-1)*{ theta_l_D_dot - F_theta_l + rtheta^(-1)*[ b_theta*theta_le + ((sin(theta_le/2)^2)/(theta_le/2))  ] };
    eq = alpha_q - q;
    % alpha_q_dot = 

    psi_b = atan2(ye,xe);
    psi_le = psi_b - psi_l;
    F_psi_l = [ (u*v_dot - u_dot*v)*cos(theta) + u*v*q*sin(theta) ] / ( (u*cos(theta)^2) + v^2 );
    alpha_r = cos(theta)*{ psi_l_D_dot - F_psi_l + gammapsi^(-1)*[ kpsi*psi_le + ((sin(psi_le/2)^2)/(psi_le/2))  ] } - r;
    er = alpha_r - r;
    % alpha_r_dot = 

    A = cos(theta_l)*cos(theta_b)*(cos(psi_le)-1) + cos(theta_le);
    ulD = uld*A + gammaR^(-1)*kr*re;
    A_dot = cos(theta_ld)*[ cos(theta_b)*sin(psi_ld - psi_b)*psi_l_D_dot - sin(theta_b)*cos(psi_ld - psi_b)*theta_l_D_dot ] + (sin(theta_ld)*cos(theta_b)*theta_l_D_dot);
    ule = ulD - ul;
    re_dot = uld*A - ul*[ cos(theta_l)*cos(theta_b)*cos(psi_le -1) + cos(theta_le) ];
    ulD_dot = uld*A_dot + gammaR^(-1)*kr*re_dot;

    Tu = bu^(-1)*sec(theta_a)*sec(psi_a)*[ ulD_dot + gammau^(-1)*[ gammaR*re + ku*ule]];

    midTq = (rq^(kq*eq + rtheta*theta_le*b_theta));
    midTr = gammar^(-1)*( kr*er + r*psile*sec(theta));

    Tq = bq^(-1)*[ midTq + alphaq_dot];
    Tr = br^(-1)*[ midTr + alphar_dot];
    
end

