function [xe,ye,ze,uld,theta_ld,psi_ld] = referencefn(t,x,y,z)

    syms x y z xe ye ze 
    
    if( t<=90 )
        xd = 0;
        yd = 0;
        zd = 0;
        uld = 1.5;
        theta_ld = 0;
        psi_ld = 0;
    else
        xd = 0;
        yd = 20;
        zd = 0;
        uld = 1.5;
        theta_ld = -10;
        psi_ld = 0.06;
    end 
    
    xe = x - xd;
    ye = y - yd;
    ze = z - zd;
end 