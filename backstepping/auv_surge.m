function durdt = auv_surge(t3,ur)
    durdt = zeros(1,1);
        
    global ureq_dot    
    durdt(1) = ureq_dot; % to find ureq
end