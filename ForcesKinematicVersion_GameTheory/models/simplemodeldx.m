function dx = simplemodeldx(x,u)
    global index
    
    % Inputs 
    u1_v1 = u(index.u_acc_v1);
    u2_v1 = u(index.u_steer_v1);
    
    u1_v2 = u(index.u_acc_v2);
    u2_v2 = u(index.u_steer_v2);
    
    import casadi.*
    if isa(x(1), 'double')
        dx = zeros(index.ns,1);
    else
        dx = SX.zeros(index.ns,1);
    end
    
    % Update states
    dx(index.x_v1-index.nu)=u1_v1;
    dx(index.y_v1-index.nu)=u2_v1;
    dx(index.x_v2-index.nu)=u2_v2;
    dx(index.y_v2-index.nu)=u1_v2;
end

