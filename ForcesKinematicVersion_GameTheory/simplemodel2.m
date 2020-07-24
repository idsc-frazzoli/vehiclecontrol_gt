function dx = simplemodel2(x,u)
    global index
    
    % Inputs 
    u1_v1 = u(index.u_acc_v1);
    u2_v1 = u(index.u_steer_v1);
    
%     % States
%     x_v1 = x(index.x_v1-index.nu);
%     y_v1 = x(index.y_v1-index.nu);

    import casadi.*
    if isa(x(1), 'double')
        dx = zeros(index.ns,1);
    else
        dx = SX.zeros(index.ns,1);
    end
    
    % Update states
    dx(index.x_v1-index.nu)=u2_v1;
    dx(index.y_v1-index.nu)=u1_v1;
end

