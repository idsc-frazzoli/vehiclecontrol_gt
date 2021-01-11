function dx = interstagedx_PG3_1(x,u)
    global index1
    
    % Inputs 
    dotab = u(index1.dotab);
    ab = x(index1.ab-index1.nu);
    dotbeta = u(index1.dotbeta);
    ds = u(index1.ds);
    
    dotab_k2 = u(index1.dotab_k2);
    ab_k2 = x(index1.ab_k2-index1.nu);
    dotbeta_k2 = u(index1.dotbeta_k2);
    ds_k2 = u(index1.ds_k2);
    
    dotab_k3 = u(index1.dotab_k3);
    ab_k3 = x(index1.ab_k3-index1.nu);
    dotbeta_k3 = u(index1.dotbeta_k3);
    ds_k3 = u(index1.ds_k3);
    
    % States
    theta = x(index1.theta-index1.nu);
    v = x(index1.v-index1.nu);
    beta = x(index1.beta-index1.nu);
    
    theta_k2 = x(index1.theta_k2-index1.nu);
    v_k2 = x(index1.v_k2-index1.nu);
    beta_k2 = x(index1.beta_k2-index1.nu);
    
    theta_k3 = x(index1.theta_k3-index1.nu);
    v_k3 = x(index1.v_k3-index1.nu);
    beta_k3 = x(index1.beta_k3-index1.nu);
    
    l = 2.5;
%     ackermannAngle = -0.58*beta*beta*beta+0.93*beta;
%     ackermannAngle_k2 = -0.58*beta_k2*beta_k2*beta_k2+0.93*beta_k2;
%     ackermannAngle_k3 = -0.58*beta_k3*beta_k3*beta_k3+0.93*beta_k3;
%     
    import casadi.*
    if isa(x(1), 'double')
        dx = zeros(index1.ns,1);
    else
        dx = SX.zeros(index1.ns,1);
    end
    
    % Update states
    dx(index1.x-index1.nu)=v*cos(theta);
    dx(index1.y-index1.nu)=v*sin(theta);
    dx(index1.theta-index1.nu)=v/l*tan(beta);
    dx(index1.v-index1.nu)=ab;
    dx(index1.beta-index1.nu)=dotbeta;
    dx(index1.s-index1.nu)=ds;
    dx(index1.ab-index1.nu)=dotab;
    
    dx(index1.x_k2-index1.nu)=v_k2*cos(theta_k2);
    dx(index1.y_k2-index1.nu)=v_k2*sin(theta_k2);
    dx(index1.theta_k2-index1.nu)=v_k2/l*tan(beta_k2);
    dx(index1.v_k2-index1.nu)=ab_k2;
    dx(index1.beta_k2-index1.nu)=dotbeta_k2;
    dx(index1.s_k2-index1.nu)=ds_k2;
    dx(index1.ab_k2-index1.nu)=dotab_k2;
    
    dx(index1.x_k3-index1.nu)=v_k3*cos(theta_k3);
    dx(index1.y_k3-index1.nu)=v_k3*sin(theta_k3);
    dx(index1.theta_k3-index1.nu)=v_k3/l*tan(beta_k3);
    dx(index1.v_k3-index1.nu)=ab_k3;
    dx(index1.beta_k3-index1.nu)=dotbeta_k3;
    dx(index1.s_k3-index1.nu)=ds_k3;
    dx(index1.ab_k3-index1.nu)=dotab_k3;
    %dx = [v*cos(theta);
    %v*sin(theta);
    %v/l*tan(ackermannAngle);
    %ab;
    %dotbeta;
    %ds;
    %braking+cooldownfunction(temp)];
end

