function dx = interstagedx_PG5(x,u)
    global index
    
    % Inputs 
    dotab = u(index.dotab);
    ab = x(index.ab-index.nu);
    dotbeta = u(index.dotbeta);
    ds = u(index.ds);
    
    dotab_k2 = u(index.dotab_k2);
    ab_k2 = x(index.ab_k2-index.nu);
    dotbeta_k2 = u(index.dotbeta_k2);
    ds_k2 = u(index.ds_k2);
    
    dotab_k3 = u(index.dotab_k3);
    ab_k3 = x(index.ab_k3-index.nu);
    dotbeta_k3 = u(index.dotbeta_k3);
    ds_k3 = u(index.ds_k3);
    
    dotab_k4 = u(index.dotab_k4);
    ab_k4 = x(index.ab_k4-index.nu);
    dotbeta_k4 = u(index.dotbeta_k4);
    ds_k4 = u(index.ds_k4);
    
    dotab_k5 = u(index.dotab_k5);
    ab_k5 = x(index.ab_k5-index.nu);
    dotbeta_k5 = u(index.dotbeta_k5);
    ds_k5 = u(index.ds_k5);
    
    % States
    theta = x(index.theta-index.nu);
    v = x(index.v-index.nu);
    beta = x(index.beta-index.nu);
    
    theta_k2 = x(index.theta_k2-index.nu);
    v_k2 = x(index.v_k2-index.nu);
    beta_k2 = x(index.beta_k2-index.nu);
    
    theta_k3 = x(index.theta_k3-index.nu);
    v_k3 = x(index.v_k3-index.nu);
    beta_k3 = x(index.beta_k3-index.nu);
    
    theta_k4 = x(index.theta_k4-index.nu);
    v_k4 = x(index.v_k4-index.nu);
    beta_k4 = x(index.beta_k4-index.nu);
    
    theta_k5 = x(index.theta_k5-index.nu);
    v_k5 = x(index.v_k5-index.nu);
    beta_k5 = x(index.beta_k5-index.nu);
    
    l = 2;
 
    import casadi.*
    if isa(x(1), 'double')
        dx = zeros(index.ns,1);
    else
        dx = SX.zeros(index.ns,1);
    end
    
    % Update states
    dx(index.x-index.nu)=v*cos(theta);
    dx(index.y-index.nu)=v*sin(theta);
    dx(index.theta-index.nu)=v/l*tan(beta);
    dx(index.v-index.nu)=ab;
    dx(index.beta-index.nu)=dotbeta;
    dx(index.s-index.nu)=ds;
    dx(index.ab-index.nu)=dotab;
    
    dx(index.x_k2-index.nu)=v_k2*cos(theta_k2);
    dx(index.y_k2-index.nu)=v_k2*sin(theta_k2);
    dx(index.theta_k2-index.nu)=v_k2/l*tan(beta_k2);
    dx(index.v_k2-index.nu)=ab_k2;
    dx(index.beta_k2-index.nu)=dotbeta_k2;
    dx(index.s_k2-index.nu)=ds_k2;
    dx(index.ab_k2-index.nu)=dotab_k2;
    
    dx(index.x_k3-index.nu)=v_k3*cos(theta_k3);
    dx(index.y_k3-index.nu)=v_k3*sin(theta_k3);
    dx(index.theta_k3-index.nu)=v_k3/l*tan(beta_k3);
    dx(index.v_k3-index.nu)=ab_k3;
    dx(index.beta_k3-index.nu)=dotbeta_k3;
    dx(index.s_k3-index.nu)=ds_k3;
    dx(index.ab_k3-index.nu)=dotab_k3;
    
    dx(index.x_k4-index.nu)=v_k4*cos(theta_k4);
    dx(index.y_k4-index.nu)=v_k4*sin(theta_k4);
    dx(index.theta_k4-index.nu)=v_k4/l*tan(beta_k4);
    dx(index.v_k4-index.nu)=ab_k4;
    dx(index.beta_k4-index.nu)=dotbeta_k4;
    dx(index.s_k4-index.nu)=ds_k4;
    dx(index.ab_k4-index.nu)=dotab_k4;
    
    dx(index.x_k5-index.nu)=v_k5*cos(theta_k5);
    dx(index.y_k5-index.nu)=v_k5*sin(theta_k5);
    dx(index.theta_k5-index.nu)=v_k5/l*tan(beta_k5);
    dx(index.v_k5-index.nu)=ab_k5;
    dx(index.beta_k5-index.nu)=dotbeta_k5;
    dx(index.s_k5-index.nu)=ds_k5;
    dx(index.ab_k5-index.nu)=dotab_k5;
end

