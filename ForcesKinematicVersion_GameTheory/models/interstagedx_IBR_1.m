function dx = interstagedx_IBR_1(x,u)
    global index_IBR_1;
    
    % Inputs 
    dotab = u(index_IBR_1.dotab);
    ab = x(index_IBR_1.ab-index_IBR_1.nu);
    dotbeta = u(index_IBR_1.dotbeta);
    ds = u(index_IBR_1.ds);
    
    % States
    theta = x(index_IBR_1.theta-index_IBR_1.nu);
    v = x(index_IBR_1.v-index_IBR_1.nu);
    beta = x(index_IBR_1.beta-index_IBR_1.nu);
    
    l = 2.5;
    %ackermannAngle = -0.58*beta*beta*beta+0.93*beta;
   
    import casadi.*
    if isa(x(1), 'double')
        dx = zeros(index_IBR_1.ns,1);
    else
        dx = SX.zeros(index_IBR_1.ns,1);
    end
    
    % Update states
    dx(index_IBR_1.x-index_IBR_1.nu)=v*cos(theta);
    dx(index_IBR_1.y-index_IBR_1.nu)=v*sin(theta);
    dx(index_IBR_1.theta-index_IBR_1.nu)=v/l*tan(beta);
    dx(index_IBR_1.v-index_IBR_1.nu)=ab;
    dx(index_IBR_1.ab-index_IBR_1.nu)=dotab;
    dx(index_IBR_1.beta-index_IBR_1.nu)=dotbeta;
    dx(index_IBR_1.s-index_IBR_1.nu)=ds;
    dx(index_IBR_1.laterror-index_IBR_1.nu)=ds;
    
    %dx = [v*cos(theta);
    %v*sin(theta);
    %v/l*tan(ackermannAngle);
    %ab;
    %dotbeta;
    %ds;
    %braking+cooldownfunction(temp)];
end

