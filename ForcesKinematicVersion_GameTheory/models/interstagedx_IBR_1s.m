function dx = interstagedx_IBR_1s(x,u)
    global index_IBR
    
    l=1.19;
    % Inputs 
    dotab = u(index_IBR.dotab);
    ab = x(index_IBR.ab-index_IBR.nu);
    dotbeta = u(index_IBR.dotbeta);
    ds = u(index_IBR.ds);
    
    % States
    theta = x(index_IBR.theta-index_IBR.nu);
    v = x(index_IBR.v-index_IBR.nu);
    beta = x(index_IBR.beta-index_IBR.nu);
   
    import casadi.*
    if isa(x(1), 'double')
        dx = zeros(index_IBR.ns,1);
    else
        dx = SX.zeros(index_IBR.ns,1);
    end
    
    % Update states
    dx(index_IBR.x-index_IBR.nu)=v*cos(theta);
    dx(index_IBR.y-index_IBR.nu)=v*sin(theta);
    dx(index_IBR.theta-index_IBR.nu)=v/l*tan(beta);
    dx(index_IBR.v-index_IBR.nu)=ab;
    dx(index_IBR.ab-index_IBR.nu)=dotab;
    dx(index_IBR.beta-index_IBR.nu)=dotbeta;
    dx(index_IBR.s-index_IBR.nu)=ds;
end

