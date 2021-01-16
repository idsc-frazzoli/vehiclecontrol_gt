function dx = interstagedx_PG3_2(x,u)
    global index1
    
    points2 = [50,50,50,50,50,50,50,50,50,50,50,50,50,50;...          %x,50,50,50,50,50
          25,30,35,40,45,50,55,60,65,70,75,80,85,90; ...    %y,75,80,85,90,95
          3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5]';  %,5,5,5,5,5

    [splx_k2,sply_k2] = casadiDynamicBSPLINE(x(index1.s_k2-index1.nu),points2);
    %[spldx_k2, spldy_k2] = casadiDynamicBSPLINEforward(x(index1.s_k2-index1.nu),points2);
    [splsx_k2, splsy_k2] = casadiDynamicBSPLINEsidewards(x(index1.s_k2-index1.nu),points2);
    
    %forward_k2 = [spldx_k2;spldy_k2];
    sidewards_k2 = [splsx_k2;splsy_k2];
    
    realPos_k2 = [x(index1.x_k2-index1.nu),x(index1.y_k2-index1.nu)]';
    centerOffset_k2 = 0.4*gokartforward(x(index1.theta_k2-index1.nu))';
    centerPos_k2 = realPos_k2+centerOffset_k2;
    wantedpos_k2 = [splx_k2;sply_k2];
    error_k2 = centerPos_k2-wantedpos_k2;
    laterror_k2 = sidewards_k2'*error_k2;
    
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
    dx(index1.laterror_k2-index1.nu)=(min(0,laterror_k2))^2;%();
    
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

