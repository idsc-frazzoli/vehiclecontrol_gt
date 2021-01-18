function dx = interstagedx_PG3(x,u,p)
    global index
    
    points = [10,15,20,25,30,35,40,45,50,55,60,65,70,75;...          %x,75,80,85,90,95
          50,50,50,50,50,50,50,50,50,50,50,50,50,50; ...    %y,50,50,50,50,50
          3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5]'; %,5,5,5,5,5  
    points=flip(points);
    points2 = [50,50,50,50,50,50,50,50,50,50,50,50,50,50;...          %x,50,50,50,50,50
              25,30,35,40,45,50,55,60,65,70,75,80,85,90; ...    %y,75,80,85,90,95
              3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5]';  %,5,5,5,5,5

    points3 = [50,50,50,50,50,50,52,55,60,65,70,75,80,85;...          %x,50,50,50,50,50,50,65,65,50
               75,70,65,60,55,52,50,50,50,50,50,50,50,50; ...    %y,25,20,15,10,5,0,5,92,95
               3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5]';%,5,5,5,5,4,3,2,2,3

    
    [splx,sply] = casadiDynamicBSPLINE(x(index.s-index.nu),points);
    %[spldx_k2, spldy_k2] = casadiDynamicBSPLINEforward(x(index1.s_k2-index1.nu),points2);
    [splsx, splsy] = casadiDynamicBSPLINEsidewards(x(index.s-index.nu),points);
    
    %forward_k2 = [spldx_k2;spldy_k2];
    sidewards = [splsx;splsy];
    
    realPos = [x(index.x-index.nu),x(index.y-index.nu)]';
    centerOffset= 0.4*gokartforward(x(index.theta-index.nu))';
    centerPos = realPos+centerOffset;
    wantedpos = [splx;sply];
    error = centerPos-wantedpos;
    laterror = sidewards'*error;
    
    [splx_k2,sply_k2] = casadiDynamicBSPLINE(x(index.s_k2-index.nu),points2);
    %[spldx_k2, spldy_k2] = casadiDynamicBSPLINEforward(x(index1.s_k2-index1.nu),points2);
    [splsx_k2, splsy_k2] = casadiDynamicBSPLINEsidewards(x(index.s_k2-index.nu),points2);
    
    %forward_k2 = [spldx_k2;spldy_k2];
    sidewards_k2 = [splsx_k2;splsy_k2];
    
    realPos_k2 = [x(index.x_k2-index.nu),x(index.y_k2-index.nu)]';
    centerOffset_k2 = 0.4*gokartforward(x(index.theta_k2-index.nu))';
    centerPos_k2 = realPos_k2+centerOffset_k2;
    wantedpos_k2 = [splx_k2;sply_k2];
    error_k2 = centerPos_k2-wantedpos_k2;
    laterror_k2 = sidewards_k2'*error_k2;
    
    [splx_k3,sply_k3] = casadiDynamicBSPLINE(x(index.s_k3-index.nu),points3);
    %[spldx_k3, spldy_k3] = casadiDynamicBSPLINEforward(x(index.s_k3-index.nu),points3);
    [splsx_k3, splsy_k3] = casadiDynamicBSPLINEsidewards(x(index.s_k3-index.nu),points3);
    
    %forward_k3 = [spldx_k3;spldy_k3];
    sidewards_k3 = [splsx_k3;splsy_k3];
    
    realPos_k3 = [x(index.x_k3-index.nu),x(index.y_k3-index.nu)]';
    centerOffset_k3 = 0.4*gokartforward(x(index.theta_k3-index.nu))';
    centerPos_k3 = realPos_k3+centerOffset_k3;
    wantedpos_k3 = [splx_k3;sply_k3];
    error_k3 = centerPos_k3-wantedpos_k3;
    laterror_k3 = sidewards_k3'*error_k3;
    
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
    
    l = 2.5;
%     ackermannAngle = -0.58*beta*beta*beta+0.93*beta;
%     ackermannAngle_k2 = -0.58*beta_k2*beta_k2*beta_k2+0.93*beta_k2;
%     ackermannAngle_k3 = -0.58*beta_k3*beta_k3*beta_k3+0.93*beta_k3;
%     
    import casadi.*
    if isa(x(1), 'double')
        dx = zeros(index.ns,1);
    else
        dx = SX.zeros(index.ns,1);
    end
    slack2=u(index.slack2);
    slack3=u(index.slack3);
    slack4=u(index.slack4);
    pslack=p(index.pslack);
    pslack2=p(index.pslack2);
    vmax=p(index.SpeedMax);
    pspeedcostMax=p(index.pSpeedMax);
    % Update states
    dx(index.x-index.nu)=v*cos(theta);
    dx(index.y-index.nu)=v*sin(theta);
    dx(index.theta-index.nu)=v/l*tan(beta);
    dx(index.v-index.nu)=ab;
    dx(index.beta-index.nu)=dotbeta;
    dx(index.s-index.nu)=ds;
    dx(index.ab-index.nu)=dotab;
    dx(index.laterror-index.nu)=pslack*(min(0,laterror))^2+max(x(index.v-index.nu)-vmax,0)^2*pspeedcostMax;
    dx(index.slack_s-index.nu)=pslack2*slack2;
%     dx(index_IBR.laterror-index_IBR.nu)=pslack*latErrorPunisher(laterror)+pspeedcostMax*max(x(index_IBR.v-index_IBR.nu)-vmax,0)^2;
%     dx(index_IBR.slack_s-index_IBR.nu)=pslack2*slack;
    
    dx(index.x_k2-index.nu)=v_k2*cos(theta_k2);
    dx(index.y_k2-index.nu)=v_k2*sin(theta_k2);
    dx(index.theta_k2-index.nu)=v_k2/l*tan(beta_k2);
    dx(index.v_k2-index.nu)=ab_k2;
    dx(index.beta_k2-index.nu)=dotbeta_k2;
    dx(index.s_k2-index.nu)=ds_k2;
    dx(index.ab_k2-index.nu)=dotab_k2;
    dx(index.laterror_k2-index.nu)=pslack*(min(0,laterror_k2))^2+max(x(index.v_k2-index.nu)-vmax,0)^2*pspeedcostMax;
    dx(index.slack_s_k2-index.nu)=pslack2*slack3;
    
    dx(index.x_k3-index.nu)=v_k3*cos(theta_k3);
    dx(index.y_k3-index.nu)=v_k3*sin(theta_k3);
    dx(index.theta_k3-index.nu)=v_k3/l*tan(beta_k3);
    dx(index.v_k3-index.nu)=ab_k3;
    dx(index.beta_k3-index.nu)=dotbeta_k3;
    dx(index.s_k3-index.nu)=ds_k3;
    dx(index.ab_k3-index.nu)=dotab_k3;
    dx(index.laterror_k3-index.nu)=pslack*(min(0,laterror_k3))^2+max(x(index.v_k3-index.nu)-vmax,0)^2*pspeedcostMax;
    dx(index.slack_s_k3-index.nu)=pslack2*slack4;
end

