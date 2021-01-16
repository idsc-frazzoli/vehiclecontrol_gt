function dx = interstagedx_IBR(x,u,p)
    global index_IBR
    points=p(index_IBR.pointsO+1:index_IBR.pointsO+3*index_IBR.pointsN);
    points=reshape(points,[index_IBR.pointsN,3]);
   % points=points';
%     points = [10,15,20,25,30,35,40,45,50,55,60,65,70,75;...          %x,75,80,85,90,95
%           50,50,50,50,50,50,50,50,50,50,50,50,50,50; ...    %y,50,50,50,50,50
%           3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5]'; %,5,5,5,5,5  
    %points(:,2)=points(:,2)+1.75;
    %points=flip(points);
    %get the fancy spline
    [splx,sply] = casadiDynamicBSPLINE(x(index_IBR.s-index_IBR.nu),points);
    [splsx, splsy] = casadiDynamicBSPLINEsidewards(x(index_IBR.s-index_IBR.nu),points);
        
    sidewards = [splsx;splsy];
    realPos = [x(index_IBR.x-index_IBR.nu);x(index_IBR.y-index_IBR.nu)];
    centerOffset = 0.4*gokartforward(x(index_IBR.theta-index_IBR.nu))';
    centerPos = realPos+centerOffset;%+0.4*forward;
    wantedpos = [splx;sply];
    error = centerPos-wantedpos;
    laterror = sidewards'*error;
    
    % Inputs 
    dotab = u(index_IBR.dotab);
    ab = x(index_IBR.ab-index_IBR.nu);
    dotbeta = u(index_IBR.dotbeta);
    ds = u(index_IBR.ds);
    
    % States
    theta = x(index_IBR.theta-index_IBR.nu);
    v = x(index_IBR.v-index_IBR.nu);
    beta = x(index_IBR.beta-index_IBR.nu);
    
    l = 2.5;
    %ackermannAngle = -0.58*beta*beta*beta+0.93*beta;
   
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
    dx(index_IBR.s-index_IBR.nu)=laterror;
    
    %dx = [v*cos(theta);
    %v*sin(theta);
    %v/l*tan(ackermannAngle);
    %ab;
    %dotbeta;
    %ds;
    %braking+cooldownfunction(temp)];
end

