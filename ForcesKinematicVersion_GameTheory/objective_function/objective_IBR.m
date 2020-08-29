function f = objective_IBR(z,points,radii,vmax, maxxacc,maxyacc,latacclim,rotacceffect,torqueveceffect, brakeeffect,plagerror, platerror, pprog, pab, pdotbeta, pspeedcost,pslack,pslack2)
     global index_IBR

    %get the fancy spline
    l = 1.19;
    [splx,sply] = casadiDynamicBSPLINE(z(index_IBR.s),points);
    [spldx, spldy] = casadiDynamicBSPLINEforward(z(index_IBR.s),points);
    [splsx, splsy] = casadiDynamicBSPLINEsidewards(z(index_IBR.s),points);
    r = casadiDynamicBSPLINERadius(z(index_IBR.s),radii);
    
    forward = [spldx;spldy];
    sidewards = [splsx;splsy];
    
    realPos = z([index_IBR.x,index_IBR.y]);
    centerOffset = 0.4*gokartforward(z(index_IBR.theta))';
    centerPos = realPos+centerOffset;%+0.4*forward;
    wantedpos = [splx;sply];
    error = centerPos-wantedpos;
    lagerror = forward'*error;
    laterror = sidewards'*error;
    
    %latdist = abs(laterror);
    %outsideTrack = max(0,latdist-r);
    %trackViolation = outsideTrack^2;
    
    %% Costs objective function
    
    %beta = z(index.beta);
    %tangentspeed = z(index.v);
    %forwardacc = z(index.ab);
    slack = z(index_IBR.slack);
    slack2= z(index_IBR.slack2);
    %dotbeta = z(index.dotbeta);
    %ackermannAngle = -0.58*beta*beta*beta+0.93*beta;
    %dAckermannAngle = -0.58*3*beta*beta*dotbeta+0.93*dotbeta;
    %latacc = (tan(ackermannAngle)*tangentspeed^2)/l;
    %rotacc = dAckermannAngle*tangentspeed/l;
    %frontaxlelatacc = abs(latacc+rotacc*rotacceffect);
    %torquevectoringcapability = torqueveccapsmooth(forwardacc)*torqueveceffect;
    %torquevectoringcapability = 0;
    %understeer = max(0,frontaxlelatacc - latacclim-torquevectoringcapability)^2;
    %accnorm = ((latacc/maxyacc)^2+(z(index.ab)/maxxacc)^2);
    %accviolation = max(0,accnorm-1)^2;
    
    speedcost = speedPunisher(z(index_IBR.v),vmax)*pspeedcost;
    lagcost = plagerror*lagerror^2;
    latcost = platerror*laterror^2;
    prog = -pprog*z(index_IBR.ds);
    reg = z(index_IBR.dotab).^2*pab+z(index_IBR.dotbeta).^2*pdotbeta;
    
    %f = error'*Q*error+reg+speedcost+over75d*over75d*0.001+1*trackViolation;
    %f = lagcost+latcost+reg+prog+over75d*over75d*0.001+speedcost+accviolation+trackViolation;
    f = lagcost+latcost+reg+prog+speedcost+pslack*slack+pslack2*slack2;
end
