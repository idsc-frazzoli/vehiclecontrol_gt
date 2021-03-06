function f = objective(z,points,vmax, plagerror, platerror, pprog, pab, pdotbeta, pspeedcost,pslack)
    global index

    %get the fancy spline
    [splx,sply] = casadiDynamicBSPLINE(z(index.s),points);
    [spldx, spldy] = casadiDynamicBSPLINEforward(z(index.s),points);
    [splsx, splsy] = casadiDynamicBSPLINEsidewards(z(index.s),points);
        
    forward = [spldx;spldy];
    sidewards = [splsx;splsy];
    
    realPos = z([index.x,index.y]);
    centerOffset = 0.4*gokartforward(z(index.theta))';
    centerPos = realPos+centerOffset;%+0.4*forward;
    wantedpos = [splx;sply];
    error = centerPos-wantedpos;
    lagerror = forward'*error;
    laterror = sidewards'*error;
    
    %% Costs objective function
    slack = z(index.slack);
    speedcost = speedPunisher(z(index.v),vmax)*pspeedcost;
    lagcost = plagerror*lagerror^2;
    latcost = platerror*latErrorPunisher(laterror);
    prog = -pprog*z(index.ds);
    reg = z(index.dotab).^2*pab+z(index.dotbeta).^2*pdotbeta;
    f = lagcost+latcost+reg+prog+pslack*slack+speedcost;
end
