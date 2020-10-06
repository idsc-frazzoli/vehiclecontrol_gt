function f = objective_IBR_alpha_LE(z,points,vmax,plagerror, platerror, pprog, pab, pdotbeta, pspeedcost,pslack,pslack2)
    global index_IBR

    [splx,sply] = casadiDynamicBSPLINE(z(index_IBR.s),points);
    [spldx, spldy] = casadiDynamicBSPLINEforward(z(index_IBR.s),points);
    [splsx, splsy] = casadiDynamicBSPLINEsidewards(z(index_IBR.s),points);
        
    forward = [spldx;spldy];
    sidewards = [splsx;splsy];
    
    realPos = z([index_IBR.x,index_IBR.y]);
    centerOffset = 0.4*gokartforward(z(index_IBR.theta))';
    centerPos = realPos+centerOffset;%+0.4*forward;
    wantedpos = [splx;sply];
    error = centerPos-wantedpos;
    lagerror = forward'*error;
    laterror = latErrorPunisher(sidewards'*error);
    %% Costs objective function
  
    slack = z(index_IBR.slack);
    slack2= z(index_IBR.slack2);
    speedcost = (z(index_IBR.v)-vmax)^2*pspeedcost;
    lagcost = plagerror*lagerror^2;
    latcost = platerror*laterror^2;
    %prog = -pprog*z(index_IBR.ds);
    reg = z(index_IBR.dotab).^2*pab+z(index_IBR.dotbeta).^2*pdotbeta;

    f = lagcost+latcost+reg+speedcost+pslack*slack+pslack2*slack2;
end
