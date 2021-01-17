function f = objective_IBR_LE_1(z,points,vmax,vdes,plagerror,platerror,...
                           pprog,pab,pdotbeta,pspeedcost,pspeedcostMax,pslack,pslack2)
     global index_IBR

    %get the fancy spline
    [splx,sply] = casadiDynamicBSPLINE(z(index_IBR.s),points);
    [spldx, spldy] = casadiDynamicBSPLINEforward(z(index_IBR.s),points);
    [splsx, splsy] = casadiDynamicBSPLINEsidewards(z(index_IBR.s),points);
        
    forward = [spldx;spldy];
    sidewards = [splsx;splsy];
    r=3.5;
    realPos = z([index_IBR.x,index_IBR.y]);
    centerOffset = 0.4*gokartforward(z(index_IBR.theta))';
    centerPos = realPos+centerOffset;%+0.4*forward;
    wantedpos = [splx;sply];
    wantedpos1= [splx;sply]+r/2*sidewards;
    error = centerPos-wantedpos;
    error1= centerPos-wantedpos1;
    lagerror = forward'*error;
    laterror = sidewards'*error;
    laterror1= sidewards'*error1;
    %% Costs objective function

    %slack = z(index_IBR.slack);
    slack2= z(index_IBR.slack2);
    
    speedcost = max(z(index_IBR.v)-vdes,0)^2*pspeedcost;
    speedcost2 = min(z(index_IBR.v)-vdes,0)^2*pprog;
    speedcost3 = max(z(index_IBR.v)-vmax,0)^2*pspeedcostMax;
    reg = z(index_IBR.dotab).^2*pab+z(index_IBR.dotbeta).^2*pdotbeta;
    lagcost = plagerror*lagerror^2;
    latcost = pslack*latErrorPunisher(laterror);
    latcost1 = platerror*laterror1^2;
    
    f = lagcost+latcost+latcost1+reg+speedcost+speedcost2+speedcost3+pslack2*slack2;%+prog
end
