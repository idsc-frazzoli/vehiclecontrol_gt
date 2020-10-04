function f = objective_PG3_LE(z,points,points2,points3,vmax,plagerror, platerror, pprog, pab, pdotbeta, pspeedcost,pslack,pslack2)
    global index

    %get the fancy spline
    [splx,sply] = casadiDynamicBSPLINE(z(index.s),points);
    [spldx, spldy] = casadiDynamicBSPLINEforward(z(index.s),points);
    [splsx, splsy] = casadiDynamicBSPLINEsidewards(z(index.s),points);
    
    [splx_k2,sply_k2] = casadiDynamicBSPLINE(z(index.s_k2),points2);
    [spldx_k2, spldy_k2] = casadiDynamicBSPLINEforward(z(index.s_k2),points2);
    [splsx_k2, splsy_k2] = casadiDynamicBSPLINEsidewards(z(index.s_k2),points2);
    
    [splx_k3,sply_k3] = casadiDynamicBSPLINE(z(index.s_k3),points3);
    [spldx_k3, spldy_k3] = casadiDynamicBSPLINEforward(z(index.s_k3),points3);
    [splsx_k3, splsy_k3] = casadiDynamicBSPLINEsidewards(z(index.s_k2),points3);
    
    forward = [spldx;spldy];
    sidewards = [splsx;splsy];
    
    realPos = z([index.x,index.y]);
    centerOffset = 0.4*gokartforward(z(index.theta))';
    centerPos = realPos+centerOffset;%+0.4*forward;
    wantedpos = [splx;sply];
    error = centerPos-wantedpos;
    lagerror = forward'*error;
    laterror = sidewards'*error;
    
    forward_k2 = [spldx_k2;spldy_k2];
    sidewards_k2 = [splsx_k2;splsy_k2];
    
    realPos_k2 = z([index.x_k2,index.y_k2]);
    centerOffset_k2 = 0.4*gokartforward(z(index.theta_k2))';
    centerPos_k2 = realPos_k2+centerOffset_k2;%+0.4*forward;
    wantedpos_k2 = [splx_k2;sply_k2];
    error_k2 = centerPos_k2-wantedpos_k2;
    lagerror_k2 = forward_k2'*error_k2;
    laterror_k2 = sidewards_k2'*error_k2;
    
    forward_k3 = [spldx_k3;spldy_k3];
    sidewards_k3 = [splsx_k3;splsy_k3];
    
    realPos_k3 = z([index.x_k3,index.y_k3]);
    centerOffset_k3 = 0.4*gokartforward(z(index.theta_k3))';
    centerPos_k3 = realPos_k3+centerOffset_k3;%+0.4*forward;
    wantedpos_k3 = [splx_k3;sply_k3];
    error_k3 = centerPos_k3-wantedpos_k3;
    lagerror_k3 = forward_k3'*error_k3;
    laterror_k3 = sidewards_k3'*error_k3;
   
    %% Costs objective function

    slack = z(index.slack);
    slack2 = z(index.slack2);
    slack3 = z(index.slack3);
    slack4 = z(index.slack4);
    slack_k2 = z(index.slack_k2);
    slack_k3 = z(index.slack_k3);
       
    speedcost = speedPunisher(z(index.v),vmax)*pspeedcost;
    lagcost = plagerror*lagerror^2;
    latcost = platerror*latErrorPunisher(laterror);
    prog = -pprog*z(index.ds);
    reg = z(index.dotab).^2*pab+z(index.dotbeta).^2*pdotbeta;
    
    speedcost_k2 = speedPunisher(z(index.v_k2),vmax)*pspeedcost;
    lagcost_k2 = plagerror*lagerror_k2^2;
    latcost_k2 = platerror*latErrorPunisher(laterror_k2);
    prog_k2 = -pprog*z(index.ds_k2);
    reg_k2 = z(index.dotab_k2).^2*pab+z(index.dotbeta_k2).^2*pdotbeta;
    
    speedcost_k3 = speedPunisher(z(index.v_k3),vmax)*pspeedcost;
    lagcost_k3 = plagerror*lagerror_k3^2;
    latcost_k3 = platerror*latErrorPunisher(laterror_k3);
    prog_k3 = -pprog*z(index.ds_k3);
    reg_k3 = z(index.dotab_k3).^2*pab+z(index.dotbeta_k3).^2*pdotbeta;
    
    f = (lagcost+latcost+reg+prog+speedcost)+pslack*slack+...
        (lagcost_k2+latcost_k2+reg_k2+prog_k2+speedcost_k2)+pslack*slack_k2+...
        (lagcost_k3+latcost_k3+reg_k3+prog_k3+speedcost_k3)+pslack*slack_k3+...
        pslack2*slack2+pslack2*slack3+pslack2*slack4;
end
