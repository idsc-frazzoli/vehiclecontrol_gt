function f = objective_PG5_LE(z,points,points2,points3,points4,points5,vmax,plagerror, platerror, pprog, pab, pdotbeta, pspeedcost,pslack,pslack2)
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
    
    [splx_k4,sply_k4] = casadiDynamicBSPLINE(z(index.s_k4),points4);
    [spldx_k4, spldy_k4] = casadiDynamicBSPLINEforward(z(index.s_k4),points4);
    [splsx_k4, splsy_k4] = casadiDynamicBSPLINEsidewards(z(index.s_k4),points4);
    
    [splx_k5,sply_k5] = casadiDynamicBSPLINE(z(index.s_k5),points5);
    [spldx_k5, spldy_k5] = casadiDynamicBSPLINEforward(z(index.s_k5),points5);
    [splsx_k5, splsy_k5] = casadiDynamicBSPLINEsidewards(z(index.s_k5),points5);
    
    forward = [spldx;spldy];
    sidewards = [splsx;splsy];
    r=3;
    realPos = z([index.x,index.y]);
    centerOffset = 0.4*gokartforward(z(index.theta))';
    centerPos = realPos+centerOffset;%+0.4*forward;
    wantedpos = [splx;sply];
    wantedpos1= [splx;sply]+r/2*sidewards;
    error = centerPos-wantedpos;
    error1= centerPos-wantedpos1;
    lagerror = forward'*error;
    laterror = sidewards'*error;
    laterror1 = sidewards'*error1;
    
    forward_k2 = [spldx_k2;spldy_k2];
    sidewards_k2 = [splsx_k2;splsy_k2];
    
    realPos_k2 = z([index.x_k2,index.y_k2]);
    centerOffset_k2 = 0.4*gokartforward(z(index.theta_k2))';
    centerPos_k2 = realPos_k2+centerOffset_k2;%+0.4*forward;
    wantedpos_k2 = [splx_k2;sply_k2];
    wantedpos1_k2= [splx_k2;sply_k2]+r/2*sidewards_k2;
    error_k2 = centerPos_k2-wantedpos_k2;
    error1_k2= centerPos_k2-wantedpos1_k2;
    lagerror_k2 = forward_k2'*error_k2;
    laterror_k2 = sidewards_k2'*error_k2;
    laterror1_k2 = sidewards_k2'*error1_k2;
    
    forward_k3 = [spldx_k3;spldy_k3];
    sidewards_k3 = [splsx_k3;splsy_k3];
    
    realPos_k3 = z([index.x_k3,index.y_k3]);
    centerOffset_k3 = 0.4*gokartforward(z(index.theta_k3))';
    centerPos_k3 = realPos_k3+centerOffset_k3;%+0.4*forward;
    wantedpos_k3 = [splx_k3;sply_k3];
    wantedpos1_k3= [splx_k3;sply_k3]+r/2*sidewards_k3;
    error_k3 = centerPos_k3-wantedpos_k3;
    error1_k3= centerPos_k3-wantedpos1_k3;
    lagerror_k3 = forward_k3'*error_k3;
    laterror_k3 = sidewards_k3'*error_k3;
    laterror1_k3 = sidewards_k3'*error1_k3;
    
    forward_k4 = [spldx_k4;spldy_k4];
    sidewards_k4 = [splsx_k4;splsy_k4];
    
    realPos_k4 = z([index.x_k4,index.y_k4]);
    centerOffset_k4 = 0.4*gokartforward(z(index.theta_k4))';
    centerPos_k4 = realPos_k4+centerOffset_k4;%+0.4*forward;
    wantedpos_k4 = [splx_k4;sply_k4];
    wantedpos1_k4= [splx_k4;sply_k4]+r/2*sidewards_k4;
    error_k4 = centerPos_k4-wantedpos_k4;
    error1_k4= centerPos_k4-wantedpos1_k4;
    lagerror_k4 = forward_k4'*error_k4;
    laterror_k4 = sidewards_k4'*error_k4;
    laterror1_k4 = sidewards_k4'*error1_k4;
    
    forward_k5 = [spldx_k5;spldy_k5];
    sidewards_k5 = [splsx_k5;splsy_k5];
    
    realPos_k5 = z([index.x_k5,index.y_k5]);
    centerOffset_k5 = 0.4*gokartforward(z(index.theta_k5))';
    centerPos_k5 = realPos_k5+centerOffset_k5;%+0.4*forward;
    wantedpos_k5 = [splx_k5;sply_k5];
    wantedpos1_k5= [splx_k5;sply_k5]+r/2*sidewards_k5;
    error_k5 = centerPos_k5-wantedpos_k5;
    error1_k5= centerPos_k5-wantedpos1_k5;
    lagerror_k5 = forward_k5'*error_k5;
    laterror_k5 = sidewards_k5'*error_k5;
    laterror1_k5 = sidewards_k5'*error1_k5;
    %latdist = abs(laterror);
    %outsideTrack = max(0,latdist-r);
    %trackViolation = outsideTrack^2;
    
    %% Costs objective function
    slack = z(index.slack);
    slack2 = z(index.slack2);
    slack3 = z(index.slack3);
    slack4 = z(index.slack4);
    slack_k2 = z(index.slack_k2);
    slack_k3 = z(index.slack_k3);
    slack_k4 = z(index.slack_k4);
    slack_k5 = z(index.slack_k5);
    
    speedcost = z(index.v)^2*pspeedcost;
    lagcost = plagerror*lagerror^2;
    latcost = platerror*latErrorPunisher(laterror);
    latcost1 = platerror/200*latErrorPunisher(laterror1);
    reg = z(index.dotab).^2*pab+z(index.dotbeta).^2*pdotbeta;
    
    speedcost_k2 = z(index.v_k2)^2*pspeedcost;
    lagcost_k2 = plagerror*lagerror_k2^2;
    latcost_k2 = platerror*latErrorPunisher(laterror_k2);
    latcost1_k2 = platerror/200*latErrorPunisher(laterror1_k2);
    reg_k2 = z(index.dotab_k2).^2*pab+z(index.dotbeta_k2).^2*pdotbeta;
    
    speedcost_k3 = z(index.v_k3)^2*pspeedcost;
    lagcost_k3 = plagerror*lagerror_k3^2;
    latcost_k3 = platerror*latErrorPunisher(laterror_k3);
    latcost1_k3 = platerror/200*latErrorPunisher(laterror1_k3);
    reg_k3 = z(index.dotab_k3).^2*pab+z(index.dotbeta_k3).^2*pdotbeta;
    
    speedcost_k4 = z(index.v_k4)^2*pspeedcost;
    lagcost_k4 = plagerror*lagerror_k4^2;
    latcost_k4 = platerror*latErrorPunisher(laterror_k4);
    latcost1_k4 = platerror/200*latErrorPunisher(laterror1_k4);
    reg_k4 = z(index.dotab_k4).^2*pab+z(index.dotbeta_k4).^2*pdotbeta;
    
    speedcost_k5 = z(index.v_k5)^2*pspeedcost;
    lagcost_k5 = plagerror*lagerror_k5^2;
    latcost_k5 = platerror*latErrorPunisher(laterror_k5);
    latcost1_k5 = platerror/200*latErrorPunisher(laterror1_k5);
    reg_k5 = z(index.dotab_k5).^2*pab+z(index.dotbeta_k5).^2*pdotbeta;

    f = lagcost   +latcost   +latcost1   +reg   +pslack*slack   +speedcost   +...
        lagcost_k2+latcost_k2+latcost1_k2+reg_k2+pslack*slack_k2+speedcost_k2+...
        lagcost_k3+latcost_k3+latcost1_k3+reg_k3+pslack*slack_k3+speedcost_k3+...
        lagcost_k4+latcost_k4+latcost1_k4+reg_k4+pslack*slack_k4+speedcost_k4+...
        lagcost_k5+latcost_k5+latcost1_k5+reg_k5+pslack*slack_k5+speedcost_k5+...
        pslack2*slack2+pslack2*slack3+pslack2*slack4;
end
