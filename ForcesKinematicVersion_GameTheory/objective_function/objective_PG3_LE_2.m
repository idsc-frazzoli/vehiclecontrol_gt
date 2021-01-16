function f = objective_PG3_LE_2(z,points,points2,points3,vmax,plagerror, platerror, pprog, pab, pdotbeta, pspeedcost,pslack,pslack2)
    global index1
    
    r=3.5;
    %get the fancy spline
    [splx,sply] = casadiDynamicBSPLINE(z(index1.s),points);
    [spldx, spldy] = casadiDynamicBSPLINEforward(z(index1.s),points);
    [splsx, splsy] = casadiDynamicBSPLINEsidewards(z(index1.s),points);
    
    [splx_k2,sply_k2] = casadiDynamicBSPLINE(z(index1.s_k2),points2);
    [spldx_k2, spldy_k2] = casadiDynamicBSPLINEforward(z(index1.s_k2),points2);
    [splsx_k2, splsy_k2] = casadiDynamicBSPLINEsidewards(z(index1.s_k2),points2);
    
    [splx_k3,sply_k3] = casadiDynamicBSPLINE(z(index1.s_k3),points3);
    [spldx_k3, spldy_k3] = casadiDynamicBSPLINEforward(z(index1.s_k3),points3);
    [splsx_k3, splsy_k3] = casadiDynamicBSPLINEsidewards(z(index1.s_k2),points3);
    
    forward = [spldx;spldy];
    sidewards = [splsx;splsy];
    %r=1.5;
    realPos = z([index1.x,index1.y]);
    centerOffset = 0.4*gokartforward(z(index1.theta))';
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
    
    realPos_k2 = z([index1.x_k2,index1.y_k2]);
    centerOffset_k2 = 0.4*gokartforward(z(index1.theta_k2))';
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
    
    realPos_k3 = z([index1.x_k3,index1.y_k3]);
    centerOffset_k3 = 0.4*gokartforward(z(index1.theta_k3))';
    centerPos_k3 = realPos_k3+centerOffset_k3;%+0.4*forward;
    wantedpos_k3 = [splx_k3;sply_k3];
    wantedpos1_k3= [splx_k3;sply_k3]+r/2*sidewards_k3;
    error_k3 = centerPos_k3-wantedpos_k3;
    error1_k3= centerPos_k3-wantedpos1_k3;
    lagerror_k3 = forward_k3'*error_k3;
    laterror_k3 = sidewards_k3'*error_k3;
    laterror1_k3 = sidewards_k3'*error1_k3;
    
    %% Costs objective function

    %slack = z(index.slack);
    slack2 = z(index1.slack2);
    slack3 = z(index1.slack3);
    slack4 = z(index1.slack4);
    %slack_k2 = z(index.slack_k2);
    %slack_k3 = z(index.slack_k3);
       
    speedcost = max(z(index1.v)-vmax,0)^2*pspeedcost;
    speedcost1 = min(z(index1.v)-vmax,0)^2*pprog;
    lagcost = plagerror*lagerror^2;
    latcost = pslack*latErrorPunisher(laterror);
    latcost1 = platerror*laterror1^2;
    %prog = -pprog*z(index.ds);
    regAB = z(index1.dotab).^2*pab;
    regBeta= z(index1.dotbeta).^2*pdotbeta;
        
    speedcost_k2 = max(z(index1.v_k2)-vmax,0)^2*pspeedcost;
    speedcost1_k2 = min(z(index1.v_k2)-vmax,0)^2*pprog;
    lagcost_k2 = plagerror*lagerror_k2^2;
    latcost_k2 = pslack*latErrorPunisher(laterror_k2);
    latcost1_k2 = platerror*laterror1_k2^2;
    %prog_k2 = -pprog*z(index.ds_k2);
    regAB_k2 = z(index1.dotab_k2).^2*pab;
    regBeta_k2= z(index1.dotbeta_k2).^2*pdotbeta;
    
    speedcost_k3 = max(z(index1.v_k3)-vmax,0)^2*pspeedcost;
    speedcost1_k3 = min(z(index1.v_k3)-vmax,0)^2*pprog;
    lagcost_k3 = plagerror*lagerror_k3^2;
    latcost_k3 = pslack*latErrorPunisher(laterror_k3);
    latcost1_k3 = platerror*laterror1_k3^2;
    regAB_k3 = z(index1.dotab_k3).^2*pab;
    regBeta_k3= z(index1.dotbeta_k3).^2*pdotbeta;
    %prog_k3 = -pprog*z(index.ds_k3);

     f = (lagcost+latcost1   +regAB   +regBeta+speedcost1+speedcost)+...prog+pslack*slack+
         (+lagcost_k2+latcost1_k2+regAB_k2+regBeta_k2+speedcost1_k2+speedcost_k2)+...+prog_k2+pslack*slack_k2+
         (+lagcost_k3+latcost1_k3+regAB_k3+regBeta_k3+speedcost1_k3+speedcost_k3);%+prog_k3+pslack*slack_k3+
     %f = +pslack2*slack2+pslack2*slack3+pslack2*slack4
end
