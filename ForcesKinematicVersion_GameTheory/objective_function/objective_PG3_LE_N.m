function f = objective_PG3_LE_N(z,points,points2,points3,vmax,plagerror, platerror, pprog, pab, pdotbeta, pspeedcost,pslack,pslack2)
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
    %r=1.5;
    realPos = z([index.x,index.y]);
    centerOffset = 0.4*gokartforward(z(index.theta))';
    centerPos = realPos+centerOffset;%+0.4*forward;
    wantedpos = [splx;sply];
    %wantedpos1= [splx;sply]+r/2*sidewards;
    error = centerPos-wantedpos;
   % error1= centerPos-wantedpos1;
    lagerror = forward'*error;
    laterror = sidewards'*error;
    %laterror1 = sidewards'*error1;
    
    forward_k2 = [spldx_k2;spldy_k2];
    sidewards_k2 = [splsx_k2;splsy_k2];
    
    realPos_k2 = z([index.x_k2,index.y_k2]);
    centerOffset_k2 = 0.4*gokartforward(z(index.theta_k2))';
    centerPos_k2 = realPos_k2+centerOffset_k2;%+0.4*forward;
    wantedpos_k2 = [splx_k2;sply_k2];
   % wantedpos1_k2= [splx_k2;sply_k2]+r/2*sidewards_k2;
    error_k2 = centerPos_k2-wantedpos_k2;
    %error1_k2= centerPos_k2-wantedpos1_k2;
    lagerror_k2 = forward_k2'*error_k2;
    laterror_k2 = sidewards_k2'*error_k2;
    %laterror1_k2 = sidewards_k2'*error1_k2;
    
    forward_k3 = [spldx_k3;spldy_k3];
    sidewards_k3 = [splsx_k3;splsy_k3];
    
    realPos_k3 = z([index.x_k3,index.y_k3]);
    centerOffset_k3 = 0.4*gokartforward(z(index.theta_k3))';
    centerPos_k3 = realPos_k3+centerOffset_k3;%+0.4*forward;
    wantedpos_k3 = [splx_k3;sply_k3];
    %wantedpos1_k3= [splx_k3;sply_k3]+r/2*sidewards_k3;
    error_k3 = centerPos_k3-wantedpos_k3;
    %error1_k3= centerPos_k3-wantedpos1_k3;
    lagerror_k3 = forward_k3'*error_k3;
    laterror_k3 = sidewards_k3'*error_k3;
    %laterror1_k3 = sidewards_k3'*error1_k3;
    
    %% Costs objective function

    %slack = z(index.slack);
    slack2 = z(index.slack2);
    slack3 = z(index.slack3);
    slack4 = z(index.slack4);
    %slack_k2 = z(index.slack_k2);
    %slack_k3 = z(index.slack_k3);
       
    speedcost = max(z(index.v)-vmax,0)^2*pspeedcost;
    speedcost1 = min(z(index.v)-vmax,0)^2*pprog;
    lagcost = plagerror*lagerror^2;
    latcost = platerror*laterror^2;
    %latcost1 = pprog*laterror1^2;
    %prog = -pprog*z(index.ds);
    regAB = z(index.dotab).^2*pab;
    regBeta= z(index.dotbeta).^2*pdotbeta;
        
    speedcost_k2 = max(z(index.v_k2)-vmax,0)^2*pspeedcost;
    speedcost1_k2 = min(z(index.v_k2)-vmax,0)^2*pprog;
    lagcost_k2 = plagerror*lagerror_k2^2;
    latcost_k2 = platerror*laterror_k2^2;
    %latcost1_k2 = pprog*laterror1_k2^2;
    %prog_k2 = -pprog*z(index.ds_k2);
    regAB_k2 = z(index.dotab_k2).^2*pab;
    regBeta_k2= z(index.dotbeta_k2).^2*pdotbeta;
    
    speedcost_k3 = max(z(index.v_k3)-vmax,0)^2*pspeedcost;
    speedcost1_k3 = min(z(index.v_k3)-vmax,0)^2*pprog;
    lagcost_k3 = plagerror*lagerror_k3^2;
    latcost_k3 = platerror*laterror_k3^2;
    %latcost1_k3 = pprog*laterror1_k3^2;
    regAB_k3 = z(index.dotab_k3).^2*pab;
    regBeta_k3= z(index.dotbeta_k3).^2*pdotbeta;
    %prog_k3 = -pprog*z(index.ds_k3);
    
%   +speedcost1+speedcost)+...prog+pslack*slack+
%   +speedcost1_k2+speedcost_k2)+...+prog_k2+pslack*slack_k2+
%   +speedcost1_k3+speedcost_k3)+...+prog_k3+pslack*slack_k3+
    f = (lagcost   +latcost   +regAB   +regBeta+ 0.1*((z(index.x)-10)^2+(z(index.y)-51.5))^2)+...prog+pslack*slack+
        (lagcost_k2+latcost_k2+regAB_k2+regBeta_k2+0.1*((z(index.x_k2)-51.5)^2+(z(index.y_k2)-80))^2)+...+prog_k2+pslack*slack_k2+
        (lagcost_k3+latcost_k3+regAB_k3+regBeta_k3+0.1*((z(index.x_k3)-80)^2+(z(index.y_k3)-48.5))^2)+...+prog_k3+pslack*slack_k3+
        pslack2*slack2+pslack2*slack3+pslack2*slack4;
end
