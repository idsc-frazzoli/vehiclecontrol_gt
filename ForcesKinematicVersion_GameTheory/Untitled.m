 global index
 for ii=1:length(outputM(:,index.x))
%get the fancy spline
    [splx,sply] = casadiDynamicBSPLINE(outputM(ii,index.s),points);
    [spldx, spldy] = casadiDynamicBSPLINEforward(outputM(ii,index.s),points);
    [splsx, splsy] = casadiDynamicBSPLINEsidewards(outputM(ii,index.s),points);
    
    [splx_k2,sply_k2] = casadiDynamicBSPLINE(outputM(ii,index.s_k2),points2);
    [spldx_k2, spldy_k2] = casadiDynamicBSPLINEforward(outputM(ii,index.s_k2),points2);
    [splsx_k2, splsy_k2] = casadiDynamicBSPLINEsidewards(outputM(ii,index.s_k2),points2);
    
    [splx_k3,sply_k3] = casadiDynamicBSPLINE(outputM(ii,index.s_k3),points3);
    [spldx_k3, spldy_k3] = casadiDynamicBSPLINEforward(outputM(ii,index.s_k3),points3);
    [splsx_k3, splsy_k3] = casadiDynamicBSPLINEsidewards(outputM(ii,index.s_k2),points3);
    r=3;
    forward = [spldx;spldy];
    sidewards = [splsx;splsy];
    middleline(ii,:)=[splx;sply];
    rightline(ii,:) = [splx;sply]+r*sidewards;
    rightline1(ii,:) = [splx;sply]+r/2*sidewards;
    leftline(ii,:) = [splx;sply]-r*sidewards;
    leftline1(ii,:) = [splx;sply]-r/2*sidewards;
    
    realPos = [outputM(ii,index.x);outputM(ii,index.y)];
    centerOffset = 0.4*gokartforward(outputM(ii,index.theta))';
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
    
    realPos_k2 = [outputM(ii,index.x_k2);outputM(ii,index.y_k2)];
    centerOffset_k2 = 0.4*gokartforward(outputM(ii,index.theta_k2))';
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
    
    realPos_k3 = [outputM(ii,index.x_k3);outputM(ii,index.y_k3)];
    centerOffset_k3 = 0.4*gokartforward(outputM(ii,index.theta_k3))';
    centerPos_k3 = realPos_k3+centerOffset_k3;%+0.4*forward;
    wantedpos_k3 = [splx_k3;sply_k3];
    wantedpos1_k3= [splx_k3;sply_k3]+r/2*sidewards_k3;
    error_k3 = centerPos_k3-wantedpos_k3;
    error1_k3= centerPos_k3-wantedpos1_k3;
    lagerror_k3 = forward_k3'*error_k3;
    laterror_k3 = sidewards_k3'*error_k3;
    laterror1_k3 = sidewards_k3'*error1_k3;
    %% Costs objective function

    slack = outputM(ii,index.slack);
    slack2 = outputM(ii,index.slack2);
    slack3 = outputM(ii,index.slack3);
    slack4 = outputM(ii,index.slack4);
    slack_k2 = outputM(ii,index.slack_k2);
    slack_k3 = outputM(ii,index.slack_k3);
       
    speedcost = speedPunisher(outputM(ii,index.v),maxSpeed)*pspeedcost;
    lagcost = plagerror*lagerror^2;
    latcost = platerror*latErrorPunisher(laterror);
    latcost1 = platerror/200*latErrorPunisher(laterror1);
    prog = -pprog*outputM(ii,index.ds);
    reg = outputM(ii,index.dotab).^2*pab+outputM(ii,index.dotbeta).^2*pdotbeta;
    
    speedcost_k2 = speedPunisher(outputM(ii,index.v_k2),maxSpeed)*pspeedcost;
    lagcost_k2 = plagerror*lagerror_k2^2;
    latcost_k2 = platerror*latErrorPunisher(laterror_k2);
    latcost1_k2 = platerror/200*latErrorPunisher(laterror1_k2);
    prog_k2 = -pprog*outputM(ii,index.ds_k2);
    reg_k2 = outputM(ii,index.dotab_k2).^2*pab+outputM(ii,index.dotbeta_k2).^2*pdotbeta;
    
    speedcost_k3 = speedPunisher(outputM(ii,index.v_k3),maxSpeed)*pspeedcost;
    lagcost_k3 = plagerror*lagerror_k3^2;
    latcost_k3 = platerror*latErrorPunisher(laterror_k3);
    latcost1_k3 = platerror/200*latErrorPunisher(laterror1_k3);
    prog_k3 = -pprog*outputM(ii,index.ds_k3);
    reg_k3 = outputM(ii,index.dotab_k3).^2*pab+outputM(ii,index.dotbeta_k3).^2*pdotbeta;
    
    f = (lagcost+latcost+latcost1+reg+prog+speedcost)+pslack*slack+...
        (lagcost_k2+latcost_k2+latcost1_k2+reg_k2+prog_k2+speedcost_k2)+pslack*slack_k2+...
        (lagcost_k3+latcost_k3+latcost1_k3+reg_k3+prog_k3+speedcost_k3)+pslack*slack_k3+...
        pslack2*slack2+pslack2*slack3+pslack2*slack4;
 end