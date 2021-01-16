points2 = [50,50,50,50,50,50,50,50,50,50,50,50,50,50;...          %x,50,50,50,50,50
          25,30,35,40,45,50,55,60,65,70,75,80,85,90; ...    %y,75,80,85,90,95
          3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5]';  %,5,5,5,5,5
 idx=38;
 AA=0;
%  for idx=1:length(outputM1)
    [splx_k2,sply_k2] = casadiDynamicBSPLINE(outputM1(idx,index1.s_k2),points2);
    %[spldx_k2, spldy_k2] = casadiDynamicBSPLINEforward(x(index1.s_k2-index1.nu),points2);
    [splsx_k2, splsy_k2] = casadiDynamicBSPLINEsidewards(outputM1(idx,index1.s_k2),points2);
    
    %forward_k2 = [spldx_k2;spldy_k2];
    sidewards_k2 = [splsx_k2;splsy_k2];
    
    realPos_k2 = [outputM1(idx,index1.x_k2),outputM1(idx,index1.y_k2)]';
    centerOffset_k2 = 0.4*gokartforward(outputM1(idx,index1.theta_k2))';
    centerPos_k2 = realPos_k2+centerOffset_k2;
    wantedpos_k2 = [splx_k2;sply_k2];
    error_k2 = centerPos_k2-wantedpos_k2;
    laterror_k2 = sidewards_k2'*error_k2;
    AA=AA+0.1*(min(0,laterror_k2))^2;
%  end
    for jj=1:length(outputM2)
        [lagcost,latcost,regAB,regBeta,speedcost,speedcost1,lagcost_k2,...
        latcost_k2,regAB_k2,regBeta_k2,speedcost_k2,speedcost1_k2,lagcost_k3,...
        latcost_k3,regAB_k3,regBeta_k3,speedcost_k3,speedcost1_k3,f,f1,f2,f3]  =...
        objective_PG_Test3(outputM2(jj,:),points,points2,points3,targetSpeed,plagerror_1,...
        platerror_1, pprog, pab, pdotbeta, pspeedcost,pslack,pslack2_1);
      
        regABA=regABA+regAB;
        regABB=regABB+regAB_k2;
        regABC=regABC+regAB_k3;
        regBetaA=regBetaA+regBeta;
        regBetaB=regBetaB+regBeta_k2;
        regBetaC=regBetaC+regBeta_k3;
        latcostA=latcostA+latcost;
        latcostB=latcostB+latcost_k2;
        latcostC=latcostC+latcost_k3;
        lagcostA=lagcostA+lagcost;
        lagcostB=lagcostB+lagcost_k2;
        lagcostC=lagcostC+lagcost_k3;
        speedcostA=speedcostA+speedcost;
        speedcostB=speedcostB+speedcost_k2;
        speedcostC=speedcostC+speedcost_k3;
        speedcostA1=speedcostA1+speedcost1;
        speedcostB1=speedcostB1+speedcost1_k2;
        speedcostC1=speedcostC1+speedcost1_k3;
        optA = optA+ f1;
        optB = optB+ f2;
        optC = optC+ f3;
        opt  = opt+ f;
    end