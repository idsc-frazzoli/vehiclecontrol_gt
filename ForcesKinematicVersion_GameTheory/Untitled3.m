optA=0;
regAB1=0;
regBetaA=0;
latcostA=0;
lagcostA=0;
slackA=0;
slackB=0;
speedcostA=0;
for uu=1:length(outputM)
     [lagcost,latcost,regAB,regBeta,slack,slack2,speedcost,f] = objective_IBR_LE_Test(outputM3(uu,:),points,maxSpeed, plagerror, platerror,...
                       pprog, pab, pdotbeta, pspeedcost,pslack,pslack2);

    regAB1=regAB1+regAB;
    regBetaA=regBetaA+regBeta;
    latcostA=latcostA+latcost;
    lagcostA=lagcostA+lagcost;
    speedcostA=speedcostA+speedcost;
    optA = optA+ f;
    slackA=slackA+pslack*slack;
    slackB=slackB+pslack2*slack2;
end