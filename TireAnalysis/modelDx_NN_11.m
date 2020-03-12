function [ACCX,ACCY,ACCROTZ] = modelDx_NN_11(VELX,VELY,VELROTZ,BETA,AB,TV, param)
    %param = [B1,C1,D1,B2,C2,D2,Ic];
    B1 = param(1);
    C1 = param(2);
    D1 = param(3);
    B2 = param(4);
    C2 = param(5);
    D2 = param(6);
    Ic = param(7);
    %maxA = param(8);
    magic = @(s,B,C,D)D.*sin(C.*atan(B.*s));
    reg = 0.5;
    capfactor = @(taccx)(1-satfun((taccx/D2)^2))^(1/2);
    simpleslip = @(VELY,VELX,taccx)-(1/capfactor(taccx))*VELY/(VELX+reg);
    %simpleslip = @(VELY,VELX,taccx)-VELY/(VELX+reg);
    simplediraccy = @(VELY,VELX,taccx)magic(simpleslip(VELY,VELX,taccx),B2,C2,D2);
    simpleaccy = @(VELY,VELX,taccx)capfactor(taccx)*simplediraccy(VELY,VELX,taccx);
    %acclim = @(VELY,VELX, taccx)(VELX^2+VELY^2)*taccx^2-VELX^2*maxA^2;
    simplefaccy = @(VELY,VELX)magic(-VELY/(VELX+reg),B1,C1,D1);
    %simpleaccy = @(VELY,VELX,taccx)magic(-VELY/(VELX+reg),B2,C2,D2);



    l = 1.19;
    l1 = 0.73;
    l2 = l-l1;
    f1n = l2/l;
    f2n = l1/l;
    w = 1;
    rotmat = @(beta)[cos(beta),sin(beta);-sin(beta),cos(beta)];
    vel1 = rotmat(BETA)*[VELX;VELY+l1*VELROTZ];
    f1y = simplefaccy(vel1(2),vel1(1));
    F1 = rotmat(-BETA)*[0;f1y]*f1n;
    F1x = F1(1);
    F1y = F1(2);
    frontabcorr = F1x;
    F2x = AB;
    F2y1 = simpleaccy(VELY-l2*VELROTZ,VELX,(AB+TV/2)/f2n)*f2n/2;
    F2y2 = simpleaccy(VELY-l2*VELROTZ,VELX,(AB-TV/2)/f2n)*f2n/2;
    F2y = simpleaccy(VELY-l2*VELROTZ,VELX,AB/f2n)*f2n;
    TVTrq = TV*w;
    
    
    ACCROTZ_NOM = (TVTrq + F1y*l1 -F2y*l2)/Ic;
    %ACCROTZ = TVTrq + F1y*l1;
    ACCX_NOM = F1x+F2x+VELROTZ*VELY;
    ACCY_NOM = F1y+F2y1+F2y2-VELROTZ*VELX;
    
    w1 = [0.029378539 0.029378537 0.2887413 0.02937854 0.02937854 0.029378539 0.02937854 0.02937854 0.02937854 0.029378539 0.35226983 0.02937854 0.029378537 0.029378539 0.029378539 0.02937854;-0.012976049 -0.012976046 0.12697396 -0.01297605 -0.01297605 -0.012976046 -0.01297605 -0.01297605 -0.01297605 -0.012976045 -0.24393772 -0.012976049 -0.012976046 -0.012976046 -0.012976045 -0.012976049;-0.009619812 -0.009619809 0.45193237 -0.009619813 -0.009619813 -0.009619807 -0.009619813 -0.009619813 -0.009619813 -0.009619807 -0.41658977 -0.009619812 -0.009619809 -0.009619809 -0.009619808 -0.009619812;-0.004344045 -0.0043440447 0.29361883 -0.0043440466 -0.004344047 -0.004344045 -0.004344047 -0.004344047 -0.004344047 -0.004344045 -0.24659336 -0.004344045 -0.0043440447 -0.0043440447 -0.004344045 -0.004344045;-0.00848084 -0.008480839 0.0063124048 -0.00848084 -0.00848084 -0.00848084 -0.008480839 -0.00848084 -0.008480839 -0.00848084 -0.061166167 -0.00848084 -0.00848084 -0.00848084 -0.008480841 -0.00848084;-0.01038454 -0.010384533 0.70850396 -0.010384545 -0.010384545 -0.010384529 -0.010384545 -0.010384545 -0.010384545 -0.010384529 -0.79405665 -0.01038454 -0.010384533 -0.010384533 -0.010384529 -0.01038454];
    b1 = [0.013768503 0.013768503 0.25082174 0.013768506 0.013768505 0.013768505 0.013768506 0.013768505 0.013768506 0.013768504 0.32373732 0.0137685025 0.013768504 0.013768504 0.013768505 0.013768502];
    w2 = [0.08572511 0.0007932616 0.00079326186 0.00079325965 0.0007932616 0.00079326186 0.0007932616 0.0007932606 0.00079325965 0.0007932622 0.00079325965 0.0007932591 0.107111104 0.0007932591 0.0007932607 0.0007932606;0.085725114 0.00079326116 0.0007932607 0.00079325965 0.00079326116 0.0007932607 0.00079326116 0.0007932606 0.00079325994 0.0007932612 0.0007932598 0.0007932598 0.1071111 0.0007932598 0.0007932605 0.0007932604;0.8386833 -0.0073592714 -0.0073592723 -0.007359274 -0.0073592723 -0.0073592723 -0.007359272 -0.0073592705 -0.007359274 -0.0073592714 -0.007359274 -0.0073592714 -0.42718312 -0.0073592714 -0.007359271 -0.0073592714;0.08572511 0.00079326093 0.00079326116 0.00079326046 0.0007932608 0.00079326116 0.0007932608 0.000793261 0.0007932598 0.0007932615 0.00079326 0.0007932596 0.10711111 0.0007932596 0.00079326093 0.000793261;0.08572511 0.00079326093 0.0007932616 0.0007932602 0.00079326093 0.0007932616 0.00079326093 0.0007932607 0.0007932598 0.0007932613 0.00079326 0.0007932595 0.10711111 0.0007932595 0.0007932607 0.0007932607;0.08572512 0.0007932614 0.00079326093 0.0007932608 0.0007932614 0.00079326093 0.00079326093 0.0007932604 0.0007932608 0.0007932615 0.00079326046 0.0007932594 0.1071111 0.0007932594 0.0007932608 0.00079326105;0.08572511 0.00079326093 0.0007932614 0.00079326046 0.00079326116 0.0007932614 0.00079326093 0.0007932607 0.0007932601 0.0007932613 0.0007932602 0.0007932591 0.10711111 0.0007932591 0.00079326093 0.0007932607;0.08572511 0.00079326093 0.0007932616 0.0007932602 0.00079326093 0.0007932616 0.00079326093 0.0007932607 0.0007932598 0.0007932613 0.00079326 0.0007932595 0.10711111 0.0007932595 0.0007932607 0.0007932607;0.08572511 0.00079326093 0.0007932614 0.00079326046 0.00079326116 0.0007932614 0.00079326093 0.0007932614 0.00079326046 0.0007932613 0.0007932602 0.0007932589 0.10711111 0.0007932589 0.0007932614 0.000793261;0.08572512 0.0007932616 0.0007932616 0.0007932605 0.00079326134 0.0007932616 0.0007932616 0.0007932604 0.0007932605 0.000793262 0.00079326006 0.0007932594 0.1071111 0.0007932594 0.0007932606 0.00079326105;-0.37409368 -0.025425997 -0.025425997 -0.02542599 -0.025425997 -0.025425997 -0.025425997 -0.025425997 -0.02542599 -0.025425997 -0.02542599 -0.025425991 0.9650265 -0.025425991 -0.025425997 -0.025425997;0.08572511 0.00079326186 0.00079326145 0.0007932594 0.0007932614 0.00079326145 0.0007932616 0.000793261 0.0007932592 0.0007932615 0.0007932594 0.00079325924 0.107111104 0.00079325924 0.0007932607 0.000793261;0.085725114 0.00079326116 0.00079326093 0.00079326046 0.00079326116 0.00079326093 0.00079326105 0.0007932606 0.00079326046 0.00079326204 0.00079326 0.00079325936 0.1071111 0.00079325936 0.00079326105 0.0007932606;0.085725114 0.00079326116 0.00079326093 0.00079326 0.00079326116 0.00079326093 0.00079326105 0.0007932606 0.00079326 0.0007932613 0.00079325965 0.0007932591 0.1071111 0.0007932591 0.0007932608 0.00079326105;0.08572512 0.00079326116 0.00079326093 0.00079326076 0.00079326116 0.00079326093 0.00079326093 0.0007932602 0.00079326046 0.0007932613 0.00079326046 0.0007932591 0.1071111 0.0007932591 0.0007932608 0.00079326105;0.08572511 0.0007932614 0.0007932616 0.0007932594 0.0007932614 0.0007932616 0.0007932614 0.000793261 0.0007932592 0.0007932615 0.0007932592 0.00079325924 0.107111104 0.00079325924 0.0007932607 0.000793261];
    b2 = [0.1306587 0.0040432736 0.004043272 0.0040432713 0.0040432736 0.004043272 0.0040432727 0.004043273 0.004043271 0.0040432736 0.0040432718 0.0040432718 0.12834089 0.0040432718 0.0040432727 0.004043272];
    w3 = [0.008478813 0.32606542 -0.83446866;-0.008090033 0.08053848 0.018680865;-0.008090034 0.08053848 0.018680865;-0.008090033 0.08053847 0.018680867;-0.008090033 0.08053848 0.018680865;-0.008090034 0.08053848 0.018680865;-0.008090034 0.08053848 0.018680865;-0.008090034 0.08053848 0.018680861;-0.008090034 0.08053847 0.018680867;-0.008090033 0.08053848 0.018680865;-0.008090033 0.08053847 0.018680867;-0.008090034 0.08053847 0.018680861;0.041020688 -0.5868233 0.84365;-0.008090034 0.08053847 0.018680861;-0.008090033 0.08053848 0.018680861;-0.008090034 0.08053848 0.018680861];
    b3 = [-0.010615546 0.10432406 0.03346752];
    means = [2.672327304368387 0.001339023281537436 -0.03454558037830045 -0.023152252504970373 0.06404004662117853 -0.0617760867753043];
    stds = [1.916055703856533 0.3142763123175873 0.547953525421002 0.20097633791253725 0.8316641470766218 0.635200176995599];

    input = [VELX,VELY,VELROTZ,BETA,AB,TV];

    normed_input = (input - means) ./ stds;

    h1 = log(exp(normed_input * w1 + b1) + 1);
    h2 = log(exp(h1 * w2 + b2) + 1);
    disturbance = h2 * w3 + b3;
    
    ACCX = ACCX_NOM + disturbance(1);
    ACCY = ACCY_NOM + disturbance(2);
    ACCROTZ = ACCROTZ_NOM + disturbance(3);
end

