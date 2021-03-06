function [ACCX,ACCY,ACCROTZ] = modelDx_NN_1x16_tanh_reg0p001(VELX,VELY,VELROTZ,BETA,AB,TV, param)
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
    
    w1 = [1.1728417 0.42591736 0.07986251 -0.26196322 -0.111046776 0.3882618 0.14955446 0.55110955 0.76614505 -0.54954004 -1.6756954 0.38198626 -0.7477387 0.39040023 -0.21965227 -0.67703044;2.0221987 -0.15497968 0.19029778 2.5350294 -0.024048682 2.637351 -0.36177856 -0.2556143 0.5592595 -0.25240284 2.3278055 -0.018579027 0.6659208 0.3453348 -0.30171245 -0.48442078;-1.0789828 0.841098 0.51536334 -1.1837924 -0.3679383 -1.2435391 -0.95330274 -0.2683116 -0.6279335 -0.7865869 -1.4909061 -0.6478992 -0.52995765 -0.72653735 -0.5739707 0.90168315;0.27301604 -0.2675288 -0.4170837 -0.08334321 0.26246232 -0.04211974 -0.16477358 1.2755725 0.47356373 -0.26790494 0.16625343 0.055532157 0.64730567 0.428596 1.606313 0.1897964;0.02369489 -0.012651676 -0.026245061 -0.003996557 0.2678374 -0.014708901 -0.047005773 -0.09877063 -0.027458642 -0.1645989 0.01834283 -0.03037514 0.04759624 -0.021039221 -0.03111372 -0.17045172;-0.23938161 0.0012546912 0.43657443 -0.077259526 0.38505265 -0.130038 0.9445111 -0.002511514 -0.21473193 0.69621414 -0.1702106 -0.07207764 -0.07940829 0.15200962 -0.06654568 -0.21843578];
    b1 = [1.8475224 -1.2549827 -0.1962633 -0.3298234 0.80673426 0.640694 0.3152667 -1.1467817 -0.4003311 -0.61594576 -2.3228838 -1.076803 0.67274034 -1.1385134 0.2423011 -0.71107];
    w2 = [0.16316189 2.13896 0.79036057;0.37449324 2.3308282 -0.8352366;0.12887228 0.5608615 -1.4616047;-0.008008634 1.9008771 0.65114677;-0.68911785 0.074288554 -0.90398175;0.0778062 2.0033126 1.0040292;0.22309507 -0.36679304 -1.2190633;-0.2020314 -0.38713527 -1.8193603;0.058975272 -1.7697493 -1.0362872;-0.00385004 0.060055308 -1.690763;-0.0021349986 2.60819 0.096386895;-0.008616919 -1.5057976 1.0487216;0.13493003 -2.0541267 -1.7367506;0.18300298 -2.4046128 -0.4321836;-0.093665764 -0.34950632 -1.4536977;0.28274417 -0.0839471 1.6701285];
    b2 = [0.6033309 -0.19436386 -0.2893267];
    means = [2.6775749832578986 0.001922168301437502 -0.04515304867653743 -0.026195759869689427 0.06460829512732788 -0.0686602322088558];
    stds = [1.907241556285694 0.3079627568782051 0.5515613235298231 0.20101736434516326 0.8309067855283836 0.634574245360539];
    
    input = [VELX,VELY,VELROTZ,BETA,AB,TV];

    normed_input = (input - means) ./ stds;

    h1 = tanh(normed_input * w1 + b1);
    disturbance = h1 * w2 + b2;
    
    ACCX = ACCX_NOM + disturbance(1);
    ACCY = ACCY_NOM + disturbance(2);
    ACCROTZ = ACCROTZ_NOM + disturbance(3);
end

