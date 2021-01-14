function v = nlconst_PG3_N_1(z,p)
global index1
%NLCONST Summary of this function goes here
%   Detailed explanation goes here
dist=p(index1.dist);
pslack2=p(index1.pslack2);
plagerror=p(index1.plag);
% l = 1.19;
% beta  = z(index.beta);
% dotbeta = z(index.dotbeta);
% forwardacc = z(index.ab);
%slack = z(index.slack);

slack2 = z(index1.slack2);
slack3 = z(index1.slack3);
slack4 = z(index1.slack4);

% beta_k2  = z(index.beta_k2);
% dotbeta_k2 = z(index.dotbeta_k2);
% forwardacc_k2 = z(index.ab_k2);
%slack_k2 = z(index.slack_k2);
% 
% beta_k3  = z(index.beta_k3);
% dotbeta_k3 = z(index.dotbeta_k3);
% forwardacc_k3 = z(index.ab_k3);
%slack_k3 = z(index.slack_k3);

% Splines Control points and radii
pointsO = index1.pointsO;
pointsN = index1.pointsN;
pointsN2 = index1.pointsN2;
pointsN3 = index1.pointsN3;

points = getPointsFromParameters(p, pointsO, pointsN);
radii = getRadiiFromParameters(p, pointsO, pointsN);

points_k2 = getPointsFromParameters(p, pointsO+3*pointsN, pointsN2);
radii_k2 = getRadiiFromParameters(p, pointsO+3*pointsN, pointsN2);

points_k3 = getPointsFromParameters(p, pointsO+3*pointsN+3*pointsN2, pointsN3);
radii_k3 = getRadiiFromParameters(p, pointsO+3*pointsN+3*pointsN2, pointsN3);
% Splines
[splx,sply] = casadiDynamicBSPLINE(z(index1.s),points);
[spldx, spldy] = casadiDynamicBSPLINEforward(z(index1.s),points);
[splsx, splsy] = casadiDynamicBSPLINEsidewards(z(index1.s),points);
r = casadiDynamicBSPLINERadius(z(index1.s),radii);

[splx_k2,sply_k2] = casadiDynamicBSPLINE(z(index1.s_k2),points_k2);
[spldx_k2, spldy_k2] = casadiDynamicBSPLINEforward(z(index1.s_k2),points_k2);
[splsx_k2, splsy_k2] = casadiDynamicBSPLINEsidewards(z(index1.s_k2),points_k2);
r_k2 = casadiDynamicBSPLINERadius(z(index1.s_k2),radii_k2);

[splx_k3,sply_k3] = casadiDynamicBSPLINE(z(index1.s_k3),points_k3);
[spldx_k3, spldy_k3] = casadiDynamicBSPLINEforward(z(index1.s_k3),points_k3);
[splsx_k3, splsy_k3] = casadiDynamicBSPLINEsidewards(z(index1.s_k3),points_k3);
r_k3 = casadiDynamicBSPLINERadius(z(index1.s_k3),radii_k3);

forward = [spldx;spldy];
sidewards = [splsx;splsy];

realPos = z([index1.x,index1.y]);
centerOffset = 0.4*gokartforward(z(index1.theta))';
centerPos = realPos+centerOffset;%+0.4*forward;
wantedpos = [splx;sply];

% Position error
error = centerPos-wantedpos;

% Projection errors
lagerror = forward'*error;
laterror = sidewards'*error;

forward_k2 = [spldx_k2;spldy_k2];
sidewards_k2 = [splsx_k2;splsy_k2];

realPos_k2 = z([index1.x_k2,index1.y_k2]);
centerOffset_k2 = 0.4*gokartforward(z(index1.theta_k2))';
centerPos_k2 = realPos_k2+centerOffset_k2;%+0.4*forward;
wantedpos_k2 = [splx_k2;sply_k2];

% Position error
error_k2 = centerPos_k2-wantedpos_k2;

% Projection errors
lagerror_k2 = forward_k2'*error_k2;
laterror_k2 = sidewards_k2'*error_k2;

forward_k3 = [spldx_k3;spldy_k3];
sidewards_k3 = [splsx_k3;splsy_k3];

realPos_k3 = z([index1.x_k3,index1.y_k3]);
centerOffset_k3 = 0.4*gokartforward(z(index1.theta_k3))';
centerPos_k3 = realPos_k3+centerOffset_k3;%+0.4*forward;
wantedpos_k3 = [splx_k3;sply_k3];

% Position error
error_k3 = centerPos_k3-wantedpos_k3;

% Projection errors
lagerror_k3 = forward_k3'*error_k3;
laterror_k3 = sidewards_k3'*error_k3;
plagcost = plagerror*lagerror^2;
plagcost_k2 = plagerror*lagerror_k2^2;
plagcost_k3 = plagerror*lagerror_k3^2;
% %parameters
% vmax =  p(index.ps);
% maxxacc = p(index.pax);
% maxyacc = p(index.pay);
% latacclim = p(index.pll);
% rotacceffect = p(index.prae);
% torqueveceffect = p(index.ptve);
% brakeeffect = p(index.pbre);
% 
% ackermannAngle = -0.58*beta*beta*beta+0.93*beta;
% dAckermannAngle = -0.58*3*beta*beta*dotbeta+0.93*dotbeta;
% tangentspeed = z(index.v);
% latacc = (tan(ackermannAngle)*tangentspeed^2)/l;
% %avoid oversteer
% accnorm = ((latacc/maxyacc)^2+(z(index.ab)/maxxacc)^2);

%avoid understeer
% rotacc = dAckermannAngle*tangentspeed/l;
% frontaxlelatacc = latacc+rotacc*rotacceffect;
% 
% %simplified 
% accbudget = (1.8-forwardacc)/1.6;
% torquevectoringcapability = accbudget*torqueveceffect;

% ackermannAngle_k2 = -0.58*beta_k2*beta_k2*beta_k2+0.93*beta_k2;
% dAckermannAngle_k2 = -0.58*3*beta_k2*beta_k2*dotbeta_k2+0.93*dotbeta_k2;
% tangentspeed_k2 = z(index.v_k2);
% latacc_k2 = (tan(ackermannAngle_k2)*tangentspeed_k2^2)/l;
% %avoid oversteer
% accnorm_k2 = ((latacc_k2/maxyacc)^2+(z(index.ab_k2)/maxxacc)^2);

%avoid understeer
% rotacc_k2 = dAckermannAngle_k2*tangentspeed_k2/l;
% frontaxlelatacc_k2 = latacc_k2+rotacc_k2*rotacceffect;
% 
% %simplified 
% accbudget_k2 = (1.8-forwardacc_k2)/1.6;
% torquevectoringcapability_k2 = accbudget_k2*torqueveceffect;

% ackermannAngle_k3 = -0.58*beta_k3*beta_k3*beta_k3+0.93*beta_k3;
% dAckermannAngle_k3 = -0.58*3*beta_k3*beta_k3*dotbeta_k3+0.93*dotbeta_k3;
% tangentspeed_k3 = z(index.v_k3);
% latacc_k3 = (tan(ackermannAngle_k3)*tangentspeed_k3^2)/l;
%avoid oversteer
% accnorm_k3 = ((latacc_k3/maxyacc)^2+(z(index.ab_k3)/maxxacc)^2);

%avoid understeer
% rotacc_k3 = dAckermannAngle_k3*tangentspeed_k3/l;
% frontaxlelatacc_k3 = latacc_k3+rotacc_k3*rotacceffect;
% 
% %simplified 
% accbudget_k3 = (1.8-forwardacc_k3)/1.6;
% torquevectoringcapability_k3 = accbudget_k3*torqueveceffect;
% %torquevectoringcapability = torqueveccapsmooth(forwardacc)*torqueveceffect;
distance_X=(z(index1.x)-z(index1.x_k2));
distance_Y=(z(index1.y)-z(index1.y_k2));

distance_X2=(z(index1.x)-z(index1.x_k3));
distance_Y2=(z(index1.y)-z(index1.y_k3));

distance_X3=(z(index1.x_k2)-z(index1.x_k3));
distance_Y3=(z(index1.y_k2)-z(index1.y_k3));

distance_X4=(z(index1.x_k2)-53);
distance_Y4=(z(index1.y_k2)-58);

squared_distance_array = sqrt(distance_X.^2+distance_Y.^2);
squared_distance_array2 = sqrt(distance_X2.^2+distance_Y2.^2);
squared_distance_array3 = sqrt(distance_X3.^2+distance_Y3.^2);
squared_distance_array4 = sqrt(distance_X4.^2+distance_Y4.^2);

%% Constraints
%v1 = (tan(z(index.beta))*z(index.v)^2/l)^2+z(index.ab)^2;
%v1=(tan(z(8))*z(7)^2/l);
v1 = plagcost+plagcost_k2+plagcost_k3+pslack2*slack2+pslack2*slack3+...
     pslack2*slack4-p(index1.pax);
v2 = z(index1.ab)-casadiGetSmoothMaxAcc(z(index1.v));
v3 = z(index1.x);
v4 = laterror-r;%-0.5*slack
v5 = -laterror-r;%-0.5*slack
%understeerright
%v6 = frontaxlelatacc - latacclim-torquevectoringcapability-slack;
% v6 = frontaxlelatacc -torquevectoringcapability- latacclim-slack;
% %understeerleft
% %v7 = -frontaxlelatacc - latacclim-torquevectoringcapability-slack;
% v7 = -frontaxlelatacc -torquevectoringcapability - latacclim-slack;

v8 = z(index1.ab_k2)-casadiGetSmoothMaxAcc(z(index1.v_k2));
v9 = z(index1.y_k2);
v10 = laterror_k2-r_k2;%-0.5*slack_k2
v11 = -laterror_k2-r_k2;%-0.5*slack_k2
% v12 = frontaxlelatacc_k2-torquevectoringcapability_k2-latacclim-slack_k2;
% v13 = -frontaxlelatacc_k2-torquevectoringcapability_k2-latacclim-slack_k2;

v14 = z(index1.ab_k3)-casadiGetSmoothMaxAcc(z(index1.v_k3));
v15 = z(index1.x_k3);
v16 = laterror_k3-r_k3;%-0.5*slack_k3
v17 = -laterror_k3-r_k3;%-0.5*slack_k3
% v18 = frontaxlelatacc_k3-torquevectoringcapability_k3-latacclim-slack_k3;
% v19 = -frontaxlelatacc_k3-torquevectoringcapability_k3-latacclim-slack_k3;
v20 = -squared_distance_array+dist-slack2;%;%
v21 = -squared_distance_array2+dist-slack3;%
v22 = -squared_distance_array3+dist-slack4;%;%
v23 = -squared_distance_array4+dist-slack3;

v = [v1;v2;v3;v4;v5;v8;v9;v10;v11;v14;v15;v16;v17;v20;v21;v22;v23];%v9;%v15;v18;v15;
end

