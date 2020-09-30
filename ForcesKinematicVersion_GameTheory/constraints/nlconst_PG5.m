function v = nlconst_PG5(z,p)
global index
%NLCONST Summary of this function goes here
%   Detailed explanation goes here
dist=p(index.dist);
slack = z(index.slack);

slack2 = z(index.slack2);
slack3 = z(index.slack3);
slack4 = z(index.slack4);

slack_k2 = z(index.slack_k2);
slack_k3 = z(index.slack_k3);

slack_k4 = z(index.slack_k4);
slack_k5 = z(index.slack_k5);
% Splines Control points and radii
pointsO = index.pointsO;
pointsN = index.pointsN;
pointsN2 = index.pointsN2;
pointsN3 = index.pointsN3;
pointsN4 = index.pointsN4;
pointsN5 = index.pointsN5;
points = getPointsFromParameters   (p, pointsO, pointsN);
radii = getRadiiFromParameters     (p, pointsO, pointsN);

points_k2 = getPointsFromParameters(p, pointsO+3*pointsN, pointsN2);
radii_k2 = getRadiiFromParameters  (p, pointsO+3*pointsN, pointsN2);

points_k3 = getPointsFromParameters(p, pointsO+3*pointsN+3*pointsN2, pointsN3);
radii_k3 = getRadiiFromParameters  (p, pointsO+3*pointsN+3*pointsN2, pointsN3);

points_k4 = getPointsFromParameters(p, pointsO+3*pointsN+3*pointsN2+3*pointsN3, pointsN4);
radii_k4 = getRadiiFromParameters  (p, pointsO+3*pointsN+3*pointsN2+3*pointsN3, pointsN4);

points_k5 = getPointsFromParameters(p, pointsO+3*pointsN+3*pointsN2+3*pointsN3+3*pointsN4, pointsN5);
radii_k5 = getRadiiFromParameters  (p, pointsO+3*pointsN+3*pointsN2+3*pointsN3+3*pointsN4, pointsN5);

% Splines
[splx,sply] = casadiDynamicBSPLINE(z(index.s),points);
[splsx, splsy] = casadiDynamicBSPLINEsidewards(z(index.s),points);
r = casadiDynamicBSPLINERadius(z(index.s),radii);

[splx_k2,sply_k2] = casadiDynamicBSPLINE(z(index.s_k2),points_k2);
[splsx_k2, splsy_k2] = casadiDynamicBSPLINEsidewards(z(index.s_k2),points_k2);
r_k2 = casadiDynamicBSPLINERadius(z(index.s_k2),radii_k2);

[splx_k3,sply_k3] = casadiDynamicBSPLINE(z(index.s_k3),points_k3);
[splsx_k3, splsy_k3] = casadiDynamicBSPLINEsidewards(z(index.s_k3),points_k3);
r_k3 = casadiDynamicBSPLINERadius(z(index.s_k3),radii_k3);

[splx_k4,sply_k4] = casadiDynamicBSPLINE(z(index.s_k4),points_k4);
[splsx_k4, splsy_k4] = casadiDynamicBSPLINEsidewards(z(index.s_k4),points_k4);
r_k4 = casadiDynamicBSPLINERadius(z(index.s_k4),radii_k4);

[splx_k5,sply_k5] = casadiDynamicBSPLINE(z(index.s_k5),points_k5);
[splsx_k5, splsy_k5] = casadiDynamicBSPLINEsidewards(z(index.s_k5),points_k5);
r_k5 = casadiDynamicBSPLINERadius(z(index.s_k5),radii_k5);
%1)
sidewards = [splsx;splsy];

realPos = z([index.x,index.y]);
centerOffset = 0.4*gokartforward(z(index.theta))';
centerPos = realPos+centerOffset;%+0.4*forward;
wantedpos = [splx;sply];

% Position error
error = centerPos-wantedpos;

% Projection errors
laterror = sidewards'*error;
% 2
sidewards_k2 = [splsx_k2;splsy_k2];

realPos_k2 = z([index.x_k2,index.y_k2]);
centerOffset_k2 = 0.4*gokartforward(z(index.theta_k2))';
centerPos_k2 = realPos_k2+centerOffset_k2;%+0.4*forward;
wantedpos_k2 = [splx_k2;sply_k2];

% Position error
error_k2 = centerPos_k2-wantedpos_k2;

% Projection errors
laterror_k2 = sidewards_k2'*error_k2;
% 3
sidewards_k3 = [splsx_k3;splsy_k3];

realPos_k3 = z([index.x_k3,index.y_k3]);
centerOffset_k3 = 0.4*gokartforward(z(index.theta_k3))';
centerPos_k3 = realPos_k3+centerOffset_k3;%+0.4*forward;
wantedpos_k3 = [splx_k3;sply_k3];

% Position error
error_k3 = centerPos_k3-wantedpos_k3;

% Projection errors
laterror_k3 = sidewards_k3'*error_k3;

% 4
sidewards_k4 = [splsx_k4;splsy_k4];

realPos_k4 = z([index.x_k4,index.y_k4]);
centerOffset_k4 = 0.4*gokartforward(z(index.theta_k4))';
centerPos_k4 = realPos_k4+centerOffset_k4;%+0.4*forward;
wantedpos_k4 = [splx_k4;sply_k4];

% Position error
error_k4 = centerPos_k4-wantedpos_k4;

% Projection errors
%lagerror_k2 = forward_k2'*error_k2;
laterror_k4 = sidewards_k4'*error_k4;

% 5
sidewards_k5 = [splsx_k5;splsy_k5];

realPos_k5 = z([index.x_k5,index.y_k5]);
centerOffset_k5 = 0.4*gokartforward(z(index.theta_k5))';
centerPos_k5 = realPos_k5+centerOffset_k5;%+0.4*forward;
wantedpos_k5 = [splx_k5;sply_k5];

% Position error
error_k5 = centerPos_k5-wantedpos_k5;

% Projection errors
%lagerror_k3 = forward_k3'*error_k3;
laterror_k5 = sidewards_k5'*error_k5;

distance_X=(z(index.x)-z(index.x_k2));
distance_Y=(z(index.y)-z(index.y_k2));

distance_X2=(z(index.x)-z(index.x_k3));
distance_Y2=(z(index.y)-z(index.y_k3));

distance_X3=(z(index.x)-z(index.x_k4));
distance_Y3=(z(index.y)-z(index.y_k4));

distance_X4=(z(index.x)-z(index.x_k5));
distance_Y4=(z(index.y)-z(index.y_k5));

distance_X5=(z(index.x_k2)-z(index.x_k3));
distance_Y5=(z(index.y_k2)-z(index.y_k3));

distance_X6=(z(index.x_k2)-z(index.x_k4));
distance_Y6=(z(index.y_k2)-z(index.y_k4));

distance_X7=(z(index.x_k2)-z(index.x_k5));
distance_Y7=(z(index.y_k2)-z(index.y_k5));

distance_X8=(z(index.x_k3)-z(index.x_k4));
distance_Y8=(z(index.y_k3)-z(index.y_k4));

distance_X9=(z(index.x_k3)-z(index.x_k5));
distance_Y9=(z(index.y_k3)-z(index.y_k5));

distance_X10=(z(index.x_k4)-z(index.x_k5));
distance_Y10=(z(index.y_k4)-z(index.y_k5));

squared_distance_array =  sqrt(distance_X .^2+distance_Y .^2);
squared_distance_array2 = sqrt(distance_X2.^2+distance_Y2.^2);
squared_distance_array3 = sqrt(distance_X3.^2+distance_Y3.^2);
squared_distance_array4 = sqrt(distance_X4.^2+distance_Y4.^2);
squared_distance_array5 = sqrt(distance_X5.^2+distance_Y5.^2);
squared_distance_array6 = sqrt(distance_X6.^2+distance_Y6.^2);
squared_distance_array7 = sqrt(distance_X7.^2+distance_Y7.^2);
squared_distance_array8 = sqrt(distance_X8.^2+distance_Y8.^2);
squared_distance_array9 = sqrt(distance_X9.^2+distance_Y9.^2);
squared_distance_array10 = sqrt(distance_X10.^2+distance_Y10.^2);
%% Constraints

v1 = z(index.ab)-casadiGetSmoothMaxAcc(z(index.v));
v2 = laterror-r-0.5*slack;
v3 = -laterror-r-0.5*slack;

v4 = z(index.ab_k2)-casadiGetSmoothMaxAcc(z(index.v_k2));
v5 = laterror_k2-r_k2-0.5*slack_k2;
v6 = -laterror_k2-r_k2-0.5*slack_k2;

v7 = z(index.ab_k3)-casadiGetSmoothMaxAcc(z(index.v_k3));
v8 = laterror_k3-r_k3-0.5*slack_k3;
v9 = -laterror_k3-r_k3-0.5*slack_k3;

v10 = z(index.ab_k4)-casadiGetSmoothMaxAcc(z(index.v_k4));
v11 = laterror_k4-r_k4-0.5*slack_k4;
v12 = -laterror_k4-r_k4-0.5*slack_k4;

v13 = z(index.ab_k5)-casadiGetSmoothMaxAcc(z(index.v_k5));
v14 = laterror_k5-r_k5-0.5*slack_k5;
v15 = -laterror_k5-r_k5-0.5*slack_k5;
%v18 = frontaxlelatacc_k3-torquevectoringcapability_k3-latacclim-slack_k3;
%v19 = -frontaxlelatacc_k3-torquevectoringcapability_k3-latacclim-slack_k3;
v16 = -squared_distance_array+dist-slack2;
v17 = -squared_distance_array2+dist-slack3;
v18 = -squared_distance_array3+dist-slack4;
v19 = -squared_distance_array4+dist-slack2;
v20 = -squared_distance_array5+dist-slack3;
v21 = -squared_distance_array6+dist-slack4;
v22 = -squared_distance_array7+dist-slack2;
v23 = -squared_distance_array8+dist-slack3;
v24 = -squared_distance_array9+dist-slack4;
v25 = -squared_distance_array10+dist-slack2;
v = [v1;v2;v3;v4;v5;v6;v7;v8;v9;v10;v11;v12;v13;v14;v15;v16;v17;v18;v19;v20;v21;v22;v23;v24;v25];%
end

