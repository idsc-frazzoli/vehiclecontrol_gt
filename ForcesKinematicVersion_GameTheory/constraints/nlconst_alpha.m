function v = nlconst_PG(z,p)
global index
%NLCONST Summary of this function goes here
%   Detailed explanation goes here
dist=p(index.dist);
slack = z(index.slack);
slack2 = z(index.slack2);
slack_k2 = z(index.slack_k2);

% Splines Control points and radii
pointsO = index.pointsO;
pointsN = index.pointsN;
pointsN2 = index.pointsN2;
points = getPointsFromParameters(p, pointsO, pointsN);
radii = getRadiiFromParameters(p, pointsO, pointsN);

points_k2 = getPointsFromParameters(p, pointsO+3*pointsN, pointsN2);
radii_k2 = getRadiiFromParameters(p, pointsO+3*pointsN, pointsN2);
% Splines
[splx,sply] = casadiDynamicBSPLINE(z(index.s),points);
[splsx, splsy] = casadiDynamicBSPLINEsidewards(z(index.s),points);
r = casadiDynamicBSPLINERadius(z(index.s),radii);

[splx_k2,sply_k2] = casadiDynamicBSPLINE(z(index.s_k2),points_k2);
[splsx_k2, splsy_k2] = casadiDynamicBSPLINEsidewards(z(index.s_k2),points_k2);
r_k2 = casadiDynamicBSPLINERadius(z(index.s_k2),radii_k2);

sidewards = [splsx;splsy];

realPos = z([index.x,index.y]);
centerOffset = 0.4*gokartforward(z(index.theta))';
centerPos = realPos+centerOffset;%+0.4*forward;
wantedpos = [splx;sply];

% Position error
error = centerPos-wantedpos;

% Projection errors
laterror = sidewards'*error;

sidewards_k2 = [splsx_k2;splsy_k2];

realPos_k2 = z([index.x_k2,index.y_k2]);
centerOffset_k2 = 0.4*gokartforward(z(index.theta_k2))';
centerPos_k2 = realPos_k2+centerOffset_k2;%+0.4*forward;
wantedpos_k2 = [splx_k2;sply_k2];

% Position error
error_k2 = centerPos_k2-wantedpos_k2;

% Projection errors
laterror_k2 = sidewards_k2'*error_k2;

distance_X=(z(index.x)-z(index.x_k2));
distance_Y=(z(index.y)-z(index.y_k2));
squared_distance_array = sqrt(distance_X.^2+distance_Y.^2);
%% Constraints
v2 = z(index.ab)-casadiGetSmoothMaxAcc(z(index.v));
v4 = laterror-r-slack;
v5 = -laterror-r-slack;

v8 = z(index.ab_k2)-casadiGetSmoothMaxAcc(z(index.v_k2));
v10 = laterror_k2-r_k2-slack_k2;
v11 = -laterror_k2-r_k2-slack_k2;

v14 = -squared_distance_array+dist-slack2;
v = [v2;v4;v5;v8;v10;v11;v14];%
end

