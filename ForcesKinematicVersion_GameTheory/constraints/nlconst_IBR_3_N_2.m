function v = nlconst_IBR_3_N_2(z,p)
global index_IBR

pslack2 = p(index_IBR.pslack2);
slack2 = z(index_IBR.slack2);
dist = p(index_IBR.dist);
pointsO = index_IBR.pointsO;
pointsN = index_IBR.pointsN;
xVehicle2 = p(index_IBR.xComp2);
yVehicle2 = p(index_IBR.yComp2);
xVehicle3 = p(index_IBR.xComp3);
yVehicle3 = p(index_IBR.yComp3);
points = getPointsFromParameters(p, pointsO, pointsN);
radii = getRadiiFromParameters(p, pointsO, pointsN);

[splx,sply] = casadiDynamicBSPLINE(z(index_IBR.s),points);
[splsx, splsy] = casadiDynamicBSPLINEsidewards(z(index_IBR.s),points);
r = casadiDynamicBSPLINERadius(z(index_IBR.s),radii);

sidewards = [splsx;splsy];
realPos = z([index_IBR.x,index_IBR.y]);
centerPos = realPos;
wantedpos = [splx;sply];
error = centerPos-wantedpos;
laterror = sidewards'*error;

distance_X=(z(index_IBR.x)-xVehicle2);
distance_Y=(z(index_IBR.y)-yVehicle2);
distance_X_3=(z(index_IBR.x)-xVehicle3);
distance_Y_3=(z(index_IBR.y)-yVehicle3);
distance_X_4=(z(index_IBR.x)-53);
distance_Y_4=(z(index_IBR.y)-58);
squared_distance_array   = sqrt(distance_X.^2+distance_Y.^2);
squared_distance_array_3   = sqrt(distance_X_3.^2+distance_Y_3.^2);
squared_distance_array_4   = sqrt(distance_X_4.^2+distance_Y_4.^2);

v1 = z(index_IBR.ab)-casadiGetSmoothMaxAcc(z(index_IBR.v));
v2 = laterror-r;%-slack
v3 = -laterror-r;%-slack
v4 = -squared_distance_array+dist-slack2;
v5 = -squared_distance_array_3+dist-slack2;
v6 = -squared_distance_array_4+dist-slack2;
v7 = z(index_IBR.x);
v8 = z(index_IBR.y);
v9 = pslack2*slack2;
v10 = z(index_IBR.laterror)-p(index_IBR.pay);
v = [v1;v2;v3;v4;v5;v6;v7;v8;v9;v10];
end

