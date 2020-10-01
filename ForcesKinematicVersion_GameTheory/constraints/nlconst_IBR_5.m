function v = nlconst_IBR_5(z,p)
global index_IBR

slack = z(index_IBR.slack);
slack2 = z(index_IBR.slack2);
dist = p(index_IBR.dist);
pointsO = index_IBR.pointsO;
pointsN = index_IBR.pointsN;
xVehicle2 = p(index_IBR.xComp2);
yVehicle2 = p(index_IBR.yComp2);
xVehicle3 = p(index_IBR.xComp3);
yVehicle3 = p(index_IBR.yComp3);
xVehicle4 = p(index_IBR.xComp4);
yVehicle4 = p(index_IBR.yComp4);
xVehicle5 = p(index_IBR.xComp5);
yVehicle5 = p(index_IBR.yComp5);
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
distance_X_4=(z(index_IBR.x)-xVehicle4);
distance_Y_4=(z(index_IBR.y)-yVehicle4);
distance_X_5=(z(index_IBR.x)-xVehicle5);
distance_Y_5=(z(index_IBR.y)-yVehicle5);
squared_distance_array   = sqrt(distance_X.^2+distance_Y.^2);
squared_distance_array_3   = sqrt(distance_X_3.^2+distance_Y_3.^2);
squared_distance_array_4   = sqrt(distance_X_4.^2+distance_Y_4.^2);
squared_distance_array_5   = sqrt(distance_X_5.^2+distance_Y_5.^2);
v1 = z(index_IBR.ab)-casadiGetSmoothMaxAcc(z(index_IBR.v));
v3 = laterror-r-0.5*slack;
v4 = -laterror-r-0.5*slack;
v5 = -squared_distance_array+dist-slack2;
v6 = -squared_distance_array_3+dist-slack2;
v7 = -squared_distance_array_4+dist-slack2;
v8 = -squared_distance_array_5+dist-slack2;
v = [v1;v3;v4;v5;v6;v7;v8];
end

