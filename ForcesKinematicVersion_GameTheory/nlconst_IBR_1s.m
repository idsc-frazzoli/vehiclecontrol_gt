function v = nlconst_IBR_1s(z,p)
global index_IBR

slack = z(index_IBR.slack);
slack2 = z(index_IBR.slack2);
dist = p(index_IBR.dist);
pointsO = index_IBR.pointsO;
pointsN = index_IBR.pointsN;
xVehicle2 = p(index_IBR.xComp);
yVehicle2 = p(index_IBR.yComp);

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
squared_distance_array   = sqrt(distance_X.^2+distance_Y.^2);

v1 = z(index_IBR.ab)-casadiGetSmoothMaxAcc(z(index_IBR.v));
v3 = laterror-r-0.5*slack;
v4 = -laterror-r-0.5*slack;
v7 = -squared_distance_array+dist-slack2;
v = [v1;v3;v4;v7];
end

