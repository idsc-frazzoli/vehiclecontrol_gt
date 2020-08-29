function v = nlconst_IBR(z,p)
global index_IBR
%NLCONST Summary of this function goes here
%   Detailed explanation goes here
% variables z = [ab,dotbeta,ds,x,y,theta,v,beta,s,braketemp]
l = 1.19;
beta  = z(index_IBR.beta);
dotbeta = z(index_IBR.dotbeta);
forwardacc = z(index_IBR.ab);
%VELX = z(index.v);
%VELY = z(index.yv);
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
[spldx, spldy] = casadiDynamicBSPLINEforward(z(index_IBR.s),points);
[splsx, splsy] = casadiDynamicBSPLINEsidewards(z(index_IBR.s),points);
r = casadiDynamicBSPLINERadius(z(index_IBR.s),radii);

forward = [spldx;spldy];
sidewards = [splsx;splsy];
realPos = z([index_IBR.x,index_IBR.y]);
centerPos = realPos;
wantedpos = [splx;sply];
error = centerPos-wantedpos;
lagerror = forward'*error;
laterror = sidewards'*error;

distance_X=(z(index_IBR.x)-xVehicle2);
distance_Y=(z(index_IBR.y)-yVehicle2);
squared_distance_array   = sqrt(distance_X.^2+distance_Y.^2);

%parameters
vmax =  p(index_IBR.ps);
maxxacc = p(index_IBR.pax);
maxyacc = p(index_IBR.pay);
latacclim = p(index_IBR.pll);
rotacceffect = p(index_IBR.prae);
torqueveceffect = p(index_IBR.ptve);
brakeeffect = p(index_IBR.pbre);

ackermannAngle = -0.58*beta*beta*beta+0.93*beta;
dAckermannAngle = -0.58*3*beta*beta*dotbeta+0.93*dotbeta;
tangentspeed = z(index_IBR.v);
latacc = (tan(ackermannAngle)*tangentspeed^2)/l;
%avoid oversteer
accnorm = ((latacc/maxyacc)^2+(z(index_IBR.ab)/maxxacc)^2);

%avoid understeer
rotacc = dAckermannAngle*tangentspeed/l;
frontaxlelatacc = latacc+rotacc*rotacceffect;

%simplified 
accbudget = (1.8-forwardacc)/1.6;
torquevectoringcapability = accbudget*torqueveceffect;

% l = 1.19;
% l1 = 0.73;
% l2 = l-l1;
% f1n = l2/l;
% f2n = l1/l;

%v1 = (tan(z(index.beta))*z(index.v)^2/l)^2+z(index.ab)^2;
%v1=(tan(z(8))*z(7)^2/l);
v1 = z(index_IBR.ab)-casadiGetSmoothMaxAcc(z(index_IBR.v));
v2 = accnorm-slack;
v3 = laterror-r-0.5*slack;
v4 = -laterror-r-0.5*slack;
v5 = frontaxlelatacc -torquevectoringcapability- latacclim-slack;
%understeerleft
%v7 = -frontaxlelatacc - latacclim-torquevectoringcapability-slack;
v6 = -frontaxlelatacc -torquevectoringcapability - latacclim-slack;
v7 = -squared_distance_array+dist-slack2;
%v4 = error'*error;
%v2 = -1;
%v = [v1;v2;v3];
v = [v1;v2;v3;v4;v5;v6;v7];
end

