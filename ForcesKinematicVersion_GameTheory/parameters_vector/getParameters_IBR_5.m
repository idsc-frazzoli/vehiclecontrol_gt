function p = getParameters_IBR_5(maxspeed, xmaxacc,ymaxacc,latacclim,...
                         rotacceffect,torqueveceffect, brakeeffect,...
                         plagerror, platerror, pprog,...
                         pab, pdotbeta, pspeedcost,pslack,pslack2,dist,...
                         xcomp,ycomp,xcomp2,ycomp2,xcomp3,ycomp3,xcomp4,...
                         ycomp4,points)
[np,~]=size(points);
p = zeros(3*np+23,1);
p(1)=maxspeed;
p(2)=xmaxacc;
p(3)=ymaxacc;
p(4)=latacclim;
p(5)=rotacceffect;
p(6)=torqueveceffect;
p(7)=brakeeffect;
p(8)=plagerror;
p(9)=platerror;
p(10)=pprog;
p(11)=pab;
p(12)=pdotbeta;
p(13)=pspeedcost;
p(14)=pslack;
p(15)=pslack2;
p(16)=dist;
p(17)=xcomp;
p(18)=ycomp;
p(19)=xcomp2;
p(20)=ycomp2;
p(21)=xcomp3;
p(22)=ycomp3;
p(23)=xcomp4;
p(24)=ycomp4;
p(24:3*np+24-1)=points(:);
end

