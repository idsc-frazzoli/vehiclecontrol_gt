function p = getParameters_simplePG(pslack, dist,xend1,yend1,xend2,yend2,pacc,psteer)
p = zeros(8,1);
p(1)=pslack;
p(2)=dist;
p(3)=xend1;
p(4)=yend1;
p(5)=xend2;
p(6)=yend2;
p(7)=pacc;
p(8)=psteer;
end

