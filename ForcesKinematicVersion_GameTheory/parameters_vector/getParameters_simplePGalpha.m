function p = getParameters_simplePGalpha(pslack, dist,xend1,yend1,xend2,...
    yend2,alpha,pacc,psteer)
p = zeros(9,1);
p(1)=pslack;
p(2)=dist;
p(3)=xend1;
p(4)=yend1;
p(5)=xend2;
p(6)=yend2;
p(7)=alpha;
p(8)=pacc;
p(9)=psteer;
end

