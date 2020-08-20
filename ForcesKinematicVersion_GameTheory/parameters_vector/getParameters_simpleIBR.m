function p = getParameters_simpleIBR(pslack, dist,xend1,yend1,xend2,...
    yend2,pacc,psteer,x_v,y_v)
p = zeros(10,1);
p(1) = pslack;
p(2) = dist;
p(3) = xend1;
p(4) = yend1;
p(5) = xend2;
p(6) = yend2;
p(7) = pacc;
p(8) = psteer;
p(9) = x_v;
p(10)= y_v;
end

