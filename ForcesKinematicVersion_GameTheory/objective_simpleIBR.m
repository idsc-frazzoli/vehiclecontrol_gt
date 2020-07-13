function f = objective_simpleIBR(z,pslack,xend1,yend1,pacc,psteer)
    global index

    x_v1 = z(index.x_v1);
    y_v1 = z(index.y_v1);
    slack= z(index.slack);
    
    xcost1=(x_v1-xend1)^2;
    ycost1=(y_v1-yend1)^2;
    
    u1cost_v1=pacc*z(index.u_acc_v1)^2;
    u2cost_v1=psteer*z(index.u_steer_v1)^2;
    
    f = xcost1+ycost1+u1cost_v1+u2cost_v1+pslack*slack;
end
