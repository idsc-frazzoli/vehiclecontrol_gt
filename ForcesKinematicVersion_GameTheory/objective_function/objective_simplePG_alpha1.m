function f = objective_simplePG_alpha1(z,pslack,xend1,xend2,yend1,yend2,alpha,pacc,psteer)
    global index

    x_v1 = z(index.x_v1);
    y_v1 = z(index.y_v1);
    x_v2 = z(index.x_v2);
    y_v2 = z(index.y_v2);
    slack = z(index.slack);
    
    xcost1=4*(x_v1-xend1)^2;
    xcost2=4*(x_v2-xend2)^2;
    ycost1=4*(y_v1-yend1)^2;
    ycost2=4*(y_v2-yend2)^2;
    
    u1cost_v1=pacc*z(index.u_acc_v1)^2;
    u2cost_v1=psteer*z(index.u_steer_v1)^2;
    u1cost_v2=psteer*z(index.u_steer_v2)^2;
    u2cost_v2=pacc*z(index.u_acc_v2)^2;
    
    f = xcost1+ycost1+u1cost_v1+u2cost_v1+alpha*(xcost2+ycost2+u1cost_v2+...
        u2cost_v2)+pslack*slack;
end
