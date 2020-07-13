function f = objective_simple(z,pslack,xend1,xend2,yend1,yend2)
    global index

    x_v1 = z(index.x_v1);
    y_v1 = z(index.y_v1);
    
    xcost1=(x_v1-xend1)^2;
    ycost1=(y_v1-yend1)^2;
    
    u1cost_v1=z(index.u_acc_v1)^2;
    u2cost_v1=z(index.u_steer_v1)^2;
    
    f = xcost1+ycost1+u1cost_v1+u2cost_v1;
end
