function [u1cost_v1,u2cost_v1,u1cost_v2,u2cost_v2,xcost1,xcost2,ycost1,...
          ycost2,slackcost,f,f1,f2] =...
          objective_PG_simpleTestAlpha(z,pslack,xend1,xend2,yend1,yend2,alpha,pacc,psteer)
     global index
    x_v1 = z(index.x_v1);
    y_v1 = z(index.y_v1);
    %[xs,time] = euler(@(x,u)simplemodeldx(x,u),xs,u,integrator_stepsize/eulersteps);
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
    slackcost=pslack*slack;
    
    f1 = xcost1+ycost1+u1cost_v1+u2cost_v1;
    f2 = xcost2+ycost2+u1cost_v2+u2cost_v2;
    f  = f1+alpha*(f2)+pslack*slack;
end
