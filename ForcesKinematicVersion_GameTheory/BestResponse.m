function [outputM] = BestResponse(problem,X2,Y2,X3,Y3,params,nextSplinePoints,model)

global index_IBR
problem.all_parameters = repmat (getParameters_IBR_3(params.targetSpeed,...
    0,0,0,params.PprogMax,0,...
    params.posX4,params.plagerror,0,0,0,0,...
    0,0,params.pslack2,params.dist,0,0,0,0,...
    nextSplinePoints),model.N ,1);
problem.all_parameters(index_IBR.xComp2:model.npar:end)=X2;
problem.all_parameters(index_IBR.yComp2:model.npar:end)=Y2;
problem.all_parameters(index_IBR.xComp3:model.npar:end)=X3;
problem.all_parameters(index_IBR.yComp3:model.npar:end)=Y3;
%TODO problem.x0, problem.xinit
%
[output,exitflag,info] = MPCPathFollowing_3v_IBR(problem);
outputM = reshape(output.alldata,[model.nvar,model.N])';
x0 = outputM';
SlackCost=outputM(60,13);
% 
problem.x0=x0(:);
problem.all_parameters = repmat (getParameters_IBR_3(params.targetSpeed,...
    SlackCost+1,0,params.maxSpeed,params.PprogMax,params.pSpeedMax,...
    params.posX4,params.plagerror,0,0,0,0,...
    0,params.pslack,params.pslack2,params.dist,0,0,0,0,...
    nextSplinePoints),model.N ,1);
problem.all_parameters(index_IBR.xComp2:model.npar:end)=X2;
problem.all_parameters(index_IBR.yComp2:model.npar:end)=Y2;
problem.all_parameters(index_IBR.xComp3:model.npar:end)=X3;
problem.all_parameters(index_IBR.yComp3:model.npar:end)=Y3;

[output,exitflag,info] = MPCPathFollowing_3v_IBR_2(problem);
outputM = reshape(output.alldata,[model.nvar,model.N])';
x0 = outputM';

problem.x0 = x0(:);
latcostA=outputM(60,12);
problem.all_parameters = repmat (getParameters_IBR_3(params.targetSpeed,...
    SlackCost+1,latcostA+0.1,params.maxSpeed,params.PprogMax,params.pSpeedMax,...
    params.posX4,params.plagerror,params.platerror,params.pprog,params.pab,params.pdotbeta,...
    params.pspeedcost,params.pslack,params.pslack2,params.dist,0,0,0,0,...
    nextSplinePoints),model.N ,1);
problem.all_parameters(index_IBR.xComp2:model.npar:end)=X2;
problem.all_parameters(index_IBR.yComp2:model.npar:end)=Y2;
problem.all_parameters(index_IBR.xComp3:model.npar:end)=X3;
problem.all_parameters(index_IBR.yComp3:model.npar:end)=Y3;

[output,exitflag,info] = MPCPathFollowing_3v_IBR_3(problem);
outputM = reshape(output.alldata,[model.nvar,model.N])';

end

