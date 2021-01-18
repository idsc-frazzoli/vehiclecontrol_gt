function [outputM,outputM2,outputM3] = IteratedBestResponse(jj,config,problem,...
    problem2,problem3,params,model,nextSplinePoints,nextSplinePoints2,...
    nextSplinePoints3,outputMold,outputMold2,outputMold3)

global index_IBR
X1=outputMold(:,index_IBR.x);
Y1=outputMold(:,index_IBR.y);
X2=outputMold2(:,index_IBR.x);
Y2=outputMold2(:,index_IBR.y);
X3=outputMold3(:,index_IBR.x);
Y3=outputMold3(:,index_IBR.y);
iter=1;
while iter<=10
    iter=iter+1;
    for ii=1:length(config(1,:))
        if config(jj,ii)==1
            outputM = BestResponse(problem,X2,Y2,X3,Y3,params,nextSplinePoints,model);
            X1=outputM(:,index_IBR.x);
            Y1=outputM(:,index_IBR.y);
            x0=outputM';
            problem.x0=x0(:);
         
        elseif  config(jj,ii)==2
            
            outputM2 = BestResponse(problem2,X1,Y1,X3,Y3,params,nextSplinePoints2,model);
            X2=outputM2(:,index_IBR.x);
            Y2=outputM2(:,index_IBR.y);
            x02=outputM2';
            problem2.x0=x02(:);
            
        elseif  config(jj,ii)==3
            
            outputM3 = BestResponse(problem3,X1,Y1,X2,Y2,params,nextSplinePoints3,model);
            X3=outputM3(:,index_IBR.x);
            Y3=outputM3(:,index_IBR.y);
            x03=outputM3';
            problem3.x0=x03(:);
        end
    end
    MakeFig(jj,outputM,outputM2,outputM3,params)
    distanceX=outputM(:,index_IBR.x)-outputMold(:,index_IBR.x);
    distanceY=outputM(:,index_IBR.y)-outputMold(:,index_IBR.y);
    distanceX2=outputM2(:,index_IBR.x)-outputMold2(:,index_IBR.x);
    distanceY2=outputM2(:,index_IBR.y)-outputMold2(:,index_IBR.y);
    distanceX3=outputM3(:,index_IBR.x)-outputMold3(:,index_IBR.x);
    distanceY3=outputM3(:,index_IBR.y)-outputMold3(:,index_IBR.y);
    
    squared_distance_arrayX    = sum((distanceX).^2);
    squared_distance_arrayY    = sum((distanceY).^2);
    squared_distance_arrayX2    = sum((distanceX2).^2);
    squared_distance_arrayY2    = sum((distanceY2).^2);
    squared_distance_arrayX3    = sum((distanceX3).^2);
    squared_distance_arrayY3    = sum((distanceY3).^2);
    par=0.05;
    if (squared_distance_arrayX<=par && squared_distance_arrayY<=par &&...
            squared_distance_arrayX2<=par && squared_distance_arrayY2<=par && ...
            squared_distance_arrayX3<=par && squared_distance_arrayY3<=par)
        iter-1
        iter=11;
    else
        outputMold=outputM;
        outputMold2=outputM2;
        outputMold3=outputM3;
    end
end

end

