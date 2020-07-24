clear all;
%% Two player intersection
%Currently it is assumed that the game stops once both Players leave the
%intersection, And the that time cost is time to leave the intersection.
%Collsions are calculated assuming player A passes ahead

dT = 0.2; 
d0 = [80,100]; %distance till end of intersection
v0 = [4,4]; %initial velocity
dV = 0:0.1:10; %change in velocities to eval
for i = 1:length(dV)
   t(i,:)= d0./(v0+dV(i));
   n(i,:) = [ceil(t(i,1)/dT),ceil(t(i,2)/dT)];
end
plot(dV,t);
W = zeros(length(dV),max(n(1,:)),2);
X =zeros(length(dV),max(n(1,:)),2);
for i = 1:length(dV)
   for j = 1:n(i)
       W(i,j,1)=(1-(j/n(i,1)))*dT;
       W(i,j,2)=(1-(j/n(i,2)))*dT;
       X(i,j,1)=dV(i)/n(i,1);
       X(i,j,2)=dV(i)/n(i,2);
   end
end


%% Derived best U
figure;
hold on
U =zeros(size(X));
%nSum = ones(size(dV));
nSum=n;
for i = 1:length(dV)
    for j = 1:n(i,1)
        nSum(i,1)= nSum(i)-(j*(2-(j/n(i,1)))/n(i,1));
    end
    for j = 1:n(i,2)
        nSum(i,2)= nSum(i)-(j*(2-(j/n(i,2)))/n(i,2));
    end
a=dV(i)./(nSum(i,:)*dT);
b=a/n(i);
for j = 1:n(i,1)
    U(i,j,1)=a(1)-(b(1)*(j));
end
for j = 1:n(i,2)
    U(i,j,2)=a(2)-(b(2)*(j));
end
subplot(1,2,1)
hold on
plot(U(i,:,1))
subplot(1,2,2)
hold on
plot(U(i,:,2))
c(i,1)=U(i,:,1)*U(i,:,1)';
c(i,2)=U(i,:,2)*U(i,:,2)';
end
%% compare optimised v.s. derived costs

figure;
subplot(1,3,1)
plot(dV,0.2*c(:,1),dV,0.2*c(:,2));
legend("Player A","Player B");ylabel("Acc Cost");xlabel("D_V");
subplot(1,3,2)
plot(dV,t(:,1),dV,t(:,2))
legend("Player A","Player B");ylabel("Time Cost");xlabel("D_V");
subplot(1,3,3)
plot(dV,t(:,1)+(0.2*c(:,1)),dV,t(:,2)+(0.2*c(:,2)))
legend("Player A","Player B");ylabel("Total Cost");xlabel("D_V");

com = zeros(length(dV),length(dV));
col = zeros(length(dV),length(dV));
costA = zeros(length(dV),length(dV));
costB = zeros(length(dV),length(dV));
for i = 1:length(dV)
    for j = 1:length(dV)
        if t(i,1)>t(j,2)
            col(i,j)=500;
        end
        com(i,j)= c(i,1)+c(j,2)+t(i,1)+t(j,2);
        costA(i,j)= c(i,1)+t(i,1);
        costB(i,j)= c(j,2)+t(j,2);
    end
end
figure;
surface(dV,dV,com)
figure;
surface(dV,dV,col)
figure;
surface(dV,dV,col+com)
figure;
surface(dV,dV,costA)
figure;
surface(dV,dV,costB)

[xx,yy] = meshgrid(dV,dV);
figure;hold on;
cA =costA-min(min(costA));
cA(12:end,:)=-cA(12:end,:);
cB =costB-min(min(costB));
cB(:,12:end)=-cB(:,12:end);
quiver(dV,dV,0*costA,cA)
quiver(dV,dV,cB,0*costB)




