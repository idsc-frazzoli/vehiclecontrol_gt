clear all;
dT = 0.2; 
d0 = 100; %distance till end of intersection
v0 = 4; %initial velocity
dV = 0:0.5:15; %change in velocities to eval
for i = 1:length(dV);
   t(i)= d0/(v0+dV(i));
   n(i) = ceil(t(i)/dT);
end
plot(dV,t);
W = zeros(length(dV),n(1));
X =zeros(length(dV),n(1));
for i = 1:length(dV);
   for j = 1:n(i);
       W(i,j)=(1-(j/n(i)))*dT;
       X(i,j)=dV(i)/n(i);
   end
end
%% Optimise for best U

figure;
hold on;
Myoptions = optimset('MaxIter',2e10);
for i = 1:length(dV);
    u(1,:) = fminsearch(@(x)(x*x') + 500*((x*W(i,:)') -dV(i))^2,X(i,:), Myoptions);
    for j =2:5
        u(j,:) = fminsearch(@(x)(x*x') + 500*((x*W(i,:)') -dV(i))^2,u(j-1,:), Myoptions);
    end
U(i,:)=sum(u(4:5,:),1)/2;
U2(i,:)=u(5,:);
plot(U(i,:))
plot(U2(i,:))
c(i)=U(i,:)*U(i,:)';
c2(i)=U2(i,:)*U2(i,:)';
end

%% Derived best U
figure;
hold on
U3 =zeros(size(U));
%nSum = ones(size(dV));
nSum=n
for i = 1:length(dV)
    for j = 1:n(i)
        nSum(i)= nSum(i)-(j*(2-(j/n(i)))/n(i));
    end
a=dV(i)/(nSum(i)*dT);
b=a/n(i);
for j = 1:n(i);
    U3(i,j)=a-(b*(j));
end
subplot(1,2,1)
hold on
plot(U3(i,:))
subplot(1,2,2)
hold on
plot(U2(i,:))
c3(i)=U3(i,:)*U3(i,:)';
end
%% compare optimised v.s. derived costs
figure;
plot(dV,0.5*c,dV,0.5*c3);legend("opt","derived");
figure;
hold on
for i = 1:length(dV);
eDV(i) =U(i,:)*W(i,:)';
eDV3(i) =U3(i,:)*W(i,:)';
end
plot(dV,dV,'.')
plot(dV,eDV)
plot(dV,eDV3)
legend("target","opt","line");

figure;
plot(dV,0.2*c,dV,t,dV,0.1*(t.^2),dV,t+(0.2*c))
com=zeros(length(dV),length(dV));
for i = 1:length(dV);
    for j = 1:length(dV);
        com(i,j)= c(i)+c(j)+t(i)+t(j);
        costA(i,j)= c(i)+t(i);
        costB(i,j)= c(j)+t(j);
    end
end
figure;
surface(com)
figure;
surface(costA)
figure;
surface(costB)

[xx,yy] = meshgrid(dV,dV);
figure;hold on;
cA =costA-min(min(costA));
cA(12:end,:)=-cA(12:end,:);
cB =costB-min(min(costB));
cB(:,12:end)=-cB(:,12:end);
quiver(dV,dV,0*costA,cA)
quiver(dV,dV,cB,0*costB)




