
figure (300)
JerkCost=[[1:7]',[Jerk1,Jerk2,Jerk3;regABA,regABB,regABC]];
for ii=1:7
JerkCost(ii,5)=mean(JerkCost(ii,2:4));
end
T2 = array2table(JerkCost);
T2.Properties.VariableNames(1:5) = {'x','blue','red','green','mean'};
writetable(T2,'JerkCost_st.txt','Delimiter',' ')
plot(1:length(JerkCost),JerkCost(:,2),'bx','Linewidth',2.5)
hold on
plot(1:length(JerkCost),JerkCost(:,3),'r*','Linewidth',2.5)
plot(1:length(JerkCost),JerkCost(:,4),'go','Linewidth',2.5)
plot(1:length(JerkCost),JerkCost(:,5),'k+','Linewidth',2.5)

grid on
%title('Costs')
xlim([0,8])
xticklabels({'','BRG','BGR','RBG','RGB','GBR','GRB','PG','','interpreter','latex'})
%legend ('V 1','V 2','V 3')
hold off
set(gca,'FontSize',15)
savefig('figures/3v_IBR_jerk')
saveas(gcf,'figures/3v_IBR_jerk','epsc')