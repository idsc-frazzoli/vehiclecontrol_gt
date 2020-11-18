close all
figure
hold on
I=imread('road06_1.png');
h=image([20 80],[60 20],I);
set(gca,'visible','off')
axis equal

B=imread('carb.png');
b=image([pstart(1)-2.5,pstart(1)+2.5],[pstart(2)-1,pstart(2)+2]-10,B);
% G=imread('carg.png');
% g=image([pstart3(1)-2,pstart3(1)+1],[pstart3(2)-10+2.5,pstart3(2)-10-2.5]-10,G);
R=imread('carr.png');
r=image([pstart2(1)-1.5,pstart2(1)+1.5],[pstart2(2)+10+2.5,pstart2(2)+10-2.5]-10,R);
savefig('figures/3v_intersection_init')
saveas(gcf,'figures/3v_intersection_init','epsc')

hold off

figure
hold on
I=imread('racetrack.png');
h=image([20 80],[60 20],I);

axis equal

B=imread('carb.png');
b=image([47+1.5,47-1.5],[25,27.2],B);
G=imread('carg_R.png');
g=image([42-1.5,42+1.5],[24,26.2],G);
R=imread('carrR.png');
r=image([34,36.2],[36,33],R);
set(gca,'visible','off')
savefig('figures/3v_race')
saveas(gcf,'figures/3v_race','epsc')