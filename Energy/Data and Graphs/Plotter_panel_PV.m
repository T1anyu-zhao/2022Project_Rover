dataplot = xlsread('Plotter.xlsx', 'Sheet2','T5:W19');

p = dataplot(:,1);
q = dataplot(:,2);
x = dataplot(:,3);
y = dataplot(:,4);

coefficients = polyfit(p,q,6);
pFit = linspace(p(1),p(end));
qFit = polyval(coefficients,pFit);

coefficients2 = polyfit(x,y,5);
xFit = linspace(x(1),x(end));
yFit = polyval(coefficients2,xFit);

plot(p,q,'r.','MarkerSize',20)
hold on
plot(x,y,'g.','MarkerSize',20)
hold on
plot(pFit,qFit,'k--','LineWidth',2);
hold on
plot(xFit,yFit,'b--','LineWidth',2);

legend({'larger angle','smaller angle','larger best fit','smaller best fit'},'Location','Northwest');

xlabel('Voltage (V)')
ylabel('Power (W)')
title('PV Characteristics of PV panel')
grid on


