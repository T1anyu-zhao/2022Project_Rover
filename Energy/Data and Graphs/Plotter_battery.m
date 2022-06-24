dataset = xlsread('Plotter.xlsx', 'Sheet1','A1:B12');

x = dataset(:,1);
y = dataset(:,2);

plot(x,y,'.','MarkerSize',20)

xlabel('Voltage (V)')
ylabel('Current (A)')
title('IV Characteristics of the Battey')
grid on
hold on

m = dataset(4:10,1);
n = dataset(4:10,2);

coefficients = polyfit(m,n,1);
mFit = linspace(min(m),max(m),1000);
nFit = polyval(coefficients,mFit);

plot(mFit,nFit,'r--','LineWidth',2);
text(4.72,0.25, 'Current = ' + string(coefficients(1,1)) +'*Voltage' + string(coefficients(1,2)))
plot(m,n,'.','MarkerSize',20)
grid on

%disp('y = ', coefficient[x],'+',coefficient[y]);


