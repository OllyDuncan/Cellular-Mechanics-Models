close all; clear all; clc

h=1; %vertical rib (relative to l)
l=1; 
thd(:,1)=-45:0.1:45; %rib angle
b=0.2; %rib width (relative to l)

ii=751; %30 degrees

K=[0.0001,0.005,0.04,999999999999999999999999999999999]; %Upper & lower bounds

%% Equations

th=thd.*pi()/180;

for i=1:4
Nu13(:,i)=(sin(th).*(cos(th).^2).*(1/K(i)-1))./((h/l+sin(th)).*((sin(th).^2)./K(i) + (cos(th).^2)));
Nu31(:,i)=((sin(th).*(h/l+sin(th)).*(1/K(i)-1)))./((cos(th).^2)./K(i) + 2*h/l+(sin(th).^2));

E1(:,i)=cos(th)./(b*(h/l+sin(th)).*((sin(th).^2)./K(i) + (cos(th).^2)));
E3(:,i)=(h/l+sin(th))./(b.*cos(th).*(cos(th).^2)./K(i) + 2*h/l+(sin(th).^2));
end

%% Plot

line={'k','k--','k:','k-.'};

for i=1:4
figure(1) 
h(i)=plot(thd(:,1),Nu13(:,i),line{i},'LineWidth',2);hold on

plot([-45,45],[0,0],'k','LineWidth',0.5)
plot([0,0],[-10,10],'k','LineWidth',0.5)
xlabel('\theta(°)');
ylabel('\nu_1_3'); 
set(gca,'FontSize',24);
set(gca,'FontName','Calibri');
set(gca,'ytick',-8:4:8)
set(gca,'xtick',-45:15:45)
axis('square')
axis([-45,45,-8,8])


figure(2)  
plot(thd(:,1),Nu31(:,i),line{i},'LineWidth',2);hold on

plot([-45,45],[0,0],'k','LineWidth',0.5)
plot([0,0],[-10,10],'k','LineWidth',0.5)
xlabel('\theta(°)');
ylabel('\nu_3_1'); 
set(gca,'FontSize',24);
set(gca,'FontName','Calibri');
set(gca,'ytick',-3:1:3)
set(gca,'xtick',-45:15:45)
axis('square')
axis([-45,45,-1,3])


figure(3)
plot(thd(:,1),E1(:,i),line{i},'LineWidth',2);hold on

plot([-45,45],[0,0],'k','LineWidth',0.5)
plot([0,0],[-10,10],'k','LineWidth',0.5)
xlabel('\theta(°)');
ylabel('E_1/E_s'); 
set(gca,'FontSize',24);
set(gca,'FontName','Calibri');
set(gca,'ytick',0:2:20)
set(gca,'xtick',-45:15:45)
axis('square')
axis([-45,45,0,8])


figure(4)
plot(thd(:,1),E3(:,i),line{i},'LineWidth',2);hold on

plot([-45,45],[0,0],'k','LineWidth',0.5)
plot([0,0],[-10,10],'k','LineWidth',0.5)
xlabel('\theta(°)');
ylabel('E_3/E_s'); 
set(gca,'FontSize',24);
set(gca,'FontName','Calibri');
set(gca,'ytick',0:0.2:0.8)
set(gca,'xtick',-45:15:45)
axis('square')
axis([-45,45,0,0.8])
end

figure(1) 
legend(h(1:4),'K_h_f/K_s \approx 0','K_h_f/K_s = 0.005','K_h_f/K_s = 0.04','K_h_f/K_s \approx \infty','FontSize',20,'Location','NorthWest');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This Matlab code was written by O. Duncan,  Department of Engineering,   %
% Manchester Metropolitan University                                       %
%  M15 6BG, UK.                                                            %
% Please sent your comments to: o.duncan@mmu.ac.uk                         %
%                                                                          %
% The code is intended for educational purposes and theoretical details    %
% are discussed in the paper                                               %
% Duncan O., Allen T., Foster L., Senior T., Alderson A. Fabrication,      %
% characterisation and modelling of uniform and gradient auxetic foam      % 
% sheets. Acta Mater. (2017).126. 426–37.                                  %
%                                                                          %
% Disclaimer:                                                              %
% The author reserves all rights but do not guaranty that the code is      %
% free from errors. Furthermore, I shall not be liable in any event        %
% caused by the use of the program.                                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%