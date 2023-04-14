close all; clear all; clc

h=1; %vertical rib (relative to l)
l=1; 
thd(:,1)=-45:0.1:45; %rib angle
thid(:,1)=0:0.1:90;  %offset
b=0.2; %rib width (relative to l)

ii=751; %30 degrees

K=[0.0001,0.005,0.04,999999999999999999999999999999999]; %Upper & lower bounds

%% on axis equatins equations

th=thd.*pi()/180;
thi=thid.*pi()/180;

for i=1:4
Nu13(:,i)=(sin(th).*(cos(th).^2).*(1/K(i)-1))./((h/l+sin(th)).*((sin(th).^2)./K(i) + (cos(th).^2)));
Nu31(:,i)=((sin(th).*(h/l+sin(th)).*(1/K(i)-1)))./((cos(th).^2)./K(i) + 2*h/l+(sin(th).^2));

E1(:,i)=cos(th)./(b*(h/l+sin(th)).*((sin(th).^2)./K(i) + (cos(th).^2)));
E3(:,i)=(h/l+sin(th))./(b.*cos(th).*(cos(th).^2)./K(i) + 2*h/l+(sin(th).^2));
G13(:,i)= ((b.*(h/l).^2.*(1+2.*h/l).*cos(th))./(K(i).*(h/l + sin(th)))+((b.*(1+h/l.*sin(th)).^2)./(cos(th).*(h/l+sin(th))))).^-1;
end


%% Off axis
for i=1:4
E1_t(:,i)=(((cos(thi).^4)./E1(ii,i))+(cos(thi).^2).*(sin(thi).^2).*((1./G13(ii,i))-(2.*Nu13(ii,i)./E1(ii,i)))+(sin(thi).^4)./E3(ii,i)).^-1;
Nu13_t(:,i)=E1_t(:,i).*[((cos(thi).^4+sin(thi).^4).*Nu13(ii,i)./E1(ii,i))-(cos(thi).^2).*(sin(thi).^2).*(1./E1(ii,i)+1./E3(ii,i)-1./G13(ii,i))];
end

%% Plot

line={'k','k--','k:','k-.'};

for i=1:4
figure(1) 
plot(thid(:,1),Nu13_t(:,i),line{i},'LineWidth',2);hold on

plot([0,90],[0,0],'k','LineWidth',0.5)
plot([0,0],[-1,1.5],'k','LineWidth',0.5)
xlabel('\phi(°)');
ylabel('\nu_1_3(\phi)'); 
set(gca,'FontSize',24);
set(gca,'FontName','Calibri');
set(gca,'ytick',-2:0.5:2)
set(gca,'xtick',0:15:90)
axis('square')
axis([0,90,-1,1.2])


figure(2)
h(i)=plot(thid(:,1),E1_t(:,i),line{i},'LineWidth',2);hold on

plot([0,90],[0,0],'k','LineWidth',0.5)
plot([0,0],[0,4],'k','LineWidth',0.5)
xlabel('\phi(°)');
ylabel('E_1(\phi)/E_s'); 
set(gca,'FontSize',24);
set(gca,'FontName','Calibri');
set(gca,'ytick',0:1:5)
set(gca,'xtick',0:15:90)
axis('square')
axis([0,90,0,4])

end

figure(2) 
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
