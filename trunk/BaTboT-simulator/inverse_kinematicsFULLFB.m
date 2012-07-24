%**************************************************************************
%BatBot: Biological inspired Bat roBot.

%Copyright RObotics and Cybernetics Group
%Julian Colorado

% Matlab simulator of bat flight behavior. 
%************

% MATLAB script for DIRECT KINEMATICS of the FLEXIBLE WING with 3DOF % It calculates and plots the coordinates of the wrist and % fingertips with respect to the body frame located in the sternum % of the bat.

clear all 
close all 
clc 
format short


Wrist_Name = '\bf{Influence of \theta_{20}}'; 
Wingtip_Name= '\bf{Wingtip trajectory, 3-DOF model}';
gm = 58*pi/180; % the inclination angle of the stroke plane 
x0 = 13; % [mm] distance along yb between sternum and shoulder
% Humerus design parameters %------------------------------------------------------------------- 
Lh = 19;% [mm] % humerus length 
deltah = 77*pi/180; % the inclination angle of humerus bone
a1 = Lh*sin(deltah); % the length of the common normal between the shoulder 
d1 = Lh*cos(deltah); % [mm] 
alpha1 = -90*pi/180;%
% Radius parameters %-------------------------------------------------------------------
Lr = 42;% the length between the elbow joint and the wrist joint, mm 
delR = 12*pi/180; % the inclination angle of the radius bone 
a2 = Lr*cos(delR); 
d2 = Lr*sin(delR);
% 3rd Finger parameters %------------------------------------------------------------------- 
L3f = 70;% mm - the length of the third digit 66.5mm 
deltaf1 = 50*pi/180;% third finger angle in x-y local plane 
deltaf2 = -10*pi/180; % third finger angle in x-z local plane
x3f = L3f*cos(deltaf1)*cos(deltaf2);

z3f = L3f*sin(deltaf2); 
y3f = L3f*sin(deltaf1)*cos(deltaf2);
% 4th Finger parameters %------------------------------------------------------------------- 
L4f = 58;%mm - the length of the fourth digit 
deltaf3 = 67*pi/180; % fourth finger angle in x-y local plane 
deltaf4 = -5*pi/180; % fourth finger angle in x-z local plane
x4f = L4f*cos(deltaf3)*cos(deltaf4); 
z4f = L4f*sin(deltaf4); 
y4f = L4f*sin(deltaf3)*cos(deltaf4);
% 5th Finger parameters %-------------------------------------------------------------------
L5f = 51;%mm - the length of the fifth finger 
deltaf5 = 112*pi/180; % fifth finger angle in x-y local plane 
deltaf6 = 5*pi/180; % fifth finger angle in x-z local plane
x5f = L5f*cos(deltaf5)*cos(deltaf6); 
z5f = L5f*sin(deltaf6); 
y5f = L5f*sin(deltaf5)*cos(deltaf6);

freq = 2.5; %Flapping Frequency (wing-beat cycle)
step_2 = (1/freq)/(36-1);
t=0:step_2:(1/freq);

%	PLECOTUS AURITUS DATA %-------------------------------------------------------------------
% % Wrist coodinates
% data1=xlsread('wrist_averages.xls'); 
% t = data1(:,1); 
% xwP = data1(:,2); 
% ywP = data1(:,3)+x0;
% zwP = data1(:,4);
% % Wingtip coodinates
% data2 = xlsread('3rd_averages.xls'); 
% x3P = data2(:,2); 
% y3P = data2(:,3)+x0; 
% z3P = data2(:,4);
% % Coodinates of fourth finger
% data3 = xlsread('4th_averages.xls'); 
% x4P = data3(:,2); 
% y4P = data3(:,3)+x0; 
% z4P = data3(:,4);
% % Coodinates of fifth finger
% data4 = xlsread('5th_averages.xls');
% x5P = data4(:,2);
% y5P = data4(:,3)+x0; z5P = data4(:,4);

for i = 1:length(t) 
    null(i)=0;
end
for i = 1:length(t)
    count(i) = i;
    th11(i) = (20+40*cos(2*pi/0.084*t(i)-0*pi/180))*pi/180; 
    th21(i) = (-59+26*cos(2*pi/0.084*t(i)-122*pi/180))*pi/180;%PHI2 is neg
    th31(i) = (10+25*cos(2*pi/0.084*t(i)+60*pi/180))*pi/180;
    % Plot a second position
    th12(i) = (5+30*cos(2*pi/0.084*t(i)-0*pi/180))*pi/180; 
    th22(i) = (-68+30*cos(2*pi/0.084*t(i)-120*pi/180))*pi/180; 
    th32(i) = (11+28*cos(2*pi/0.084*t(i)+60*pi/180))*pi/180;
    % Plot a third position
    th13(i) = (5+30*cos(2*pi/0.084*t(i)-0*pi/180))*pi/180; 
    th23(i) = (-68+30*cos(2*pi/0.084*t(i)-60*pi/180))*pi/180; 
    th33(i) = (11+28*cos(2*pi/0.084*t(i)+120*pi/180))*pi/180;
    % Plot a fourth position
    th14(i) = (5+30*cos(2*pi/0.084*t(i)-0*pi/180))*pi/180; 
    th24(i) = (-68+30*cos(2*pi/0.084*t(i)-0*pi/180))*pi/180; 
    th34(i) = (11+28*cos(2*pi/0.084*t(i)+180*pi/180))*pi/180;
    xw1(i) = (-cos(gm)*sin(th11(i)).*cos(th21(i))- cos(gm)*cos(th11(i)).*...
            sin(th21(i))*cos(alpha1)+sin(gm)*sin(th21(i))*sin(alpha1))*a2- cos(gm)...
            *sin(th11(i))*a1-cos(gm)*cos(th11(i))*d2*sin(alpha1)- sin(gm)*d2*...
            cos(alpha1)-sin(gm)*d1;
    yw1(i) = (cos(th11(i)).*cos(th21(i))-sin(th11(i)).*sin(th21(i))*... 
            cos(alpha1))*a2+cos(th11(i))*a1-sin(th11(i))*d2*sin(alpha1);
    zw1(i) = (sin(gm)*sin(th11(i)).*cos(th21(i))+sin(gm)*cos(th11(i)).*...
            sin(th21(i))*cos(alpha1)+cos(gm)*sin(th21(i))*sin(alpha1))*a2+sin(gm )...
            *sin(th11(i))*a1+sin(gm)*cos(th11(i))*d2*sin(alpha1)- cos(gm)*d2*...
            cos(alpha1)-cos(gm)*d1;
    xw2(i) = (-cos(gm)*sin(th12(i)).*cos(th22(i))- cos(gm)*cos(th12(i)).*...
            sin(th22(i))*cos(alpha1)+sin(gm)*sin(th22(i))*sin(alpha1))*a2- cos(gm)...
            *sin(th12(i))*a1-cos(gm)*cos(th12(i))*d2*sin(alpha1)- sin(gm)*d2*...
            cos(alpha1)-sin(gm)*d1;
    yw2(i) = (cos(th12(i)).*cos(th22(i))-sin(th12(i)).*sin(th22(i))*... 
            cos(alpha1))*a2+cos(th12(i))*a1-sin(th12(i))*d2*sin(alpha1)+x0;
    zw2(i) = (sin(gm)*sin(th12(i)).*cos(th22(i))+sin(gm)*cos(th12(i)).*...
            sin(th22(i))*cos(alpha1)+cos(gm)*sin(th22(i))*sin(alpha1))*a2+sin(gm )...
            *sin(th12(i))*a1+sin(gm)*cos(th12(i))*d2*sin(alpha1)- cos(gm)*d2*...
            cos(alpha1)-cos(gm)*d1;
    xw3(i) = (-cos(gm)*sin(th13(i)).*cos(th23(i))- cos(gm)*cos(th13(i)).*...
            sin(th23(i))*cos(alpha1)+sin(gm)*sin(th23(i))*sin(alpha1))*a2- cos(gm)...
            *sin(th13(i))*a1-cos(gm)*cos(th13(i))*d2*sin(alpha1)- sin(gm)*d2*...
            cos(alpha1)-sin(gm)*d1;
    yw3(i) = (cos(th13(i)).*cos(th23(i))-sin(th13(i)).*sin(th23(i))*... 
         cos(alpha1))*a2+cos(th13(i))*a1-sin(th13(i))*d2*sin(alpha1)+x0;
    zw3(i) = (sin(gm)*sin(th13(i)).*cos(th23(i))+sin(gm)*cos(th13(i)).*...
            sin(th23(i))*cos(alpha1)+cos(gm)*sin(th23(i))*sin(alpha1))*a2+sin(gm )...
            *sin(th13(i))*a1+sin(gm)*cos(th13(i))*d2*sin(alpha1)- cos(gm)*d2*...
            cos(alpha1)-cos(gm)*d1;
    xw4(i) = (-cos(gm)*sin(th14(i)).*cos(th24(i))- cos(gm)*cos(th14(i)).*...
            sin(th24(i))*cos(alpha1)+sin(gm)*sin(th24(i))*sin(alpha1))*a2- cos(gm)...
            *sin(th14(i))*a1-cos(gm)*cos(th14(i))*d2*sin(alpha1)- sin(gm)*d2*...
            cos(alpha1)-sin(gm)*d1;
    yw4(i) = (cos(th14(i)).*cos(th24(i))-sin(th14(i)).*sin(th24(i))*... 
            cos(alpha1))*a2+cos(th11(i))*a1-sin(th11(i))*d2*sin(alpha1)+x0;
    zw4(i) = (sin(gm)*sin(th14(i)).*cos(th24(i))+sin(gm)*cos(th14(i)).*...
            sin(th24(i))*cos(alpha1)+cos(gm)*sin(th24(i))*sin(alpha1))*a2+sin(gm )...
            *sin(th14(i))*a1+sin(gm)*cos(th14(i))*d2*sin(alpha1)- cos(gm)*d2*...
            cos(alpha1)-cos(gm)*d1; 
    xtip(i) = ((-cos(gm)*sin(th11(i)).*cos(th21(i))-...
            cos(gm)*cos(th11(i)).*...
            sin(th21(i))*cos(alpha1)+sin(gm)*sin(th21(i))*sin(alpha1)).*cos(th31 (i))...
            +(cos(gm)*sin(th11(i)).*sin(th21(i))-cos(gm)*cos(th11(i)).*... 
            cos(th21(i))*cos(alpha1)+sin(gm)*cos(th21(i))*sin(alpha1)).*... 
            sin(th31(i)))*x3f+(-(-cos(gm)*sin(th11(i)).*cos(th21(i))-...
            cos(gm)*... 
            cos(th11(i)).*sin(th21(i))*cos(alpha1)+sin(gm)*sin(th21(i))*... 
            sin(alpha1)).*sin(th31(i))+(cos(gm)*sin(th11(i)).*sin(th21(i))-...
            cos(gm)*cos(th11(i)).*cos(th21(i))*cos(alpha1)+sin(gm)*cos(th21(i))*...
            sin(alpha1)).*cos(th31(i)))*y3f+(cos(gm)*cos(th11(i))*sin(alpha1)+...
            sin(gm)*cos(alpha1))*z3f+(-cos(gm)*sin(th11(i)).*cos(th21(i))- cos(gm)*cos(th11(i)).*sin(th21(i))*cos(alpha1)+sin(gm)*sin(th21(i))*...
            sin(alpha1))*a2-cos(gm)*sin(th11(i))*a1- cos(gm)*cos(th11(i))*d2*...
        sin(alpha1)-sin(gm)*d2*cos(alpha1)-sin(gm)*d1;
    ytip(i) = ((cos(th11(i)).*cos(th21(i))- sin(th11(i)).*sin(th21(i))*...
            cos(alpha1)).*cos(th31(i))+(-cos(th11(i)).*sin(th21(i))- sin(th11(i)).*...
            cos(th21(i))*cos(alpha1)).*sin(th31(i)))*x3f+(- (cos(th11(i)).*cos(th21(i))...
            -sin(th11(i)).*sin(th21(i))*cos(alpha1)).*sin(th31(i))+(- cos(th11(i)).*...
            sin(th21(i))- sin(th11(i)).*cos(th21(i))*cos(alpha1)).*cos(th31(i)))*...
            y3f+sin(th11(i))*sin(alpha1)*z3f+(cos(th11(i)).*cos(th21(i))-... 
            sin(th11(i)).*sin(th21(i))*cos(alpha1))*a2+cos(th11(i))*a1-... 
            sin(th11(i))*d2*sin(alpha1)+x0;
    ztip(i) = ((sin(gm)*sin(th11(i)).*cos(th21(i))+sin(gm)*cos(th11(i)).*...
            sin(th21(i))*cos(alpha1)+cos(gm)*sin(th21(i))*sin(alpha1)).*...
            cos(th31(i))+(- sin(gm)*sin(th11(i)).*sin(th21(i))+sin(gm)*cos(th11(i)).*...
            cos(th21(i))*cos(alpha1)+cos(gm)*cos(th21(i))*sin(alpha1)).*...
            sin(th31(i)))*x3f+(- (sin(gm)*sin(th11(i)).*cos(th21(i))+sin(gm)*...
            cos(th11(i)).*sin(th21(i))*cos(alpha1)+cos(gm)*sin(th21(i))*...
            sin(alpha1)).*sin(th31(i))+(- sin(gm)*sin(th11(i)).*sin(th21(i))+...
            sin(gm)*cos(th11(i)).*cos(th21(i))*cos(alpha1)+cos(gm)*cos(th21(i))*... 
            sin(gm)*cos(th11(i))*sin(alpha1)+...
            sin(alpha1)).*cos(th31(i)))*y3f+(- cos(gm)*cos(alpha1))*z3f+(sin(gm)*sin(th11(i)).*cos(th21(i))+sin(gm)*... 
            sin(alpha1))*a2+sin(gm)*sin(th11(i))*a1+sin(gm)*cos(th11(i))*d2*...
            cos(th11(i)).*sin(th21(i))*cos(alpha1)+cos(gm)*sin(th21(i))*... 
            sin(alpha1)-cos(gm)*d2*cos(alpha1)-cos(gm)*d1;
    x4d(i) = ((-cos(gm)*sin(th11(i)).*cos(th21(i))- cos(gm)*cos(th11(i)).*...
            sin(th21(i))*cos(alpha1)+sin(gm)*sin(th21(i))*sin(alpha1)).*cos(th31 (i))...
            +(cos(gm)*sin(th11(i)).*sin(th21(i))-cos(gm)*cos(th11(i)).*... 
            cos(th21(i))*cos(alpha1)+sin(gm)*cos(th21(i))*sin(alpha1)).*... 
            sin(th31(i)))*x4f+(-(-cos(gm)*sin(th11(i)).*cos(th21(i))-...
            cos(gm)*... 
            cos(th11(i)).*sin(th21(i))*cos(alpha1)+sin(gm)*sin(th21(i))*... 
            sin(alpha1)).*sin(th31(i))+(cos(gm)*sin(th11(i)).*sin(th21(i))-...
            cos(gm)*cos(th11(i)).*cos(th21(i))*cos(alpha1)+sin(gm)*cos(th21(i))*...
            sin(alpha1)).*cos(th31(i)))*y4f+(cos(gm)*cos(th11(i))*sin(alpha1)+...
            sin(gm)*cos(alpha1))*z4f+(-cos(gm)*sin(th11(i)).*cos(th21(i))- cos(gm)*cos(th11(i)).*sin(th21(i))*cos(alpha1)+sin(gm)*sin(th21(i))*...
            sin(alpha1))*a2-cos(gm)*sin(th11(i))*a1- cos(gm)*cos(th11(i))*d2*...
            sin(alpha1)-sin(gm)*d2*cos(alpha1)-sin(gm)*d1;
    y4d(i) = ((cos(th11(i)).*cos(th21(i))-sin(th11(i)).*sin(th21(i))*... 
            cos(alpha1)).*cos(th31(i))+(-cos(th11(i)).*sin(th21(i))-...
            sin(th11(i)).*... 
            cos(th21(i))*cos(alpha1)).*sin(th31(i)))*x4f+(-(cos(th11(i)).*cos(th21(i))... 
            -sin(th11(i)).*sin(th21(i))*cos(alpha1)).*sin(th31(i))+(-cos(th11(i)).*... 
            sin(th21(i))-sin(th11(i)).*cos(th21(i))*cos(alpha1)).*cos(th31(i)))*... 
            y4f+sin(th11(i))*sin(alpha1)*z4f+(cos(th11(i)).*cos(th21(i))-... 
            sin(th11(i)).*sin(th21(i))*cos(alpha1))*a2+cos(th11(i))*a1-... 
            sin(th11(i))*d2*sin(alpha1)+x0;
    z4d(i) = ((sin(gm)*sin(th11(i)).*cos(th21(i))+sin(gm)*cos(th11(i)).*...
            sin(th21(i))*cos(alpha1)+cos(gm)*sin(th21(i))*sin(alpha1)).*...
            cos(th31(i))+(- sin(gm)*sin(th11(i)).*sin(th21(i))+sin(gm)*cos(th11(i)).*...
            cos(th21(i))*cos(alpha1)+cos(gm)*cos(th21(i))*sin(alpha1)).*...
            sin(th31(i)))*x4f+(- (sin(gm)*sin(th11(i)).*cos(th21(i))+sin(gm)*...
            cos(th11(i)).*sin(th21(i))*cos(alpha1)+cos(gm)*sin(th21(i))*...
            sin(alpha1)).*sin(th31(i))+(- sin(gm)*sin(th11(i)).*sin(th21(i))+...
            sin(gm)*cos(th11(i)).*cos(th21(i))*cos(alpha1)+cos(gm)*cos(th21(i))*... 
            sin(gm)*cos(th11(i))*sin(alpha1)+...
            sin(alpha1)).*cos(th31(i)))*y4f+(- cos(gm)*cos(alpha1))*z4f+(sin(gm)*sin(th11(i)).*cos(th21(i))+sin(gm)... 
            *sin(alpha1))*a2+sin(gm)*sin(th11(i))*a1+sin(gm)*cos(th11(i))*d2*...
            cos(th11(i)).*sin(th21(i))*cos(alpha1)+cos(gm)*sin(th21(i))*... 
            sin(alpha1)-cos(gm)*d2*cos(alpha1)-cos(gm)*d1;
    x5d(i) = ((-cos(gm)*sin(th11(i)).*cos(th21(i))- cos(gm)*cos(th11(i)).*...
            sin(th21(i))*cos(alpha1)+sin(gm)*sin(th21(i))*sin(alpha1)).*cos(th31 (i))...
            +(cos(gm)*sin(th11(i)).*sin(th21(i))-cos(gm)*cos(th11(i)).*... 
            cos(th21(i))*cos(alpha1)+sin(gm)*cos(th21(i))*sin(alpha1)).*...
            sin(th31(i)))*x5f+(-(-cos(gm)*sin(th11(i)).*cos(th21(i))-...
            cos(gm)*... 
            cos(th11(i)).*sin(th21(i))*cos(alpha1)+sin(gm)*sin(th21(i))*... 
            sin(alpha1)).*sin(th31(i))+(cos(gm)*sin(th11(i)).*sin(th21(i))-...
            cos(gm)*cos(th11(i)).*cos(th21(i))*cos(alpha1)+sin(gm)*cos(th21(i))*...
            sin(alpha1)).*cos(th31(i)))*y5f+(cos(gm)*cos(th11(i))*sin(alpha1)+...
            sin(gm)*cos(alpha1))*z5f+(-cos(gm)*sin(th11(i)).*cos(th21(i))- cos(gm)*cos(th11(i)).*sin(th21(i))*cos(alpha1)+sin(gm)*sin(th21(i))*...
            sin(alpha1))*a2-cos(gm)*sin(th11(i))*a1- cos(gm)*cos(th11(i))*d2*...
            sin(alpha1)-sin(gm)*d2*cos(alpha1)-sin(gm)*d1;
    y5d(i) = ((cos(th11(i)).*cos(th21(i))-sin(th11(i)).*sin(th21(i))*... 
            cos(alpha1)).*cos(th31(i))+(-cos(th11(i)).*sin(th21(i))-sin(th11(i)).*... 
            cos(th21(i))*cos(alpha1)).*sin(th31(i)))*x5f+(-(cos(th11(i)).*cos(th21(i))... 
            -sin(th11(i)).*sin(th21(i))*cos(alpha1)).*sin(th31(i))+(-...
            cos(th11(i)).*... 
            sin(th21(i))-sin(th11(i)).*cos(th21(i))*cos(alpha1)).*cos(th31(i)))*... 
            y5f+sin(th11(i))*sin(alpha1)*z5f+(cos(th11(i)).*cos(th21(i))-... 
            sin(th11(i)).*sin(th21(i))*cos(alpha1))*a2+cos(th11(i))*a1-... 
            sin(th11(i))*d2*sin(alpha1)+x0;
    z5d(i) = ((sin(gm)*sin(th11(i)).*cos(th21(i))+sin(gm)*cos(th11(i)).*...
            sin(th21(i))*cos(alpha1)+cos(gm)*sin(th21(i))*sin(alpha1)).*...
            cos(th31(i))+(- sin(gm)*sin(th11(i)).*sin(th21(i))+sin(gm)*cos(th11(i)).*...
            cos(th21(i))*cos(alpha1)+cos(gm)*cos(th21(i))*sin(alpha1)).*...
            sin(th31(i)))*x5f+(- (sin(gm)*sin(th11(i)).*cos(th21(i))+sin(gm)*...
            cos(th11(i)).*sin(th21(i))*cos(alpha1)+cos(gm)*sin(th21(i))*...
            sin(alpha1)).*sin(th31(i))+(- sin(gm)*sin(th11(i)).*sin(th21(i))+...
            sin(gm)*cos(th11(i)).*cos(th21(i))*cos(alpha1)+cos(gm)*cos(th21(i))*... 
            sin(gm)*cos(th11(i))*sin(alpha1)+...
            sin(alpha1)).*cos(th31(i)))*y5f+(- cos(gm)*cos(alpha1))*z5f+(sin(gm)*sin(th11(i)).*cos(th21(i))+sin(gm)... 
            *sin(alpha1))*a2+sin(gm)*sin(th11(i))*a1+sin(gm)*cos(th11(i))*d2*...
            cos(th11(i)).*sin(th21(i))*cos(alpha1)+cos(gm)*sin(th21(i))*... 
            sin(alpha1)-cos(gm)*d2*cos(alpha1)-cos(gm)*d1;
    xtip2(i) = ((-cos(gm)*sin(th11(i)).*cos(th21(i))- cos(gm)*cos(th11(i)).*...
            sin(th21(i))*cos(alpha1)+sin(gm)*sin(th21(i))*sin(alpha1)).*cos(th32 (i))...
            +(cos(gm)*sin(th11(i)).*sin(th21(i))-cos(gm)*cos(th11(i)).*... 
            cos(th21(i))*cos(alpha1)+sin(gm)*cos(th21(i))*sin(alpha1)).*... 
            sin(th32(i)))*x3f+(-(-cos(gm)*sin(th11(i)).*cos(th21(i))-...
            cos(gm)*... 
            cos(th11(i)).*sin(th21(i))*cos(alpha1)+sin(gm)*sin(th21(i))*... 
            sin(alpha1)).*sin(th32(i))+(cos(gm)*sin(th11(i)).*sin(th21(i))-...
            cos(gm)*cos(th11(i)).*cos(th21(i))*cos(alpha1)+sin(gm)*cos(th21(i))*...
            sin(alpha1)).*cos(th32(i)))*y3f+(cos(gm)*cos(th11(i))*sin(alpha1)+...
            sin(gm)*cos(alpha1))*z3f+(-cos(gm)*sin(th11(i)).*cos(th21(i))- cos(gm)*cos(th11(i)).*sin(th21(i))*cos(alpha1)+sin(gm)*sin(th21(i))*...
            sin(alpha1))*a2-cos(gm)*sin(th11(i))*a1- cos(gm)*cos(th11(i))*d2*...
            sin(alpha1)-sin(gm)*d2*cos(alpha1)-sin(gm)*d1;
    ytip2(i) = ((cos(th11(i)).*cos(th21(i))- sin(th11(i)).*sin(th21(i))*...
            cos(alpha1)).*cos(th32(i))+(-cos(th11(i)).*sin(th21(i))- sin(th11(i)).*...
            cos(th21(i))*cos(alpha1)).*sin(th32(i)))*x3f+(- (cos(th11(i)).*cos(th21(i))...
            -sin(th11(i)).*sin(th21(i))*cos(alpha1)).*sin(th32(i))+(- cos(th11(i)).*...
            sin(th21(i))- sin(th11(i)).*cos(th21(i))*cos(alpha1)).*cos(th32(i)))*...
            y3f+sin(th11(i))*sin(alpha1)*z3f+(cos(th11(i)).*cos(th21(i))-... 
            sin(th11(i)).*sin(th21(i))*cos(alpha1))*a2+cos(th11(i))*a1-... 
            sin(th11(i))*d2*sin(alpha1)+x0;
    ztip2(i) = ((sin(gm)*sin(th11(i)).*cos(th21(i))+sin(gm)*cos(th11(i)).*...
            sin(th21(i))*cos(alpha1)+cos(gm)*sin(th21(i))*sin(alpha1)).*...
            cos(th32(i))+(- sin(gm)*sin(th11(i)).*sin(th21(i))+sin(gm)*cos(th11(i)).*...
            cos(th21(i))*cos(alpha1)+cos(gm)*cos(th21(i))*sin(alpha1)).*...
            sin(th32(i)))*x3f+(- (sin(gm)*sin(th11(i)).*cos(th21(i))+sin(gm)*...
            cos(th11(i)).*sin(th21(i))*cos(alpha1)+cos(gm)*sin(th21(i))*...
            sin(alpha1)).*sin(th32(i))+(- sin(gm)*sin(th11(i)).*sin(th21(i))+...
            sin(gm)*cos(th11(i)).*cos(th21(i))*cos(alpha1)+cos(gm)*cos(th21(i))*... 
            sin(gm)*cos(th11(i))*sin(alpha1)+...
            sin(alpha1)).*cos(th32(i)))*y3f+(- cos(gm)*cos(alpha1))*z3f+(sin(gm)*sin(th11(i)).*cos(th21(i))+sin(gm)... 
            *sin(alpha1))*a2+sin(gm)*sin(th11(i))*a1+sin(gm)*cos(th11(i))*d2*...
            cos(th11(i)).*sin(th21(i))*cos(alpha1)+cos(gm)*sin(th21(i))*... 
            sin(alpha1)-cos(gm)*d2*cos(alpha1)-cos(gm)*d1;
    xtip3(i) = ((-cos(gm)*sin(th11(i)).*cos(th21(i))- cos(gm)*cos(th11(i)).*...
            sin(th21(i))*cos(alpha1)+sin(gm)*sin(th21(i))*sin(alpha1)).*cos(th33 (i))...
            +(cos(gm)*sin(th11(i)).*sin(th21(i))-cos(gm)*cos(th11(i)).*... 
            cos(th21(i))*cos(alpha1)+sin(gm)*cos(th21(i))*sin(alpha1)).*... 
            sin(th33(i)))*x3f+(-(-cos(gm)*sin(th11(i)).*cos(th21(i))-...
            cos(gm)*... 
            cos(th11(i)).*sin(th21(i))*cos(alpha1)+sin(gm)*sin(th21(i))*... 
            sin(alpha1)).*sin(th33(i))+(cos(gm)*sin(th11(i)).*sin(th21(i))-...
            cos(gm)*cos(th11(i)).*cos(th21(i))*cos(alpha1)+sin(gm)*cos(th21(i))*...
            sin(alpha1)).*cos(th33(i)))*y3f+(cos(gm)*cos(th11(i))*sin(alpha1)+...
            sin(gm)*cos(alpha1))*z3f+(-cos(gm)*sin(th11(i)).*cos(th21(i))- cos(gm)*cos(th11(i)).*sin(th21(i))*cos(alpha1)+sin(gm)*sin(th21(i))*...
            sin(alpha1))*a2-cos(gm)*sin(th11(i))*a1- cos(gm)*cos(th11(i))*d2*...
            sin(alpha1)-sin(gm)*d2*cos(alpha1)-sin(gm)*d1;
    ytip3(i) = ((cos(th11(i)).*cos(th21(i))- sin(th11(i)).*sin(th21(i))*...
            cos(alpha1)).*cos(th33(i))+(-cos(th11(i)).*sin(th21(i))- sin(th11(i)).*...
            cos(th21(i))*cos(alpha1)).*sin(th33(i)))*x3f+(- (cos(th11(i)).*cos(th21(i))...
            -sin(th11(i)).*sin(th21(i))*cos(alpha1)).*sin(th33(i))+(- cos(th11(i)).*...
            sin(th21(i))- sin(th11(i)).*cos(th21(i))*cos(alpha1)).*cos(th33(i)))*...
            y3f+sin(th11(i))*sin(alpha1)*z3f+(cos(th11(i)).*cos(th21(i))-... 
            sin(th11(i)).*sin(th21(i))*cos(alpha1))*a2+cos(th11(i))*a1-... 
            sin(th11(i))*d2*sin(alpha1)+x0;
    ztip3(i) = ((sin(gm)*sin(th11(i)).*cos(th21(i))+sin(gm)*cos(th11(i)).*...
            sin(th21(i))*cos(alpha1)+cos(gm)*sin(th21(i))*sin(alpha1)).*...
            cos(th33(i))+(- sin(gm)*sin(th11(i)).*sin(th21(i))+sin(gm)*cos(th11(i)).*...
            cos(th21(i))*cos(alpha1)+cos(gm)*cos(th21(i))*sin(alpha1)).*...
            sin(th33(i)))*x3f+(- (sin(gm)*sin(th11(i)).*cos(th21(i))+sin(gm)*...
            cos(th11(i)).*sin(th21(i))*cos(alpha1)+cos(gm)*sin(th21(i))*...
            sin(alpha1)).*sin(th33(i))+(- sin(gm)*sin(th11(i)).*sin(th21(i))+...
            sin(gm)*cos(th11(i)).*cos(th21(i))*cos(alpha1)+cos(gm)*cos(th21(i))*... 
            sin(gm)*cos(th11(i))*sin(alpha1)+...
            sin(alpha1)).*cos(th33(i)))*y3f+(-...
            cos(gm)*cos(alpha1))*z3f+(sin(gm)*sin(th11(i)).*cos(th21(i))+sin(gm)...
            *cos(th11(i)).*sin(th21(i))*cos(alpha1)+cos(gm)*sin(th21(i))*... 
            sin(alpha1))*a2+sin(gm)*sin(th11(i))*a1+sin(gm)*cos(th11(i))*d2*...
            sin(alpha1)-cos(gm)*d2*cos(alpha1)-cos(gm)*d1; 
    xtip4(i) = ((-cos(gm)*sin(th11(i)).*cos(th21(i))-cos(gm)*cos(th11(i)).*...
            sin(th21(i))*cos(alpha1)+sin(gm)*sin(th21(i))*sin(alpha1)).*cos(th34 (i))...
            +(cos(gm)*sin(th11(i)).*sin(th21(i))-cos(gm)*cos(th11(i)).*... 
            cos(th21(i))*cos(alpha1)+sin(gm)*cos(th21(i))*sin(alpha1)).*... 
            sin(th34(i)))*x3f+(-(-cos(gm)*sin(th11(i)).*cos(th21(i))-...
            cos(gm)*... 
            cos(th11(i)).*sin(th21(i))*cos(alpha1)+sin(gm)*sin(th21(i))*... 
            sin(alpha1)).*sin(th34(i))+(cos(gm)*sin(th11(i)).*sin(th21(i))-...
            cos(gm)*cos(th11(i)).*cos(th21(i))*cos(alpha1)+sin(gm)*cos(th21(i))*...
            sin(alpha1)).*cos(th34(i)))*y3f+(cos(gm)*cos(th11(i))*sin(alpha1)+...
            sin(gm)*cos(alpha1))*z3f+(-cos(gm)*sin(th11(i)).*cos(th21(i))- cos(gm)*cos(th11(i)).*sin(th21(i))*cos(alpha1)+sin(gm)*sin(th21(i))*... 
            sin(alpha1))*a2-cos(gm)*sin(th11(i))*a1- cos(gm)*cos(th11(i))*d2*...
            sin(alpha1)-sin(gm)*d2*cos(alpha1)-sin(gm)*d1;
    ytip4(i) = ((cos(th11(i)).*cos(th21(i))- sin(th11(i)).*sin(th21(i))*...
            cos(alpha1)).*cos(th34(i))+(-cos(th11(i)).*sin(th21(i))- sin(th11(i)).*...
            cos(th21(i))*cos(alpha1)).*sin(th34(i)))*x3f+(- (cos(th11(i)).*cos(th21(i))...
            -sin(th11(i)).*sin(th21(i))*cos(alpha1)).*sin(th34(i))+(- cos(th11(i)).*...
            sin(th21(i))- sin(th11(i)).*cos(th21(i))*cos(alpha1)).*cos(th34(i)))*...
            y3f+sin(th11(i))*sin(alpha1)*z3f+(cos(th11(i)).*cos(th21(i))-...
            sin(th11(i)).*sin(th21(i))*cos(alpha1))*a2+cos(th11(i))*a1-... 
            sin(th11(i))*d2*sin(alpha1)+x0;
            ztip4(i) = ((sin(gm)*sin(th11(i)).*cos(th21(i))+sin(gm)*cos(th11(i)).*...
            sin(th21(i))*cos(alpha1)+cos(gm)*sin(th21(i))*sin(alpha1)).*...
            cos(th34(i))+(- sin(gm)*sin(th11(i)).*sin(th21(i))+sin(gm)*cos(th11(i)).*...
            cos(th21(i))*cos(alpha1)+cos(gm)*cos(th21(i))*sin(alpha1)).*...
            sin(th34(i)))*x3f+(- (sin(gm)*sin(th11(i)).*cos(th21(i))+sin(gm)*...
            cos(th11(i)).*sin(th21(i))*cos(alpha1)+cos(gm)*sin(th21(i))*...
            sin(alpha1)).*sin(th34(i))+(- sin(gm)*sin(th11(i)).*sin(th21(i))+...
            sin(gm)*cos(th11(i)).*cos(th21(i))*cos(alpha1)+cos(gm)*cos(th21(i))*...
            sin(gm)*cos(th11(i))*sin(alpha1)+...
            sin(alpha1)).*cos(th34(i)))*y3f+(- cos(gm)*cos(alpha1))*z3f+(sin(gm)*sin(th11(i)).*cos(th21(i))+sin(gm)...
            *sin(alpha1))*a2+sin(gm)*sin(th11(i))*a1+sin(gm)*cos(th11(i))*d2*...
            cos(th11(i)).*sin(th21(i))*cos(alpha1)+cos(gm)*sin(th21(i))*... 
            sin(alpha1)-cos(gm)*d2*cos(alpha1)-cos(gm)*d1;
    th11(i) = th11(i)*180/pi; 
    th21(i) = th21(i)*180/pi; 
    th22(i) = th22(i)*180/pi; 
    th23(i) = th23(i)*180/pi; 
    th24(i) = th24(i)*180/pi;
    th31(i) = th31(i)*180/pi; 
end
scrsz = get(0,'ScreenSize'); % Get current screen size
%=================================================================== %	Create a Table to display the angle values %------------------------------------------------------------------- Result = [ count(:)	t(:)	th11(:)	th21(:)	th31(:)]; disp('	count	t(s)	th_1	th_2	th_3	') disp(' ----------------------------------------------------------') disp(Result) %------------------------------------------------------------------- xlswrite('Angle_values',Result)
% +-------------------------------------------+ % % |----	Figure 1: Wrist trajectories	----| % % +-------------------------------------------+ %

figure('Name','Wrist trajectories','NumberTitle','on','OuterPosition',[1 scrsz(3) scrsz(3)/3 scrsz(4)/2]) % Postion figure on the left side of the screen
clf
plot3(xtip(:),ytip(:),ztip(:),'-k','linewidth',5) 
hold on 
plot3(xtip2(:),ytip2(:),ztip2(:),'-','linewidth',5,'color',[0.4 0.4 0.4]) 
plot3(xtip3(:),ytip3(:),ztip3(:),'-','linewidth',5,'color',[0.7 0.7 0.7]) 
plot3(xtip4(:),ytip4(:),ztip4(:),'.k','markersize',20)
%plot3(xwP,ywP,zwP,'dk','markerfacecolor','g','markersize',12)
%plot3(xwP(1),ywP(1),zwP(1),'ok','markerfacecolor','r','markersize',15)
plot3(null-10,yw1(:),zw1(:),'-k','linewidth',3) 
plot3(xw1(:),null-10,zw1(:),'-k','linewidth',3) 
plot3(xw1(:),yw1(:),null-30,'-k','linewidth',3)
%plot3(null-10,ywP,zwP,'dk','markerfacecolor','g','markersize',8) 
%plot3(xwP,null-10,zwP,'dk','markerfacecolor','g','markersize',8) 
%plot3(xwP,ywP,null-30,'dk','markerfacecolor','g','markersize',8)
plot3(null-10,yw2(:),zw2(:),'-','linewidth',3,'color',[0.4 0.4 0.4])
plot3(xw2(:),null-10,zw2(:),'-','linewidth',3,'color',[0.4 0.4 0.4])
plot3(xw2(:),yw2(:),null-30,'-','linewidth',3,'color',[0.4 0.4 0.4])
plot3(null-10,yw3(:),zw3(:),'-','linewidth',3,'color',[0.7 0.7 0.7])
plot3(xw3(:),null-10,zw3(:),'-','linewidth',3,'color',[0.7 0.7 0.7])
plot3(xw3(:),yw3(:),null-30,'-','linewidth',3,'color',[0.7 0.7 0.7])
plot3(null-10,yw4(:),zw4(:),'.k','markersize',15) 
plot3(xw4(:),null-10,zw4(:),'.k','markersize',15) 
plot3(xw4(:),yw1(:),null-30,'.k','markersize',15)
xlabel('\bf{x_b, [mm]}','position',[15 65 -38],'FontSize',16) 
ylabel('\bf{y_b, [mm]}','position',[66 40 -27],'FontSize',16)
zlabel('\bf{z_b, [mm]}','position',[56 -12 5],'FontSize',16)
set(gca,'LineWidth',3,'FontSize',10) 
set(gca,'YTick',[-10;10;30;50;70]) 
set(gca,'XTick',[-10;10;30;50]) 
set(gca,'ZTick',[-30;-10;10;30;50])
axis([-10 50 -10 70 -30 50]);
grid on 
view([138 20])
legend('\phi_{2} = -180^o','\phi_{2} = - 120^o','\phi_{2} = - 60^o','\phi_{2} = 0^o','Plecotus')

% +---------------------------------------------+ % % |----	Figure 2: Wingtip trajectories	----| % % +---------------------------------------------+ %

figure('Name','Wingtip trajectories','NumberTitle','on','OuterPosition',[scrsz(3)/3 scrsz(3) scrsz(3)/3 scrsz(4)/2]) % Postion figure on the left side of the screen
clf
plot3(xtip(:),ytip(:),ztip(:),'-r','linewidth',5)
hold on 
%plot3(x3P,y3P,z3P,'dk','markerfacecolor','g','markersize',12) 
plot3(null-70,ytip(:),ztip(:),'-r','linewidth',3) 
plot3(xtip(:),null+50,ztip(:),'-r','linewidth',3)
plot3(xtip(:),ytip(:),null-70,'-r','linewidth',3)
%plot3(null-70,y3P,z3P,'dk','markerfacecolor','g','markersize',8)
%plot3(x3P,null+50,z3P,'dk','markerfacecolor','g','markersize',8) 
%plot3(x3P,y3P,null-70,'dk','markerfacecolor','g','markersize',8)
title('\bf{\theta_{30}= 10^o, A_{3}= 25^o}','position',[-10 90 120],'BackgroundColor',[1 1 1],'FontSize',22) 
xlabel('\bf{x_b, [mm]}','position',[-10 145 -90],'FontSize',16) 
ylabel('\bf{y_b, [mm]}','position',[95 90 -85],'FontSize',16) 
zlabel('\bf{z_b, [mm]}','position',[100 43 10],'FontSize',16) 
set(gca,'LineWidth',3,'FontSize',12)
grid on 
axis([-80 90 50 160 -70 120]); 
set(gca,'YTick',[50;80;110;140]) 
set(gca,'XTick',[-70;-30;10;50;90]) 
set(gca,'ZTick',[-70;-30;10;50;90]) 
view([138 12])

% +-------------------------------------------------------+ % % |----	Figure 3: Fourth finger tip trajectories	----| % % +-------------------------------------------------------+ %

figure('Name','Fourth finger trajectories','NumberTitle','on','OuterPosition',[scrsz(3)/1.5 scrsz(3) scrsz(3)/3 scrsz(4)/2]) % Postion figure on the left side of the screen
clf
plot3(x4d(:),y4d(:),z4d(:),'-r','linewidth',5) 
hold on
%plot3(x4P,y4P,z4P,'dk','markerfacecolor','g','markersize',12)
plot3(null-60,y4d(:),z4d(:),'-r','linewidth',3)
plot3(x4d(:),null+30,z4d(:),'-r','linewidth',3) 
plot3(x4d(:),y4d(:),null-50,'-r','linewidth',3)
%plot3(null-60,y4P,z4P,'dk','markerfacecolor','g','markersize',8) 
%plot3(x4P,null+30,z4P,'dk','markerfacecolor','g','markersize',8)
%plot3(x4P,y4P,null-50,'dk','markerfacecolor','g','markersize',8)
title('\bf{Trajectory of 4^{th} finger, 3-DOF model}','FontSize',16) 
xlabel('\bf{x_b, [mm]}','position',[-10 145 -60],'FontSize',16) 
ylabel('\bf{y_b, [mm]}','position',[55 90 -60],'FontSize',16) 
zlabel('\bf{z_b, [mm]}','position',[55 20 10],'FontSize',16) 
set(gca,'LineWidth',3,'FontSize',16)
legend('Model','Plecotus')
grid on 
axis([-60 50 30 140 -50 90]); 
set(gca,'YTick',[50;80;110;140]) 
set(gca,'XTick',[-60;-30;10;50]) 
set(gca,'ZTick',[-70;-30;10;50;90]) 
view([138 12])

% +--------------------------------------------------+ % % |----	Figure 4: Fifth finger trajectories	----| % % +--------------------------------------------------+ %

figure('Name','Fifth finger trajecories','NumberTitle','on','OuterPosition',[1 -20 scrsz(3)/3 scrsz(4)/2]) % Postion figure on the left side of the screen
clf
plot3(x5d(:),y5d(:),z5d(:),'-r','linewidth',5) 
hold on
%plot3(x5P,y5P,z5P,'dk','markerfacecolor','g','markersize',12)
plot3(null-70,y5d(:),z5d(:),'-r','linewidth',3)
plot3(x5d(:),null+0,z5d(:),'-r','linewidth',3) 
plot3(x5d(:),y5d(:),null-70,'-r','linewidth',3)
%plot3(null-70,y5P,z5P,'dk','markerfacecolor','g','markersize',8) 
%plot3(x5P,null+0,z5P,'dk','markerfacecolor','g','markersize',8) 
%plot3(x5P,y5P,null-70,'dk','markerfacecolor','g','markersize',8)
title('\bf{Trajectory of the 5^{th} finger, 3-DOF model}','FontSize',16) 
xlabel('\bf{x_b, [mm]}','position',[-10 105 -75],'FontSize',16)
ylabel('\bf{y_b, [mm]}','position',[35 50 -72],'FontSize',16) 
zlabel('\bf{z_b, [mm]}','position',[27 -5 0],'FontSize',16) 
set(gca,'LineWidth',3,'FontSize',16) 
legend('Model','Plecotus')
grid on 
axis([-70 20 0 90 -70 50]); 
set(gca,'XTick',[-70;-40;-10;20])
set(gca,'YTick',[0;30;60;90])
set(gca,'ZTick',[-70;-30;10;50]) 
view([138 12])


figure(5) 
clf 
plot(t,th11,'.r','markersize',20) 
hold on 
plot(t,th21,'dk','markerfacecolor','k','markersize',8) 
% title('\bf{Joint Angle vs Time, natural bones}','Fontsize',16)
plot(t,th22,'dk','markerfacecolor',[0.4 0.4 0.4],'markersize',8) 
plot(t,th23,'ok','markerfacecolor',[0.6 0.6 0.6],'markersize',8) 
plot(t,th24,'sk','markerfacecolor',[0.8 0.8 0.8],'markersize',8)
xlabel('\bf{Time, [s]}','FontSize',16) 
ylabel('\bf{Joint Angle, [^o]}','FontSize',16) 
legend('\bf{\theta_1}','\bf{\theta_2 = - 180^o}','\bf{\theta_2 = - 120^o}','\bf{\theta_2 = - 60^o}','\bf{\theta_2 = 0^o}') 
grid on 
set(gca,'LineWidth',2,'FontSize',12)
