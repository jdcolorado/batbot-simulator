%**************************************************************************
%BatBot: Biological inspired Bat roBot.

%Copyright RObotics and Cybernetics Group
%Julian Colorado

% Matlab simulator of bat flight behavior. 
%************


% Direct Kinematics of 3-DOF wing with new configuration natural bones. % It calculates the homogenous transformation matrices between the % coordinate frames.

% The coordinate frames are: % xbybzb - body attached frame with origin in the sternum; % 
%x0y0z0 - body attached frame with origin in the shoulder; 
% x1y1z1 - humerus attached frame; 
% x2y2z2 - radius attached frame with origin in the elbow; 
% x3y3z3 - palmwing attached frame with origin in the wrist;
% Since the xb axis of the sternum frame is not along the common normal
% % % % % % %
% % % % % % % % %
%between the sternum frame and the shoulder frame x0y0z0 we use an intermediate frame xb'yb'zb'to bring xb along this common normal. 
%For this the sternum frame xbybzb is rotated with 90 deg about zb. Then the intermediate frame xb'yb'zb' is rotated with angle gm about xb'
%and then translated with xo along xb' to reach the shoulder frame x0y0z0

clear all 
clc 
syms th1 th2 th3 th4 gm alpha1 alpha2 
a1 = sym('a1','real'); 
a2 = sym('a2','real');
d1 = sym('d1','real'); xw2 = sym('xw2','real'); zw2 = sym('zw2','real'); d2 = sym('d2','real');

d3 = sym('d3','real');
x0 = sym('x0','real'); % translation along xb' with x0 = 13 mm
x2f = sym('x2f','real'); 
y2f = sym('y2f','real'); 
z2f = sym('z2f','real'); 
Lw = sym('Lw','real');
% Homogenous transformation from xbybzb (located in sternum) to the % intermediary frame xb'yb'zb'(rotation with 90 about zb)
%===================================================================
Tb_bprime = [0 -1 0 0;
             1  0 0 0;
             0  0 1 0;
             0  0 0 1];
         
% Homogenous transformation from xb'yb'zb' (located in sternum) % to the shoulder frame x0y0z0
%===================================================================
Tbprime_0 = [1       0         0    x0; 
             0    cos(gm) -sin(gm)  0; 
             0    sin(gm)  cos(gm)  0;
             0       0         0    1];
% Homogenous transformation from body attached frame xbybzb % located in sternum to the body attached frame x0y0z0 located in % the shoulder joint
%===================================================================
Tb0 = Tb_bprime*Tbprime_0;
% Homogenous transformation from fixed shoulder frame x0y0z0 to % the humerus attached frame x1y1z1
%===================================================================
T01 = [cos(th1) -sin(th1) 0 0; 
        sin(th1) cos(th1) 0 0;
         0	       0	  1 -d1; 
         0         0      0  1];

% Homogenous transformation from humerus frame x1y1z1 to % the radius attached frame x2y2z2 located in the elbow joint 
%===================================================================
T12 = [cos(th2)	-sin(th2)	0	a1; 
       sin(th2)*0	cos(th2)*0	-(-1)	0; 
       sin(th2)*(-1) cos(th2)*(-1) 0	0;
           0          0           0     1];

Tb2 = simple(Tb0*T01*T12);

Tb1 = simple(Tb0*T01); 
Xe1 = [a1 0 0 1]'; 
Xe2 = [0 0 d2 1]'; 
Xeb = simple(Tb1*Xe1); 
Xeb2 = simple(Tb2*Xe2);
% Wingtip coordinates with respect to the local frame x3y3z3
X2d3 = [x2f y2f z2f 1]'; 
Xw2 = [xw2 0 zw2 1]'; % Xwb = simple(Tb2*Xw2)
% Wingtip coordinates with respect to the body frame xbybzb
X2b = simple(Tb2*X2d3);