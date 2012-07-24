%**************************************************************************
%BatBot: Biological inspired Bat roBot.

%Copyright RObotics and Cybernetics Group
%Julian Colorado

% Matlab simulator of bat flight behavior. 
%**************************************************************************

%This program generates cartersian trajectories for Kinematics model computation of flapping motion (2 DoF at shoulder):
%spherical motion of the shoulder

function [Q_flap dQ_flap d2Q_flap] = BatWings_flappingFB(f,freq)


sampled_time = 1/freq;
Time = [0:(sampled_time/f):sampled_time];
step = Time(2)-Time(1);
Per = 1/freq;


%for i = 1:84 
%    t = step*(i);
    %Q_flap  = (5+35*cos(2*pi/0.084*Time-0*pi/180))*pi/180;
    Q_flap(:,1)  = (5+50*cos(2*pi/Per*Time-0*pi/180))*pi/180; %freq allows 1 wingbeat cycle synced with the morphing (FLAPPING) 
    Q_flap(:,2)  = ((5+30*cos(2*pi/Per*Time-25*pi/180))*pi/180); %freq allows 1 wingbeat cycle synced with the morphing 
    
     c = length(Q_flap(:,1));
     %Differentiation to obtain joint velocities
     %Initial speed conditions
     dQ_flap(1,1) = 0;
     dQ_flap(1,2) = Q_flap(1,2)/0.2809;    %boundary condition depends on initial angle defined in: Q_flap(1,2)
     
     for k=1:c-1
        dQ_flap(k+1,1)  = .6347*((Q_flap(k+1,1)-Q_flap(k,1))/step);  %(.6347 is a scale factor for adjustment)
        dQ_flap(k+1,2)  = .5*((Q_flap(k+1,2)-Q_flap(k,2))/step);  %(.5 is a scale factor for adjustment)
     end
     %Differentiation to obtain joint accelerations
     c = length(dQ_flap(:,1));
     %Initial accel conditions
     d2Q_flap(1,1) = -85.84;
     d2Q_flap(1,2) = -dQ_flap(1,2)/0.073;    %boundary condition depends on initial angle defined in: Q_flap(1,2);
     for k=1:c-1
        d2Q_flap(k+1,1)  = .6347*((dQ_flap(k+1,1)-dQ_flap(k,1))/step); %(.6347 is a scale factor for adjustment)
        d2Q_flap(k+1,2)  = .5*(dQ_flap(k+1,2)-dQ_flap(k,2))/step; %(.5 is a scale factor for adjustment)
     end
     
end