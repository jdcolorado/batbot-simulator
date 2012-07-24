%**************************************************************************
%BatBot: Biological inspired Bat roBot.

%Copyright: Julian Colorado, PhDc
%contact: jdcolorado11@gmail.com

%Dynamics simulator of bat-like morphing-wings.
%**************************************************************************
%Kinematics: 
%  1)Perter Corke toolbox: Denavit Hartenberg based on basic rotation matrices are used.
%  2)Featherstone toolbox: quaternions are used.
%**************************************************************************
%Dynamics:
%  1)Featherstone toolbox: Floating base dynamics equations of motion based
%  on Rigid Body EOM. 
%**************************************************************************
%Featherstone toolbox: Copyright: Roy Featherstone
%http://royfeatherstone.org/spatial/v1/documentation.html
%The dynamics functions described in this folder are intended as an accompaniment to the 
%book Rigid Body Dynamics Algorithms (RBDA):
%http://www.springer.com/engineering/robotics/book/978-0-387-74314-1
%**************************************************************************

clear
%**************************************************************************
%**************************************************************************
%PARAMETERS OF THE ROBOT
%**************************************************************************
%**************************************************************************
freq = 5;  %Flapping+morphing wing frequency during a wingbeat cycle.
F_flapping = 1000;   %flapping factor; 100: matlab code, 1000; from simulink
Theta = 0;           %Position of angle q2 apound gravity axis
alpha1 = 0.069;      %Angle of attack of the wings: 4deg by default
%Parameters for wing morphology (humerus and radius bones)
l1=0.055;
l2=0.07;
lc1=l1/2;
lc2=l2/2;
m1=0.005; %humerus+SMA muscles
m2=0.003; %radius+digits
i1=(1/12)*m1*l1^2;  
i2=(1/12)*m2*l2^2;  
% Parameters for shoulder joint
m_s=0.006;
r_s=0.02;
i_s_z=0.5*m_s*r_s^2; 
i_s_x=0.083*m_s*(3*r_s^2+0.05^2);
i_s_y=0.083*m_s*(3*r_s^2+0.05^2);
% Parameters of body
m_b = 0.017;  % 17grams
lw = 0.015; %[m] x-axis
lh = 0.047; %[m] y-axis
ld = 0.015; %[m] z-axis
i_b_x=0.083*m_b*(lh^2+ld^2);        
i_b_y=0.083*m_b*(lw^2+ld^2);        
i_b_z=0.083*m_b*(lh^2+lw^2);    
%Parameters of MCP-III+digits  
Ltip =.11; %MCP-III; wrist to wingtip distance
m_dig = 0.001;
i3 = (1/12)*m_dig*Ltip^2; 
%Dynamics paraneters
rh = 0.005;  %radio del eslab?n: humerus
Ih = i1;    %humerus inertia
c = 0.007;  %Torsional damping coeficient
mh = 0.008; %total wing: humerus+radius+digits
Xh = lc1;  %distance to link CM
g=9.81;
%Parameters for controller setup PID
Kp = 70;
Kd = 0.003;
Ki = 0.0115;
%Shape Memory alloy actuators parameters 
Rsma = 8; %[ohms]
volt = 4.8; %[Voltage applied]
Ro = 1;
Tau = 0.758;
Ksm = 1.7;

%**************************************************************************
%**************************************************************************
%TRAJECTORY GENERATION
% Cartesian: the function BatWings_FULLbeatFB()generates Cartesian
% trajectories by interpolating data points. Data points are
% biologically-extracted from in-vivo bat flight experiments
%**************************************************************************
%**************************************************************************
% DO NOT MODIFY
% Bio-inspired data points that describe the Cartesian motion of wing
% joints of Carpus(wrist)--angle q4, and the wingtip(MCP_III digit)
Carpus = [1.8 6.7 0; 4.8 5 0; 6.8 1.9 0;7 0.5 0; 6.5 3 0; 5.5 4.5 0; 4 6 0; 2.5 7 0]*(0.01);
MCP_III = [13 6.5 0; 15.5 2.8 0; 17.5 -0.5 0; 17.5 -4 0; 16.8 -1.5 0; 16.3 1.4 0; 15 3.5 0; 13 5.5 0]*(0.01); 
time = [1 2 3 4 5 6 7 8];
flag = 2;
%**************************************************************************
%Calculating a single period of the wingbeat (flag_fusion = 0), or several
%periods (flag_fusion = 1)
%**************************************************************************
flag_fusion = 1;  %1) ON,  0) OFF    
if flag_fusion == 1 %(ON)
    k_wing = 2;   % 2-loops, for both wings: right and left
    times = 10;   % Multiple the number of repeting periods: i.e. (1/t)*times.  
    %Define the maneuver type for each wing [left, right]
    %1: both wings flap symmetrically, ej. flag_maneuover = [1 1];
    %2: wing expanded
    %3: wing conntracted
    %4: (1,2,3) merged with reduction factor
    % ejemplo: flag_maneuover = [3 2]: ala izquierda contraida, ala derecha extendida
    flag_maneuover = [1 4]; 
else
    k_wing = 1; %only 1-loop, 1-wing
    times = 1;
end
%**************************************************************************
step_time = 0.1;
end_time = 1;
%Forward kinematics via spline interpolation
%**************************************************************************
traj_carpus = BatWings_FULLbeatFB(Carpus,time,step_time,end_time);
traj_MCPIII = BatWings_FULLbeatFB(MCP_III,time,step_time,end_time);
[f, c] = size(traj_carpus);
%**************************************************************************
%Inverse kinematics: it retuns jount trajectories of the wings (q3,q4)
[Q_C, Q_MPCIII, dQ_C, dQ_MPCIII,d2Q_C, d2Q_MPCIII, step] = i_kine_bat_wingsFB(traj_carpus,traj_MCPIII,freq,flag,3);
%Fixing data points after numerical derivation
[f, c] = size(Q_C.signals.values(:,1));
[f2, c2] = size(Q_MPCIII.signals.values(:,1));
if f > f2
    f = f2;
else
    f2 = f;
end
%it retuns jount trajectories of the wings (q1,q2)
[Q_flap dQ_flap d2Q_flap] = BatWings_flappingFB(f-1,freq);
%**************************************************************************
[fil, col] = size(Q_flap);
% SET of wing joint trajectories: q1...q5
%joint positions:
Q(:,1) = Theta*ones(fil,1);  %shoulder wing forward/backward (q2)
Q(:,2) = Q_flap(:,1);        %Shoulder Flapping motion  (q1) 
Q(:,3) = Q_C.signals.values(1:f,1)+1.0471; %elbow joint (q3)
Q(:,4) = Q_MPCIII.signals.values(1:f,1)-1.0471; %wrist+digits (q4,5,6)
Q(:,5) = zeros(fil,1);  %(CANCELED) (END EFFECTOR) joint -n-
%Joints angular velocities (dq)
dQ(:,1) = zeros(fil,1);  %(CANCELED)
dQ(:,2) = dQ_flap(:,1);   
dQ(:,3) = dQ_C.signals.values(1:f,1);
dQ(:,4) = dQ_MPCIII.signals.values(1:f,1);
dQ(:,5) = zeros(fil,1);  %(CANCELED)
%Joints angular accelerations (d2q)
d2Q(:,1) = zeros(fil,1);  %(CANCELED)
d2Q(:,2) = d2Q_flap(:,1);
d2Q(:,3) = d2Q_C.signals.values(1:f,1);
d2Q(:,4) = d2Q_MPCIII.signals.values(1:f,1);
d2Q(:,5) = zeros(fil,1);  %(CANCELED)

%**************************************************************************
%**************************************************************************
%SETTING UP ROBOT TOPOLOGY: FLOATING BASE
%**************************************************************************
%Denavit&Hartenberg parameters
  %      alpha           a       q        d   rota    m      sx      sy     sz    Ixx      Iyy     Izz    Ixy   Ixz   Iyz fric
%Fixed base 
L1=link([0               0     Q(1,1)     0    0     m_b      0        0     0    i_b_x    i_b_y   i_b_z    0     0     0  0],'modified');  
%right wing skeleton
L2=link([pi/2+alpha1   0.035   Q(1,2)     0    0     m_s      0        0     0    i_s_x    i_s_y   i_s_z    0     0     0  0],'modified');  
L3=link([-pi/2          l1     Q(1,3)     0    0     m1      l1/2      0     0      0       0       i1      0     0     0  0],'modified');  
L4=link([0              l2     Q(1,4)     0    0     m2      l2/2      0     0      0       0       i2      0     0     0  0],'modified');  
L5=link([0              Ltip   Q(1,5)     0    0     m_dig   Ltip/2    0     0      0       0       i3      0     0     0  0],'modified');  
%**************************************************************************
%Assembling robot topology
BAT_corke=robot({L1 L2 L3 L4 L5}); 
BAT_corke.name='BAT_corke';
n=BAT_corke.n; %Degrees of freedom of each wing
%Time Vector
step_2 = (1/freq)/(fil-1);
Ti=0:step_2:(1/freq);
GRAV = [0 0 0 0 0 -g]'; %gravity vector
F_load = [0 0 0 0 0 0]'; %applied external forces [nx,ny,nz,Fx,Fy,Fz]
Links = BAT_corke.link;
for i=1:n
    DHC(i,:) = Links{i}.dh;
end
%**************************************************************************
%Dynamics Simulations (Peter corke toolbox fixed-base)
%**************************************************************************
%Computing Forces to generate motion within the Follow-The-Leader assigment
%n: number of robots within the MRS.
%DHC:   alpha  a teta  d  sigma  m  Ixx  Iyy  Izz  sx  sy  sz
%Joint Trajectory:  Q,dQ,d2Q  of nxm, where n=# of trajectory points and m=#degrees of freedom   
%6-dimensional External Force: Fext
%6-dimensional gravity vector: GRAV
%**************************************************************************
%Dynamics Simulations (Roy Featherstone Fixed-base (toolbox) 1-WING
%**************************************************************************

BAT_Feat.NB = n;
BAT_Feat.parent = [0:n-1];
BAT_Feat.pitch = zeros(1,n);

for i = 1:n
    DH_row = Links{i}.dh;
    BAT_Feat.Xtree{i} = Xtrans([0,0,DH_row(4)]) * Xrotx(DH_row(1)) * Xtrans([DH_row(2),0,0]);
end

BAT_Feat.I{1} = mcI( m_b,    [0,      0,     0], diag([i_b_x,    i_b_y,      i_b_z]) );
BAT_Feat.I{2} = mcI( m_s,    [0,      0,     0], diag([i_s_x,    i_s_y,      i_s_z]) );
BAT_Feat.I{3} = mcI( m1,     [l1/2,   0,     0], diag([0,      0,          i1]) );
BAT_Feat.I{4} = mcI( m2,     [l2/2,   0,     0], diag([0,      0,          i2]) );
BAT_Feat.I{5} = mcI( m_dig,  [Ltip/2, 0,     0], diag([0,      0,          i3]) );

%Computing forces of the fixed-base system
% for i = 1:fil
%     Torques_feat(i,:) = ID( BAT_Feat, Q(i,1:5), dQ(i,1:5), d2Q(i,1:5) );
% end
%**************************************************************************
%Dynamics Simulations (Roy Featherstone Floating-base (toolbox) 1-WING
%**************************************************************************
%transforming fixed-based bat model to floating base 
fb_BAT_Feat = floatbase( BAT_Feat );
%wing loop  
for k=1:k_wing     
     if flag_fusion == 1 %(ON)
          [Q_new, dQ_new, d2Q_new] = longer_traj_FB(times, flag_maneuover(k), Q, dQ, d2Q);
     else
          Q_new  = Q;
          dQ_new  = dQ;
          d2Q_new  = d2Q;
     end
     QT(:,(5*k)-4:5*k) = Q_new;
    
    [fil, col] = size(Q_new); 
    for i = 1:fil    %# trajectory points loop    
        %adding 6-dimensional virtual base beetween the fixed base and the
        %floating base of the robot
        Q_f = [0 0 0 0 0 0 Q_new(i,1) Q_new(i,2) Q_new(i,3) Q_new(i,4) Q_new(i,5)]';
        dQ_f = [0 0 0 0 0 0 dQ_new(i,1) dQ_new(i,2) dQ_new(i,3) dQ_new(i,4) dQ_new(i,5)]';
        d2Q_f = [0 0 0 0 0 0 d2Q_new(i,1) d2Q_new(i,2) d2Q_new(i,3) d2Q_new(i,4) d2Q_new(i,5)]';

        %Computing floating base forward dynamics to set the 6D position,
        %velocity and acceleration of the floating base
        [X,v,a] = fbKin( Q_f, dQ_f, d2Q_f );

        %Computing floating base inverse dynamics to find the joint forces and
        %the acceleration of the floating base produced by those forces 
        [fbf,afb1,tau] = IDf( fb_BAT_Feat, X, v, Q_new(i,:), dQ_new(i,:), d2Q_new(i,:) );       

        Torques_feat_f(i,:) = tau; 
        base_accel(:,i) = afb1;
        base_force(:,i) = fbf;
    end

    if k == 1  %contribution of the right (R) wing
       Torque_RW = Torques_feat_f;
       base_accel_RW = Xrotx( -pi/2-alpha1)*base_accel;
       base_force_RW = Xrotx( -pi/2-alpha1)*base_force;

       %z-contribution, yaw is positive contribution
       base_force_RW(3,:) = -1*base_force_RW(3,:);
       base_accel_RW(3,:) = -1*base_accel_RW(3,:);
       
    else       %contribution of the left (L) wing (applying the proper orientation)
        Torque_LW = Torques_feat_f;
        base_accel_LW = Xrotz( pi )* (Xrotx( -pi/2-alpha1 )*base_accel);
        base_force_LW = Xrotz( pi )* (Xrotx( -pi/2-alpha1 )*base_force);

       %x-contribution, pitch is positive contribution
       base_force_LW(1,:) = -1*base_force_LW(1,:);
       base_accel_LW(1,:) = -1*base_accel_LW(1,:);
    end
end
%filling new time vector
[fil, col] = size(base_accel_RW); 
tf = times*(1/freq); 
step_2 = tf/(col-1);
Ti_new = 0:step_2:tf;

%**************************************************************************
% effects of both wing contributions projected onto the CENTER OF MASS of
% the floating base of Batbot
%**************************************************************************
if flag_fusion == 1 %(ON)
    accel_T = base_accel_RW + base_accel_LW;
    force_T = base_force_RW + base_force_LW;
else
    accel_T = base_accel_RW;
    force_T = base_force_RW;
end
%Integrating base accelerations to analyze CM motion
%EULER V(i+1)=V(i)*step*A(i) in order to %obtain velocities and positions
[dof, pt] = size(accel_T); 
vel_T(1:6,1) = 0; %Initial vel
for k=1:pt-1
     vel_T(:,k+1) = vel_T(:,k)+(accel_T(:,k)*step_2);
end

[dof, pt] = size(vel_T);

pos_T(1:6,1) = 0; %Initial Pos
for k=1:pt-1
     pos_T(:,k+1) = pos_T(:,k)+(vel_T(:,k)*step_2);    
end

%**************************************************************************
% Plotting variables of interest
%**************************************************************************
    figure(1), clf
    set(gcf,'DoubleBuffer','on')

    %Joint positions of the right and Left wings
    %subplot(8,2,15)
    subplot(3,2,1)
    hold on
    grid on;
    plot(Ti_new, QT(:,1:4)*180/pi, Ti_new, QT(:,6:9)*180/pi);
    ylabel('q[deg]')
    xlabel('t[s]')
    legend('q_{1,R}', 'q_{2,R}', 'q_{3,R}', 'q_{4-6,R}','q_{1,L}', 'q_{2,L}', 'q_{3,L}', 'q_{4-6,L}')
    Title('Wing joint positions')
    set(get(gca,'YLabel'),'Rotation',90);    
 
    %Joint torques of the right wing
    subplot(3,2,3)
    hold on
    grid on;
    plot(Ti_new, Torque_RW(:,1:4),Ti_new,Torque_LW(:,1:4) );
    ylabel('\tau[Nm]')
    xlabel('t[s]')
    legend('\tau_{1,R}', '\tau_{2,R}', '\tau_{3,R}', '\tau_{4-6,R}','\tau_{1,L}', '\tau_{2,L}', '\tau_{3,L}', '\tau_{4-6,L}')
    Title('Wing torques')
    set(get(gca,'YLabel'),'Rotation',90);     
        
    %%Base torques produced by each wings
    subplot(3,2,5)
    hold on
    grid on;
    plot(Ti_new, base_force_RW(1:3,:),Ti_new, base_force_LW(1:3,:));
    ylabel('\tau_{cm}[Nm]')
    xlabel('t[s]')
    legend('\tau_{x,R}', '\tau_{y,R}', '\tau_{z,R}','\tau_{x,L}', '\tau_{y,L}', '\tau_{z,L}')
    Title('body torques produced by each wing')
    set(get(gca,'YLabel'),'Rotation',90);     
     
    %Base_forces produced by both wings
    %subplot(8,2,15)
    subplot(3,2,2)
    hold on
    grid on;
    plot(Ti_new,force_T(1:6,:));
    ylabel('F')
    xlabel('t[s]')
    legend('F_x', 'F_y', 'F_z', '\tau_x','\tau_y','\tau_z')
    Title('Body 6D-forces produced by both wings')
    set(get(gca,'YLabel'),'Rotation',90);  
    
    %Base_accel produced by both wings
    %subplot(8,2,15)
    subplot(3,2,4)
    hold on
    grid on;
    plot(Ti_new,accel_T(1:3,:));
    ylabel('d^2(\theta,\phi,\psi)[rad/s^2]')
    xlabel('t[s]')
    legend('pitch', 'roll', 'yaw')
    Title('Body angular accelerations')
    set(get(gca,'YLabel'),'Rotation',90);  
    
    %Base_angular position.
    %subplot(8,2,15)
    subplot(3,2,6)
    hold on
    grid on;
    plot(Ti_new, pos_T(1:3,:));
    ylabel('Attitude}[deg]')
    xlabel('t[s]')
    legend('pitch', 'roll', 'yaw')
    Title('Body angular positions')
    set(get(gca,'YLabel'),'Rotation',90);  

%**************************************************************************
% Writting output structures
%**************************************************************************
 %Right wing
 Q_R_1_e = struct('values', [QT(:,1)*180/pi], 'dimensions', [1]);
 Q_R_1 = struct('time', [Ti_new], 'signals', Q_R_1_e);
 
 Q_R_2_e = struct('values', [QT(:,2)*180/pi], 'dimensions', [1]);
 Q_R_2 = struct('time', [Ti_new], 'signals', Q_R_2_e);
 
 Q_R_3_e = struct('values', [QT(:,3)*180/pi], 'dimensions', [1]);
 Q_R_3 = struct('time', [Ti_new], 'signals', Q_R_3_e);
 
 Q_R_4_e = struct('values', [QT(:,4)*180/pi], 'dimensions', [1]);
 Q_R_4 = struct('time', [Ti_new], 'signals', Q_R_4_e);
 
 
 %left wing
 Q_L_1_e = struct('values', [QT(:,7)*180/pi], 'dimensions', [1]);
 Q_L_1 = struct('time', [Ti_new], 'signals', Q_L_1_e); 
 
 Q_L_2_e = struct('values', [QT(:,8)*180/pi], 'dimensions', [1]);
 Q_L_2 = struct('time', [Ti_new], 'signals', Q_L_2_e); 

 Q_L_3_e = struct('values', [QT(:,9)*180/pi], 'dimensions', [1]);
 Q_L_3 = struct('time', [Ti_new], 'signals', Q_L_3_e); 

  %Center of mass motion 
 Pos_CM_pitch_e = struct('values', [pos_T(1,:)'], 'dimensions', [1]);
 Pos_CM_pitch =   struct('time', [Ti_new], 'signals', Pos_CM_pitch_e);

 Pos_CM_roll_e = struct('values', [pos_T(2,:)'], 'dimensions', [1]);
 Pos_CM_roll =   struct('time', [Ti_new], 'signals', Pos_CM_roll_e); 
 