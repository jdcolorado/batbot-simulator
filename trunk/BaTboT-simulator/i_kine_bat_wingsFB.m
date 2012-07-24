%**************************************************************************
%BatBot: Biological inspired Bat roBot.

%Copyright RObotics and Cybernetics Group
%Julian Colorado

% Matlab simulator of bat flight behavior. 
%**************************************************************************


%BAT-wing inverse kinematics computation
%This program generates angular trajectories for
%both CARPUS and MCP-III wing-bones joints of the BAT (planar motion) within
%a wing-beat cycle: Downstroke and Upstroke with a sampled frecuency=3.03Hz

%**************************************************************************

function  [Q_C, Q_MPCIII, dQ_C, dQ_MPCIII,d2Q_C, d2Q_MPCIII, step] = i_kine_bat_wingsFB(traj_carpus,traj_MCPIII,freq,flag,Flag_test)

    [f, c] = size(traj_carpus);
    sampled_time = 1/freq;
    Time = [0:(sampled_time/f):sampled_time];
    step = Time(2)-Time(1);

    
    for i=1:f
        Q(i,1) = atan2(traj_carpus(i,flag),traj_carpus(i,1));       
        Q(i,2) = atan2((traj_MCPIII(i,flag)-traj_carpus(i,flag)),(traj_MCPIII(i,1)-traj_carpus(i,1)))-Q(i,1);   
        Q(i,3) = Time(i);   
    end
    

    %1st filter: Fixing data continuity 
    cont = 1;
    i = 1;
    while i <= f-1       
        if Q(i+1,1)-Q(i,1) > 0 
           Qc(cont) = Q(i,1);
           cont = cont+1;
        else
           Qc(cont) = Q(i,1);
           cont = cont+1;
           i = i+1;
        end
        i = i+1;  
    end
    
    %2nd filter
    cont = 1;
    i = 1;
    while i <= f-1       
        if Q(i+1,2)-Q(i,2) < 0 
           QM(cont) = Q(i,2);
           cont = cont+1;
        else
           QM(cont) = Q(i,2);
           cont = cont+1;
           i = i+1;
        end
        i = i+1;       
    end
    
       
    %Fixing Data sizes
    a = length(Qc); 
    b = length(QM);
    if a<b
        c = a;
    else
        c = b;
    end
    
    
        
    %scale factor for adjustments
    %**********************************************************************
    %Biological Specimen:
    if Flag_test ==1
        fact = 1; 
    end
    %Bat-robot (Simulation)
    if Flag_test ==2
        fact = 0.5; 
    end
    % % %Bat-robot: V3 (ROBOT)
    if Flag_test ==3
        fact = 0.38; 
    end
    %**********************************************************************
    Qc = Qc.*fact;
    QM = QM.*fact;
    
    
    %Differentiation to obtain joint velocities
    for k=1:c-1
        dQc(k)  = (Qc(k+1)-Qc(k))/step; 
        dQM(k)  = (QM(k+1)-QM(k))/step;  
    end

    
    %Filtering (determine which is higher than the 30% of increasing)
    c1 = length(dQc);    
    cont = 1;
    i = 1;
    while i <= c1-1              
        if abs(dQc(i+1)-dQc(i)) > abs(dQc(i)*0.3)
           dQc2(cont) = dQc(i);
           cont = cont+1;
           i = i+1;
        else
           dQc2(cont) = dQc(i);
           cont = cont+1;
        end
        i = i+1;  
    end
           

    %Filtering (determine which is higher than the 30% of increasing)
    c1 = length(dQM);    
    cont = 1;
    i = 1;
    while i <= c1-1              
        if abs(dQM(i+1)-dQM(i)) > abs(dQM(i)*0.3)
           dQM2(cont) = dQM(i);
           cont = cont+1;
           i = i+1;
        else
           dQM2(cont) = dQM(i);
           cont = cont+1;
        end
        i = i+1;  
    end

    
    c = length(dQM2);    
    %Differentiation to obtain joint accelerations
    for k=1:c-1
        d2Qc(k) = (dQc2(k+1)-dQc2(k))/step; 
        d2QM(k) = (dQM2(k+1)-dQM2(k))/step;  
    end

    
    c1 = length(d2Qc);
    Q_carpus(:,1) = Qc(1:c1);
    Q_carpus(:,2) = dQc2(1:c1);
    Q_carpus(:,3) = d2Qc*0.1;
    
    
    [f, c2] = size(d2QM);
    Q_MCP(:,1) = QM(1:c1);
    Q_MCP(:,2) = dQM2(1:c1);
    Q_MCP(:,3) = d2QM*0.1;
     
    Time1(:,1) = Time(1:c1);
    Time2(:,1) = Time(1:c1);
          

    %Creating structures for angular data
    
    % Strcuture for angular positions
    Pos_signals_C = struct('values', [Q_carpus(:,1)], 'dimensions', [1]);
    Pos_signals_MPIII = struct('values', [Q_MCP(:,1)], 'dimensions', [1]);
    
    Q_C = struct('time', [Time1], 'signals', Pos_signals_C);
    Q_MPCIII = struct('time', [Time2], 'signals', Pos_signals_MPIII);
    
    % Strcuture for angular velocities
    Vel_signals_C = struct('values', [Q_carpus(:,2)], 'dimensions', [1]);
    Vel_signals_MPIII = struct('values', [Q_MCP(:,2)], 'dimensions', [1]);
    
    dQ_C = struct('time', [Time1], 'signals', Vel_signals_C);
    dQ_MPCIII = struct('time', [Time2], 'signals', Vel_signals_MPIII);
    
    % Strcuture for angular Acelerations
    Acel_signals_C = struct('values', [Q_carpus(:,3)], 'dimensions', [1]);
    Acel_signals_MPIII = struct('values', [Q_MCP(:,3)], 'dimensions', [1]);
    
    d2Q_C = struct('time', [Time1], 'signals', Acel_signals_C);
    d2Q_MPCIII = struct('time', [Time2], 'signals', Acel_signals_MPIII);
    
 
end

  