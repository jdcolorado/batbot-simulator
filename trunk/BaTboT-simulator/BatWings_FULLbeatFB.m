%**************************************************************************
%BatBot: Biological inspired Bat roBot.

%Copyright RObotics and Cybernetics Group
%Julian Colorado

% Matlab simulator of bat flight behavior. 
%**************************************************************************

%BAT-wing trajectory Generator
%This program generates cartersian trajectories for
%both CARPUS and MCP-III wing-bones of the BAT (planar motion) within
%a wing-beat cycle: Downstroke and Upstroke with a sampled frecuency=3.03Hz

%**************************************************************************

%Entrance:
%cartesian control-points: [x1 y1 z1; xn yn zn] 
%vector time per point [t1 tn]                                 
%sampled time (Ts=1/F)
%End simulation Time (Tf)

%Output:
%traj    -> [X Y Z Vx Vy Vz Ax Ay Az] (forward kinematics: cartesian motion)

%Examples for Carpus and WingTip Downstroke
%Run to compute downstroke-beat for Carpus
%traj=splines3D([0.1 0 0.25; 0.18 0 0.22; 0.28 0 0.1; 0.3 0 -0.02; 0.29 0 -0.08],[1 2 3 4 5],0.001,1,0,0,0);
%Run to compute downstroke-beat for Wing-Tip
%traj=splines3D([0.35 0 0.2; 0.4 0 0.33; 0.5 0 0.2; 0.55 0 0.02; 0.48 0 -0.25],[1 2 3 4 5],0.001,1,0,0,0);

%Examples for Carpus and WingTip Upstroke
%Run to compute Upstroke-beat for Carpus
%traj=splines3D([0.3 0 -0.07; 0.28 0 0.09; 0.2 0 0.2; 0.1 0 0.25],[1 2 3 4],0.001,1,0,0,0);
%Run to compute Upstroke-beat for Wing-Tip
%traj=splines3D([0.45 0 -0.28; 0.25 0 -0.18; 0.22 0 -0.05; 0.35 0 0.15],[1 2 3 4],0.001,1,0,0,0);

%**************************************************************************

function traj = BatWings_FULLbeat(points,time,step_time,end_time)

[m,n]=size(time);
A=zeros(n,n);
B=zeros(n,3);
C=zeros(n,3);
h=zeros(n-1);

%defining time vector to eval polys after building
t=[0:step_time:end_time];
Npoints=(end_time/step_time)+1;
   
%obtaining vector h->(t(i+1)-t(i))
for i=1:n-1
    h(i)=time(i+1)-time(i);
end

%obtaining the matrix A (Time matrix)
for i=1:n
    for j=1:n       
        %filling lower diagonal
        if i-j==1
            if ((i==2 && j==1) || (i==3 && j==2) || (i==n && j==n-1))
                if (i==n && j==n-1)
                     A(i,j)=0;
                else
                     A(i,j)=h(1);
                end
            else
                A(i,j)=h(i-1);
            end
        end             
        %filling the diagonal
        if i==j
            if ((i==1 && j==1) || (i==n && j==n))
                A(i,j)=1;
            else
                A(i,j)=2*(h(i-1)+h(i));
            end
        end
        %filling upper diagonal
        if j-i==1
            if (i==1 && j==2)
                A(i,j)=0;
            else
                A(i,j)=h(i);
            end
        end

    end
end
        
%obtaining the vector Bx,By,Bz:
for i=1:n
    if (i==1 || i==n)
        B(i,1)=0;
        B(i,2)=0;
        B(i,3)=0;
    else
        B(i,1)=(3/h(i)*(points(i+1,1)-points(i,1)))-(3/h(i-1)*(points(i,1)-points(i-1,1)));
        B(i,2)=(3/h(i)*(points(i+1,2)-points(i,2)))-(3/h(i-1)*(points(i,2)-points(i-1,2)));
        B(i,3)=(3/h(i)*(points(i+1,3)-points(i,3)))-(3/h(i-1)*(points(i,3)-points(i-1,3)));
    end
end

%solving the system A.C=B, to obtain the coefficients C
C(:,1)=inv(A)*B(:,1);
C(:,2)=inv(A)*B(:,2);
C(:,3)=inv(A)*B(:,3);

%Computing trajectory 
for i=1:n-1
    %obtaining the coefficients b and d
    b(i,1)=1/h(i)*(points(i+1,1)-points(i,1))-h(i)/3*(2*C(i,1)+C(i+1,1));
    b(i,2)=1/h(i)*(points(i+1,2)-points(i,2))-h(i)/3*(2*C(i,2)+C(i+1,2));
    b(i,3)=1/h(i)*(points(i+1,3)-points(i,3))-h(i)/3*(2*C(i,3)+C(i+1,3));
    %**********************************************************************
    %****
    d(i,1)=(C(i+1,1)-C(i,1))/(3*h(i));
    d(i,2)=(C(i+1,2)-C(i,2))/(3*h(i));
    d(i,3)=(C(i+1,3)-C(i,3))/(3*h(i));
    %**********************************************************************
    %mixing coefficients to compute third grade poly S=a+bt+ct^2+dt^3, where a
    %corresponds to the f(x)->points[]
    %Positions
    polyx(i,:)=points(i,1)+b(i,1)*t+C(i,1)*t.^2+d(i,1)*t.^3;
    polyy(i,:)=points(i,2)+b(i,2)*t+C(i,2)*t.^2+d(i,2)*t.^3;
    polyz(i,:)=points(i,3)+b(i,3)*t+C(i,3)*t.^2+d(i,3)*t.^3;
    %Derivating Positions: Velocities
    polydx(i,:)=b(i,1)+2*C(i,1)*t+3*d(i,1)*t.^2;
    polydy(i,:)=b(i,2)+2*C(i,2)*t+3*d(i,2)*t.^2;
    polydz(i,:)=b(i,3)+2*C(i,3)*t+3*d(i,3)*t.^2;
    %Derivating velocities: Accelerations
    polyd2x(i,:)=2*C(i,1)+6*d(i,1)*t;
    polyd2y(i,:)=2*C(i,2)+6*d(i,2)*t;
    polyd2z(i,:)=2*C(i,3)+6*d(i,3)*t;    
    %************************************************************************** 
    %Return x-y-z trajectory of the whole spline3D with the respective
    %Time: [x y z Time]
    tempX=[polyx(i,:)' polyy(i,:)' polyz(i,:)' t'+(i-1)];   
    tempV=[polydx(i,:)' polydy(i,:)' polydz(i,:)' t'+(i-1)];
    tempA=[polyd2x(i,:)' polyd2y(i,:)' polyd2z(i,:)' t'+(i-1)];
    X((((i-1)*Npoints)+1):i*Npoints,1:4)=tempX;
    V((((i-1)*Npoints)+1):i*Npoints,1:4)=tempV;
    Ac((((i-1)*Npoints)+1):i*Npoints,1:4)=tempA;
end
%**************************************************************************


%BAT Carpus and MCP-III trajectory profile
[pt co]=size(X);
for i=1:pt-1
    traj(i,1)=X(i,1);
    traj(i,2)=X(i,2);
    traj(i,3)=X(i,3);
    traj(i,4)=V(i,1);
    traj(i,5)=V(i,2);
    traj(i,6)=V(i,3);
    traj(i,7)=Ac(i,1);
    traj(i,8)=Ac(i,2);
    traj(i,9)=Ac(i,3);
end  

%Three-dimensional plotting of the curve
%   plot3(polyx(1,:),polyy(1,:),polyz(1,:),...
%         polyx(2,:),polyy(2,:),polyz(2,:),...
%         polyx(3,:),polyy(3,:),polyz(3,:),...
%         polyx(4,:),polyy(4,:),polyz(4,:));


    
end

%**************************************************************************


       
