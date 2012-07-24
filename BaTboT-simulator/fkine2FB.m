%**************************************************************************
%BatBot: Biological inspired Bat roBot.

%Copyright RObotics and Cybernetics Group
%Julian Colorado

% Matlab simulator of bat flight behavior. 
%**************************************************************************

%This program compute de forward kinematics using Denavit-hartenberg
%parameters DH and a set joint position Q
%**************************************************************************

function T=fkine2(DH,q)

[n,m]=size(DH);   

               
  T=eye(4);                 
  
  for i=1:n
    a=DH(i,2);          %obtain the distance a 
    alfa=DH(i,1);       %Obtain alpha angle
    
    %Evaluate for Rotational or prismatic joint
    if(DH(i,5)==0)     %Rotational
        teta=q(i);   
        d=DH(i,4);      
    else
        teta=DH(i,3);   %Prismatic
        d=q(i);      
    end
  
  %Compute homogeuneus transformation matriz  
  TR=[cos(teta)    -cos(alfa)*sin(teta)    sin(alfa)*sin(teta)      a*cos(teta);        
      sin(teta)    cos(alfa)*cos(teta)    -sin(alfa)*cos(teta)      a*sin(teta);
         0               sin(alfa)                cos(alfa)             d;
         0               0                           0                  1 ];
    
  T=T*TR;
  end


end