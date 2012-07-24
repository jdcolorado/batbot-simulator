%**************************************************************************
%BatBot: Biological inspired Bat roBot.

%Copyright RObotics and Cybernetics Group
%Julian Colorado

% Matlab simulator of bat flight behavior. 
%**************************************************************************

%This program compute de forward kinematics using Denavit-hartenberg
%parameters DH and a set joint position Q for just 1 DoF
%**************************************************************************
function [R] = fkine_1_DoF(teta, dof)

 if dof ==1
    alfa = 0;
 end
  if dof ==2
    alfa = 0;
  end
  if dof ==3
    alfa = 0;
  end
  if dof ==4
    alfa = 0;
  end
  if dof ==5
    alfa = 0;
  end
 
 R=[cos(teta)    -cos(alfa)*sin(teta)    sin(alfa)*sin(teta)...        
      sin(teta)    cos(alfa)*cos(teta)    -sin(alfa)*cos(teta)...
         0               sin(alfa)                cos(alfa)]';
     
  end