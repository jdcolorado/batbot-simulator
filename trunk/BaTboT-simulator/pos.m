function [p] = pos(teta, dof)
if dof ==1
    a = 0;
 end
  if dof ==2
    a = 0.035; 
  end
  if dof ==3
    a = 0.055;
  end
  if dof ==4
    a = 0.07;
  end
  if dof ==5
    a = 0.11;
  end
  p=[a*cos(teta)        a*sin(teta)                 0]';
     

end
