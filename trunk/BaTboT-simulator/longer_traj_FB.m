%**************************************************************************
%BatBot: Biological inspired Bat roBot.

%Copyright RObotics and Cybernetics Group
%Julian Colorado

% Matlab simulator of bat flight behavior. 
%**************************************************************************

function  [Q_new, dQ_new, d2Q_new] = longer_traj_FB(times, flag_maneuover, Q, dQ, d2Q)


 %definining different tests profiles regarding asymmetries on bat wing's
 %motion
 len_q = length(Q(:,1));
 cont_q = 1;
 cont2 = 1;
 switch(flag_maneuover) %1:(symmetrical) 2:wing expanded,  3: wing conntracted  4: starts symmetrical + fully expanded + fully contracted
    
     case 1
                        
                while (cont2 <= times)
                    Q_new(cont_q:cont2*len_q,:) = Q(1:len_q,:);
                    dQ_new(cont_q:cont2*len_q,:) = dQ(1:len_q,:);
                    d2Q_new(cont_q:cont2*len_q,:) = d2Q(1:len_q,:);
                    
                    cont_q = cont_q+len_q;    
                    cont2  = cont2+1;
                end
                       
     case 2

            elbow_fact = pi/2; 
            wrist_fact = -pi/2;
                while (cont2 <= times)
                    Q_new(cont_q:cont2*len_q,1) = Q(1:len_q,1); 
                    Q_new(cont_q:cont2*len_q,2) = Q(1:len_q,2); 
                    Q_new(cont_q:cont2*len_q,3) = elbow_fact; 
                    Q_new(cont_q:cont2*len_q,4) = wrist_fact; 
                    Q_new(cont_q:cont2*len_q,5) = Q(1:len_q,5);
                    
                    dQ_new(cont_q:cont2*len_q,1) = dQ(1:len_q,1); 
                    dQ_new(cont_q:cont2*len_q,2) = dQ(1:len_q,2); 
                    dQ_new(cont_q:cont2*len_q,3) = 0; 
                    dQ_new(cont_q:cont2*len_q,4) = 0; 
                    dQ_new(cont_q:cont2*len_q,5) = dQ(1:len_q,5);
                    
                    d2Q_new(cont_q:cont2*len_q,1) = d2Q(1:len_q,1); 
                    d2Q_new(cont_q:cont2*len_q,2) = d2Q(1:len_q,2); 
                    d2Q_new(cont_q:cont2*len_q,3) = 0; 
                    d2Q_new(cont_q:cont2*len_q,4) = 0; 
                    d2Q_new(cont_q:cont2*len_q,5) = d2Q(1:len_q,5);                    
                    
                    cont_q = cont_q+len_q;    
                    cont2  = cont2+1;
                end
                        
     case 3

            elbow_fact = 1.0471;
            wrist_fact = -1.0471;
            
            while (cont2 <= times)
                    Q_new(cont_q:cont2*len_q,1) = Q(1:len_q,1); 
                    Q_new(cont_q:cont2*len_q,2) = Q(1:len_q,2); 
                    Q_new(cont_q:cont2*len_q,3) = elbow_fact; 
                    Q_new(cont_q:cont2*len_q,4) = wrist_fact; 
                    Q_new(cont_q:cont2*len_q,5) = Q(1:len_q,5);
                    
                    dQ_new(cont_q:cont2*len_q,1) = dQ(1:len_q,1); 
                    dQ_new(cont_q:cont2*len_q,2) = dQ(1:len_q,2); 
                    dQ_new(cont_q:cont2*len_q,3) = 0; 
                    dQ_new(cont_q:cont2*len_q,4) = 0; 
                    dQ_new(cont_q:cont2*len_q,5) = dQ(1:len_q,5);
                    
                    d2Q_new(cont_q:cont2*len_q,1) = d2Q(1:len_q,1); 
                    d2Q_new(cont_q:cont2*len_q,2) = d2Q(1:len_q,2); 
                    d2Q_new(cont_q:cont2*len_q,3) = 0; 
                    d2Q_new(cont_q:cont2*len_q,4) = 0; 
                    d2Q_new(cont_q:cont2*len_q,5) = d2Q(1:len_q,5);                    
                    
                    cont_q = cont_q+len_q;    
                    cont2  = cont2+1;
                end
            
     case 4
                
                while (cont2 <= times)
                    
                    if cont2 <= times/3
                        Q_new(cont_q:cont2*len_q,:) = Q(1:len_q,:);
                        dQ_new(cont_q:cont2*len_q,:) = dQ(1:len_q,:);
                        d2Q_new(cont_q:cont2*len_q,:) = d2Q(1:len_q,:);
                    end
                    
                    if cont2 > times/3 && cont2 <= times/2
                        fact_reduc = 0.55; %reduction factor of morphing motion
                        Q_new(cont_q:cont2*len_q,1) = Q(1:len_q,1); 
                        Q_new(cont_q:cont2*len_q,2) = Q(1:len_q,2); 
                        Q_new(cont_q:cont2*len_q,3) = Q(1:len_q,3)*fact_reduc; 
                        Q_new(cont_q:cont2*len_q,4) = Q(1:len_q,4)*fact_reduc;  
                        Q_new(cont_q:cont2*len_q,5) = Q(1:len_q,5);

                        dQ_new(cont_q:cont2*len_q,1) = dQ(1:len_q,1); 
                        dQ_new(cont_q:cont2*len_q,2) = dQ(1:len_q,2); 
                        dQ_new(cont_q:cont2*len_q,3) = dQ(1:len_q,3)*fact_reduc;  
                        dQ_new(cont_q:cont2*len_q,4) = dQ(1:len_q,4)*fact_reduc; 
                        dQ_new(cont_q:cont2*len_q,5) = dQ(1:len_q,5);

                        d2Q_new(cont_q:cont2*len_q,1) = d2Q(1:len_q,1); 
                        d2Q_new(cont_q:cont2*len_q,2) = d2Q(1:len_q,2); 
                        d2Q_new(cont_q:cont2*len_q,3) = d2Q(1:len_q,3)*fact_reduc;
                        d2Q_new(cont_q:cont2*len_q,4) = d2Q(1:len_q,4)*fact_reduc; 
                        d2Q_new(cont_q:cont2*len_q,5) = d2Q(1:len_q,5);  
                    end
                    
                    if cont2 > times/2 
                        fact_reduc = 0.15; %reduction factor of morphing motion
                        Q_new(cont_q:cont2*len_q,1) = Q(1:len_q,1); 
                        Q_new(cont_q:cont2*len_q,2) = Q(1:len_q,2); 
                        Q_new(cont_q:cont2*len_q,3) = Q(1:len_q,3)*fact_reduc; 
                        Q_new(cont_q:cont2*len_q,4) = Q(1:len_q,4)*fact_reduc;  
                        Q_new(cont_q:cont2*len_q,5) = Q(1:len_q,5);

                        dQ_new(cont_q:cont2*len_q,1) = dQ(1:len_q,1); 
                        dQ_new(cont_q:cont2*len_q,2) = dQ(1:len_q,2); 
                        dQ_new(cont_q:cont2*len_q,3) = dQ(1:len_q,3)*fact_reduc;  
                        dQ_new(cont_q:cont2*len_q,4) = dQ(1:len_q,4)*fact_reduc; 
                        dQ_new(cont_q:cont2*len_q,5) = dQ(1:len_q,5);

                        d2Q_new(cont_q:cont2*len_q,1) = d2Q(1:len_q,1); 
                        d2Q_new(cont_q:cont2*len_q,2) = d2Q(1:len_q,2); 
                        d2Q_new(cont_q:cont2*len_q,3) = d2Q(1:len_q,3)*fact_reduc;
                        d2Q_new(cont_q:cont2*len_q,4) = d2Q(1:len_q,4)*fact_reduc; 
                        d2Q_new(cont_q:cont2*len_q,5) = d2Q(1:len_q,5);  
                    end
                    
                                        
                    cont_q = cont_q+len_q;    
                    cont2  = cont2+1;
                end
            
 end
  
end






