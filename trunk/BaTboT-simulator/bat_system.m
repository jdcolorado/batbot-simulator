
function [fb_BAT_Feat, freq] = bat_system()

%Denavit&Hartenberg parameters
  %      alpha   a    teta        d  rota        m      sx      sy     sz      Ixx      Iyy        Izz    Ixy   Ixz   Iyz fric
%Fixed base  
L0=link([0       0     Q(1,1)     0    0        m_b      0        0     0       i_b_x    i_b_y      i_b_z    0     0     0  0],'modified');   

%right wing skeleton
L1=link([pi/2   0.035  Q(1,2)     0    0        m_s      0        0     0       i_s_x    i_s_y      i_s_z    0     0     0  0],'modified');  
L2=link([-pi/2   l1    Q(1,3)     0    0        m1      l1/2      0     0         0       0          i1      0     0     0  0],'modified');  
L3=link([0       l2    Q(1,4)     0    0        m2      l2/2      0     0         0       0          i2      0     0     0  0],'modified');  
L4=link([0       Ltip  Q(1,5)     0    0       m_dig   Ltip/2     0     0         0       0          i3      0     0     0  0],'modified');  


BAT_corke=robot({L0 L1 L2 L3 L4}); 
BAT_corke.name='BAT_corke';
n=BAT_corke.n;

Links = BAT_corke.link;

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


%transforming fixed-based bat model to floating base 
fb_BAT_Feat = floatbase( BAT_Feat );

%%flapping frequency
freq = 5;

end