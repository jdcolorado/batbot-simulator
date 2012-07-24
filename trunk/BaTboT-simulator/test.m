
clear L;
% creating a single body structure with 1 DOF with fixed base
L{1} = link([0 0 0 0 0], 'modified');
L{1}.m = 5;
L{1}.r = [0 0 0.05];
L{1}.I = [0.00833 0 0;0 0.00833 0;0 0 0.00833];

Model_Feat.Xtree{1} = Xtrans([0 0 0]);
Icm = diag([0.00833,0.00833,0.00833]);
Model_Feat.I{1} = mcI( 5, [0,0,0.05], Icm );

Model_Corke=robot(L);

%Creating the multibody system according to Roy Featherstone Toolbox [2]
Model_Feat.NB = Model_Corke.n;
Model_Feat.n = Model_Corke.n;
Model_Feat.pitch = zeros(1,Model_Feat.NB);
Model_Feat.parent(1)=0;