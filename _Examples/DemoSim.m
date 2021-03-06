% DEMOSIM Demo script for the simulation and alignement of point clouds.
%
% In particular :
% - the creation of a mesh (point cloud) based on a geometrical shape (target), 
% - copy the point cloud and add noise it, 
% - rotate the copy and try to align the copy (source) with the target.
% - try out some smoothing.
% - view the results
%
%AUTHOR  : Neal Gauvin (at a3 dot epfl dot ch)
%VERSION : 2.16
%STATUS  : Ok
%DATE    : 25 mai 2011

clear; clear global;
% Define a path to your data...
%Data = 'F:\ngauvin\Matlab\Data\'; %A Windows path.
%Data = '/Data/ngauvin/Matlab/Data/';  %A Linux path.


%% Create a point cloud (3xn matrix of n 3D points)
% Have a look in the ShapeSimulator repertory.
% from a plan with a pyramid in the middle
%Target = GenPlanePyramid(); 
% or a cornet de frites
%Target = GenCornetdeFrites(1000)
% or two planes with a V-shape
Target = GenValley(1000); 
% a simple plane
%Target = GenPlane('Arg',[5,5,1,1]); 
% a terrain
% Target = GenTerrain('Size',25);
% waves
%Target = GenWaves('W',[100 20]);
%Target = GenWaves('W',[100 20],'Peaks',10); % with sublying peaks or sphere
% Give it a name, useful for plotting...
Target.Name = 'Target Point Cloud';


%% Here you may choose the way your point cloud will be rotated.
% Note : this choice has large impact on some ICP options (see below).
% 0 : wrst point (0,0,0).
% 1 : wrst mass center of the point cloud.
% [x,y,z] : wrst to the given point.
% Target.TMode = 0;


%% You may want to create a subsample of your original point cloud.
% Note : for simulated PointClouds, you better do it before copying the
% PointCloud if you are interested into comparing the true positions (they
% need to be the same).
%[Target,~] = SubSampling(Target,'Type','UseAll');
%[Target,~] = SubSampling(Target,'Type','Auto','Arg',{100});
%[Target,~] = SubSampling(Target,'Type','Uniform','Arg',{10} );
%[Target,~] = SubSampling(Target,'Type','Random','Arg',{0.1} );
%[Target,~] = SubSampling(Target,'Type','Normal','Arg',{100,3});
%[Target,~] = SubSampling(Target,'Type','Curv','Arg',{-1000,12});
%[Target,~] = SubSampling(Target,'Type','Box','Arg',{[5,5,inf],5});
% Perform a smoothing of your point cloud using GridFit (sub and over sampling !)
%[Target,~] = SubSampling(PC,'Type','GridFit','Arg',{1000, {'Smoothness',1}});
% You also have the possibility to perform a color or intensity filtering.


%% Copy the Target, translate and rotate it
% Translation
%GenTM = TransformMatrix([],[0,0,10]);                                     %#ok<*NOPTS>
% Rotation + Translation
GenTM = TransformMatrix([pi/12,0,0],[0,0,50]);                             %#ok<*NOPTS>
% Rotation + Translation + resize. Shear is also possible.
%GenTM = TransformMatrix([0,0,0],[0,0,50],[2 2 2]);                        %#ok<*NOPTS>
Source_=GenValley(600);
Source = AffinTransform(Source_, GenTM);
% Give it a name, useful for plotting...
Source.Name = 'Source Point Cloud'; 


%% Add some noise to both of them
Source.addNoise('OutlierProb',0.01, 'GaussSmear', [0.01 1],'DropOutProb', 0 );
Target.addNoise('OutlierProb',0.01, 'GaussSmear', [0.01 1],'DropOutProb', 0 );
% Alignement is done only on the measured (or simulated) positions.
% If you wish to perform the alignment on the true positions, copy the true
% positions to the measured position location :
%Source.copyTrue2MeasPos();
%Target.copyTrue2MeasPos();


%% Maybe you just want to load existing PointCLouds...
% See below how to save any created Matlab object.
% Matlab object
%load([Data 'GenPlanePyramid_wNoiseNorm.mat']); %A PointCloud called Target
%load([Data 'GenPlanePyramid_wNoiseNorm_T.mat']); %A PointCloud called Source
% Create a PointCloud object from given ASCII file.
%Source = ImportPointCloudFromASCII([Data 'test.txt']);


%% To compute the full set of (true) distances (see ICPVarOut.Dist(4:5)), 
% you need to precise the link between the two PointClouds.
% By applying those indices to the source, its true positions must become 
% exactly the same as the Target's ones, modulo the spatial transformation.
% In our example, they're just a copy of each other. It's trivial.
% Source.TrueM = 1:size(Target.P,2);
% Note that in this trivial case, you do not need to specify it. It will be
% assumed by default.


%% Visualise the clouds (true positions)
Plot3DPointClouds(Target, Source, 'TruePos1', true, 'TruePos2', true);
% or the simulated positions
% Plot3DPointClouds(Target, Source);
% This is a quick and easy plot tool. For a more elaborated one, check
% PlotMultiPointClouds.


%% Create a object to store the output of the ICP.
Output = ICPVarOut();


%% You may want to perform a first alignment. 
% For instance, move the source cloud to the center of gravity of the target. 
% Think about using quaternions for systematic intial translations and rotations.
%Source.MoveToCM(Target, Output);


%% Keep a copy of source before alignment
% Note the way it is done :
% SourceIni = PointCloud('Source before alignment',Source);
% This creates a new independant point cloud with the properties.
% SourceIni = Source
% Would just copy the pointer (handle). Modifying Source would modify
% SourceIni.

%% Perform the alignment with the ICP algorithm.
% Note that setting Source.TMode = 1 might help the convergence for lma, nm
% and bfgs minimisations. Do not use it for the other options.
ICParg = {'Verbose',true,'TruePos', true,'Tol',0.01,'TimeLimit',10,'Matching', 'kDtree','Minimise',{'plane',1}};
% A few options to set :
% ,'RejectNWorstPairs',0.1
% ,'RejectDistPairs',20
% ,'RejectRMSPairs',2
% ,'RejectNormals', pi/4
% ,'RejectIsolated', 0.1
% ,'Weight','dist'
% ,'Weight','normal'
% ,'Weight','curv'
% ,'Extrapolation',true
% ,'Matching', 'kDtree' || 'GLtree'
% ,'Minimise',{'plane',1}
% ,'Minimise',{'lma',1}
% ,'Minimise',{'nm',1}
% 'Minimise',{'bfgs',2}
% ,'Tol',-1,
% 'NbIterMax', 10
% doc ICP for extensive documentation !
% ICP(Target,Source,Output,ICParg{:});


%% Perform the alignment with multi-resolutions.
% Get your own feeling by trying various level of smoothing.
% Type doc MultiResoICP for extensive explanations.
% Try GenWaves with peaks or sphere, set with lower frequency than the waves ones.
% ,'Type', 'GridFit'} || 'Box'
% ,'SmoothOpt', {'Smoothness',1}
% ,'Verbose', 0 || 1 || 2
MultiResoICP(Target,Source,Output,'Ratio', {10 5 0}, 'ICPOpt',ICParg, 'Verbose', 2,'Type', 'GridFit','SmoothOpt', {'Smoothness',1})


%% Test that RecTM is really the inverse of GenTM
% Retrieve the last total transformation
% RecTM = Output.LastTM                                                      %#ok<NOPTS>
% Looks quite different thant GenTM, he ?
% Aligned = AffinTransform(SourceIni,RecTM);
% Aligned.Name = 'Aligned Source';
% Plot3DPointClouds(Target,Aligned);
% Ok, well. They're the same !


%% Visualise the clouds after alignment
%Plot3DPointClouds(Target, Source, 'TruePos1', true, 'TruePos2', true);
Plot3DPointClouds(Target, Source);


%% Display the mean distances between the coincident points and CPU with respect to the nb of iterations.
% Check the numerous plot options in ICPVarOut and in its member fonctions.
Output.PlotDist;
Output.PlotCPU;


%% You may want to save the PointCloud objects (with related computed variables, such as the normals or a GLtree).
% Good if you want reproduceable results.
% Save it before the alignment is processed, if for alignment purpose !
% save([Data 'GenPlanePyramid_wNoiseNorm'],'Target')
% save([Data 'GenPlanePyramid_wNoiseNorm_T'],'Source')


%% Let's play a little bit with GridFit directly.
% With GridFit you can estimate the z coordinate on a x,y grid for data
% that can be mapped as z = f(x,y). This alloes you to perform some
% smoothing, sub and over sampling.
% doc GridFit !

% First fix some control points. Nb of points in x,y or vectors of points.
% Spacing can be variable.
xc = 50; yc = 50;

% Now let's turn the scanned point data into a surface
% Some options :
% ,'Smoothness',1
% ,'interp','bilinear','triangle','nearest'
% ,'regularizer', 'diffusion','gradient','springs'
% ,'solver','normal','lsqr','symmlq'
% ,'extend','warning','never','always'
% ,'tilesize', 10
% ,'overlap',0.25
% ,'TrueP',true
% There's different ways to access the very same technology.
% MPC = GridFit(Target,xc,yc,'Smoothness',1); % directly
% Or via the PointCloud class.
MPC = Target.MeshPointCloud(xc,yc,'Smoothness',10);
MPC.Name = 'Target Smoothed';
% You are left with a MeshPointCloud object. Try doc MeshPointCloud !
% Note that (MultiResol)ICP accepts MeshPointCloud objects as well.

%% Display the result
% You can call directly MPC to plot itself
%MPC.surf; % or with the contour MPC.surfc;
% Or plot the surface mesh with the original PointCloud
myopt = {
    {'MarkerSize',2,'Marker','*','LineStyle','none','Color',[0.75 0 0.75]},...
    {'FaceLighting','phong','MarkerSize',2,'Marker','*','LineStyle','none','FaceColor','interp','EdgeColor','none'}
    };       
PlotMultiPointClouds({Target,MPC},'Title','A Point Cloud with its Smoothed Surface','PlotOpt',myopt);
% Some stuff to call for a nice surf
%colormap(hot(256));
%colormap(jet(256));
%colormap(hsv(256));
%camlight right;
%lighting phong;
%shading interp

% Note that you can turn your MeshPointCloud into a PointCloud, by calling
% PC = MPC.PointCloud;


%% Good to know...
% Compute and plot the normals
% Target.ComputeNormals;
% Target.PlotNormals; 
% The curvatures
% Target.ComputeCurvature;
% Target.PlotCurvature;
% If you have some Colors or Intensities ...
% Target.PlotPositionsWithColors;
% Target.PlotPositionsWithIntensities;


