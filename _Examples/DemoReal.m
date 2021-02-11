% DEMOREAL Demo script to test the alignement of point clouds.
%
% In particular :
% - load some real point clouds from Lidar measurement, 
% - perform the registration of the point clouds,
% - view the results
%
% Note   : No data is provided yet. This demo gives you a hint to develop
%   your own script on your own data.
%   Have also a look on DemoSim to play with the various options.
%   They are complementary.
%
%AUTHOR  : Neal Gauvin (at a3 dot epfl dot ch)
%VERSION : 1.2
%STATUS  : OK
%DATE    : 7 july 2011

clear; clear global;
% Define a path to your data...
%Data = 'F:\ngauvin\Matlab\Data\'; %A Windows path.
%Data = '/Data/ngauvin/Matlab/Data/';  %A Linux path.


%% Load existing data in format txt.
% Beware : this may be a lengthy process for nb points O(1M)...
% Advise : load the point cloud as described here, compute all relevant
% data, like the normals (PC{1}.ComputeNormals), and the save the object as
% a binary file (see below). Finally, load the binary object.
pc1 = ImportPointCloudFromASCII([Data 'scan1.txt'],'Scan 1',{'P','Colors'});
pc2 = ImportPointCloudFromASCII([Data 'scan2.txt'],'Scan 2',{'P','Colors'});


%% Maybe you just want to load existing PointCloud objects...
%load([Data 'scan1.mat']); %A PointCloud called pc1
%load([Data 'scan2.mat']); %A PointCloud called pc2

%% Have a quick look at the points... There's a bunch of them !
PlotMultiPointClouds({pc1 pc2});
% Don't even try to display the clouds with their associated colors when > O(1000)) points. 
% Your computer may get stuck. You need to subsample before visualising.

%% These PointClouds are really heavy... maybe you want to reduce them
% Advise : keep a high precision on the target, and sample the source (to be aligned).
% Some option possible (type doc SubSampling for more info):
% ,'Type','UseAll'
% ,'Type','Auto','Arg',{1000}
% ,'Type','Uniform','Arg',{100}
% ,'Type','Random','Arg',{0.01}
% ,'Type','Normal','Arg',{1000,3}
% ,'Type','Curv','Arg',{-1000,12}
% ,'Type','Box','Arg',{[5,5,inf],5};
% Perform a smoothing of your point cloud using GridFit (sub and over sampling !)
% ,'Type','GridFit','Arg',{1000, {'Smoothness',1}};
% You also have the possibility to perform a color or intensity filtering.
% ,'Type','Color','Arg',{[100 50 85],[255 200 95]}
pcs1 = pc1;
% keep a wide range of curvature : good way to keep special features.
[pcs2,~] = SubSampling(pc2,'Type','Curv','Arg',{-1000,12});

%% Another way to subsample is to select desired points manually.
% Plot the point cloud and select the desired area with the Matlab data selection tool (brush).
% Right-click on the selected area and choose "Create variable". Give it a
% name, for instant tmp. Then create a new PointCloud out of tmp :
% rs1 = PointCloud('Rock 1',tmp);
% However, you do not have the colors, present in the original point cloud.
% You need to get them back.
% rs1.GetMissingPropFromPC(pc1);


%% Introduce some artificial translation or apply some initial transformations.
%TM = TransformMatrix([],[0,0,10])
%pcs2.transform(TM);


%% Visualise the clouds before alignment. You may pass some plot options.
myopt = {
    %{'MarkerSize',2,'Marker','*','LineStyle','none','Color',[0.75 0 0.75]},...
    %{'MarkerSize',1,'Marker','.','LineStyle','none','Color',[0.75 0 0.75]},...
    {'MarkerSize',4,'Marker','+','LineStyle','none','Color',[0 0.5 0]},...
    {'MarkerSize',4,'Marker','x','LineStyle','none','Color',[0 0 1]}
    };
PlotMultiPointClouds({pcs1,pcs2},'Title','Before Alignment','PlotOpt',myopt);


%% Create a object to store the output of the ICP.
Output = ICPVarOut();


%% Perform the alignment with the ICP algorithm.
% We suppose they are already pre-aligned.
% Check out all the options to feed ICP by typing doc ICP
ICParg = {'Verbose',true,'TimeLimit',10,'RejectIsolated', 0.1 };
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
% ,'Tol',-1,'NbIterMax', 10
% doc ICP for extensive documentation !
ICP(pcs1,pcs2,Output,'Matching','kDtree',ICParg{:});

%% Perform the alignment with multi-resolutions.
% and to test smoothing, see DemoSim.

%% Display the mean distances between the coincident points and CPU with respect to the nb of iterations.
% Check the numerous plot options in ICPVarOut and in its member fonctions.
Output.PlotDist('Log',false);
Output.PlotCPU;


%% Visualise the clouds after alignment
PlotMultiPointClouds({pcs1,pcs2},'Title','After Alignment','PlotOpt',myopt);

%% You may want to save the PointCloud objects (with related computed variables, such as the normals or a GLtree).
% save([Data 'scan1'],'pc1');
% save([Data 'scan2'],'pc2');

%% Good to know...
% Compute and plot the normals
% Target.ComputeNormals;
% Target.PlotNormals; 
% The curvatures
% Target.ComputeCurvature;
% Target.PlotCurvature;
% On real data, think about increasing the number of neighbours used to
% compute the normals/curvatures, if you want to be less sensitive to local
% fluctuations.
% If you have some Colors or Intensities ...
% Target.PlotPositionsWithColors;
% Target.PlotPositionsWithIntensities;

% Note that you can "add" PointClouds by doing pc1.Add(pc2);
% useful when you want to merge tow point clouds after the alignment.

% You have plotted a point cloud and want to know the color/intensity
% associated to a position ?
% Select a data point, right-click and export its position p. Then
% pc1.WhatColor(p.Position)
% A good way to select the limits of a color filter : plot the point cloud
% and ask for the color of some interesting points.

%% How to spatially transform a point cloud ?
% Sometimes, they are more easy to visualize when moved in another frame.
% Here, make a rotation along the x axis and get a new object.
% pc1t = AffinTransform(pc1,TransformMatrix([Deg2Rad(90-25),0,0]));
% Check the result :
% pc1t.plot3(false,'MarkerSize',1,'Marker','x','LineStyle','none','Color',[0 0 1]);
% If you do not wish a new PointCloud
%pc1.transform(TransformMatrix([Deg2Rad(90-25),0,0]));



