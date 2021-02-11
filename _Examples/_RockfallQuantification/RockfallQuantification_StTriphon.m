% RockfallQuantification is a script for estimation of rockfall volume with multi-temporal point cloud
% acquisition using ICP algorithm, Euclidian distance to classify points to create a new point cloud
% representing the fallen block and alpha-shapehull to compute the volume of the block pointcloud.
%
% INPUT: - a (Mesh)PointCloud object for post-failure surface Pc1.
%        - a (Mesh)PointCloud object for pre-failure surface Pc2.
%        - T1 first threshold
%        - T2 second threshold
%        - k number of objects in a neighborhood of an object
%        - Eps neighborhood radius, if not known avoid this parameter or put []
%        - R radius of reserch alpha-shape
%        - fig if you want to plot the histograms or not
%
% OUTPUT: - a PointCloud object for block of fallen rocks.
%         - the estimated volume.
%
% Remarks:
% - ESTIMATION VOLUME works with libraries of PointCloudToolBox developed at
%   the Institute of Geomatics and Analysis of Risk - University of Lausanne
%   (IGAR-UNIL) by Neal Gauvin and Dario Carrea. All the PointCloudToolBox is
%   propriety of IGAR-UNIL.
%
% Update:
%   1.1: For faster computing, replace line knnsearch (KDTree) by nearestNeighbor(Delaunay Tri). 
%   1.2: fonction(DBSCAN) for clustering data & multi measurement blocks. 
%   2.0: Implemented for (Mesh)PointCloud object. Faster computing.
%        Possibility to use Point-to-Surface ('P2S') comaprison for a better 
%        segementation of fallen block but increase computing time. 
%        ( => see Comparison()).
%
%   References:
%   [1] D. Carrea, A. Abellán, M.-H. Derron, M. Jaboyedoff. Automatic rockfalls volume
%       estimation based on terrestrial laser scanning data, In  Engineering Geology
%       for Society and Territory - Vol. 2, Eds. G. Lollino, D. Giordan, G.B. Crosta, J.Corominas,
%       R. Azzam, J. Wasowski, N. Sciarra, Springer International Publishing, 2015.
%   [2] M. Tonini & A.,Abellán  Rockfall detection from terrestrial
%       LiDAR point clouds: a clustering approach using R. Journal of Spatial
%       Information Science, 2014.
%
% AUTHOR  : Dario Carrea (at unil dot ch)
% VERSION : 3.0
% STATUS  : OK
% DATE    : 14 December 2017
tic
clear all; close all;
Pc1=PointCloud;
Pc2=PointCloud;
Pc1=ImportPointCloudFromASCII('2010_StTriphon_Nord- Cloud.txt','Pc1',{'P','Colors','UsefulVar','Normals'});
Pc2=ImportPointCloudFromASCII('2017_StTriphon_Nord- Cloud.txt','Pc2',{'P','Intensities','Normals'});
Pc1.TLSPos=[39.959904;3.074811;2.038273];
Pc2.TLSPos=[39.959904;3.074811;2.038273];

[PCRockfall,T1] = RockfallExtract(Pc1,Pc2,[],[],'P2S',0); %%#ok<MSNU>
figure;
[Rockfalls_events,Eps] = RockfallSegment(PCRockfall,25,[],0); %#o%#ok<MSNU> k<NASGU>
figure;
[BlocksVolume, SS] = RockfallVolume(Rockfalls_events,[],0);
toc