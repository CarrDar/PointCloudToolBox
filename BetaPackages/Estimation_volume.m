function [V,S] = Estimation_volume(Pc1,Pc2,varargin)
% ESTIMATION VOLUME is a script for estimation of rockfall volume with multi-temporal point cloud
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
% VERSION : 2.0
% STATUS  : OK
% DATE    : 14 December 2014

%% Validate input arguments.
ip = inputParser;

ip.addRequired( 'Pc1', @(x)isa(x,'PointCloud') || isa(x,'MeshPointCloud') );
ip.addRequired( 'Pc2', @(x)isa(x,'PointCloud') || isa(x,'MeshPointCloud')  );
ip.addParamValue('T1' , {}, @(x)iscell(x));
ip.addParamValue('T2' , {}, @(x)iscell(x));
ip.addParamValue('k'  , {}, @(x)iscell(x));
ip.addParamValue('Eps', {}, @(x)iscell(x));
ip.addParamValue('R'  , {}, @(x)iscell(x));
ip.addParamValue('fig', {}, @(x)iscell(x));
ip.parse(Pc1,Pc2,varargin{:});
%% %%%%%%%%%%%%%%%%%%%%%% SANITY CHECK %%%%%%%%%%%%%%%%%%%%%%

% If MeshPointCloud are given, change them into PointCloud
if isa(Pc1,'MeshPointCloud'), Pc1 = Pc1.PointCloud; end
if isa(Pc2,'MeshPointCloud'), Pc1 = Pc2.PointCloud; end

% Following sanity check must be modified!!!!!!

% Sanity check of the thresholds
if ~isempty(arg.T1) && arg.T1 <= 0
    error('Estimation_volume:threshold','First threshold T1 must be positive!');
end

if ~isempty(arg.T2) && arg.T2 <= 0
    error('Estimation_volume:threshold2','Second threshold T2 must be positive!');
end

%   Sanity check of the number of objects in a neighborhood of an object
if ~isempty(arg.k) && arg.k <= 0 && integer(arg.k)
    error('Estimation_volume:searchRadius_Cluster','Number of objects in a neighborhood k must be positive integer!');
end

%   Sanity check of the neighborhood radius
if ~isempty(arg.Eps) && arg.Eps <= 0
    error('Estimation_volume:searchNeighbor_Cluster','Neighborhood radius must be positive!');
end

if isempty(arg.k) && isempty(arg.Eps)
    error('Estimation_volume:searchRadius_Cluster','You need at least a value for variable k or Eps');
end    

% Sanity check of the radius
if ~isempty(arg.R) && arg.R <= 0
    error('Estimation_volume:searchRadius','Search radius R must be positive!');
end

% Plot figure option check
if isempty(arg.fig) || arg.fig ~= 1
    fig =0;
    display('Histograms will not be plotted!');
end
%% %%%%%%%%%%%%%%%%%%%%%%%% IDENTIFICATION OF THE BLOCK %%%%%%%%%%%%%%%%%%%%%%

tic
% The method for point cloud distribution is Point-to-Point method.
% If you want a better segmentation choose Point-to-Surface method but it
% will increase time of computing.
%
%ComparePoint2Surface(Pc2,Pc1,'Method',{'P2S'});
%
ComparePoint2Point(Pc1,Pc2);
ComparePoint2Point(Pc2,Pc1); 

D  = Pc1.UsefulVar(1,:)>0;
D2 = Pc1.UsefulVar(1,D==1);
DD  = Pc2.UsefulVar(1,:)>0;
DD2 = Pc2.UsefulVar(1,DD==1);

% Plot the distribution of euclidian distance between points
% Allow to estimate the noise for the next IF loop

if isempty(T1)
    % Create the edges of the histogram
    MAX = max(D2);
    MIN = min(D2);
    step=(MAX-MIN)/100;
    edges= MIN:step:MAX;
    [~,bin] = histc(D2,edges);
    m = mode(bin);
    mode_= edges([m, m+1]);
    mode_1= mean(mode_);
    
     MAX_ = max(DD2);
     MIN_ = min(DD2);
     step=(MAX_-MIN_)/100;
     edges= MIN_:step:MAX_;
     [~,bin] = histc(DD2,edges);
     m_ = mode(bin);
     mode__= edges([m_, m_+1]);             %                                        ^
     mode_2= mean(mode__);                  %                                       /¦\
                                            %                                      / ¦ \
    Threshold = 2*((mode_1+mode_2)/2);      % distance between 0 and mode + mode  /  ¦  \/\____
    if fig == 1                             %                                    0   m  2m
        figure;
        hax=axes;
        hist(D2,length(edges));
        hold on
        line([Threshold Threshold],get(hax,'YLim'),'Color',[1 0 0]);
        str= 'Distribution of distance between 2 point clouds';
        str = sprintf('%s   -   Threshold 1 = %.3f',str,Threshold);
        title(str,'fontsize',12);
        hold off
    end
    
else
    Threshold = T1;
    MAX = max(D2);
    MIN = min(D2);
    step=(MAX-MIN)/100;
    edges= MIN:step:MAX;
    if fig == 1
        figure;
        hax=axes;
        hist(D2,length(edges));
        hold on
        line([Threshold Threshold],get(hax,'YLim'),'Color',[1 0 0]);
        str= 'Distribution of distance between 2 point clouds';
        str = sprintf('%s   -   Threshold 1 = %.3f',str,Threshold);
        title(str,'fontsize',12);
        hold off
    end
end

toc
%%
tic
%%% Create Point cloud with point belonging to the fallen block.
% Post rockfall surface
Pc_Block_post=PointCloud('',Pc1.P(:,Pc1.UsefulVar(1,:) > Threshold | Pc1.UsefulVar(1,:) < -(Threshold)));

% Pre rockfall surface
Pc_Block_pre = PointCloud('',Pc2.P(:,Pc2.UsefulVar(1,:) > Threshold | Pc2.UsefulVar(1,:) < -(Threshold)));


% Merging both (pre-,post-)to have one point cloud with the shape of the block
PcBlock = Add(Pc_Block_post,Pc_Block_pre);

clear Pc_Block_post Pc_Block_pre

% Save it in ACSII *.txt
SaveInASCII(PcBlock,'BLOCK\BLOCK.txt',{'P'});

toc
%% %%%%%%%%%%%%%%%%%%%%%%%%  REMOVE NON BLOCK POINTS %%%%%%%%%%%%%%%%%%%%%%%%
tic
PcBlock.ComputeKDTree;

[~, D3] = knnsearch(PcBlock.KDTree,(PcBlock.P)', 'k', 20);

if isempty(T2)
    MAX = max(D3(:,10));
    MIN = min(D3(:,10));
    step=(MAX-MIN)/100;
    edges= MIN:step:MAX;
    [~,bin] = histc(D3(:,10),edges);
    m = mode(bin);
    mode_= edges([m, m+1]);                %                                        ^
    mode_3= mean(mode_);                   %                                       /¦\
                                           %                                      / ¦ \
    Threshold =  2*(mode_3);               % distance between 0 and mode + mode  /  ¦  \______
                                           %                                    0   m    3m
    if fig == 1
        figure;
        hax=axes;
        hist(D3(:,10),length(edges));
        hold on
        line([Threshold Threshold],get(hax,'YLim'),'Color',[1 0 0]);
        str= 'Distribution of point spacing';
        str = sprintf('%s   -   Threshold 2 = %.3f',str,Threshold);
        title(str,'fontsize',12);
        hold off
    end
else
    MAX = max(D3(:,10));
    MIN = min(D3(:,10));
    step=(MAX-MIN)/100;
    edges= MIN:step:MAX;
    Threshold = T2;
    
    
    if fig == 1
        figure;
        hax=axes;
        hist(D3(:,10),length(edges));
        hold on
        line([Threshold Threshold],get(hax,'YLim'),'Color',[1 0 0]);
        str= 'Distribution of point spacing';
        str = sprintf('%s   -   Threshold 2 = %.3f',str,Threshold);
        title(str,'fontsize',12);
        hold off
    end
end

toc
%%
tic

PcBlock_Final = PointCloud('',PcBlock.P(:,D3(:,10) < Threshold));

clear PcBlock

PcBlock_Final.PlotPCLViewer({'P'});

toc
%% %%%%%%%%%%%%%%%%%%%%%%%% IDENTIFICATION & SEPARATION OF THE DIFFERENTS BLOCKS %%%%%%%%%%%%%%%%%%%%%%

%   DBSCAN method based on pre-existing code written by:
%
%   Michal Daszykowski - December 2004
%   Department of Chemometrics, Institute of Chemistry, The University of Silesia  - http://www.chemometria.us.edu.pl
%
%   Based on following papers:
%
% References:
% [1] M. Ester, H. Kriegel, J. Sander, X. Xu, A density-based algorithm for
% discovering clusters in large spatial databases with noise, proc.
% 2nd Int. Conf. on Knowledge Discovery and Data Mining, Portland, OR, 1996,
% p. 226, available from:
% www.dbs.informatik.uni-muenchen.de/cgi-bin/papers?query=--CO
% [2] M. Daszykowski, B. Walczak, D. L. Massart, Looking for
% Natural Patterns in Data. Part 1: Density Based Approach,
% Chemom. Intell. Lab. Syst. 56 (2001) 83-92
%
%
% Input:
% x - data set (m,n); m-objects, n-variables
% k - number of objects in a neighborhood of an object
% (minimal number of objects considered as a cluster)
% Eps - neighborhood radius, if not known avoid this parameter or put []
% -------------------------------------------------------------------------
% Output:
% class - vector specifying assignment of the i-th object to certain
% cluster (m,1)
% type - vector specifying type of the i-th object
% (core: 1, border: 0, outlier: -1)
%

tic

[class,~] = dbscan((PcBlock_Final.P)',k,Eps);

toc

if fig == 1
    X= PcBlock_Final.P(1,:)';
    Y= PcBlock_Final.P(2,:)';
    Z= PcBlock_Final.P(3,:)';
	figure;
    scatter3(X,Y,Z,6,class,'filled');
    axis equal;
end

PcBlock_Final.UsefulVar(1,:)=class;

% SaveInASCII(PcBlock_Final,'BLOCK\BLOCK_FIN.txt',{'P'});

for i=1:max(class)
    PCBLOCK(i)= PointCloud('',PcBlock_Final.P(:,class(1,:)==i));
end


%% %%%%%%%%%%%%%%%%%%%%%%%% VOLUME ESTIMATION WITH ALPHA-SHAPE HULL %%%%%%%%%%%%%%%%%%%%%%

%   ALPHAVOL method based on previous code written by:
%   Jonas Lundgren - 2010 
%   splinefit@gmail.com 
%
%   Based on following papers:
%
%   References:
%   [1] W.Shen, Building boundary extraction based on lidar point cloud
%       data,Int. Arch. of Photogram., Rem. Sens. and Spac. Inf. Sci., Vol.
%       37, Beijing, 2008.
%   [2] L. Lin, Conformal Alpha Shape-based Multi-scale Curvature Estimation
%       from Point Clouds, Journal of Computers, Vol. 7, N# 6, June 2012.

if isempty(R)
    R = inf;
end

hold on
for i=1:length(PCBLOCK)
    [V S] = alphavol((PCBLOCK(i).P)',R,1);
    axis equal;
    Volumes_des_blocs(i,1) = V;
end
hold off

savefile= 'Volume des blocs';
save(savefile,'Volumes_des_blocs','-ascii');

end
%%
