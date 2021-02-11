% DEFORMATION is a script using ICP algorithm to track non-rigid target 
% in multi-temporal pointcloud. After a first automatic selection
% DEFORMATION detect automatically non-rigid body position (X,Y,Z) in each pointcloud.
%
% Remarks:
% - DEFORMATION work with libraries of PointCloudToolBox developed at
%   the Institute of Geomatics and Analysis of Risk - University of Lausanne
%   (IGAR-UNIL) by Neal Gauvin. All the PointCloudToolBox is propriety of
%   IGAR-UNIL.
% 
%
%AUTHOR  : Dario Carrea (at unil dot ch)
%VERSION : 2.0
%STATUS  : OK
%DATE    : 16 January 2015
%% %%%%%%%%%%%%%%%%%%%%%% LOADING DATA %%%%%%%%%%%%%%%%%%%%%%
close all;
clc;
tic
cd 'C:\Users\Michel\Documents\MATLAB\Matlab_pctb\Data\Perraire'
files = dir('*.txt');

filesPC = length(files);

matlabpool open local
    
for i=1:filesPC
        pc(i) = ImportPointCloudFromASCII(files(i).name,'',{'P'});
end

% %Let's compute the normals! For the moment not necessary, but maybe can be useful later
% % parfor i=1:filesPC
% %     if isempty(pc(i).Normals)
% %             pcn(i) = ComputeOptimalNormals(pc(i));  
% %     else
% %         pcn(i) = pc(i); 
% %     end
% % end
pcn = pc;

% % Clear data to release memory.
clear pc;

% % Let's correct the position and alignment according to RT matrix export
% %from an other software! (i.e. PolyWorks ImInspect).
%
% % Don't forget to load them!

pct=pcn;
% % pct(1) = transform(pcn(1),[0 0 0]); %Insert matrix here
% % pct(2) = transform(pcn(2),[0 0 0]); %Insert matrix here

% % Clear data to release memory.
clear pcn;

for i =1:filesPC
    if isempty(pct(i).TLSPos)
    pct(i).TLSPos= [587890 100930 2000]';%error('Please provide TLS position [3x1] for Point Cloud #%1.0f',i); %
    end
end

for i =1:filesPC
    pct(i).TLSAttribute= ones(1,length(pct(i).P));
end

for i =1:filesPC
    PCS(i)= SubSampling(pct(i),'Type','Uniform','Arg',{1000});
end

matlabpool close
toc
%% %%%%%%%%%%%%%%%%%%%%%% DEFINITION OF INITIAL RESEARCH AREA %%%%%%%%%%%%%%%%%%%%%%
% % Find in full point cloud (pct) the 100 closest point from subsampled
% % point cloud (PCS)
% % 
% % Select a data point area size and export its position.
% %
% % Define exploration window size.
tic
window = 1000; % Test with different window size to see precision vs time of computation.
[PI,~] = knnsearch((pct(1).P)',(PCS(1).P)','K',window,'NSMethod','kdtree','Distance','euclidean');
toc       
%% %%%%%%%%%%%%%%%%%%%%%% TRACKING DEFORMATION %%%%%%%%%%%%%%%%%%%%%%
% Tracks one part of cloud in all timelaps scans starting from its previous position.
% Then selected another one and track it in all timelaps scans starting from
% its previous position. etc...
%
matlabpool open

% %Test with different ICP parameters!
myarg = struct('NbIterMax',200,'Tol',{0.001},'Matching','kDtree');
% %
% % ,'Extrapolation',true
% % ,'Matching','kDtree'
% % ,'Matching','Delaunay'
% % ,'Matching','GLtree'
% % ,'Minimise',{'plane',1}
% % ,'Minimise',{'lma',1}
% % ,'Minimise',{'nm',1}
% % ,'Minimise',{'bfgs',2}
% % ,'NbIterMax',{30}
% % ,'RejectDistPairs',20
% % ,'RejectRMSPairs',2
% % ,'RejectNWorstPairs',0.5
% % ,'RejectNormals', pi/4
% % ,'RejectIsolated', 0.1
% % ,'TimeLimit',{inf}
% % ,'Tol',{0.01}
% % ,'Verbose',true
% % ,'Weight','dist'
% % ,'Weight','normal'
% % ,'Weight','curv'
%
% % If you are interested in viewing the results!
myopt = {
    %{'MarkerSize',1,'Marker','.','LineStyle','none','Color',[0.75 0 0.75]},...
    %{'MarkerSize',4,'Marker','+','LineStyle','none','Color',[0 0.5 0]},...
    %{'MarkerSize',4,'Marker','x','LineStyle','none','Color',[0 0 1]}};
    {'MarkerSize',4,'Marker','o','LineStyle','none','Color',[0.75 0 0.75]},...
    {'MarkerSize',3,'Marker','.','LineStyle','none','Color',[0 0 1]}
    };
    


barre= waitbar(0,'Initialization...','Name','Tracking in progress...');
fin=size(PI,1);

% Vtic
% parfor Area = 1:fin
%     ReconAREA(Area) = PointCloud('Initial Scan',pct(1).P(:,PI(Area,:)));
% end
% toc

tic
parfor Area = 1:fin
    % % Only if you need to understand what will happen.
    % %
    % % ReconAREA.GetMissingPropFromPC(pct(1));
    % %
    % % ReconAREA.plot3(false,'MarkerSize',5,'Marker','.','LineStyle','none','Color',[0 0 1]);
    ReconAREA = PointCloud('Initial Scan',pct(1).P(:,PI(Area,:)));
    %%% Create the shape that will be use to fit to the scan.
    %GenTM = TransformMatrix([0,0,0],[mean(ReconAREA.P(1,:)),mean(ReconAREA.P(2,:)),mean(ReconAREA.P(3,:))]); % Center of mass of the point cloud. 
    GenTM = TransformMatrix([0,0,0],[ReconAREA.P(1,1),ReconAREA.P(2,1),ReconAREA.P(3,1)]); % The initial point from which you have built the shape. Best Solution!
    %GenTM = TransformMatrix([0,0,0],[ReconAREA.P(1,'...'),ReconAREA.P(2,'...'),ReconAREA.P(3,'...')]); % A selected point in the point cloud.
    shp = ReconAREA;
    shp.Name = 'Shape';
    %fprintf('Center of the shape before ICP : %.4f %.4f %.4f\n',GenTM(1,4),GenTM(2,4),GenTM(3,4));
    %PlotMultiPointClouds({shp,ReconShape},'PlotOpt',myopt);
    
    % % ICP align !
    Output = ICPVarOut();
    ICP(ReconAREA,shp,Output,myarg);
    RecTM = Output.LastTM;
    
    %PlotMultiPointClouds({shp,ReconShape},'PlotOpt',myopt);
    
    % % Where is the center of the shape ?
    TMTot_initial = AffinTransform(GenTM,RecTM);
    %fprintf('Center of the shape after ICP : %.4f %.4f %.4f\n',TMTot_initial(1,4),TMTot_initial(2,4),TMTot_initial(3,4));
    
    
    TMTot=[];
    Results=[];
    for j= 1:filesPC
        
        if isempty(TMTot)
            
            %fprintf('Center of the shape before ICP : %.1f %.1f %.1f\n',TMTot_initial(1),TMTot_initial(2),TMTot_initial(3));
            %PlotMultiPointClouds({shp,pct(1,j)},'PlotOpt',myopt);
            
            % %bICP align !
            Output = ICPVarOut();
            ICP(pct(1,j),shp,Output,myarg);
            RecTM = Output.LastTM;
            
            %PlotMultiPointClouds({shp,pct(1,j)},'PlotOpt',myopt);
            %axis equal
            % % Where is the center of the shape ?
            TMTot = AffinTransform(TMTot_initial,RecTM);
            %fprintf('Center of the shape after ICP : %.4f %.4f %.4f\n',TMTot(1,4),TMTot(2,4),TMTot(3,4));
            
            
            
        else
            %fprintf('Center of the shape before ICP : %.1f %.1f %.1f\n',TMTot(1),TMTot(2),TMTot(3));
            %PlotMultiPointClouds({shp,pct(1,j)},'PlotOpt',myopt);
            
            % % ICP align !
            Output = ICPVarOut();
            ICP(pct(1,j),shp,Output,myarg);
            RecTM = Output.LastTM;
            %Output.PlotDist;
            %PlotMultiPointClouds({shp, pct(1,j)},'PlotOpt',myopt);
            %axis equal
            % % Where is the center of the shape ?
            TMTot = AffinTransform(TMTot,RecTM);
            %fprintf('Center of the shape after ICP : %.4f %.4f %.4f\n',TMTot(1,4),TMTot(2,4),TMTot(3,4));
            
        end
        
        X=TMTot(1,4); Y=TMTot(2,4); Z=TMTot(3,4);
        Results(1:3,j) = [X,Y,Z];
        
    end
    
    Displacement(Area,1) = {Results'};
    %ReconAREA(Area)=nan(1:3,window);
    %waitbar(Area/fin,barre,sprintf('Window #%1.0f',Area))
    
    
end

%close(barre);
toc
%% %%%%%%%%%%%%%%%%%%%%%% Results in Point Cloud %%%%%%%%%%%%%%%%%%%%%%
% % Transform results of "Deformaion algorithm" into variables & color for visualization
% % in other software.

Final=PointCloud;
Final.P=(cell2mat(Displacement))';
Final_=PointCloud;
fin=length(Final.P);

for i =1:filesPC    
    PCS(i).P = Final.P(:,i:filesPC:fin);
end
for i =1:filesPC-1
    PCS(i+1).Normals=(PCS(i+1).P)-(PCS(i).P);
end

for i =1:filesPC
    PCS(i).TLSPos = pct(i).TLSPos;
end

for i =1:filesPC-1
    PCS(i+1).UsefulVar = EuclDist(PCS(i+1).P,PCS(i).P);
end

for i =1:filesPC-1
    for g = 1:length(PCS(i+1).UsefulVar(1,:))
        if EuclDist(PCS(i).TLSPos(1:3,1),PCS(i).P(1:3,g)) < EuclDist(PCS(i+1).TLSPos(1:3,1),PCS(i+1).P(1:3,g))  % 2011 > 2009 Négatif
            PCS(i+1).UsefulVar(1,g) = PCS(i+1).UsefulVar(1,g)*(-1);
        end
    end
end

parfor i=2:filesPC
    [R,G,B] = SDC((PCS(i).Normals(1,:))',(PCS(i).Normals(3,:))',(PCS(i).Normals(2,:))',(PCS(i).UsefulVar(1,:))',0.2,0.0,'','');
    PCS(i).Colors= [R G B];
    
    [direction,dip] = Orientations((PCS(i).Normals(1,:))',(PCS(i).Normals(3,:))',(PCS(i).Normals(2,:))',(PCS(i).UsefulVar(1,:))');
    PCS(i).UsefulVar(2,:)= direction';
    PCS(i).UsefulVar(3,:)= dip';
end

parfor i=2:filesPC
    SaveInASCII(PCS(i),sprintf('Comparison_1000x21366_c_%1.0f.txt',i),{'P','Normals','Colors','UsefulVar'});
end

matlabpool close
%% %%%%%%%%%%%%%%%%%%%%%% Plot  %%%%%%%%%%%%%%%%%%%%%%
% % Plot the final pointcloud with colors according to direction of
% % displacement in PCL Viewer. Need PCL 1.6.0 or higher installed.

PCS(2).PlotPCLViewer({'P','Colors'});
