% FINDSHAPE is a script using ICP algorithm to track spherical target
% in multi-temporal pointcloud. After a first manual selection FINDSPHERE
% detect automatically position (X,Y,Z) of center of mass of rigid body shape
% in each pointcloud acquisition.
%
% Remarks:
% - FINDSHAPE work with libraries of PointCloudToolBox developed at
%   the Institute of Geomatics and Analysis of Risk - University of Lausanne
%   (IGAR-UNIL) by Neal Gauvin. All the PointCloudToolBox is propriety of
%   IGAR-UNIL.
%
% AUTHOR  : Dario Carrea (at unil dot ch)
% VERSION : 1.0
% STATUS  : OK
% DATE    : 7 March 2012
%% %%%%%%%%%%%%%%%%%%%%%% LOADING DATA %%%%%%%%%%%%%%%%%%%%%%
close all;
clc;

matlabpool open local

cd 'E:\Thèse\Scan_LiDAR_Thèse\Sites_Terrains_Rocheux\Orsières (VS)\Doc_MATLAB\Disp_bloc1_'
files = dir('*.txt');
tic

filesPC = length(files);

parfor i=1:filesPC
    pc(i) = ImportPointCloudFromASCII(files(i).name,'',{'P'});
end

% parfor i=1:filesPC
%     pcn(i)= ComputeOptimalNormals(pc(i),'Iteration',10,'Threshold',30);
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Let's correct the position of the dataset according to 3D digitizer position.
%More easy for visulalization.
%!!!Warning!!! Do not forget to note the inclinaison angle of the 3D digitizer
% during experiment.
%
parfor i=1:filesPC
    pct(i) = AffinTransform(pc(i),TransformMatrix([0,0,0]));
end

parfor i=1:filesPC
    pct(i).GetMissingPropFromPC(pc(i));
end


toc
matlabpool close
%
%% %%%%%%%%%%%%%%%%%%%%%% SETECTION OF INITIAL SHAPE POSITION %%%%%%%%%%%%%%%%%%%%%%
%
% % Try to find back the shape.
% % At this point you have to select manually:
% %  - 1/ select one point closer to shape to detect.
% %  - 2/ save as "shape.mat".
% %  - 3/ reload the matlab script.
% %
% %
% %
ReconShape = PointCloud('Initial Scan',pct(1));
%
% ReconShape.plot3(false,'MarkerSize',1,'Marker','o','LineStyle','none','Color',[0 0 1]);
%
%% %%%%%%%%%%%%%%%%%%%%%% TARGET TRACKING %%%%%%%%%%%%%%%%%%%%%%
% Tracks one sphere in all laps scans starting from its previous position.
% Then selected another one and track it in all laps scans starting from
% its previous position. etc...
%
% %Test with different ICP parameters!
myarg = struct('Tol',{0.000000001},'Matching','kDtree');
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
%
%%%% will identiied the number of time laps scans done used for j loop.


    barre= waitbar(0,'Initialization...','Name','Tracking in progress...');
    
    
    %eval(sprintf('Pos_temp = shape.Position;'));
    
    %%% Create the shape that will be use to fit to the scan.
    Shape =ImportPointCloudFromASCII('E:\Thèse\Scan_LiDAR_Thèse\Sites_Terrains_Rocheux\Orsières (VS)\Doc_MATLAB\Bloc\shape_1_.txt','Shape',{'P'});
    SHAPE = AffinTransform(Shape,TransformMatrix([0,0,0]));
    GenTM = [mean(SHAPE.P(1,:)),mean(SHAPE.P(2,:)),mean(SHAPE.P(3,:))]';
    shp = SHAPE;
    shp.Name = 'Shape';
    fprintf('Center of the shape before ICP : %.4f %.4f %.4f\n',GenTM(1),GenTM(2),GenTM(3));
    %PlotMultiPointClouds({shp,ReconShape},'PlotOpt',myopt);
    
    % %ICP align !
    Output = ICPVarOut();
    ICP(ReconShape,shp,Output,myarg);
    RecTM = Output.LastTM;
    
    %PlotMultiPointClouds({shp,ReconShape},'PlotOpt',myopt);
    
    % %where is the center of the sphere ?
    TMTot_initial = AffinTransform(GenTM,RecTM);
    fprintf('Center of the shape after ICP : %.4f %.4f %.4f\n',TMTot_initial(1),TMTot_initial(2),TMTot_initial(3));
    
    
    TMTot=[];
    for j= 1:filesPC
        %%% Create the sphere that will be use to fit to the scan.
        %sph = GenSphere(5,25);
        % %     SHAPE = PointCloud('Shape');
        % %     SHAPE.TrueP =[Shape(:,1) Shape(:,2) Shape(:,3)];
        % %     shp = SHAPE;
        % %     shp.Name = 'Shape';
        % %     shp.copyTrue2MeasPos;
        if isempty(TMTot)
            
            %fprintf('Center of the shape before ICP : %.1f %.1f %.1f\n',TMTot_initial(1),TMTot_initial(2),TMTot_initial(3));
            %PlotMultiPointClouds({shp,pct(1,j)},'PlotOpt',myopt);
            
            % %ICP align !
            Output = ICPVarOut();
            ICP(pct(1,j),shp,Output,myarg);
            RecTM = Output.LastTM;
            
            %PlotMultiPointClouds({shp,pct(1,j)},'PlotOpt',myopt);
            
            % %where is the center of the shape ?
            TMTot = AffinTransform(TMTot_initial,RecTM);
            fprintf('Center of the shape after ICP : %.4f %.4f %.4f\n',TMTot(1),TMTot(2),TMTot(3));
            
            
            
        else
            %fprintf('Center of the shape before ICP : %.1f %.1f %.1f\n',TMTot(1),TMTot(2),TMTot(3));
            %PlotMultiPointClouds({shp,pct(1,j)},'PlotOpt',myopt);
            
            % %%%%ICP align !
            Output = ICPVarOut();
            ICP(pct(1,j),shp,Output,myarg);
            RecTM = Output.LastTM;
            % Output.PlotDist;
            PlotMultiPointClouds({shp, pct(1,j)},'PlotOpt',myopt);
            
            % %where is the center of the shape ?
            TMTot = AffinTransform(TMTot,RecTM);
            fprintf('Center of the shape after ICP : %.4f %.4f %.4f\n',TMTot(1),TMTot(2),TMTot(3));
            
        end
        
        X=TMTot(1); Y=TMTot(2); Z=TMTot(3);
        Results(1:3,j) = [X,Y,Z] ;
        
        waitbar(j/filesPC,barre,sprintf('PC#%1.0f',j))
        save(['Matrice_' num2str(j) '.mat'],'TMTot')
    end
close(barre);

Displacement = Results';
%%
% 
% for g = 1:fin
%     PCS(1).UsefulVar(:,g) = sqrt(  (Displacement{g,1}(1,1) - Displacement{g,1}(2,1))^2   +   (Displacement{g,1}(1,2)-Displacement{g,1}(2,2))^2   +   (Displacement{g,1}(1,3)-Displacement{g,1}(2,3))^2   );
% end
% 
% 
% for g = 1:fin
%     PCS(1).Normals(1,g) = Displacement{g,1}(1,1) - Displacement{g,1}(2,1);
% end
% 
% for g = 1:fin
%     PCS(1).Normals(2,g) = Displacement{g,1}(1,2)-Displacement{g,1}(2,2);
% end
% 
% for g = 1:fin
%     PCS(1).Normals(3,g) = Displacement{g,1}(1,3)-Displacement{g,1}(2,3);
% end
% 
% 
% SaveInASCII(PCS(1,1),'Comp\Comparison_100x21810.txt',{'P','Normals','UsefulVar'});
% %%
% % % PCS(1).plot3(false,'MarkerSize',1,'Marker','o','LineStyle','none','Color',[0 1 0]);
% % % hold on; 
% % % PCS(2).plot3(false,'MarkerSize',1,'Marker','o','LineStyle','none','Color',[D2 0 0]);
% % % hold off
% % % 
% % % 
% % % 
% scatter3(PCS(1).P(1,1:fin),PCS(1).P(2,1:fin),PCS(1).P(3,1:fin), 3, PCS(1).UsefulVar(1,1:fin),'filled');
% axis equal 
% % % 
% % % scatter3(PCS(2).P(1,:),PCS(2).P(2,:),PCS(2).P(3,:), 3, D(:,1),'filled');
% % % axis equal
% % % 
% % % PCS(1,1).UsefulVar = D2';

