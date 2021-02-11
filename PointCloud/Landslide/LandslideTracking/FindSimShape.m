% FINDSPHERE is a script using ICP algorithm to track spherical target 
% in multi-temporal pointcloud. After a first manual selection FINDSPHERE
% detect automatically position (X,Y,Z) of center of mass of rigid body shape
% in each pointcloud acquisition.
%
% Remarks:
% - FINDSPHERE work with libraries of PointCloudToolBox developed at
%   the Institute of Geomatics and Analysis of Risk - University of Lausanne
%   (IGAR-UNIL) by Neal Gauvin. All the PointCloudToolBox is propriety of
%   IGAR-UNIL.
%
% - FINDSPHERE provide much better results with scan acquiered with
%   colors proprieties. 
%
% AUTHOR  : Dario Carrea (at unil dot ch)
% VERSION : 2.2
% STATUS  : OK
% DATE    : 30 september 2011
%% %%%%%%%%%%%%%%%%%%%%%% LOADING DATA %%%%%%%%%%%%%%%%%%%%%%
close all;
clc;

matlabpool open local

cd 'D:\Dario_unil\Thèse\Article\Article_main_author\Peer_Papers\5_Carrea_Sandbox_Landslide_shortnote_in-prep_3.049\Data\Velocity_test_quint\Segmented'
files = dir('*.txt');
tic
%barre= waitbar(0,'Initialization...','Name','Creation PointCloud...'); 
parfor i=1:length(files) 
        pc(i) = ImportPointCloudFromASCII(files(i).name,'',{'P','Colors'});
    %waitbar(i/length(files),barre,sprintf('PC#%1.0f',i))
end
%close(barre);


% % % % % %barre= waitbar(0,'Initialization...','Name','Computing Normals...');
% % % % % parfor i=1:length(files)  
% % % % %     ComputeOptimalNormals(pc(1,i),'Iteration',10,'Threshold',30,'D1',3,'D2',1,'epsilon',0.05,'sigma',0.5);
% % % % %     %waitbar(i/2,barre,sprintf('PC#%1.0f',i))
% % % % % end
% % % % % %close(barre);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Let's correct the position of the dataset according to 3D digitizer position.
%More easy for visulalization. 
%!!!Warning!!! Do not forget to note the inclinaison angle of the 3D digitizer
% during experiment.
%
%barre= waitbar(0,'Initialization...','Name','Correcting PointCloud position...'); 
parfor i=1:length(files)
        pct(i) = AffinTransform(pc(i),TransformMatrix([Deg2Rad(90-0),0,0])); %[Deg2Rad(90-5),0,0]    [Deg2Rad(90-35),0,0]
    %waitbar(i/length(files),barre,sprintf('PCT#%1.0f',i))
end
%close(barre);
matlabpool close
toc
%
%% %%%%%%%%%%%%%%%%%%%%%% SETECTION OF INITIAL SPHERE POSITION %%%%%%%%%%%%%%%%%%%%%%
%
% % Try to find back the spheres.
% % At this point you have to select manually:
% %  - 1/ select one point closer to each sphere and save as "sphere#NN".
% %  - 2/ save all file "sphere#NN" as "spheres.mat".
% %  - 3/ reload the matlab script.
% %
% %
% %
ReconSphere = PointCloud('Initial Scan',pct(1));
% % Get back the colors
% % Test to identify if there is a color information or not.
%
%
%
% Insert manually the number of sphere point selected so call sphereNN° --> Can be improved by saving all the sphereNN° in a file  *.mat
%
filesSphere = 7;
%
%
%
if isempty(ReconSphere.Colors) && isempty(ReconSphere.Intensities)
    
    ReconSphere.plot3(false,'MarkerSize',1,'Marker','o','LineStyle','none','Color',[0 0 1]); axis equal;
    
    barre= waitbar(0,'Initialization...','Name','Correcting PointCloud position...');
    for i=1:length(files)
        pcs(i) = pct(i);
        waitbar(i/length(files),barre,sprintf('PCS#%1.0f',i))
    end
    close(barre);
    delete('pct');
    
elseif isempty(ReconSphere.Intensities)
    % Try to find back the spheres by the color.
    %
    % Important note : you cannot visualise the data clouds with the
    % associated colors when > O(1000)) points. You need to subsample, for
    % instance using a loose color filter :
    %
    %
    
    RSS = SubSampling(ReconSphere,'Type','Color','Arg',{[150 150 150]});
    RSS.PlotPositionsWithColors;
    
    barre= waitbar(0,'Initialization...','Name','Detection Position with Color...');
    for k=1:filesSphere
        eval(sprintf('Pos_temp = sphere%1.0f.Position;',k));
        InfoSphere(:,k)= ReconSphere.WhatColor(Pos_temp);
        
        waitbar(k/filesSphere,barre,sprintf('PCT#%1.0f',k))
    end
    close(barre);
    close all
    
    %%% Once you get an idea of what is the typical colors of your sphere.
    R= round(mean(InfoSphere(1,1:k))); Rbuffer= round(std(InfoSphere(1,1:k)));
    G= round(mean(InfoSphere(2,1:k))); Gbuffer= round(std(InfoSphere(2,1:k)));
    B= round(mean(InfoSphere(3,1:k))); Bbuffer= round(std(InfoSphere(3,1:k)));
    
    
    barre= waitbar(0,'Initialization...','Name','Isolating Targets by Color...');
    for i=1:length(files)
        %pcs(i) = SubSampling(pct(i),'Type','Color','Arg',{[R G B]});
        %pcs(i) = SubSampling(pct(i),'Type','Color','Arg',{[R+Rbuffer G+Gbuffer B+Bbuffer]},'Arg',{[R-2*Rbuffer G-2*Gbuffer B-2*Bbuffer]});
        pcs(i) = SubSampling(pct(i),'Type','Color','Arg',{[150 150 150]});
        waitbar(i/length(files),barre,sprintf('PCS#%1.0f',i))
    end
    close(barre);
    parfor i=1:length(files)
        pcs(i).PlotPositionsWithColors;
    end
    
    delete('pct')
else
    RSS = SubSampling(ReconSphere,'Type','Intensity','Arg',{[0 255]});
    RSS.PlotPositionsWithIntensities;
    
    barre= waitbar(0,'Initialization...','Name','Detection Position with Intensities...');
    for k=1:filesSphere
        eval(sprintf('Pos_temp = sphere%1.0f.Position;',k));
        InfoSphere(:,k)= ReconSphere.WhatIntensity(Pos_temp);
        
        waitbar(k/filesSphere,barre,sprintf('PCT#%1.0f',k))
    end
    close(barre);
    close all
    
    %%% Once you get an idea of what is the typical intensity of your sphere.
    Intense = round(mean(InfoSphere(1,1:k))); %Intensebuffer= round(std(InfoSphere(1,1:k)));
    
    
    barre= waitbar(0,'Initialization...','Name','Isolating Targets by Intensity...');
    for i=1:length(files)
        pcs(i) = SubSampling(pct(i),'Type','Intensity','Arg',{[Intense]});
        waitbar(i/length(files),barre,sprintf('PCS#%1.0f',i))
    end
    close(barre);
    pcs(1).PlotPositionsWithIntensities;
    delete('pct')
end

%

%
%% %%%%%%%%%%%%%%%%%%%%%% TARGET TRACKING %%%%%%%%%%%%%%%%%%%%%%
% Tracks one sphere in all laps scans starting from its previous position.
% Then selected another one and track it in all laps scans starting from
% its previous position. etc...
load('spheres.mat');
%
% %Test with different ICP parameters!
myarg = struct('NbIterMax',200,'Tol',{0.000001},'Matching','kDtree'); %
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
% % ,'RejectNormals', pi/4
% % ,'RejectIsolated', 0.1
% % ,'TimeLimit',{inf}
% % ,'Tol',{0.01}
% % ,'Verbose'
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

filesPC = length(pcs); % 111  106 49 length(pcs)
Spheres = filesSphere; % 7 10 11  filesSphere

barre= waitbar(0,'Initialization...','Name','Tracking in progress...');
for k= 1:Spheres  
    
    eval(sprintf('Pos_temp = sphere%1.0f.Position;',k));
    %%% Create the sphere that will be use to fit to the scan.
    %sph = GenSphere(5,25);
    sph = GenHalfSphere(5,50); %working with a half sphere improves things much better.
    sph.Name = 'Test Sphere';
    sph.copyTrue2MeasPos;
    %%%
    GenTM = TransformMatrix([0,0,0],Pos_temp); % best
    %fprintf('Center of the sphere before ICP : %.1f %.1f %.1f\n',GenTM(1,4),GenTM(2,4),GenTM(3,4));
    sph.transform(GenTM);
    
    %PlotMultiPointClouds({sph,ReconSphere},'PlotOpt',myopt);
    
    % %ICP align !
    Output = ICPVarOut();
    
    ICP(ReconSphere,sph,Output,myarg);
    %ICP(recsph1,sph,Output,myarg);
    RecTM = Output.LastTM;
    %PlotMultiPointClouds({sph,ReconSphere},'PlotOpt',myopt);
    
    % %where is the center of the sphere ?
    TMTot_initial = AffinTransform(GenTM,RecTM);
    %fprintf('Center of the sphere after ICP : %.1f %.1f %.1f\n',TMTot_initial(1,4),TMTot_initial(2,4),TMTot_initial(3,4));
    
    
    TMTot=[];
    for j= 1:1:filesPC
        %%% Create the sphere that will be use to fit to the scan.
        %sph = GenSphere(5,25);
        sph = GenHalfSphere(5,50); %working with a half sphere improves things much better.
        sph.Name = 'Test Sphere';
        sph.copyTrue2MeasPos;
        if isempty(TMTot)
            
            GenTM = TransformMatrix([0,0,0],[TMTot_initial(1,4),TMTot_initial(2,4),TMTot_initial(3,4)]); % best
            %fprintf('Center of the sphere before ICP : %.1f %.1f %.1f\n',GenTM(1,4),GenTM(2,4),GenTM(3,4));
            sph.transform(GenTM);
            
            %PlotMultiPointClouds({sph,pcs(1,j)},'PlotOpt',myopt);
            
            % %ICP align !
            Output = ICPVarOut();
            ICP(pcs(j),sph,Output,myarg); %pcs
            %ICP(recsph1,sph,Output,myarg);
            RecTM = Output.LastTM;
            %PlotMultiPointClouds({sph,pcs(1,j)},'PlotOpt',myopt);
            % %where is the center of the sphere ?
            TMTot = AffinTransform(GenTM,RecTM);
            %fprintf('Center of the sphere after ICP : %.1f %.1f %.1f\n',TMTot(1,4),TMTot(2,4),TMTot(3,4));
            
            
            
        else
            TMTtotx= TMTot(1,4); %TMTot_initial(1,4);%
            TMTtoty= TMTot(2,4); %TMTot_initial(2,4);%
            TMTtotz= TMTot(3,4); %TMTot_initial(3,4);%
% % %                         if j>3
% % %                             VectTmp1=[Results(1,k,j-2),Results(2,k,j-2),Results(3,k,j-2)]';
% % %                             VectTmp2=[Results(1,k,j-3),Results(2,k,j-3),Results(3,k,j-3)]';
% % %                             VectTmp3= abs(VectTmp2 - VectTmp1);
% % %                             Norm= 0.3; % Changing according to result quality!!!!!!!
% % %                             TMTmp= Norm*VectTmp3;
% % %                             TMTtot= VectTmp1-TMTmp;
% % %                             TMTtotx= TMTot(1,4);
% % %                             TMTtoty= TMTtot(2,1);
% % %                             TMTtotz= TMTtot(3,1);
% % %                         end
            GenTM = TransformMatrix([0,0,0],[TMTtotx,TMTtoty,TMTtotz]); % best
            %fprintf('Center of the sphere before ICP : %.1f %.1f %.1f\n',GenTM(1,4),GenTM(2,4),GenTM(3,4));
            sph.transform(GenTM);
            
            %PlotMultiPointClouds({sph,pcs(1,j)},'PlotOpt',myopt);
            
            % %ICP align !
            Output = ICPVarOut();
            ICP(pcs(j),sph,Output,myarg); %pcs
            RecTM = Output.LastTM;
            close;
            Output.PlotDist;
            
            %PlotMultiPointClouds({sph,pcs(j)},'PlotOpt',myopt);
            % %where is the center of the sphere ?
            TMTot = AffinTransform(GenTM,RecTM);
            %fprintf('Center of the sphere after ICP : %.1f %.1f %.1f\n',TMTot(1,4),TMTot(2,4),TMTot(3,4));
            
        end
        X=TMTot(1,4); Y=TMTot(2,4); Z=TMTot(3,4);
        Results(1:3,j)=[X,Y,Z];
        %save(['Matrice_' num2str(j) '.mat'],'TMTot')
    end

    Displacement(k,1) = {Results'};
    waitbar(k/Spheres,barre,sprintf('Target#%1.0f',k))
end
close(barre);
