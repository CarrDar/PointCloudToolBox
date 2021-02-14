 %AUTHOR  : Dario Carrea (at unil dot ch)
 %VERSION : 0.1
 %STATUS  : OK
 %DATE    : 14 fevrier 2021


close all;
clc;
cd % Add your directories link here;

pc=ImportPointCloudFromASCII('2015-06-11_Up_10cm.txt','Test',{'P','Intensities'});
pc.ComputeKDTree;
pc.TLSPos=[0 0 0]'; % add TLS position here
tic; pc.ComputeOptimalNormals; toc
pc.NormalsOutTopo;
[N3, ~] = knnsearch(pc.KDTree,(pc.P)', 'k', 50);

%%
for i=1:length(pc.P)
d(1,:)= pc.Normals(1,N3(i,1:10))- pc.Normals(1,i);
d(2,:)= pc.Normals(2,N3(i,1:10))- pc.Normals(2,i);
d(3,:)= pc.Normals(3,N3(i,1:10))- pc.Normals(3,i);
D=d';
F(i,1)={D};

end

for i=1:length(pc.P)
dd(1,:)= pc.P(1,N3(i,:))- pc.P(1,i);
dd(2,:)= pc.P(2,N3(i,:))- pc.P(2,i);
dd(3,:)= pc.P(3,N3(i,:))- pc.P(3,i);
DD=dd';
FF(i,1)={DD};
end

tic
for i=1:length(pc.P)
[~,~,EIG]=pca(FF{i,1});
EIGvalue(i,1:3)=EIG;
end
toc

for i=1:length(pc.P)
Result_inter1(1,i)=EIGvalue(i,3)/(EIGvalue(i,1)+EIGvalue(i,2)+EIGvalue(i,3));
end
Gamma_rock=(Result_inter1)';

for i=1:length(pc.P)
Result_inter3(1,i)=EIGvalue(i,2)/(EIGvalue(i,1)+EIGvalue(i,2)+EIGvalue(i,3));
end
Gamma_rock2=(Result_inter3)';

for i=1:length(pc.P)
Result_inter5(1,i)=EIGvalue(i,1)/(EIGvalue(i,1)+EIGvalue(i,2)+EIGvalue(i,3));
end
Gamma_rock3=(Result_inter5)';

for i=1:length(pc.P)
STDNormals_rock(i,1:3)= std(F{i,1});
end

for i=1:length(pc.P)
GammaNormals_rock(i,1)= sum(STDNormals_rock(i,1:3));
end


clear F FF D DD


figure;hist(STDNormals_rock,100);figure;hist(GammaNormals_rock,100);figure;hist(Gamma_rock,100);


for i=1:length(pc.P)
    Result_inter2(1,i)=EIGvalue(i,2)/EIGvalue(i,1);
end
Gamma_rock5=(Result_inter2)';

for i=1:length(pc.P)
    Result_inter3(1,i)=EIGvalue(i,3)/EIGvalue(i,2);
end
Gamma_rock6=(Result_inter3)';

for i=1:length(pc.P)
    Result_inter8(1,i)=EIGvalue(i,1)/EIGvalue(i,2);
end
Gamma_rock7=(Result_inter8)';


% % Show the resluts for segmentation
threshold=GammaNormals_rock+(Gamma_rock+Gamma_rock6);
figure;hist(threshold,200);

threshold2=(Gamma_rock-Gamma_rock5);
figure;hist(threshold2,200);

hist3d=horzcat(threshold,threshold2);
[N ~]=hist3(hist3d,[500 500]);

figure;imagesc(N);

%%

seuil=threshold(:,1)> 1; % <-- Change Here
Var(1,seuil)=1;
Var(1,~seuil)=0;
pc.UsefulVar(1,:)=Var(1,:);

seuil=threshold2(:,1)> -0.5; % <-- Change Here
Var(1,seuil)=1;
Var(1,~seuil)=0;
pc.UsefulVar(2,:)=Var(1,:);
pc.UsefulVar(3,:)=pc.UsefulVar(2,:)+pc.UsefulVar(1,:);

tic
for i=1:length(pc.P)
second_filter(1,i)= mode(pc.UsefulVar(3,N3(i,:)));
end
toc

seuil2=pc.UsefulVar(3,:)==second_filter(1,:);
Var(1,seuil2)=pc.UsefulVar(3,seuil2);
Var(1,~seuil2)=second_filter(1,~seuil2);
pc.UsefulVar(4,:)=Var(1,:);

veg=pc.UsefulVar(4,:)>=1;
Colors(1,veg)=0;
Colors(2,veg)=255;
Colors(3,veg)=0;
veg=pc.UsefulVar(4,:)<1;
Colors(1,veg)=255;
Colors(2,veg)=255;
Colors(3,veg)=255;

pc.Colors=Colors;

clear Colors

pc_clean=PointCloud;
pc_clean.P(1:3,:)=pc.P(1:3,pc.UsefulVar(4,:)<1);
pc_clean.GetMissingPropFromPC(pc);

pc_clean.PlotPCLViewer({'P','Intensities'});
pc.PlotPCLViewer({'P','Colors'});

% % To see what are the segementation parameters
veg=pc.UsefulVar(3,:)==2;
Colors(1,veg)=0;
Colors(2,veg)=255;
Colors(3,veg)=0;
veg=pc.UsefulVar(3,:)==1;
Colors(1,veg)=0;
Colors(2,veg)=0;
Colors(3,veg)=255;

veg=pc.UsefulVar(3,:)<1;
Colors(1,veg)=255;
Colors(2,veg)=255;
Colors(3,veg)=255;

pc.Colors=Colors;

clear Colors

pc.PlotPCLViewer({'P','Colors'});

% Apply transformation before saving
%TM = eye(4);
%TM(1:3,4) = -(pc_clean.TLSPos);
%pc_clean.transform(TM);
SaveInASCII(pc_clean,'test_veg_scene.txt',{'P','Colors','Intensities','Normals','Curv'});
