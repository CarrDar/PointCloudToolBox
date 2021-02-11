close all;
clc;
cd 'C:\Users\Dario\Documents\Sauvegarde_unil\Thèse\Modelisation\MATLAB\Data_Test\Végétation_test';

pc=ImportPointCloudFromASCII('Filtered_manually_ROCK.txt','Test',{'P','Intensities'});
pc.ComputeKDTree;
pc.TLSPos=[0 0 0]';
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

threshold_rock=GammaNormals_rock+(Gamma_rock+Gamma_rock6);
figure;hist(threshold_rock,200);

% % % % threshold2_rock=(Gamma_rock-Gamma_rock5);%GammaNormals_rock+
% % % % figure;hist(threshold2_rock,200);


%%
pc2=ImportPointCloudFromASCII('Filtered_manually_VEGETATION.txt','Test',{'P','Intensities'});
pc2.ComputeKDTree;
pc2.TLSPos=[0 0 0]';
tic; pc2.ComputeOptimalNormals; toc
pc2.NormalsOutTopo;
[N32, ~] = knnsearch(pc2.KDTree,(pc2.P)', 'k', 50);
%%

for i=1:length(pc2.P)
d(1,:)= pc2.Normals(1,N32(i,1:10))- pc2.Normals(1,i);
d(2,:)= pc2.Normals(2,N32(i,1:10))- pc2.Normals(2,i);
d(3,:)= pc2.Normals(3,N32(i,1:10))- pc2.Normals(3,i);
D=d';
F(i,1)={D};

end

for i=1:length(pc2.P)
dd(1,:)= pc2.P(1,N32(i,:))- pc2.P(1,i);
dd(2,:)= pc2.P(2,N32(i,:))- pc2.P(2,i);
dd(3,:)= pc2.P(3,N32(i,:))- pc2.P(3,i);
DD=dd';
FF(i,1)={DD};
end

tic
for i=1:length(pc2.P)
[~,~,EIG]=pca(FF{i,1});
EIGvalue2(i,1:3)=EIG;
end
toc

for i=1:length(pc2.P)
Result_inter22(1,i)=EIGvalue2(i,3)/(EIGvalue2(i,1)+EIGvalue2(i,2)+EIGvalue2(i,3));
end
Gamma_veg=(Result_inter22)';

for i=1:length(pc2.P)
Result_inter42(1,i)=EIGvalue2(i,2)/(EIGvalue2(i,1)+EIGvalue2(i,2)+EIGvalue2(i,3));
end
Gamma_veg2=(Result_inter42)';

for i=1:length(pc2.P)
Result_inter62(1,i)=EIGvalue2(i,1)/(EIGvalue2(i,1)+EIGvalue2(i,2)+EIGvalue2(i,3));
end
Gamma_veg3=(Result_inter62)';

for i=1:length(pc2.P)
STDNormals_veg(i,1:3)= std(F{i,1});
end

for i=1:length(pc2.P)
GammaNormals_veg(i,1)= sum(STDNormals_veg(i,1:3));
end

clear F FF D DD

figure;hist(STDNormals_veg,100);figure;hist(GammaNormals_veg,100);figure;hist(Gamma_veg,100);

for i=1:length(pc2.P)
    Result_inter22(1,i)=EIGvalue2(i,2)/EIGvalue2(i,1);
end
Gamma_veg5=(Result_inter22)';

for i=1:length(pc2.P)
    Result_inter32(1,i)=EIGvalue2(i,3)/EIGvalue2(i,2);
end
Gamma_veg6=(Result_inter32)';

for i=1:length(pc2.P)
    Result_inter82(1,i)=EIGvalue2(i,1)/EIGvalue2(i,2);
end
Gamma_veg7=(Result_inter82)';


threshold_veg=GammaNormals_veg+(Gamma_veg+Gamma_veg6); %GammaNormals_veg+
figure;hist(threshold_veg,200);


% % % % % threshold2_veg=(Gamma_veg-Gamma_veg5); %GammaNormals_veg+
% % % % % figure;hist(threshold2_veg,200);

%%
threshold_rock=GammaNormals_rock+(Gamma_rock+Gamma_rock6);
figure;hist(threshold_rock,200);
threshold_veg=GammaNormals_veg+(Gamma_veg+Gamma_veg6); %GammaNormals_veg+
figure;hist(threshold_veg,200);

threshold2_rock=(Gamma_rock-Gamma_rock5);%GammaNormals_rock+
figure;hist(threshold2_rock,200);hold on;
threshold2_veg=(Gamma_veg-Gamma_veg5); %GammaNormals_veg+
hist(threshold2_veg,200);


hist3d_veg=horzcat(threshold_veg,threshold2_veg);
hist3d_rock=horzcat(threshold_rock,threshold2_rock);



[N_veg ~]=hist3(hist3d_veg,[500 500]);

[N_rock ~]=hist3(hist3d_rock,[500 500]);


awesome_results=vertcat(hist3d_veg,hist3d_rock);

[N ~]=hist3(awesome_results,[500 500]);

hold on
figure;imagesc(N);

%figure;imagesc(N_rock);
%figure;imagesc(N_veg);
