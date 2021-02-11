function Cloud = TargetModel( input_args )
%% TARGET MODEL
% Creat Parametric surface
%Cloud = Gen_Surface(1000,90,500,500);
% % % % % TARGET DEM
[xk yk zk]=peaks(50);
crd(1,:)=reshape(xk,1,size(xk,2)*size(xk,2)).*100;
crd(2,:)=reshape(yk,1,size(yk,2)*size(yk,2)).*100;
crd(3,:)=reshape(zk,1,size(zk,2)*size(zk,2)).*10;
pc=PointCloud('',crd);

% % % % % TARGET PLANE
% % % % pc=GenPlane('Arg',[500,500,1000,1000]);
% % % % pc.copyTrue2MeasPos;
% % % % dt_plane = DelaunayTri(pc.P(1,:)',pc.P(2,:)');
% % % % tri_plane = dt_plane(:,:);
% % % % tri_plane_save = tri_plane;
% % % % tr_plane = TriRep(tri_plane, pc.P(1,:)',pc.P(2,:)',pc.P(3,:)');
% % % % P_plane = incenters(tr_plane);
% % % % fn_plane = faceNormals(tr_plane);
% % % % 
% % % % % TARGET CUBE
% % % % pc2=GenCube([100 300 100],[0 0 0]);
% % % % pc2.copyTrue2MeasPos;
% % % % T_cube= [70,8.5,0];
% % % % R_cube= [0,0,-pi/3];
% % % % GenTM = TransformMatrix(R_cube,T_cube);
% % % % pc2.transform(GenTM);
% % % % pc2.ComputeDelaunayTriangulation;
% % % % dt_cu = pc2.DT;
% % % % [tri_cu,Xb_cu] = freeBoundary(dt_cu);
% % % % tri_cu_save = tri_cu;
% % % % tr_cu = TriRep(tri_cu,Xb_cu);
% % % % P_cu = incenters(tr_cu);
% % % % fn_cu = faceNormals(tr_cu);
% % % % 
% % % % % TARGET PYRAMID
% % % % % pc2=GenPyramid;
% % % % % pc2.copyTrue2MeasPos;
% % % % % T_pyr= [50,250,0];
% % % % % R_pyr= [0,0,-pi/6];
% % % % % GenTM = TransformMatrix(R_pyr,T_pyr);
% % % % % pc2.transform(GenTM);
% % % % % dt_pyr = DelaunayTri(pc2.P(1,:)',pc2.P(2,:)');
% % % % % tri_pyr = dt_pyr(:,:);
% % % % % tri_pyr_save = tri_pyr;
% % % % % tr_pyr = TriRep(tri_pyr, pc2.P(1,:)',pc2.P(2,:)',pc2.P(3,:)');
% % % % % P_pyr = incenters(tr_pyr);
% % % % % fn_pyr = faceNormals(tr_pyr);
% % % % 
% % % % 
% % % % % COMBINATION OF MULTIPLE TARGETS
% % % % 
% % % % PCloud=PointCloud('',horzcat(P_plane'));
% % % % %Cloud.Normals= horzcat(fn_cu');
% % % % length_cld_2=length(pc.P);
% % % % pc.Add(pc2);
% % % % 
% % % % PCloud=PointCloud('',horzcat(P_plane',P_cu'));
% % % % Cloud.Normals= horzcat(fn_plane',fn_cu');
% % % % length_cld_3=length(pc.P);
% % % % %pc.Add(pc3);
% % % % Cloud=PointCloud('',pc.P);
% % % % % PCloud=PointCloud('',horzcat(P_plane',P_pyr',P_cu')); 
% % % % % Cloud.Normals= horzcat(fn_plane',fn_pyr',fn_cu'); 
% % % % % Cloud.Normals= horzcat(fn_plane');
% % % % 
% % % % 
% % % %
% % % % % % % % % % PCloud = Cloud;
% % % % % % % % % % pCloud = Cloud;
% % % % % % % % % % R_Cloud       = [0,0,-pi/10];                         %<-- Rx Ry Rz in radian
% % % % % % % % % % T_Cloud       = [0,0,0];                           %<-- Tx Ty Tz
% % % % % % % % % % GenTM = TransformMatrix(R_Cloud,T_Cloud);
% % % % % % % % % % PCloud.TrueP=[];
% % % % % % % % % % pCloud.P=[];
% % % % % % % % % % 
% % % % % % % % % % PCloud.transform(GenTM);
% % % % % % % % % % pCloud.transform(GenTM);
% % % % % % % % % % 
% % % % % % % % % % Cloud.P=PCloud.P;
% % % % PCloud.transform(GenTM);
% % % % Cloud.TrueP=PCloud.P;
% % % % 
% % % % Cloud.DT=vertcat(tri_plane_save);
% % % % Cloud.DT=vertcat(tri_plane_save,tri_cu_save+(length_cld_2)); 
% % % % % Cloud.DT=vertcat(tri_plane_save,tri_pyr_save+(length_cld_2),tri_cu_save+(length_cld_3)); 
% % % % 
% % % % tri_cloud = Cloud.DT;
% % % % tr_Cloud = TriRep(tri_cloud, Cloud.P(1,:)',Cloud.P(2,:)',Cloud.P(3,:)');
% % % % 
% % % % toc
% % % % 
% % % % clear fe P fn tr tri triRT dt pc2 pc GenTM PCloud
% % % % 
% % % % Cloud.ComputeKDTree;
% % % % 
% % % % % PLOT YOUR PARAMETRIC SURFACE
% % % % %
tri_cloud = DelaunayTri(Cloud.P(1,:)',Cloud.P(3,:)');
DT_cloud=tri_cloud(:,:);
tr_Cloud = TriRep(DT_cloud, Cloud.P(1,:)',Cloud.P(3,:)',Cloud.P(2,:)');
P_clo = incenters(tr_Cloud);
fn_clo = faceNormals(tr_Cloud);
figure;
trisurf(tr_Cloud, 'FaceColor', 'cyan', 'EdgeColor',[0.749 0 0.749],'FaceAlpha', 0.2); axis equal;
hold on;
Cloud.plot3;

quiver3(Cloud.P(1,:),Cloud.P(2,:),Cloud.P(3,:),Cloud.Normals(1,:),Cloud.Normals(2,:),Cloud.Normals(3,:),0.5, 'color','r');
hold off;
axis equal

end

