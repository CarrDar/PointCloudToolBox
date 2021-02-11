%% TARGET MODEL
% Creat Parametric surface

% % % % % TARGET DEM
function CP = Gen_Surface(range,angle,width,length)

pCloud=Createsurface(width,length);
PCloud=PointCloud;
DT=pCloud.DT;
PCloud.P=pCloud.TrueP;
pCloud.TrueP=[];


if isempty(angle)
    R_Cloud       = [0,-pi/2,0];                         %<-- Rx Ry Rz in radian
else
    R_Cloud       = [0,angle,0];                         %<-- Rx Ry Rz in radian
end
if isempty(range)
    T_Cloud       = [500,0,0];                           %<-- Tx Ty Tz
else
    T_Cloud       = [range,0,0];                           %<-- Tx Ty Tz
end

GenTM = TransformMatrix(R_Cloud,T_Cloud);
pCloud.transform(GenTM);
PCloud.transform(GenTM);
pCloud.TrueP=PCloud.P;
pCloud.DT=DT;
pCloud.ComputeKDTree;

% % figure;
% % trisurf(pCloud.DT, 'FaceColor', 'cyan', 'EdgeColor',[0.749 0 0.749],'FaceAlpha', 0.2); axis equal;
% % hold on;
% % Cloud.plot3;
% % quiver3(pCloud.TrueP(1,:),pCloud.TrueP(2,:),pCloud.TrueP(3,:),pCloud.Normals(1,:),pCloud.Normals(2,:),pCloud.Normals(3,:),0.5, 'color','r');
% % hold off;
% % axis equal

CP=pCloud;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% .................................................................................
%                                       SURFACE GENERATOR
%                                           2014.04.02
% ...............................................................................
%
function surface_param=Createsurface(xsize,ysize)

% IM=zeros(xsize,ysize);
% background=[0 0 0;0 ysize 0;xsize 0 0;ysize xsize 0]';

% define number of SQUARES
NUM=single(round(random('normal', 100,5)*2)); % define number of target here

% Create upper left coordinates
Corner_X=round(random('unif',0,300,NUM,1));
Corner_Y=round(random('unif',0,300,NUM,1));

% Create square size
Size_X=abs(round(random('Gamma',10,2,NUM,1).^2));
Size_Y=abs(round(random('Gamma',10,2,NUM,1).^2));
Size_Z=abs(round(random('Gamma',10,2,NUM,1)));

% create IMAGE datasets with all the squares
Size_X=sort(Size_X); Size_Y=sort(Size_Y); Size_Z=sort(Size_Z);

for i=1:NUM
    pc=GenCube([Size_X(i)*0.1 Size_Y(i)*0.1 Size_Z(i)*0.5],[Corner_X(i)+(Size_X(i)/2)*0.1 Corner_Y(i)+(Size_Y(i)/2)*0.1 (Size_Z(i)*0.5)/2]);
    pc.copyTrue2MeasPos;pc.TrueP=[];
    pc.ComputeDelaunayTriangulation;
    dt_cu = pc.DT;
    [tri_cu,Xb_cu] = freeBoundary(dt_cu);
    tri_cu_save = tri_cu;
    tr_cu = TriRep(tri_cu,Xb_cu);
    P_cu = incenters(tr_cu);
    fn_cu = faceNormals(tr_cu);
    if i==1
        pc1=GenPlane('Arg',[xsize,ysize,2*xsize,2*ysize]);
        pc1.copyTrue2MeasPos;pc1.TrueP=[];
        dt_plane = DelaunayTri(pc.P(1,:)',pc.P(2,:)');
        tri_plane = dt_plane(:,:);
        tr_plane = TriRep(tri_plane, pc.P(1,:)',pc.P(2,:)',pc.P(3,:)');
        P_plane = incenters(tr_plane);
        fn_plane = faceNormals(tr_plane);
        P = horzcat(pc1.P,pc.P);
        Normals = horzcat(fn_plane',fn_cu');
        TrueP= horzcat(P_plane',P_cu');
        DT_= vertcat(tri_plane,tri_cu+size(pc1.P,2));
    else
        P = horzcat(P,pc.P);
        Normals= horzcat(Normals,fn_cu');
        TrueP = horzcat(TrueP,P_cu');
        DT_= vertcat(DT_,(tri_cu_save+size(P,2)));
    end

end
pc1=PointCloud;

pc1.P=P;
pc1.TrueP=TrueP(:,1:end-12);
pc1.Normals=Normals(:,1:end-12);
pc1.DT=DT_(1:end-12,:);
pc1.plot3;
surface_param=pc1;

end











