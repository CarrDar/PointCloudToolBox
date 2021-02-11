%function ScannerModel()
%% SCANNER MODEL
%%Create PointCloud Parameters
% % Input parameters
% Shooting direction: in degree from 0° to 360° and scanner position in space
% or if you want to introduce a shifting effect in your point cloud.

TLSPos  = [100,0,0];                               %<-- TLS Position X Y Z cartesian
R_TLS   = [90,0];                                %<-- Shooting direction Azimuth/elevation in dgree

if isempty(R_TLS)
    azimut = 90;                 %<-- in degree
else
    azimut =R_TLS(1,1);
end
az= 360-azimut;
rad_az= deg2rad(az);
def_az= rad_az+(pi/2);
if def_az>=(2*pi)
    def_az= def_az-(2*pi);
end

if isempty(R_TLS)
    elevation = 0;              %<-- in degree
else
    elevation =R_TLS(1,2);
end

elev= -elevation;
rad_elev= deg2rad(elev);
def_elev= rad_elev;

[Ttx,Tty,Ttz]= sph2cart(def_az,def_elev,1);
Tt=[Ttx;Tty;Ttz];
    
%% SOURCE MODEL
%%Angular step in degree:
%
ang_step =0.25;              %<-- in degree min value = 0.001146°
% Vertical angular step:
phi=deg2rad(ang_step);
% Horizontal angular step:
theta=deg2rad(ang_step);
%
% Scan window extention:
%
% % Mettre au maximum de -90 Left/Down à 90 Right/Up sinon ça merde....
%
% Left side:
leftXlim=deg2rad(-30);      %<-- in degree negative value
% Right side:
rightXlim=deg2rad(30);      %<-- in degree positive value
% Up side:
upYlim=deg2rad(30);         %<-- in degree negative value
% Down side:
downYlim=deg2rad(-30);      %<-- in degree positive value

% Let's start!!
openingX=(rightXlim-leftXlim);
openingY=(upYlim-downYlim);
spacingX=round(openingX/phi);
spacingY=round(openingY/theta);
shooting_Win_theta=zeros(spacingY,spacingX);
shooting_Win_phi=zeros(spacingY,spacingX);

for i=1:spacingX
    shooting_Win_theta(:,i)=-(leftXlim+i*theta+def_az);
end
for i=1:spacingY
    shooting_Win_phi(i,:)= (upYlim-i*phi-def_elev);
end



shooting_Win_range =zeros(spacingY,spacingX);
shooting_Win_inc_angle =zeros(spacingY,spacingX);

clear leftXlim rightXlim upYlim downYlim openingX openingY
%% Add simulated noise to source model
%
% Possible noise to simulate :
% - Gaussian noise in 3D scan acquisition. vector of 2 real, the sigma and
%   norm parameters ([0.01,1]) of the gaussian distribution.

% Add error on theta (gaussian)
m=0;
sig_theta= 8*10^-5;
Np = size(shooting_Win_theta);
shooting_Win_theta_err = random('Normal',m, sig_theta, Np);

shooting_Win_theta = shooting_Win_theta + shooting_Win_theta_err;

% Add error on phi (gaussian)
m=0;
sig_phi= 8*10^-5;
Np = size(shooting_Win_phi);
shooting_Win_phi_err = random('Normal',m, sig_phi, Np);

shooting_Win_phi = shooting_Win_phi + shooting_Win_phi_err;

clear m sig Np
%% PROPAGATION AND REFLECTION MODEL
%%Let's create the simulated point cloud

tic

check_point=false(1,size(Cloud.Normals,2));

% % Find faces parallel to or back-facing  the ray
for h=1:size(Cloud.Normals,2)
    check_point(1,h) = -dot(Cloud.Normals(1:3,h),Tt)> 10^-7;
end

% Triangle
P(1:3,:)=Cloud.TrueP(1:3,check_point); % one point in triangle
N(1:3,:)=Cloud.Normals(1:3,check_point); % orientation of  triangle normal 
dt_(:,1:3)=Cloud.DT(check_point',1:3);    % vertices of triangle


% Check if P_intersection is in triangle find vectors for two edges sharing vert0
    u=nan(3,size(N,2));
    v=nan(3,size(N,2));
    w0=nan(3,size(N,2));
    a=nan(3,size(N,2));
    uu=nan(3,size(N,2));
    uv=nan(3,size(N,2));
    vv=nan(3,size(N,2));
    D=nan(1,size(N,2));

parfor h=1:size(N,2)
    u(1:3,h)= Cloud.P(1:3,dt_(h,2))-Cloud.P(1:3,dt_(h,1)); %edge1 = vert1-vert0;         
    v(1:3,h)= Cloud.P(1:3,dt_(h,3))-Cloud.P(1:3,dt_(h,1)); %edge2 = vert2-vert0;
    w0(1:3,h)= TLSPos'-Cloud.P(1:3,dt_(h,1));              % distance from vert0 to ray origin
    a(1:3,h) = -dot(N(1:3,h),w0(1:3,h));
    uu(1:3,h) = dot(u(1:3,h),u(1:3,h));
    uv(1:3,h) = dot(u(1:3,h),v(1:3,h));
    vv(1:3,h) = dot(v(1:3,h),v(1:3,h));
    D(1,h) = uv(1,h) .* uv(1,h) - uu(1,h) .* vv(1,h);
end    

La=cell(spacingY,spacingX);

for i=1:spacingX
    for j=1:spacingY
        [x,y,z]= sph2cart(shooting_Win_theta(j,i),shooting_Win_phi(j,i),1);
        La{j,i}=[x,y,z];
    end
end

for i=1:spacingX
    for j=1:spacingY
        b=nan(1,size(N,2));
        r=nan(1,size(N,2)); 
        I=nan(size(N,2),3);
        w=nan(3,size(N,2)); 
        wu=nan(3,size(N,2));
        wv=nan(3,size(N,2));
        s=nan(1,size(N,2)); 
        t=nan(1,size(N,2)); 
        check_s=false(1,size(N,2)); 
        check_t=false(1,size(N,2)); 
        check_I=false(1,size(N,2)); 
        for h=1:size(N,2)
            % Plücker coordinates 
            b(1,h) = dot(N(1:3,h),La{j,i}');                                %%%%%%% Low line
        %end
            r(1,h) = a(1,h)./b(1,h);
            
            I(h,1:3) = TLSPos + r(1,h).* La{j,i};
            
            w(1:3,h) = I(h,1:3) - Cloud.P(1:3,dt_(h,1))';                   %%%%%%% Low line
            wu(1:3,h) = dot(w(1:3,h),u(1:3,h));                             %%%%%%% Low line
            wv(1:3,h) = dot(w(1:3,h),v(1:3,h));                             %%%%%%% Low line
            
            s(1,h) = (uv(1,h) .* wv(1,h) - vv(1,h) .* wu(1,h)) ./ D(1,h);
            t(1,h) = (uv(1,h) .* wu(1,h) - uu(1,h) .* wv(1,h)) ./ D(1,h);
            
            check_s(1,h)= s(1,h) >= 0.0 & s(1,h) <= 1.0;                    % I is outside Triangle
            check_t(1,h)= t(1,h) >= 0.0 & (s(1,h) + t(1,h)) <= 1.0;         % I is outside Triangle
            check_I(1,h) = check_s(1,h)==1 && check_t(1,h)==1;
        end

        I_=I(check_I(1,:),1:3);                                       % I is inside Triangle
        b_ =b(1,check_I(1,:));
        if isempty(I_);
            I_=[NaN,NaN,NaN];                                         % I no intresection with Triangle
        end
        if isempty(b_);
            b_=NaN;                                         
        end
        
        range_ = EuclDist((I_)',TLSPos');
        
        if ~isnan(range_)
            [~,check_range]=find(range_==min(min(range_)));
            if size(check_range,2)>=2
                shooting_Win_inc_angle(j,i)=90*(1-(abs((b_(1,1)))));
            else
                shooting_Win_inc_angle(j,i)=90*(1-(abs((b_(1,check_range)))));
            end
        end
        
        if shooting_Win_inc_angle(j,i)>=89                          % limit angle for detection related to code stability
            shooting_Win_inc_angle(j,i)=NaN;
        end
        range=min(range_);
        if range>= 2500
            range=0;
        end
        shooting_Win_range(j,i)=range;
        
    end
end
toc

%% Add error + error ppm range  (Gaussian noise function of range)
m=0;

for i=1:spacingX
    parfor j=1:spacingY
        if shooting_Win_range(j,i)>100
            sig = sqrt(power(shooting_Win_range(j,i)*5*10^-5,2)+ power(0.007,2));
        else
            sig = 0.007;
        end
        shooting_Win_range_err(j,i) = random('Normal',m,sig,1);
    end
end

%% Add error based on incidence and distance
%  Depend on the type of spatial pulse distribution

for i=1:spacingX
    parfor j=1:spacingY
        shooting_Win_inc_angle_range_err(j,i) = +(shooting_Win_range(j,i)*(0.8*(90-shooting_Win_inc_angle(j,i)).^(-0.958)-0.0107))/100; % fitting on error in percent
    end
end

%% Calculated range (dist. theoric + overall error)

shooting_Win_range = shooting_Win_range + shooting_Win_range_err + shooting_Win_inc_angle_range_err;


%% Convert into a XYZ point cloud 
[x,y,z] = sph2cart(shooting_Win_theta,shooting_Win_phi,shooting_Win_range);

X=reshape(x,1,(spacingY*spacingX));
Y=reshape(y,1,(spacingY*spacingX));
Z=reshape(z,1,(spacingY*spacingX));
Point(1,:)=X;
Point(2,:)=Y;
Point(3,:)=Z;

%% Plot results

[IDX, D] = knnsearch(Point',Point','NSMethod','kdtree','K',2);
figure;
scatter3(X,Y,Z,4,D(:,2),'filled');
hold on
scatter3(0,0,0,'r*');
hold off
axis equal

figure;imagesc(shooting_Win_range);

SimCloud=PointCloud('',Point);
SimCloud.RemoveNans;
SimCloud.PlotPCLViewer('P');

clear x y z X Y Z;
%% SAVE POINT CLOUD
SaveInASCII(SimCloud,'D:\Dario_unil\Thèse\Modelisation\Simulator_LiDAR\Point_cloud\SimCloud_dstpts03_400.txt');

