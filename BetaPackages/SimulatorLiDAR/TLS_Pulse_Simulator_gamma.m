%% TLS Pulse Simulator
%
%
% TLS Pulse Simulator is built to understand how the pulse is influenced by
% the incidence angle as function of different distance. The method is to
% divide a laser beam in several sub-beam. Each sub-beam has it own
% power. The range is measured for each sub-beam.
%
%
%
%AUTHOR  : Dario Carrea (at unil dot ch)
%VERSION : 1.5
%STATUS  : OK
%DATE    : 17 november 2014

clear all
close all
clc

%% SOURCE MODEL (PULSE MODEL)

% TLS Postition and shooting direction
TLSPos  = [0,0,0];                               %<-- TLS Position X Y Z cartesian
Tt=[1;0;0];

% IN SPACE DOMAIN
% Beam division into sub-beam in degree:

beam_width=sqrt(0.012^2+(0.00017*1)^2);  % beam divergence 0.00017 microrad @ 1 meter range for beam waist @ 0.006 => 0.012 (see Larsson et al. 2007 p.11)

ang_step = atand(beam_width/2)/100;                 %<-- in degree
% Vertical angular step:
phi = deg2rad(ang_step);                            %<-- in degree
% Horizontal angular step:
theta = deg2rad(ang_step);                          %<-- in degree

% Beam extention:
% Left side:
leftXlim        =deg2rad(-atand(beam_width/2));      %<-- in degree negative value
% Right side:
rightXlim       =deg2rad(atand(beam_width/2));       %<-- in degree positive value
% Up side:
upYlim          =deg2rad(atand(beam_width/2));       %<-- in degree positive value
% Down side:
downYlim        =deg2rad(-atand(beam_width/2));      %<-- in degree negative value

openingX                    =(rightXlim-leftXlim);
openingY                    =(upYlim-downYlim);
spacingX                    =round(openingX/phi);
spacingY                    =round(openingY/theta);
shooting_Win_theta          =zeros(spacingY,spacingX);
shooting_Win_phi            =zeros(spacingY,spacingX);
shooting_Win_theta_power    =zeros(spacingY,spacingX);
shooting_Win_phi_power      =zeros(spacingY,spacingX);

for i=1:spacingX
    shooting_Win_theta(:,i) = -(leftXlim+i*(theta));
end
for i=1:spacingY
    shooting_Win_phi(i,:)   = (upYlim-i*(phi));
end

shooting_Win_range      = zeros(spacingY,spacingX);
shooting_Win_size       =  zeros(spacingY,spacingX);
shooting_Win_angle      = zeros(spacingY,spacingX);
shooting_Win_inc_angle  = zeros(spacingY,spacingX);
shooting_Win_time       = zeros(spacingY,spacingX);

angle_limite    = atan((beam_width/2)/1);

for i=1:spacingX
    for j=1:spacingY
        [x,y,z]= sph2cart(shooting_Win_theta(j,i),shooting_Win_phi(j,i),1);
        La{j,i}= [x,y,z];
    end
end

for i=1:spacingX
    shooting_Win_theta_power(:,i)=-2+i*4/spacingX;
end
for i=1:spacingY
    shooting_Win_phi_power(i,:)= -2+i*4/spacingY;
end

% % GAUSSIAN
for i=1:spacingX
    for j=1:spacingY
        shooting_Win_pulse_transmitted(j,i) = 1.*exp(-(shooting_Win_theta_power(j,i)...   % Keeping only 1/e^2 = 86.5% of max power apmplitude (see Resshetyuk 2006 p.21)
            .^2+shooting_Win_phi_power(j,i).^2)*0.5);
    end
end

% UNIFORM   A modifier
% for i=1:spacingX
%     for j=1:spacingY
%         shooting_Win_power(j,i) = 0.1;
%     end
% end

%% IN TIME DOMAIN

lambda= 1064e-9; %[m] wavelength
c=299792458; %lightspeed in atmosphere [m/sec]
PRF = 10000; % Hz

Pulse_duration=5.0e-9; %[sec] see Larsson et al. 2007 p.14

% GAUSSIAN
% interval=-(Pulse_duration/2):1.0e-12:(Pulse_duration/2);
% Pulse = 1*exp(-(interval.^2/(2*Pulse_duration^2))*200);
% Pulse_func = @(x) exp(-(x.^2/(2.*1.6e3.^2)).*200);
% Pulse(2,:)=(-Pulse_duration/2:(Pulse_duration)/(length(interval)-1):Pulse_duration/2).*1.0e12;

% Q-SWITCH
interval=-(Pulse_duration/2):1.0e-12:(Pulse_duration/2);
Pulse = 1*exp(-(interval.^2/(2*Pulse_duration^2))*200);
tau=((length(Pulse(1,Pulse(1,:)>=0.5)))/2)/3.5;
interval=-0:1:3000;
clear Pulse;Pulse = horzcat(zeros(1,3000),((interval./tau).^2).*exp(-((interval)./tau)));
Pulse = Pulse/max(Pulse);
Pulse(2,:)=(-Pulse_duration/2:(Pulse_duration)/(2*(length(interval)-1)):Pulse_duration/2).*1.0e12;


% NOISE
% Add noise random gaussian of amplitude xx  (SNR Signal-Noise Ratio)
for i=1:length(Pulse(1,:))
    SNR(1,i)=random('norm',0,1.0e-4);
end
Pulse(1,:)=Pulse(1,:)+SNR;

% THRESHOLD
threshold=0.1;  % SNR

mu_threshold = (Pulse(1,Pulse(1,:)>=threshold));
time_threshold = Pulse(2,Pulse(1,:)== mu_threshold(1,1));

% MAXIMUM
mu_max = max(Pulse(1,:));
time_max = Pulse(2,Pulse(1,:)==mu_max);

% CENTER OF GRAVITY                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        OF GRAVITY
mu_cg = Pulse(1,Pulse(1,:)>=threshold);
time_cg =Pulse(2,Pulse(1,:)>=threshold);
% Aire = integral(Pulse_func,Pulse(2,1),Pulse(2,end));
% time_cg=Mx/Aire;
% mu_cg=My/Aire;
[Aire,Timec,Yc] = Polycenter(time_cg,mu_cg);

% ZERO CROSSING
shifting=round((length(Pulse(1,Pulse(1,:)>=0.5)))/2);
cfd_Pulse(1,:)=horzcat(-Pulse(1,shifting:end),zeros(1,shifting-1));
cfd_line= Pulse(1,:)+cfd_Pulse(1,:);
check_zero_cross=(cfd_line)>threshold;
mu_zc=Pulse(1,check_zero_cross);
time_zc=Pulse(2,check_zero_cross);

% CONSTANT FRACTION DISCRIMINANT (CFD) @ 50% Y0

Y0=0.5;
mu_cfd_=Pulse(1,check_zero_cross);
mu_cfd=mu_cfd_(1,1)*Y0;
time_cfd=Pulse(2,Pulse(1,:)>=mu_cfd);
mu_cfd=Pulse(1,Pulse(2,:)==time_cfd(1,1));

% PLOT
figure;plot(Pulse(2,:),Pulse(1,:),'DisplayName','Emitted Pulse');grid; ylim([-0.1 1]);xlim([-200 1500]);
xlabel('Time [picosec]'); ylabel('Emitted Intensity [a.u.]');title('Emitted Pulse - Starting Time');
hold on;
%plot(Pulse(2,:),cfd_line,'m','DisplayName','CFD Result');
%plot(Pulse(2,:),cfd_Pulse,'Color',[1 0.7 0.4],'DisplayName','Invert Delay');
plot(time_zc(1,1),mu_zc(1,1),'k^','markerfacecolor',[0 1 1],'DisplayName','Zero-crossing');
plot(time_max,mu_max,'k^','markerfacecolor',[1 0 0],'DisplayName','Max');
plot(Timec,Yc,'k^','markerfacecolor',[0 1 0],'DisplayName','Center of gravity');
plot(time_cfd(1,1),mu_cfd(1,1),'k^','markerfacecolor',[1 0 1],'DisplayName','CFD 50%');
%plot(time_threshold(1,1),mu_threshold(1,1),'k^','markerfacecolor',[0 0 1],'DisplayName','Threshold');
%plot(time_threshold(1,2),mu_threshold(1,1),'k^','markerfacecolor',[0 0 1],'DisplayName','Threshold');
legend show;
hold off;


%% FIRE!!!

figure;
for h=1:1:85
    %% TARGET MODEL
    
    % STEP
%     pc=GenPlane('Arg',[200,200,400,400]);
%     pc.copyTrue2MeasPos;
%     dt_plane = DelaunayTri(pc.P(1,:)',pc.P(2,:)');
%     tri_plane = dt_plane(:,:);
%     tri_plane_save = tri_plane;
%     tr_plane = TriRep(tri_plane, pc.P(1,:)',pc.P(2,:)',pc.P(3,:)');
%     P_plane = incenters(tr_plane);
%     fn_plane = faceNormals(tr_plane);
%     
%     pc2=GenCube([200 400 0.2],[0 0 0]);
%     pc2.copyTrue2MeasPos;
%     T_cube= [100,0,0.1];
%     R_cube= [0,0,0];
%     GenTM = TransformMatrix(R_cube,T_cube);
%     pc2.transform(GenTM);
%     pc2.ComputeDelaunayTriangulation;
%     dt_cu = pc2.DT;
%     [tri_cu,Xb_cu] = freeBoundary(dt_cu);
%     tri_cu_save = tri_cu;
%     tr_cu = TriRep(tri_cu,Xb_cu);
%     P_cu = incenters(tr_cu);
%     fn_cu = faceNormals(tr_cu);
%     
%     length_cld_2=length(pc.P);
%     pc.Add(pc2);
%     
%     PCloud=PointCloud('',horzcat(P_plane',P_cu'));
%     Cloud=pc;
%     Cloud.Normals= horzcat(fn_plane',fn_cu');
    
    
    % PLANE
        pc=GenPlane('Arg',[2000,2000,4000,4000]);
        pc.copyTrue2MeasPos;
        dt_plane = DelaunayTri(pc.P(1,:)',pc.P(2,:)');
        tri_plane = dt_plane(:,:);
        tri_plane_save = tri_plane;
        tr_plane = TriRep(tri_plane, pc.P(1,:)',pc.P(2,:)',pc.P(3,:)');
        P_plane = incenters(tr_plane);
        fn_plane = faceNormals(tr_plane);
    
        Cloud=PointCloud('',pc.P);
        PCloud=PointCloud('',horzcat(P_plane'));
        Cloud.Normals= horzcat(fn_plane');
    
    R_Cloud       = [0,-deg2rad(90-0),0];                    %<-- Rx Ry Rz in radian
    T_Cloud       = [h,0,0];                                 %<-- Tx Ty Tz
    GenTM = TransformMatrix(R_Cloud,T_Cloud);
    Cloud.transform(GenTM);
    PCloud.transform(GenTM);
    Cloud.TrueP=PCloud.P;
    
    % % TRIANGLE FACES
    %Cloud.DT=vertcat(tri_plane_save,tri_cu_save+(length_cld_2));
    Cloud.DT=vertcat(tri_plane_save);
    
    tri_cloud = Cloud.DT;
    tr_Cloud = TriRep(tri_cloud, Cloud.P(1,:)',Cloud.P(2,:)',Cloud.P(3,:)');
    
            drawnow;
            subplot(1,2,1);
            trisurf(tr_Cloud, 'FaceColor', 'cyan', 'EdgeColor',[0.749 0 0.749],'FaceAlpha', 0.2); axis equal;
            hold on;
            Cloud.plot3;
            quiver3(Cloud.TrueP(1,:),Cloud.TrueP(2,:),Cloud.TrueP(3,:),Cloud.Normals(1,:),Cloud.Normals(2,:),Cloud.Normals(3,:),0.5, 'color','r');
            plot(TLSPos,'*r');
            hold off;
            axis equal
    
    clear fe P fn tr tri triRT dt pc GenTM PCloud
    
    
    %% PROPAGATION AND REFLECTION MODEL
    
    % Material and device properties
    rho=1;      %Reflectance of target surface [0 1]
    D_aprture=0.051;    %Aperture diameter [m]  from spec. sheet sketch Optech (see \\nas.unil.ch\CALLISTO\Common\Admin\Informatique\Logiciels\Windows\Optech\Manuals_ILRIS_LR\ILRIS-LR dimensions.pdf)
    
    % % Find faces parallel to or back-facing  the ray
    check_point=false(1,size(Cloud.Normals,2));
    for H=1:size(Cloud.Normals,2)
        check_point(1,H) = -dot(Cloud.Normals(1:3,H),Tt)> 10^-7;
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
    
    for H=1:size(N,2)
        u(1:3,H)= Cloud.P(1:3,dt_(H,2))-Cloud.P(1:3,dt_(H,1)); %edge1 = vert1-vert0;
        v(1:3,H)= Cloud.P(1:3,dt_(H,3))-Cloud.P(1:3,dt_(H,1)); %edge2 = vert2-vert0;
        w0(1:3,H)= TLSPos'-Cloud.P(1:3,dt_(H,1));              % distance from vert0 to ray origin
        a(1:3,H) = -dot(N(1:3,H),w0(1:3,H));
        uu(1:3,H) = dot(u(1:3,H),u(1:3,H));
        uv(1:3,H) = dot(u(1:3,H),v(1:3,H));
        vv(1:3,H) = dot(v(1:3,H),v(1:3,H));
        D(1,H) = uv(1,H) .* uv(1,H) - uu(1,H) .* vv(1,H);
    end
    
    La=cell(spacingY,spacingX);
    
    for i=1:spacingX
        for j=1:spacingY
            [x,y,z]= sph2cart(shooting_Win_theta(j,i),shooting_Win_phi(j,i),1);
            La{j,i}=[x,y,z];
        end
    end
    
    % Ray to plane distance for sub-beam matrix
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
            for H=1:size(N,2)
                % Plücker coordinates
                b(1,H) = dot(N(1:3,H),La{j,i}');                                %%%%%%% Low line
                %end
                r(1,H) = a(1,H)./b(1,H);
                
                I(H,1:3) = TLSPos + r(1,H).* La{j,i};
                
                w(1:3,H) = I(H,1:3) - Cloud.P(1:3,dt_(H,1))';                   %%%%%%% Low line
                wu(1:3,H) = dot(w(1:3,H),u(1:3,H));                             %%%%%%% Low line
                wv(1:3,H) = dot(w(1:3,H),v(1:3,H));                             %%%%%%% Low line
                
                s(1,H) = (uv(1,H) .* wv(1,H) - vv(1,H) .* wu(1,H)) ./ D(1,H);
                t(1,H) = (uv(1,H) .* wu(1,H) - uu(1,H) .* wv(1,H)) ./ D(1,H);
                
                check_s(1,H)= s(1,H) >= 0.0 & s(1,H) <= 1.0;                    % I is outside Triangle
                check_t(1,H)= t(1,H) >= 0.0 & (s(1,H) + t(1,H)) <= 1.0;         % I is outside Triangle
                check_I(1,H) = check_s(1,H)==1 && check_t(1,H)==1;
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
                    shooting_Win_inc_angle(j,i)=pi/2*(1-(abs((b_(1,1)))));
                else
                    shooting_Win_inc_angle(j,i)=pi/2*(1-(abs((b_(1,check_range)))));
                end
            end
            
            if shooting_Win_inc_angle(j,i)>=89.9                          % limit angle for detection related to code stability
                shooting_Win_inc_angle(j,i)=NaN;
            end
            range=min(range_);
            shooting_Win_range(j,i)     = range;
            shooting_Win_time(j,i)      = ((2.*shooting_Win_range(j,i))./c)*10^12;
            shooting_Win_angle(j,i)     = acos(dot(Tt,La{j,i}'));
            shooting_Win_size(j,i)      = shooting_Win_angle(j,i)./shooting_Win_range(j,i);
            
            if shooting_Win_angle(j,i)>angle_limite;
                shooting_Win_time(j,i)= NaN;
                shooting_Win_range(j,i)= NaN;
                shooting_Win_inc_angle(j,i)= NaN;
                shooting_Win_pulse_transmitted(j,i)= NaN;
                shooting_Win_size(j,i)= NaN;
            end
        end
    end
    
    shooting_Win_pulse_received = (shooting_Win_pulse_transmitted.*cos(shooting_Win_inc_angle).*rho.*D_aprture)./((power(shooting_Win_range,2)));
    [m,n]=size(shooting_Win_time);
    hist_data(1,:) = reshape(shooting_Win_time,n*m,1);
    hist_data(2,:) = reshape(shooting_Win_pulse_received,n*m,1);
    hist_data=hist_data';                                   % transpose
    [~,G]=sort(hist_data);                                  % sort the data according to time
    result(:,1)=hist_data(G(:,1),1);                        %   ¦
    result(:,2)=hist_data(G(:,1),2);                        %   v
    results(:,1:2)=result(~isnan(result(:,1)),1:2);
    
    min_bin=min(results(:,1));%664000;%
    max_bin=max(results(:,1));%670000;%
    BINM(1,2)=0;BINm(1,2)=0;
    BINM(1,1)=ceil(max_bin); BINm(1,1)=floor(min_bin); % 1 measure every 1 ps
    results_=vertcat(BINm,results,BINM);
    
    [nn,ctrs] =hist3(results_,[BINM(1,1)-BINm(1,1) 100]);
    for k=1:size(nn,1)
        resultss_(k,1)  = sum(ctrs{1,2}(1,:).*nn(k,:));
        resultss_(k,2)  = ctrs{1, 1}(1,k);
    end
    
    if length(resultss_)<length(Pulse)
        equal_size=single(floor(length(Pulse)/2));
        resultss_a(:,1)=vertcat(zeros(equal_size,1),resultss_(:,1),zeros(equal_size,1));
        resultss(:,2)=conv(resultss_a(:,1),Pulse(1,:)','same');
        resultss(:,1)=vertcat([(min(resultss_(:,2))-1)-(length(Pulse)/2-1):1:min(resultss_(:,2))-1]',resultss_(:,2),...
            [max(resultss_(:,2))+1:1:max(resultss_(:,2))+1+(length(Pulse)/2-1)]');
    else
        resultss(:,2)=conv(resultss_(:,1),Pulse(1,:)','same');
        resultss(:,1)=resultss_(:,2);
    end
    
    %I_M_T(:,1+(h))=resultss(:,2);
    
    % NOISE
    %Add noise random gaussian of amplitude xx  (SNR Signal-Noise Ratio)
    %         for i=1:length(resultss(:,1))
    %             SNR(i,1)=random('norm',0,0.005);
    %         end
    %         resultss(:,2)=resultss(:,2)+SNR(:,1);
    
    % MEASUREMENT RANGE METHODS
    
    % THRESHOLD
    mu_ = resultss(resultss(:,2)>=threshold,2);
    if isempty(mu_)
        mu_=0;
        dist= NaN;
    else
        time_ = resultss(resultss(:,2)==mu_(1,1),1);
        if size(time_,1)>=2
            dist=(((time_(end,1)-time_threshold(end,1))*10^-12)*c)/2;
        else
            dist=(((time_-time_threshold(end,1))*10^-12)*c)/2;
        end
    end
    
    % MAXIMUM
    mu_1 = max(resultss(:,2));
    time_1 = resultss(resultss(:,2)==mu_1,1);
    
    if size(time_1,1)>=2
        dist_1=(((time_1(end,1)-time_max)*10^-12)*c)/2;
    else
        dist_1=(((time_1-time_max)*10^-12)*c)/2;
    end
    
    % CENTER OF GRAVITY                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         OF GRAVITY
    mu = resultss(resultss(:,2)>=threshold,2);
    if isempty(mu)
        mu=0;
        dist_2= NaN;
    else
        time = resultss(resultss(:,2)>=threshold,1);
        [Area,Timec_,Yc_]=Polycenter(time,mu);
        dist_2=(((Timec_-Timec)*10^-12)*c)/2;
    end
    
    % ZERO-CROSSING
    shifting=round((length(resultss(resultss(:,2)>=max(resultss(:,2)/2))))/2);
    cfd_resultss(:,1) = vertcat(-resultss(:,2),zeros(shifting,1));
    corrected_resultss(:,1) = vertcat([(min(resultss(:,1))-1)-(shifting-1):1:min(resultss(:,1))-1]',resultss(:,1));
    corrected_resultss(:,2) = vertcat(zeros(shifting,1),resultss(:,2));
    cfd_line_=corrected_resultss(:,2)+cfd_resultss(:,1);
    check_zero_cross = (cfd_line_)>threshold;
    
    mu_3 = corrected_resultss(check_zero_cross,2);
    time_3 = corrected_resultss(check_zero_cross,1);
    if size(time_3,1)>=2
        dist_3=(((time_3(1,1)-time_zc(1,1))*10^-12)*c)/2;
    else
        dist_3=(((time_3-time_zc(1,1))*10^-12)*c)/2;
    end
    
    % CONSTANT FRACTION DISCRIMINATOR (CFD) @ Y0 = 50%
    
    mu_4_ = corrected_resultss(check_zero_cross,2);
    mu_4=mu_4_(1,1)*Y0;
    time_4=resultss(resultss(:,2)>=mu_4,1);
    mu_4=resultss(resultss(:,1)>=time_4(1,1),2);
    
    if size(time_4,1)>=2
        dist_4=(((time_4(1,1)-time_cfd(end,1))*10^-12)*c)/2;
    else
        dist_4=(((time_4-time_cfd(end,1))*10^-12)*c)/2;
    end
    
    int=single((1+(h)));
    results_100(1,int)= dist;
    results_100(2,int)= dist_1;
    results_100(3,int)= dist_2;
    results_100(4,int)= dist_3;
    results_100(5,int)= dist_4;
    results_100(6,int)= h;
    
    drawnow;
    subplot(1,2,2);
    plot(resultss(:,1),resultss(:,2),'DisplayName','Recieved pulse');
    grid;ylim([-0.01 1]);%xlim([min_bin max_bin]);
    xlabel('Time [picoseconde]'); ylabel('Returned intensity [a.u.]');title(sprintf('Incidance angle %1.2f',h));
    hold on;
    %plot(corrected_resultss(:,1),cfd_line_,'m','DisplayName','CFD Result');
    %plot(corrected_resultss(:,1),cfd_resultss,'Color',[1 0.7 0.4],'DisplayName','Invert Delay');
    plot(time_1,mu_1,'k^','markerfacecolor',[1 0 0],'DisplayName','Max');
    plot(Timec_,Yc_,'k^','markerfacecolor',[0 1 0],'DisplayName','Center of gravity');
    plot(time_3(1,1),mu_3(1,1),'k^','markerfacecolor',[0 1 1],'DisplayName','Zero-crossing');
    plot(time_4(1,1),mu_4(1,1),'k^','markerfacecolor',[1 0 1],'DisplayName','CFD 50%');
    %plot(time_(1,1),mu_(1,1),'k^','markerfacecolor',[0 0 1],'DisplayName','Threshold');
    %plot(time_(1,end),mu_(1,end),'k^','markerfacecolor',[0 0 1],'DisplayName','Threshold');
    legend show;
    hold off;
    clear check_I check_ok check_point check_range check_s check_t N dt dt_ Mat result results resultss hist_data mode Mat_sum Mat_sum_ SNR resultss_ cfd_resultss corrected_resultss cfd_line_ mu_2 resultss_a
end
%%

figure;scatter(results_100(6,:),results_100(2,:),'r.');
hold on
plot(results_100(6,:),results_100(2,:),'r');
hold on
scatter(results_100(6,:),results_100(1,:),'b.');
hold on
plot(results_100(6,:),results_100(1,:),'b');
hold on
scatter(results_100(6,:),results_100(3,:),'g.');
hold on
plot(results_100(6,:),results_100(3,:),'g');
hold on
scatter(results_100(6,:),results_100(4,:),'c.');
hold on
plot(results_100(6,:),results_100(4,:),'c');
hold on
scatter(results_100(6,:),results_100(5,:),'m.');
hold on
plot(results_100(6,:),results_100(5,:),'m');

%% COMPLEX SURFACES

%[mu_,locs] = findpeaks(resultss(:,2));
%time_= resultss(locs,1);

range_10(1,:)=results_100(1,:)-results_100(1,1);
range_10(2,:)=results_100(2,:)-results_100(2,1);
range_10(3,:)=results_100(3,:)-results_100(3,1);
range_10(4,:)=results_100(4,:)-results_100(4,1);
range_10(5,:)=results_100(5,:)-results_100(5,1);
range_10(6,:)=90-results_100(6,:);

figure;scatter(range_10(6,:),range_10(2,:),'r.');
hold on
plot(range_10(6,:),range_10(2,:),'r');
hold on
scatter(range_10(6,:),range_10(1,:),'b.');
hold on
plot(range_10(6,:),range_10(1,:),'b');
hold on
scatter(range_10(6,:),range_10(3,:),'g.');
hold on
plot(range_10(6,:),range_10(3,:),'g');
hold on
scatter(range_10(6,:),range_10(4,:),'c.');
hold on
plot(range_10(6,:),range_10(4,:),'c');
hold on
scatter(range_10(6,:),range_10(5,:),'m.');
hold on
plot(range_10(6,:),range_10(5,:),'m');


X=range_10(6,:);
Y=range_10(3,:);

save('range_10','Y')

