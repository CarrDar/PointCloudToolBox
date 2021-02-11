%% TLS Pulse SImulator
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
%VERSION : 0.5
%STATUS  : OK
%DATE    : 03 march 2014

clear all
close all
clc
%%
for h=0:1:89
    %% TARGET MODEL
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
    
    R_Cloud       = [0,-deg2rad(90-h),0];                      %<-- Rx Ry Rz in radian
    T_Cloud       = [100,0,0];                                 %<-- Tx Ty Tz
    GenTM = TransformMatrix(R_Cloud,T_Cloud);
    Cloud.transform(GenTM);
    PCloud.transform(GenTM);
    Cloud.TrueP=PCloud.P;
    
    Cloud.DT = vertcat(tri_plane_save); 
    tri_cloud = Cloud.DT;
    tr_Cloud = TriRep(tri_cloud, Cloud.P(1,:)',Cloud.P(2,:)',Cloud.P(3,:)');

%     figure;
%     trisurf(tr_Cloud, 'FaceColor', 'cyan', 'EdgeColor',[0.749 0 0.749],'FaceAlpha', 0.2); axis equal;
%     hold on;
%     Cloud.plot3;
%     quiver3(Cloud.TrueP(1,:),Cloud.TrueP(2,:),Cloud.TrueP(3,:),Cloud.Normals(1,:),Cloud.Normals(2,:),Cloud.Normals(3,:),0.5, 'color','r');
%     hold off;
%     axis equal  

    clear fe P fn tr tri triRT dt pc GenTM PCloud
    
    %% SCANNER MODEL
    %%Shooting direction: in degree from 0° to 360° & Define TLS Position:
    % Input parameters
    
    TLSPos  = [0,0,0];                               %<-- TLS Position X Y Z cartesian
    
    azimut = 90.0;                                   %<-- in degree
    az= 360-azimut;
    rad_az= deg2rad(az);
    def_az= rad_az+(pi/2);
    if def_az>=(2*pi)
        def_az= def_az-(2*pi);
    end
    
    elevation = 0.0;                                   %<-- in degree
    elev= -elevation;
    rad_elev= deg2rad(elev);
    def_elev= rad_elev;
    
    [Ttx,Tty,Ttz]= sph2cart(def_az,def_elev,1);
    Tt=[Ttx;Tty;Ttz];
     
    %% SOURCE MODEL (PULSE MODEL)
    % Beam division into sub-beam in degree:
    
    beam_width= 2*T_Cloud(1,1).*tan((0.00025/2));       % beam divergence
    
    ang_step = atand((beam_width/2)/T_Cloud(1,1))/100;  %<-- in degree
    % Vertical angular step:
    phi = deg2rad(ang_step);                            %<-- in degree
    % Horizontal angular step:
    theta = deg2rad(ang_step);                          %<-- in degree
    
    % Beam extention:
    % Left side:
    leftXlim        =deg2rad(-atand((beam_width/2)/T_Cloud(1,1)));      %<-- in degree negative value
    % Right side:
    rightXlim       =deg2rad(atand((beam_width/2)/T_Cloud(1,1)));       %<-- in degree positive value
    % Up side:
    upYlim          =deg2rad(atand((beam_width/2)/T_Cloud(1,1)));       %<-- in degree positive value
    % Down side:
    downYlim        =deg2rad(-atand((beam_width/2)/T_Cloud(1,1)));      %<-- in degree negative value
      
    openingX                    =(rightXlim-leftXlim);
    openingY                    =(upYlim-downYlim);
    spacingX                    =round(openingX/phi);
    spacingY                    =round(openingY/theta);
    shooting_Win_theta          =zeros(spacingY,spacingX);
    shooting_Win_phi            =zeros(spacingY,spacingX);
    shooting_Win_theta_power    =zeros(spacingY,spacingX);
    shooting_Win_phi_power      =zeros(spacingY,spacingX);
    
    for i=1:spacingX
        shooting_Win_theta(:,i) = -(leftXlim+i*(theta+def_az));
    end
    for i=1:spacingY
        shooting_Win_phi(i,:)   = (upYlim-i*(phi-def_elev));
    end
    
    shooting_Win_range      = zeros(spacingY,spacingX);
    shooting_Win_size       =  zeros(spacingY,spacingX);
    shooting_Win_angle      = zeros(spacingY,spacingX);
    shooting_Win_inc_angle  = zeros(spacingY,spacingX);
    shooting_Win_time       = zeros(spacingY,spacingX);
    
    angle_limite    = atan((beam_width/2)/T_Cloud(1,1));
    
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
    
    for i=1:spacingX
        for j=1:spacingY
            shooting_Win_power(j,i) = 0.1.*exp(-(shooting_Win_theta_power(j,i)...   % keeping only the FWHM part of the energy pulse
                                      .^2+shooting_Win_phi_power(j,i).^2)/2);       % Full Width at Half Maximum of the pulse (Carlsson et al. 2001)
        end
    end
    
    clear leftXlim rightXlim upYlim downYlim openingX openingY
    
    %% PROPAGATION AND REFLECTION MODEL 
    
    % Triangle
    P(1:3,:)=Cloud.TrueP(1:3,:); % one point in triangle
    N(1:3,:)=Cloud.Normals(1:3,:); % orientation of  triangle normal
    
    % Ray to plane distance for sub-beam matrix
    for i=1:spacingX
        for j=1:spacingY
            
            b =  dot(N(1:3,1),La{j,i}');
            a = -dot(N(1:3,1),P(1:3,1));
            
            if abs(b) < 1*10^-7
                I(1,1:3)= [NaN NaN NaN];
            end
            
            sI = a / b;
            
            if sI < 0
                I(1,1:3)= [NaN NaN NaN];
            end
            
            I(1,1:3) = TLSPos + sI.* La{j,i};
            
            range                       = EuclDist((I)',TLSPos');
            shooting_Win_range(j,i)     = range;
            shooting_Win_inc_angle(j,i) = pi/2*(1-(abs(b)));
            shooting_Win_time(j,i)      = ((2.*shooting_Win_range(j,i))./299792458)*10^9;
            shooting_Win_angle(j,i)     = acos(dot(Tt,La{j,i}'));
            shooting_Win_size(j,i)      = shooting_Win_angle(j,i)./shooting_Win_range(j,i);
            
            if shooting_Win_angle(j,i)>angle_limite;
                shooting_Win_time(j,i)= NaN;
                shooting_Win_range(j,i)= NaN;
                shooting_Win_inc_angle(j,i)= NaN;
                shooting_Win_power(j,i)= NaN;
                shooting_Win_size(j,i)= NaN;
            end
        end
    end
       
    shooting_Win_pulse = (shooting_Win_power.*cos(shooting_Win_inc_angle))./power(shooting_Win_range,2); % to do: Add noise random gaussian of
                                                                                                         %        amplitude xx  (SNR Signal-Noise Ratio)
    [m,n]=size(shooting_Win_time);
    hist_data(1,:) = reshape(shooting_Win_time,n*m,1);
    hist_data(2,:) = reshape(shooting_Win_pulse,n*m,1);
    time=mode(hist_data(1,:));
    dist=((time*10^-9)*299792458)/2;
    
    hist_data=hist_data';                                   % transpose
    [~,G]=sort(hist_data);                                  % sort the data according to time
    result(:,1)=hist_data(G(:,1),1);                        %   ¦
    result(:,2)=hist_data(G(:,1),2);                        %   v
    min_bin=min(result(:,1));max_bin=max(result(:,1));
    BINM(1,2)=0;BINm(1,2)=0;
    BINM(1,1)=(ceil(max_bin*10^4))/10^4; BINm(1,1)=(floor(min_bin*10^4)/10^4); % 1 measure every 1 ps
    k=0.0001;
    largeur_bin_dif=k;
    interval_dif=BINm:largeur_bin_dif:BINM;
    nombre_bin_dif=length(interval_dif);
    
    [~,Gz]=sort(result);                                    % sort the data according to time
    results(:,1)=result(Gz(:,1),1);                        %   ¦
    results(:,2)=result(Gz(:,1),2);                        %   v
     
     interval=-5:1.0e-3:5;
     Pulse= 0.1*exp(-(interval.^2)/2); %Pulse=0.152.*lognpdf(interval,0.1);%
     Pulse(2,:)=-5:(10/(length(interval)-1)):5;
     resultss=conv(results(:,2),Pulse(1,:)','same');
     resultss(:,2)=results(:,1);    
    
    
    [Nn,C]=hist3(resultss,[nombre_bin_dif 300]);
    for g=1:300
        Mat(:,g)=C{1,2}(1,g).*Nn(:,g);
    end
    
    for k=1:nombre_bin_dif
        Mat_sum(k,1)= sum(Mat(k,:));
        Mat_sum(k,2)= C{1,1}(1,k);
        if Mat_sum(k,1)==0;
            Mat_sum(k,1)=NaN; 
        end        
    end
    
    Mat_sum=[fliplr(BINm);Mat_sum;fliplr(BINM)];
    Mat_sum_(:,1:2)=Mat_sum(~isnan(Mat_sum(:,1)),1:2);
    
%    mu = max(Mat_sum(:,1));
%    time = Mat_sum(Mat_sum(:,1)==mu,2);
    


    % Constant Fraction Discrimiination (CFD)
    % the following part could be replaced by
    % using the Constant Fraction Discrimination (CFD)

     [mu,locs] = findpeaks(Mat_sum_(:,1));
     time= Mat_sum_(locs,2);
     
    if size(time,1)>=2
        dist_=((time(end,1)*10^-9)*299792458)/2;
    else
        dist_=((time*10^-9)*299792458)/2;
    end
    int=single((1+(h)));
    results_100(1,int)= dist;
    results_100(2,int)= dist_;
    results_100(3,int)= h;
    
    drawnow;plot(Mat_sum_(:,2),Mat_sum_(:,1));
    grid;%xlim([667 667.20]);ylim([0 0.02]);
    xlabel('Nanoseconds'); ylabel('Returned power');title(sprintf('Incidance angle %1.2f',h));
    hold on;plot(time,mu,'k^','markerfacecolor',[1 0 0]);hold off;
    %clear check_I check_ok check_point check_range check_s check_t N dt Mat result results resultss XX YY hist_data mode Mat_sum Mat_sum_ TOTO
end
%%

figure(gcf);scatter(results_100(3,:),results_100(2,:),'b.');
hold on
plot(results_100(3,:),results_100(2,:),'b');
hold on
scatter(results_100(3,:),results_100(1,:),'g.');
hold on
plot(results_100(3,:),results_100(1,:),'g');
 