function [I_corr, sigma, Incidence_deg] = Correction_Intensity(PC,sigma_deg)
% Function to correct raw intensities from LiDAR point cloud. This
%   function use optimization function to correct intensity. The function
%   was originally design to correct "Raw 16bits intensity" coming from Optech ILRIS
%   device using Oren-Nayar BDRF (Oren & Nayar, 1994). Other intensities
%   coming from other devices could lead to unexpected results. The BDRF model is a
%   generalisation of the diffuse reflection. sigma is the std.dev. of the
%   slope surface roughness. Recommanded values are between 0° and 55°. For
%   more information have a look at following references.
%
%   INPUT:
%   - PC: class PointCloud object 
%   - Raw Intensity (For Optech devices "Raw 16bits intensity")
%   - sigma_deg: slope angle std. dev. in degree
%
%   OUTPUT:
%   - A relative corrected Intensity acccording to range, incidence
%   angle and roughness based on Oren-Nayar BDRF
%
%
% Reference :
% 
% Carrea, D., Abellán, A., Humair, F., Matasci, B., Derron, M.-H., Jaboyedoff, M., (2016)
%   Correction of terrestrial LiDAR intensity channel using Oren–Nayar reflectance model: An application to lithological differentiation.
%   ISPRS Journal of Photogrammetry and Remote Sensing 113, 17–29. doi:10.1016/j.isprsjprs.2015.12.004
% OREN M. & NAYAR S.K., (1994a) Generalization of Lambert's Reflectance Model,
%   ACM 21st Annual Conference on Computer Graphics and Interactive Techniques (SIGGRAPH), pp.239-246
% BRENT, R. P., (1973) Algorithms for Minimization without Derivatives, Prentice-Hall,
%   Englewood Cliffs, New Jersey.  
%
%
% Disclaimer :
% This function comes with no warranty whatsoever. The responsability is
% upon the user to test thoroughly that it yields results consistent with expectations.
% Please signal any bug encountered.
% Please feel free to adapt and distribute.
%
%AUTHOR  : Dario Carrea (at unil dot ch)
%VERSION : 2.0
%STATUS  : OK
%DATE    : 21 May 2015
%% Sanity Check
if ~isobject(PC)
    error('Only object class "PointCloud" are accepted');
elseif isempty(PC.Intensities)
    error('Please provide Intensities');
elseif isempty(PC.Normals)
    error('Please provide Normals');
elseif isempty(PC.TLSPos)
    error('Please provide Position of TLS');
end
% N.B.: The above section could be modified to work with arrays instead of "pointCloud" class object

%% Compute temporary variables

% Compute Range between points and scanner position
Range=EuclDist(PC.P,PC.TLSPos);

% Remove point with too high or null range
idx_range=Range<=30 | Range>4000;
Range(1,idx_range)=NaN;

% Compute Intensity
I_Raw=PC.Intensities;

% Compute Incidence angle
PC.NormalsOutTopo;
Vect=zeros(3,length(PC.P));UsefulVar=zeros(3,length(PC.P));Normals=zeros(3,length(PC.P));
Vect_n=PC.Normals;
Vect_p=PC.P; TLS=PC.TLSPos;
for i=1:length(PC.P)
    Vect(1:3,i)=TLS - Vect_p(1:3,i);
    UsefulVar(1:3,i)=Vect(1:3,i)/norm(Vect(1:3,i));
    Normals(1:3,i)=Vect_n(1:3,i)/norm(Vect_n(1:3,i));
end

Incidence(1,:)=(acos(dot(UsefulVar(1:3,:),Normals(1:3,:))));

Incidence_deg=round(rad2deg(Incidence));

% Remove point with too high inicdence angle
idx_ia= Incidence_deg(1,:)>85;
Incidence_deg(1,idx_ia)=NaN;

switch exist('sigma_deg','var')
    case 1
        if sigma_deg<0 || sigma_deg>55;
            error('Please provide a std dev. slope roughness value between 0 and 55 degree');
        end
        [I_corr, sigma]=Correction_sigma();
    case 0
        [I_corr, sigma]=Optimisateur_sigma();
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function [I_corr, sigma]=Correction_sigma()
        % INCIDENCE ANGLE CORRECTION MODEL according to Oren-Nayar BDRF
        sigma=deg2rad(sigma_deg);
        A= 1-0.5*(sigma^2/(sigma^2+0.33));
        B=0.45*(sigma^2/(sigma^2+0.09));
        attenuation=(cosd(Incidence_deg).*(A+B.*sind(Incidence_deg).*tand(Incidence_deg)));
        Corr_I_Inc=I_Raw./attenuation;
        % RANGE CORRECTION MODEL
        Range_Corr_fact=Range.^2;
        
        % APPLY CORRECTION
        Corr_I_Tot=Corr_I_Inc.*Range_Corr_fact;
        
        % Remove zero value
        idx_0=Corr_I_Tot==0;
        Corr_I_Tot(idx_0)=NaN;
        
        % Give Intensity to Point Cloud
        I_corr=single(round(Corr_I_Tot));
        
        sigma=rad2deg(sigma);

    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function [I_corr, sigma]=Optimisateur_sigma()
         % INCIDENCE ANGLE CORRECTION MODEL according to Oren-Nayar BDRF
         
        % OPTIMIZATION
        idx =Incidence_deg<10    & ~isnan(Incidence_deg);
        idx2= Incidence_deg<10  & ~isnan(Incidence_deg);
        idx3= Incidence_deg>=10 & Incidence_deg<20 & ~isnan(Incidence_deg);
        idx4= Incidence_deg>=20 & Incidence_deg<30 & ~isnan(Incidence_deg);
        idx5= Incidence_deg>=30 & Incidence_deg<40 & ~isnan(Incidence_deg);
        idx6= Incidence_deg>=40 & Incidence_deg<50 & ~isnan(Incidence_deg);
        idx7= Incidence_deg>=50 & Incidence_deg<60 & ~isnan(Incidence_deg);
        idx8= Incidence_deg>=60 & Incidence_deg<70 & ~isnan(Incidence_deg);
        idx9= Incidence_deg>=70 & ~isnan(Incidence_deg);

        find_sigma=@(sigma)abs(mean(single(round((I_Raw(idx)./(cosd(Incidence_deg(idx)).*((1-...
        0.5*(sigma^2/(sigma^2+0.33)))+(0.45*(sigma^2/(sigma^2+0.09))).*sind(Incidence_deg(idx)).*tand...
        (Incidence_deg(idx))))).*(Range(idx).^2))))...
        -(mean([...
        mean(single(round((I_Raw(idx2)./(cosd(Incidence_deg(idx2)).*((1-0.5*(sigma^2/(sigma^2+0.33)))+(0.45*(sigma^2/(sigma^2+0.09))).*sind...
        (Incidence_deg(idx2)).*tand(Incidence_deg(idx2))))).*(Range(idx2).^2)))),...
        mean(single(round((I_Raw(idx3)./(cosd(Incidence_deg(idx3)).*((1-0.5*(sigma^2/(sigma^2+0.33)))+(0.45*(sigma^2/(sigma^2+0.09))).*sind...
        (Incidence_deg(idx3)).*tand(Incidence_deg(idx3))))).*(Range(idx3).^2)))),...
        mean(single(round((I_Raw(idx4)./(cosd(Incidence_deg(idx4)).*((1-0.5*(sigma^2/(sigma^2+0.33)))+(0.45*(sigma^2/(sigma^2+0.09))).*sind...
        (Incidence_deg(idx4)).*tand(Incidence_deg(idx4))))).*(Range(idx4).^2)))),...
        mean(single(round((I_Raw(idx5)./(cosd(Incidence_deg(idx5)).*((1-0.5*(sigma^2/(sigma^2+0.33)))+(0.45*(sigma^2/(sigma^2+0.09))).*sind...
        (Incidence_deg(idx5)).*tand(Incidence_deg(idx5))))).*(Range(idx5).^2)))),...
        mean(single(round((I_Raw(idx6)./(cosd(Incidence_deg(idx6)).*((1-0.5*(sigma^2/(sigma^2+0.33)))+(0.45*(sigma^2/(sigma^2+0.09))).*sind...
        (Incidence_deg(idx6)).*tand(Incidence_deg(idx6))))).*(Range(idx6).^2)))),...
        mean(single(round((I_Raw(idx7)./(cosd(Incidence_deg(idx7)).*((1-0.5*(sigma^2/(sigma^2+0.33)))+(0.45*(sigma^2/(sigma^2+0.09))).*sind...
        (Incidence_deg(idx7)).*tand(Incidence_deg(idx7))))).*(Range(idx7).^2)))),...
        mean(single(round((I_Raw(idx8)./(cosd(Incidence_deg(idx8)).*((1-0.5*(sigma^2/(sigma^2+0.33)))+(0.45*(sigma^2/(sigma^2+0.09))).*sind...
        (Incidence_deg(idx8)).*tand(Incidence_deg(idx8))))).*(Range(idx8).^2)))),...
        mean(single(round((I_Raw(idx9)./(cosd(Incidence_deg(idx9)).*((1-0.5*(sigma^2/(sigma^2+0.33)))+(0.45*(sigma^2/(sigma^2+0.09))).*sind...
        (Incidence_deg(idx9)).*tand(Incidence_deg(idx9))))).*(Range(idx9).^2))))]...
        )));
        
        [sigma_, ~] = fminbnd(find_sigma,0,1);
        
        A= 1-0.5*(sigma_^2/(sigma_^2+0.33));
        B=0.45*(sigma_^2/(sigma_^2+0.09));
        attenuation=(cosd(Incidence_deg).*(A+B.*sind(Incidence_deg).*tand(Incidence_deg)));
        Corr_I_Inc=I_Raw./attenuation;
        
        % RANGE CORRECTION MODEL
        Range_Corr_fact=Range(1,:).^2;
        % APPLY CORRECTION
        Corr_I_Tot=Corr_I_Inc.*Range_Corr_fact;
        
        % Remove zero value
        idx_0=Corr_I_Tot==0;
        Corr_I_Tot(idx_0)=NaN;
        
        % Give Intensity to Point Cloud
        I_corr=single(round(Corr_I_Tot));

        sigma=round(rad2deg(sigma_));
    end
end


