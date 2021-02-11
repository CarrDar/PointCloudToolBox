function [dip, direction] = Orientations(Dx,Dy,Dz,Disp)
%   ORIENTATION of the surface deformation for 3D point cloud.
%   [Dip, Direction] = Orientations(Dx,Dy,Dz,Disp)
%   Give the direction/dip for point according to displacement obtain by
%   Deformation or Shape tracking algorithm.
%
% INPUT: - Dx,Dy,Dz,Disp: correspond to displacement vector between two same
%          points
%
% OUTPUT: - Direction/Dip for each point according to the displacement.
%
% Remarks:
% - ORIENTATION works with libraries of PointCloudToolBox developed at
%   the Institute of Geomatics and Analysis of Risk - University of Lausanne
%   (IGAR-UNIL) by Neal Gauvin. All the PointCloudToolBox is propriety of
%   IGAR-UNIL.
% Update:
%   1.1: Angle Dip from 0° (top) to 180°(down)
%
%
% AUTHOR  : Dario Carrea (at unil dot ch)
% VERSION : 1.1
% STATUS  : OK
% DATE    : 21 January 2013
%

%% Find Dip (From 90° to -90°)

A = Dz./abs(Disp); % Normalized
phi = acos(A); %phi = lightness from 0 (down) to +pi (up)

% for 0° to 180°
dip = fix(((phi/(pi/180))-90)*-1);

% For 0° to 90°
%
% fin =length(Dz);
% for i=1:fin
%     if phi(i)> pi/2
%         Dhi(i)= phi(i)-pi/2;
%     else
%         Dhi(i)= phi(i);
%     end
% end
% dip = round(Dhi/(pi/180))';

%% Find Azimuth (From 1° to 360°)

theta =(atan2(Dy,Dx)./(pi/180));    % azimuth from -pi to +pi in counter-clockwise and 0° at East
azimuth_ =  360-(floor(theta));     % azimuth from 1° to 360° in    clockwise      and 0° at East  (=> 360° - XXX)
azimuth = (azimuth_+90)/360;        % azimuth from 1° to 360° in    clockwise      and 0° at North (=> XXX + 90° && -360° if > 360°)
for i=1:length(Dx)                  %   |
	X(i) = Correction(azimuth(i));  %   |
end                                 %   V
direction = round(X.*360);

function X = Correction(x)
    if x > 1
        X = (x-1);
    else
        X = x;
    end
end
end