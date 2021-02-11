function [R,G,B] = SDC(Dx,Dy,Dz,Disp,Limite_max,Limite_min,rotation,ouverture)
%SDF Surface Deforamtion Color for 3D point cloud.
%   V [R,G,B] = SDC(Dx,Dy,Dz,Disp,Limite_max,Limite_min,rotation,ouverture)
%   Give the color (RGB) for point according to displacement obtain by
%   Deformation or Shape tracking algorithm. The representation is
%   according to a spherical distribution of the color. It is possible to
%   rotate the sphere color in function of the order you put Dx, Dy, Dz in
%   the SDF function.
%
% INPUT: - Dx,Dy,Dz,Disp: correspond to displacement vector between two same
%          points
%        - Limite_max: is the max length of displacement used for saturation
%        - Limite_max: is the min length of displacement considered as a
%          displacement
%        - rotation: if you want to modified the orientation the colorsphere.
%          clockwise rotation according to red color for reference.
%        - ouverture: corrispond to openning angle between which you have
%          all the color
%
% OUTPUT: - Color RGB [0 255] for each point according to the displacement.
%
% Remarks:
% - SDF works with libraries of PointCloudToolBox developed at
%   the Institute of Geomatics and Analysis of Risk - University of Lausanne
%   (IGAR-UNIL) by Neal Gauvin. All the PointCloudToolBox is propriety of
%   IGAR-UNIL.
% Update:
%   1.1: Possibility to rotate the colorsphere  
%   2.0: Fast computing with parallel computing
%   Up coming: Possibility to vary the openning with beta angle
%
%
% AUTHOR  : Dario Carrea (at unil dot ch)
% VERSION : 2.0
% STATUS  : OK
% DATE    : 12 January 2013


% Dx,Dy,Dz,Disp are supposed to have the same length
% If don't, check problem!

[a,fin]= size(Dx);



% Find Saturation    From 0 to 1
for i=1:fin
    if abs(Disp(1,i))< Limite_min
        disp(i,1)=0;
    else
        disp(i,1)=Disp(1,i);
    end
end

fullsaturation=max(abs(disp));                  %
if fullsaturation > Limite_max                  %
    fullsaturation=Limite_max;                  %
    Saturation(:,1)= abs(disp)/fullsaturation;  % Normalized [0 1]
    for i=1:fin                              %
        if Saturation(i,1)> 1                   %
            Saturation(i,1)=1;                  %
        else                                    %
            Saturation(i,1)=(Saturation(i,1));  %
        end
    end
else
    Saturation = abs(disp)/fullsaturation;
end


% Find Lightness    From 0 to 1

lightness = -Dz./abs(Disp);
phi = acos(lightness); %phi = lightness from 0 (up) to +pi (down)
Lightness = phi/pi; %Lightness= azimuth from 1° to 180° to [0 1]


% Find Hue From 1° to 360°

theta =(atan2(-Dx,-Dy))./(pi/180); %theta = azimuth from -pi to +pi in counter-clockwise
Hue = 360 - (floor((theta)+180)); %Hue = azimuth from 1° to 360°  in clockwise (=> 360° - XXX)

% HSL:
HSL(:,1)= Hue; HSL(:,2)= Saturation; HSL(:,3)= Lightness;

% Following code is based on the tutorial found at:
% http://colorgrader.net/index.php/dictionary-a-tutorials/color-theory/...
% ...93-math-behind-colorspace-conversions-rgb-hsl.html
% written by © 2011 Nikolai Waldman
%
% //Input value of HSL from 0 to 1
% //Output results of RGB results given from 0 to 255
%
H = HSL(:,1)/360;   %[0 1]
S = HSL(:,2);       %[0 1]
I = HSL(:,3);       %[0 1]



if isempty(rotation)
    alpha = 0;
else
    alpha = rotation/360;
end

if isempty(ouverture) % Do not work for the moment!!! 
    beta = 1/3;
else
    beta = 1/3;
end

temporary_R = H + 1/3 + alpha; %Centered on red color and red on North (=> 1/3 for R G B)
temporary_G = H + 1/3 + beta + alpha ; %  For alpha clockwise rotation with 
temporary_B = H + 1/3 - beta + alpha ; %  color as follow: red -> green -> blue -> red

for i=1:fin
    if  I(i) < 0.5
        var_2(i) = I(i) * ( 1 + S(i) ); % Lower Hemisphere
    else
        var_2(i) = ( I(i) + S(i) ) - ( S(i) * I(i) ); % Upper Hemisphere
    end    
    var_1(i) = 2 * I(i) - var_2(i); % Inside Sphere 
end
for i=1:fin
    if S(i) == 0
        r(i) = 0.5;
        g(i) = 0.5;
        b(i) = 0.5;
    end
    r(i)= (Hue_2_RGB( var_1(i), var_2(i), temporary_R(i)));%% \
    g(i)= (Hue_2_RGB( var_1(i), var_2(i), temporary_G(i)));%%  |- With white up
    b(i)= (Hue_2_RGB( var_1(i), var_2(i), temporary_B(i)));%% /   and black down
end
R = (round(r*255))';
G = (round(g*255))';
B = (round(b*255))';

end

%--------------------------------------------------------------------------
function value = Hue_2_RGB(v1,v2,vH)
% Hue_2_RGB Convert Hue in color and combine them with ligthness and
% saturation.

% First loop used for normalization 3 color zones [0 1]
if vH < 0
    VH = (vH+1);
elseif vH > 1
    VH = (vH-1);
else
    VH=vH;
end

% Second loop used for color computation for evrey Hue [0 1]
if (6 * VH) < 1                                     % Decreasing color
    value = v1 + ( v2 - v1 ) * 6 * VH;              % /
elseif ( 2 * VH ) < 1                               % Zero color
    value= v2;                                      % /
elseif ( 3 * VH ) < 2                               % Increasing color
    value= v1 + ( v2 - v1 ) * ( ( 2/3 ) - VH ) * 6; % /
else                                                %Full color
    value= v1;                                      % /
end
end