function TM  = TransformMatrix( R, T, Sz, Sh )
%TRANSFORMMATRIX Given the rotation angles and a translation vector, provides a transformation matrix.
%
%Output :
%   TM : 4x4 transform matrix.
%
% Input : 
% R  : rotation angles in radians about the x, y, and z-axis, respectively.
% T  : translation vector along x,y,z.
% Sz : resizing vector along x,y,z.
% Sh : shearing vector [xy,xz,yx,yz,zx,zy].
%
% All input arguments are optional.
%
% Usage :
%   Translation only :
%   GenTM = TransformMatrix([],[0,0,50]);    
%   Rotation, translation and resize :
%   GenTM = TransformMatrix([pi/12,0,0],[0,0,50],[2 2 2]);    
%
%    
%AUTHOR  : Neal Gauvin (at a3 dot epfl dot ch)
%VERSION : 2.1
%STATUS  : OK
%DATE    : 26 mai 2011

%No sanity checks to save CPU...

% preallocation
TM = double(eye(4,4));

% buid the transformation matrix
if nargin > 0
    % Rotation
    if ~isempty(R), TM(1:3,1:3) = RotationMatrix( R(1), R(2), R(3) ); end
    if nargin > 1
        % Translation
        TM(1:3,4) = T';
        if nargin > 2
            % Resize
            TM = TM * [Sz(1) 0 0 0; 0 Sz(2) 0 0; 0 0 Sz(3) 0; 0 0 0 1];
            if nargin > 3
                % Shear
                TM = TM * [1 Sh(1) Sh(2) 0; Sh(3) 1 Sh(4) 0; Sh(5) Sh(6) 1 0; 0 0 0 1];
            end
        end
    end
end

end



