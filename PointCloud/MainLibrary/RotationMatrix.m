function [ R ] = RotationMatrix( alpha, beta, gamma )
%ROTATIONMATRIX Given the rotation angles, provides a rotation matrix.
%
% Output :
% 3x3 Rotation matrix in the form
%        Rxx Rxy Rxz
%        Ryx Ryy Ryz
%        Rzx Rzy Rzz
%
% Input : 
%   alpha, beta and gamma in radians are the rotation angles about the x,
%   y, z-axis , respectively.
%    
%AUTHOR  : Neal Gauvin (at a3 dot epfl dot ch)
%VERSION : 1.0
%STATUS  : OK
%DATE    : 26 mai 2011

% preallocation
R = zeros(3,3);

% fill the matrix
cosa = cos(alpha); cosb = cos(beta); cosg = cos(gamma); 
sina = sin(alpha); sinb = sin(beta); sing = sin(gamma);

R(1,1) = cosg*cosb;
R(1,2) = -sing*cosa + cosg*sinb*sina;
R(1,3) = sing*sina + cosg*sinb*cosa;
R(2,1) = sing*cosb;
R(2,2) = cosg*cosa + sing*sinb*sina;
R(2,3) = -cosg*sina + sing*sinb*cosa;
R(3,1) = -sinb;
R(3,2) = cosb*sina;
R(3,3) = cosb*cosa;

end

