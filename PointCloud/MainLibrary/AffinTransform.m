function [ Out ] = AffinTransform( In, AT )
% AFFINTRANSFORM Apply an affine transformation.
%
% AT : Affine transformation matrix 4 x 4 in the form
%        Rxx Rxy Rxz Tx
%        Ryx Ryy Ryz Ty
%        Rzx Rzy Rzz Tz
%        0   0   0   1
%   
%Case
%1) In  : a CloudPoint. Out : a CloudPoint.
%2) In  : a transform matrix in the same form as AT. 
%   Out : the transformed transform matrix.
%3) In  : a set of n 3D points in the form of a 3xn matrix.
%   Out : the transformed set of n 3D points.
   
%AUTHOR  : Neal Gauvin (at a3 dot epfl dot ch)
%VERSION : 2.5
%STATUS  : OK
%DATE    : 25 mai 2011, modified 1 june.

if isa(In,'PointCloud')
    Out = PointCloud(In.Name,In); % create a copy
    Out.transform(AT);
elseif isequal(size(In),[4 4])
    % a transform matrix
    Out = eye(4,4);
    Out(1:3,1:3) = AT(1:3,1:3)*In(1:3,1:3);
    Out(1:3,4) = AT(1:3,1:3)*In(1:3,4) + AT(1:3,4);
elseif size(In,1)==3
    % a set of 3D points
    Out = zeros(size(In));
    Out(1,:) = In(1,:)*AT(1,1)+In(2,:)*AT(1,2)+In(3,:)*AT(1,3)+AT(1,4);
    Out(2,:) = In(1,:)*AT(2,1)+In(2,:)*AT(2,2)+In(3,:)*AT(2,3)+AT(2,4);
    Out(3,:) = In(1,:)*AT(3,1)+In(2,:)*AT(3,2)+In(3,:)*AT(3,3)+AT(3,4);
else
    display('Please provide a PointCloud or a transform matrix')
end

end

