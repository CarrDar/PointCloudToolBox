function quaternion = Rot2Quat( R )
%ROT2QUAT Converts (orthogonal) rotation matrices R to (unit) quaternion representations.
%
% Input: A 3x3xn matrix of rotation matrices
%
% Output: A 4xn matrix of n corresponding quaternions
%
% The mathematics behind is explained <a href="matlab: 
% web('http://en.wikipedia.org/wiki/Rotation_matrix#Quaternion')">here</a>.
%
%AUTHOR  : Neal Gauvin (at a3 dot epfl dot ch)
%VERSION : 1.1
%STATUS  : OK
%DATE    : 9 juin 2011

Qxx = R(1,1,:);
Qxy = R(1,2,:);
Qxz = R(1,3,:);
Qyx = R(2,1,:);
Qyy = R(2,2,:);
Qyz = R(2,3,:);
Qzx = R(3,1,:);
Qzy = R(3,2,:);
Qzz = R(3,3,:);

w = 0.5 * sqrt(1+Qxx+Qyy+Qzz);
x = 0.5 * sign(Qzy-Qyz) .* sqrt(1+Qxx-Qyy-Qzz);
y = 0.5 * sign(Qxz-Qzx) .* sqrt(1-Qxx+Qyy-Qzz);
z = 0.5 * sign(Qyx-Qxy) .* sqrt(1-Qxx-Qyy+Qzz);

quaternion = reshape([w;x;y;z],4,[]);

end

