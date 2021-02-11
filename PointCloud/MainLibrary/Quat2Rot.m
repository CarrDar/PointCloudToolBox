function R = Quat2Rot( quaternion )
%QUAT2ROT Convert (unit) quaternion representations to (orthogonal) rotation matrices R.
% 
% Input: A 4xn matrix of n quaternions
% Output: A 3x3xn matrix of corresponding rotation matrices
%
% The mathematics behind is explained <a href="matlab: 
% web('http://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#From_a_quaternion_to_an_orthogonal_matrix')">here</a>.
%
%AUTHOR  : Neal Gauvin (at a3 dot epfl dot ch)
%VERSION : 1.1
%STATUS  : OK
%DATE    : 9 juin 2011

q0(1,1,:) = quaternion(1,:);
qx(1,1,:) = quaternion(2,:);
qy(1,1,:) = quaternion(3,:);
qz(1,1,:) = quaternion(4,:);

R = [q0.^2+qx.^2-qy.^2-qz.^2 2*qx.*qy-2*q0.*qz 2*qx.*qz+2*q0.*qy;
     2*qx.*qy+2*q0.*qz q0.^2-qx.^2+qy.^2-qz.^2 2*qy.*qz-2*q0.*qx;
     2*qx.*qz-2*q0.*qy 2*qy.*qz+2*q0.*qx q0.^2-qx.^2-qy.^2+qz.^2];

% Alternative formula:
% http://en.wikipedia.org/wiki/Rotation_matrix#Quaternion
%  R = [1-2*qy^2-2*qz^2 2*qx*qy-2*q0*qz 2*qx*qz+2*q0*qy;
%      2*qx*qy+2*q0*qz 1-2*qx^2-2*qz^2 2*qy*qz-2*q0*qx;
%      2*qx*qz-2*q0*qy 2*qy*qz+2*q0*qx 1-2*qx^2-2*qy^2];
end

