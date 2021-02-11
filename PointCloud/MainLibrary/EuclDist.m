function Z = EuclDist(X,Y)
% EUCLDIST Compute the euclidean distance between two vectors of 3D points.
%
% Input: X,Y 3xn matrices 
%
% Output : a 1xn array.
%
%AUTHOR  : Neal Gauvin (at a3 dot epfl dot ch)
%VERSION : 1.0
%STATUS  : OK
%DATE    : 30 mai 2011

Z = sqrt(power( X(1,:)-Y(1,:), 2) + power( X(2,:)-Y(2,:), 2) + power( X(3,:)-Y(3,:), 2));

end

