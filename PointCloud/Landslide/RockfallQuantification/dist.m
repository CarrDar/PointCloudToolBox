% Euclidean dist function to compute distance between point on all points
% in the point cloud matrix x
function [D]=dist(i,x)

% function: [D]=dist(i,x)
%
% Aim:
% Calculates the Euclidean distances between the i-th point and all points
% in PC
%
% Input:
% i - an object (1,n)
% x - data matrix (m,n); m-objects, n-variables
%
% Output:
% D - Euclidean distance (m,1)

[m,n]=size(x);

D=EuclDist(i',x');
if n==1
   D=abs((ones(m,1)*i-x))';
end
