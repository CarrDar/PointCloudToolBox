% Epsilon function to compute epslion radius based on the gamma function
% approximation
function [Eps]=epsilon(RD,x,k)

% Function: [Eps]=epsilon(x,k)
%
% Aim:
% Analytical way of estimating neighborhood radius for DBSCAN
%
% Input:
% RD - vector with reachability distances (m,1)
% x - data matrix (m,n); m-objects, n-variables
% k - number of objects in a neighborhood of an object
% (minimal number of objects considered as a cluster)



[m,n]=size(x);

Eps=((prod(max(RD)-min(RD))*k*gamma(.5*n+1))/(m*sqrt(pi.^n))).^(1/n);


