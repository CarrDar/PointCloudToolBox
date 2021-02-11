function [RD,CD,order]=optics(x,k)
%%%%%%%%%%%%%%%%%%%%%%%% OPTICS: Ordering Points To Identify the Clustering Structure %%%%%%%%%%%%%%%%%%%%%%
% Input:
% x - data set (m,n); m-objects, n-variables
% k - number of objects in a neighborhood of an object
% -------------------------------------------------------------------------
% Output:
% RD - vector with reachability distances (1,m)
% CD - vector with core distances (1,m)
% order - vector specifying the order of objects (1,m)
% -------------------------------------------------------------------------
% References:
% [1] M. Ankrest, M. Breunig, H. Kriegel, J. Sander, 
% OPTICS: Ordering Points To Identify the Clustering Structure, 
% available from www.dbs.informatik.uni-muenchen.de/cgi-bin/papers?query=--CO
% [2] M. Daszykowski, B. Walczak, D. L. Massart, Looking for
% Natural Patterns in Data. Part 1: Density Based Approach,
% Chemom. Intell. Lab. Syst. 56 (2001) 83-92
% -------------------------------------------------------------------------
% Written by Michal Daszykowski
% -------------------------------------------------------------------------
%UPDATED BY AUTHOR  : Dario Carrea(at unil dot ch)
%VERSION : 1.0
%STATUS  : OK
%DATE    : 15 fevrier 2015
% -------------------------------------------------------------------------


[m,~]=size(x);
CD=zeros(1,m);
RD=ones(1,m)*10^10;

% Calculate Core Distances
for i=1:m
    D=sort(dist(x(i,:),x));
    CD(i)=D(k+1);
end

% Calculate Reachability Distances
order=[];
seeds=[1:m];
ind=1;

while ~isempty(seeds)
    ob=seeds(ind);
    seeds(ind)=[];
    order=[order ob];
    mm=max([ones(1,length(seeds))*CD(ob);dist(x(ob,:),x(seeds,:))]);
    ii=(RD(seeds))>mm;
    RD(seeds(ii))=mm(ii);
    [~, ind]=min(RD(seeds));
end

RD(1)=max(RD(2:m))+.1*max(RD(2:m));