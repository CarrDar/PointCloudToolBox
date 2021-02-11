
% -------------------------------------------------------------------------
% Function: 
% [RD,CD,order]=optics(x,k,Eps)
% -------------------------------------------------------------------------
% Aim: 
% Ordering objects of a data set to obtain the clustering structure 
% -------------------------------------------------------------------------
% Input: 
% x - data set (m,n); m-objects, n-variables
% k - number of objects in a neighborhood of the selected object
% (minimal number of objects considered as a cluster)
% Eps - neighborhood radius, if not known avoid this parameter or put []
% -------------------------------------------------------------------------
% Output: 
% RD - vector with reachability distances (m,1)
% CD - vector with core distances (m,1)
% order - vector specifying the order of objects (1,m)
% class - vector specifying assignment of the i-th object to certain
% cluster (m,1)
% type - vector specifying type of the i-th object
% (core: 1, border: 0, outlier: -1)
% -------------------------------------------------------------------------
% Example of use:
% x=[randn(30,2)*.4;randn(40,2)*.5+ones(40,1)*[4 4]];
% [RD,CD,order]=optics(x,4)
% clusteringfigs('Dbscan',x,[1 2],class,type)
% -------------------------------------------------------------------------
% References: 
% [1] M. Ankrest, M. Breunig, H. Kriegel, J. Sander, 
% OPTICS: Ordering Points To Identify the Clustering Structure, 
% available from www.dbs.informatik.uni-muenchen.de/cgi-bin/papers?query=--CO
% [2] M. Ester, H. Kriegel, J. Sander, X. Xu, A density-based algorithm for
% discovering clusters in large spatial databases with noise, proc.
% 2nd Int. Conf. on Knowledge Discovery and Data Mining, Portland, OR, 1996,
% p. 226, available from:
% www.dbs.informatik.uni-muenchen.de/cgi-bin/papers?query=--CO
% [3] M. Daszykowski, B. Walczak, D.L. Massart, Looking for natural  
% patterns in analytical data. Part 2. Tracing local density 
% with OPTICS, J. Chem. Inf. Comput. Sci. 42 (2002) 500-507
% -------------------------------------------------------------------------
% Written by Michal Daszykowski
% Department of Chemometrics, Institute of Chemistry, 
% The University of Silesia
% December 2004
% http://www.chemometria.us.edu.pl
%
% Updated part: dbscan/dist
% Updated by: Dario dot Carrea at unil dot ch
% updated date: February 2013

function [RD,CD,order]=optics(x,k)

[m,~]=size(x);

% if nargin<3 || isempty(Eps)%nargin<3 | isempty(Eps) %
%    [Eps]=epsilon(x,k);
% end

CD=zeros(1,m);
RD=ones(1,m)*10^10;

x=([[1:m]' x]);     %gpuArray.
[m,n]=size(x);
type=zeros(1,m);    %gpuArray.
no=1;
touched=zeros(m,1); %gpuArray.
class=zeros(1,m);   %gpuArray.

% Calculate Core Distances
for i=1:m	
    D=sort(dist(x(i,:),x));
    CD(i)=D(k+1);  
end

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
    [i1 ind]=min(RD(seeds));
end   

RD(1)=max(RD(2:m))+.1*max(RD(2:m));

%...........................................
% function [Eps]=epsilon(x,k)
% 
% % Function: [Eps]=epsilon(x,k)
% %
% % Aim:
% % Analytical way of estimating neighborhood radius for DBSCAN
% %
% % Input:
% % x - data matrix (m,n); m-objects, n-variables
% % k - number of objects in a neighborhood of an object
% % (minimal number of objects considered as a cluster)
% 
% 
% 
% [m,n]=size(x);
% 
% Eps=((prod(max(x)-min(x))*k*gamma(.5*n+1))/(m*sqrt(pi.^n))).^(1/n);

%...........................................
function [D]=dist(i,x)

% function: [D]=dist(i,x)
%
% Aim: 
% Calculates the Euclidean distances between the i-th object and all objects in x	 
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