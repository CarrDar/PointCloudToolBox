function [class,type,SetOfSteepDownAreas,SetOfClusters,RD,CD,order,Eps]=dbscan_optics(x,k,Eps)
%function [class,type,RD,CD,order,Eps]=dbscan_optics(x,k,Eps)
% Function: [class,type,RD,CD,order,Eps]=dbscan_optics(x,k,Eps)
% -------------------------------------------------------------------------
% Aim:
% Clustering the data with Density-Based Scan Algorithm with Noise (DBSCAN)
% -------------------------------------------------------------------------
% Input:
% x - data set (m,n); m-objects, n-variables
% k - number of objects in a neighborhood of an object
% (minimal number of objects considered as a cluster)
% Eps - neighborhood radius, if not known avoid this parameter or put []
% -------------------------------------------------------------------------
% Output:
% class - vector specifying assignment of the i-th object to certain cluster (m,1)
% type - vector specifying type of the i-th object (core: 1, border: 0, outlier: -1)
% RD - vector with reachability distances (m,1)
% Eps - neighborhood radius, when automatically comupted
% order - vector specifying the order of objects (1,m)
% -------------------------------------------------------------------------
% Example of use:
% x=[randn(300,3)*.8;randn(400,3)*2+ones(400,3)*5];
% [class,type,RD,CD,order,Eps]=dbscan_optics(x,5,[]);
% figure;scatter3(x(:,1),x(:,2),x(:,3),10*(2+type(1,:)),class(1,:));
% figure;plot(RD(order));
% -------------------------------------------------------------------------
% References:
% [1] M. Ester, H. Kriegel, J. Sander, X. Xu, A density-based algorithm for
% discovering clusters in large spatial databases with noise, proc.
% 2nd Int. Conf. on Knowledge Discovery and Data Mining, Portland, OR, 1996,
% p. 226, available from:
% www.dbs.informatik.uni-muenchen.de/cgi-bin/papers?query=--CO
% [2] M. Daszykowski, B. Walczak, D. L. Massart, Looking for
% Natural Patterns in Data. Part 1: Density Based Approach,
% Chemom. Intell. Lab. Syst. 56 (2001) 83-92
% -------------------------------------------------------------------------
% Written by Alex Kendall University of Cambridge 18 Feb 2015 http://mi.eng.cam.ac.uk/~agk34/
% This software is licensed under GPLv3, see included glpv3.txt.
% Written by Michal Daszykowski
% Department of Chemometrics, Institute of Chemistry,
% The University of Silesia
% December 2004
% http://www.chemometria.us.edu.pl
%
% Updated part: dbscan_optics/dist
%               dbscan_optice/optics
%               dbscan_optics/epsilon
% Updated by: Dario dot Carrea at unil dot ch
% updated date: February 2015
[m,~]=size(x);
% Use OPTICS classification hierachical clustering structure for cluster
% analysis.
[RD,CD,order]=optics(x,k);

if nargin<3 || isempty(Eps)
    [Eps]=epsilon(RD,x,k);
end


mib = 0;
i = 1;
SetOfSteepDownAreas = struct();
SetOfClusters = struct();

while i < m-1
    mib = max([mib, RD(order(i))]);
    if RD(order(i))*(1-Eps) >= RD(order(i+1))

% update mib values and filter down areas
        for q=2:size(SetOfSteepDownAreas,2)
            SetOfSteepDownAreas(q).mib = max(RD(order((SetOfSteepDownAreas(q).end+1):i)));
        end
        q=2;
        while q<=size(SetOfSteepDownAreas,2)
            if RD(order(SetOfSteepDownAreas(q).start))*(1-Eps) < mib
                if q==size(SetOfSteepDownAreas,2)
                    SetOfSteepDownAreas = SetOfSteepDownAreas(1:q-1);
                else
                    SetOfSteepDownAreas = SetOfSteepDownAreas([1:q-1, q+1:size(SetOfSteepDownAreas,2)]);
                end
            else
                q = q+1;
            end
        end
        
        newD = size(SetOfSteepDownAreas,2)+1;
        SetOfSteepDownAreas(newD).start = i;
        SetOfSteepDownAreas(newD).mib = 0;
        
% find end of downward area
        while i < m-1
            if RD(order(i))*(1-Eps) >= RD(order(i+1))
                i = i+1;
            else
                j = i;
                while j < m-1
                    if or(j-i>k, RD(order(j)) < RD(order(j+1)))
         % if the downward area that isn't steep is longer than k, or no longer downward
                        j=-1;
                        break;
                    elseif RD(order(j))*(1-Eps) >= RD(order(j+1))
         % if it is a steepdownward area
                        break;
                    else
                        j = j+1;
                    end
                end
                
                if or(j == -1, j == m-1)
         % end of downward area
                    break;
                else
                    i = j;
                end
            end
        end
        
        SetOfSteepDownAreas(newD).end = i-1;
        mib = RD(order(i));
        
    elseif RD(order(i)) <= RD(order(i+1))*(1-Eps)
        % up area
        upAreaStart = i;
        % update mib values and filter down areas
        for q=2:size(SetOfSteepDownAreas,2)
            SetOfSteepDownAreas(q).mib = max(RD(order(SetOfSteepDownAreas(q).end:i)));
        end
        q=2;
        while q<=size(SetOfSteepDownAreas,2)
            if RD(order(SetOfSteepDownAreas(q).start))*(1-Eps) < mib
                if q==size(SetOfSteepDownAreas,2)
                    SetOfSteepDownAreas = SetOfSteepDownAreas(1:q-1);
                else
                    SetOfSteepDownAreas = SetOfSteepDownAreas([1:q-1, q+1:size(SetOfSteepDownAreas,2)]);
                end
            else
                q = q+1;
            end
        end
        
        
        % find end of upward area
        while i < m-1
            if RD(order(i)) <= RD(order(i+1))*(1-Eps)
                i = i+1;
            else
                j = i;
                while j < m-1
                    if or(j-i>k, RD(order(j)) > RD(order(j+1)))
                        % if the upward area that isn't steep is longer than k, or no longer upward
                        j=-1;
                        break;
                    elseif RD(order(j)) <= RD(order(j+1))*(1-Eps)
                        % if it is a steepdownward area
                        break;
                    else
                        j = j+1;
                    end
                end
                
                if or(j == -1, j== m-1)
                    % end of downward area
                    break;
                else
                    i = j;
                end
            end
        end
        
        mib = RD(order(i));
        
        for q=2:size(SetOfSteepDownAreas,2)
            if RD(order(i))*(1-Eps) > SetOfSteepDownAreas(q).mib
                if and(RD(order(SetOfSteepDownAreas(q).start)) >= RD(upAreaStart) , RD(order(SetOfSteepDownAreas(q).end)) <= RD(order(i)))
                    if abs(RD(order(SetOfSteepDownAreas(q).start))-RD(order(i))) <= Eps*max(RD(order(SetOfSteepDownAreas(q).start)),RD(order(i)))
                        % condition a
                        clusterStart = SetOfSteepDownAreas(q).start;
                        clusterEnd = i;
                    elseif RD(order(SetOfSteepDownAreas(q).start))*(1-Eps) > RD(order(i))
                        % condition b
                        tmp = abs(RD(SetOfSteepDownAreas(q).start:SetOfSteepDownAreas(q).end)-RD(order(i)));
                        [~, clusterStart] = min(tmp); %index of closest value
                        clusterStart = clusterStart+SetOfSteepDownAreas(q).start-1;
                        clusterEnd = i;
                    elseif RD(order(SetOfSteepDownAreas(q).start)) < RD(order(i))*(1-Eps)
                        % condition c
                        clusterStart = SetOfSteepDownAreas(q).start;
                        tmp = abs(RD(upAreaStart:i)-RD(order(SetOfSteepDownAreas(q).start)));
                        [~, clusterEnd] = min(tmp); %index of closest value
                        clusterEnd = clusterEnd+upAreaStart;
                    else
                        error('ERROR\n');
                    end
                    
                    if abs(clusterEnd - clusterStart) >= k
                        newD = size(SetOfClusters,2)+1;
                        SetOfClusters(newD).start = clusterStart;
                        SetOfClusters(newD).end = clusterEnd;
                    end
                end
            end
        end
        
    else
        i = i+1;
    end
end

SetOfClusters = SetOfClusters(2:size(SetOfClusters,2));

%end

X=([(1:m)' x]);     % inuput point cloud + one added column for point position
[m,n]=size(X);
type=zeros(1,m);    % define type point in this array
no=1;               % cluster number start at one
touched=zeros(m,1); % gpuArray.
class=zeros(1,m);   % define class cluster and noise in this array 

for i=1:m
    if touched(i)==0;
        ob=X(i,:);
        D=dist(ob(2:n),X(:,2:n));
        ind=find(D<=Eps);
        
        if length(ind)>1 && length(ind)<k+1
            type(i)=0;
            class(i)=0;
        end
        if length(ind)==1
            type(i)=-1;
            class(i)=-1;
            touched(i)=1;
        end
        
        if length(ind)>=k+1;
            type(i)=1;
            class(ind)=ones(length(ind),1)*max(no);
            
            while ~isempty(ind)
                ob=X(ind(1),:);
                touched(ind(1))=1;
                ind(1)=[];
                D=dist(ob(2:n),X(:,2:n));
                i1=find(D<=Eps);
                
                if length(i1)>1
                    class(i1)=no;
                    if length(i1)>=k+1;
                        type(ob(1))=1;
                    else
                        type(ob(1))=0;
                    end
                    
                    for i=1:length(i1)
                        if touched(i1(i))==0
                            touched(i1(i))=1;
                            ind=[ind i1(i)];
                            class(i1(i))=no;
                        end
                    end
                end
            end
            no=no+1;
        end
    end
end
i1=find(class==0);
class(i1)=-1;
type(i1)=-1;
end%