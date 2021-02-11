function [Rockfalls,Eps] = RockfallSegment(PCRockfall,k,Eps,fig)
% %%%%%%%%%%%%%%%%%%%%%%%% IDENTIFICATION & SEPARATION OF THE DIFFERENTS BLOCKS %%%%%%%%%%%%%%%%%%%%%%
%   Method applied to rockfall segmentation
%   by dario dot carrea at unil dot ch
%
%
% Input:
% PCRockfall - dataset with multiple rockfall events to segment
% k - number of objects in a neighborhood of an object
% (minimal number of point considered as a single rockfall event)
% Eps - neighborhood radius, if not known avoid this parameter or put []
% fig - if you want to plot the results (1) else [] 
% -------------------------------------------------------------------------
% Output:
% Rockfalls - a class point cloud with each rockfall event individualized
% Eps - neighborhood radius automatically defined
% -------------------------------------------------------------------------

%   Sanity check of the number of objects in a neighborhood of an object

if ~isempty(k) && k <= 0 || floor(k)~=k
    error('RockfallSegment:searchRadius_Cluster','Number of objects in a neighborhood k must be positive integer!');

end
if isempty(k)
    error('RockfallSegment:searchRadius_Cluster','You need at least a value for variable k!');
end
%   Sanity check of the neighborhood radius
if ~isempty(Eps) && Eps <= 0
    error('RockfallSegment:searchNeighbor_Cluster','Neighborhood radius "epsilon" must be positive!');
end


[class,type,RD,CD,order,Eps]=dbscan_optics((PCRockfall.P)',k,Eps);

PCRockfall.UsefulVar(1,:)=class;
PCRockfall.UsefulVar(2,:)=type;
PCRockfall.UsefulVar(3,:)=RD;
PCRockfall.UsefulVar(4,:)=CD;
PCRockfall.UsefulVar(5,:)=order;

if fig == 1
    idx=(PCRockfall.UsefulVar(1,:)>0)';
    X= PCRockfall.P(1,:)';
    Y= PCRockfall.P(2,:)';
    Z= PCRockfall.P(3,:)';
    V=PCRockfall.UsefulVar(1,:)';
    T=PCRockfall.UsefulVar(2,:)';
    subplot(2,1,1);
    scatter3(X(idx),Y(idx),Z(idx),6,V(idx),'filled');
    axis equal;
    subplot(2,1,2);
    scatter3(X,Y,Z,6,T,'filled');
    axis equal;
end
Temp=PointCloud;
Temp.P=PCRockfall.P(1:3,PCRockfall.UsefulVar(2,:)~=-1);
Temp.GetMissingPropFromPC(PCRockfall);
% Save each rockfall event
if Temp.UsefulVar(1,:)< 0
   for i=min(Temp.UsefulVar(1,:)):max(Temp.UsefulVar(1,:))
        a=i+2;
        Rockfalls(a)= PointCloud('',Temp.P(1:3,Temp.UsefulVar(1,:)==i));
        Rockfalls(a).GetMissingPropFromPC(Temp);
    end
end

for i=1:max(Temp.UsefulVar(1,:))
    Rockfalls(i)= PointCloud('',Temp.P(1:3,Temp.UsefulVar(1,:)==i));
    Rockfalls(i).GetMissingPropFromPC(Temp);
end

Eps;
end

