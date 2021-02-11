function vol = volumes_tetra(T,X)
%VOLUMES Volumes of tetrahedra

% Empty case
if isempty(T)
    vol = zeros(0,1);
    return
end
% Local coordinates
A = X(T(:,1),:);
B = X(T(:,2),:) - A;
C = X(T(:,3),:) - A;
D = X(T(:,4),:) - A;
% 3D Volume
vol = 1/6*abs(dot(B,cross(C,D,2),2));
end


