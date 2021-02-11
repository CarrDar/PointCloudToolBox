function B = trueboundary(B,X)
%TRUEBOUNDARY True boundary faces
%   Remove false boundary caused by the removal of zero volume
%   tetrahedra. The input B is the output of TriRep/freeBoundary.

% Surface triangulation
facerep = triangulation(B,X);

% Find edges attached to two coplanar faces
E0 = edges(facerep);
E1 = featureEdges(facerep, 1e-6);
E2 = setdiff(E0,E1,'rows');

% Nothing found
if isempty(E2)
    return
end

% Get face pairs attached to these edges
% The edges connects faces into planar patches
facelist = edgeAttachments(facerep,E2);
pairs = cell2mat(facelist);

% Compute planar patches (connected regions of faces)
n = size(B,1);
C = sparse(pairs(:,1),pairs(:,2),1,n,n);
C = C + C' + speye(n);
[~,p,r] = dmperm(C);

% Count planar patches
iface = diff(r);
num = numel(iface);

% List faces and vertices in patches
facelist = cell(num,1);
vertlist = cell(num,1);
for k = 1:num

    % Neglect single face patches, they are true boundary
    if iface(k) > 1
        
        % List of faces in patch k
        facelist{k} = p(r(k):r(k+1)-1);
        
        % List of unique vertices in patch k
        vk = B(facelist{k},:);
        vk = sort(vk(:))';
        ik = [true,diff(vk)>0];
        vertlist{k} = vk(ik);
        
    end
end

% Sort patches by number of vertices
ivert = cellfun(@numel,vertlist);
[ivert,isort] = sort(ivert);
facelist = facelist(isort);
vertlist = vertlist(isort);

% Group patches by number of vertices
p = [0;find(diff(ivert));num] + 1;
ipatch = diff(p);

% Initiate true boundary list
ibound = true(n,1);

% Loop over groups
for k = 1:numel(ipatch)

    % Treat groups with at least two patches and four vertices
    if ipatch(k) > 1 && ivert(p(k)) > 3

        % Find double patches (identical vertex rows)
        V = cell2mat(vertlist(p(k):p(k+1)-1));
        [V,isort] = sortrows(V);
        id = ~any(diff(V),2);
        id = [id;0] | [0;id];
        id(isort) = id;

        % Deactivate faces in boundary list
        for j = find(id')
            ibound(facelist{j-1+p(k)}) = 0;
        end
        
    end
end

% Remove false faces
B = B(ibound,:);