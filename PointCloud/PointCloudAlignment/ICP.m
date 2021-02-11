function ICP(tpc,spc,out,varargin)
% ICP Perform the alignment of a source point cloud to a target point cloud with the Iterative Closest Point class of algorithms.
%
% To add : 
% - Possibility to make random sampling in each iteration in order to prevent any bias from outliers.
% - Weighting based on the expected effect of scanner noise on the uncertainty in the error metric [Rus 01].
% - Implement Pseudo plane to plane minimisation [Low 03]. 
%
%
%
% Ouput :
%   The CPU time consumed, the transformation matrices, mean distances and status in each
%   iteration are put in the given ICPVarOut object (out).
%   Important : the transform matrices (TM) put in "out" are the ones that
%   allows to transform the original source point cloud provided to ICP
%   into the aligned point cloud in the corresponding iteration :
%   PC_aligned = PC_source * TM
%
% Input : 
%   tpc : the target, a (Mesh)PointCloud object.
%   spc : the source, a (Mesh)PointCloud object.
%   Note : 
%       the given source will be aligned, ie the result of the alignment
%       procedure will be applied to the source.
%       nans in the positions are not taken into account and wont cause any problem.
%       Given MeshPointClouds will be turned into PointCloud objects.
%
% Additional parameters that can be set via their field name :
%
% NbIterMax
%       {30} | scalar > 0
%       Nb of iteration maximum that can be performed.
%
% Tol 
%       {0.01} | real in ]0; 1[ || < 0 (disabled)
%       The iteration loop stops when the relative difference in the min
%       distance (d_k - d_k-1)/d_k is smaller than Tol.
%
% DistToMin
%       {2} | 1
%       Here you can set on which distance you want the minimisation to be
%       performed :
%       1 : the mean distance between measured matched positions.
%       2 : the standard deviation between measured matched positions.
%       Note that they are the one put in ICPVarOut.Dist(x), x=1,2.
%
%
% Matching
%       {bruteForce} | Delaunay | GLtree | kDtree
%       Specifies how point matching should be done. 
%       bruteForce is usually the slowest and GLtree is the fastest.
%       Note :
%           To enable the GLtree option you need to install GLTreePro
%           (Matlab Central File ID: #29115).
%           The kDtree option depends on the Statistics Toolbox v. 7.3 or higher.
%
% Minimise
%       Gets its arguments as cells. In the first entry, you may pass
%       {point} | plane | lma | nm | bfgs
%       Defines which minimization should be performed. 
%        - point is based on the point to point method [Besl 92] with an SVD 
%       approach. It is usually the fastest with low nb of points. 
%        - plane is based on point to plane minimisation, 
%       more precisely on the sum of squared distances from each source point 
%       to the plane containing the destination point and oriented perpendicular 
%       to the destination normal [Chen 91].
%       Surface normals for all points in the target are computed by 
%       using a KDTree, which requires some preprocessing.
%       Think that this process is dependant on the number of nearest
%       neighbours used. Default is 4, which is fine for geometrical
%       (simulated) figures. Consider increasing this number for landscape
%       measurement with a high point density, to smooth a bit the terrain.
%       Search for t.ComputeNormals in this file, read the comments how to proceed.
%       For the problem to have an analytical solution, the rotation matrix
%       is linearised, supposing that the initial alignement is already
%       pretty good and rotation angles are small.
%       Note that it was found that this option works for angles as high as
%       pi/3, but with slower convergence.
%       To have the translation as unique degrees of freedom (instead of
%       both rotations and translations), set the second entry to 0
%       (default 1).
%       - lma does point to point minimisation using the non
%       linear least squares Levenberg Marquardt algorithm [Fitz 03]. 
%       More possibilities with this option is descibed in Min_lma.
%       - nm does point to point minimisation using the Nelder-Mead method
%       (multidimensional unconstrained nonlinear minimisation)[Lag 98].
%       - bfgs does point to point minimisation using the Quasi Newton Broyden 
%       Fletcher Goldfarb Shanno (BFGS) (limited memory BFGS (L-BFGS) and Steepest Gradient Descent are also available) optimisation methods.  
%       This optimiser has developed for image registration methods with large amounts of unknown variables.
%
%       Note on lma, nm and bfgs options :
%           - You cannot use weights.
%           - They are not ICP methods. lma uses least-squares methods, while nm and bfgs looks for function local minima. 
%           - They allows you to also perform resizing and shears by
%           setting the second entry appropriately :
%               0 : translation only.
%               1 : rotation added (default).
%               2 : resizing added.
%               3 : shearing added.
%           - You can provide the initial transformation by setting the
%           third entry appropriately with respect to the second entry. 
%           Have a look to the default initialisation below or ParVec2TransMat for an example. 
%           - In lma, it would be possible to provide lower and upper
%           bounds. To be implemented. Check out Min_lma
%           - Setting Source.TMode = 1 might help the convergence for lma,
%           nm and bfgs options.
%               
%       Examples :
%           'Minimise',{'plane',1}
%           'Minimise',{'nm',2}  (with resizing)
%           'Minimise',{'nm',2,[0 0 0  0 0 1  0.5 0.5 0.5]}  (with resizing)
%
%
% Extrapolation
%       {false} | true
%       If Extrapolation is true, the iteration direction will be evaluated
%       and extrapolated if possible using the method outlined by [Besl 92]. 
%       You may also want to try the modifications proposed in [Rus 01].
%
% RejectEdges
%       {false} | true
%       If RejectEdges is true, point matches to edge vertices of the target are
%       ignored. It requires that boundary points of the target are
%       specified in t.Boundaries.
%
% RejectNWorstPairs
%       {0} | scalar in ]0, 1[
%       Reject a given percentage of the worst point pairs, based on their
%       Euclidean distance.
%
% RejectDistPairs
%       {0} | scalar in ]0, inf[
%       Reject pairs with distances greater than a threshold.
%
% RejectRMSPairs
%       {0} | scalar in ]0, inf[
%       Reject pairs with distances greater than the standard deviation
%       times a factor.
%
% RejectNormals
%       {0} | scalar in ]0, pi/2]
%
% RejectIsolated
%       {0} | scalar in ]0, 1[
%       Reject pairs not consistent with neighbouring pairs (assuming surface move rigidly) [Dorai 98].
%       Two correspondances are inconsistant iff |Dist(s1,s2)-Dist(t1,t2)|
%       > Arg * max(Dist(s1,s2),Dist(t1,t2)). We advise to use Arg = 0.1
%       By default, when nb of point is enough, each pair is compared with 10 closest neighbours and
%       rejected if found to be incompatible with more than 5.
%       Note that this option needs both point cloud to be quite well
%       aligned. The alignment will be refined on s U t.
%
% Weight
%       {constant} | dist | normal | curv
%       Apply a weight to the pairs according to the following criteria:
%       constant : every pairs get full weight (w=1)
%       dist   : based on distance, w=1-dist(p1,p2)/distmax
%       normal : based on compatibility of normals, w=n1*n2
%       curv   : based on compatibility of curvatures, w = e^-(cs-ct)^2 (for c=curv/curv_max)
%       Note : does not work with Minimise{1} = 'lma','nm' and 'bfgs'
%
% TruePos
%       {true} | false
%       Compute all stuff that uses the true positions, if they are
%       provided (PointCloud.TrueP not empty).
%
% Verbose
%       {false} | true
%       Tell me your life in the Command Window.
%
% TimeLimit
%       {inf} | scalar in ]0, inf[
%       Time limit [s] allowed for an iteration.
%
%
% Advise :
% - Perform an initial alignement before starting the algorithm. For
%   instance, move the source cloud to the center of gravity of the target.
% - Make a subsample of your clouds if they are big (>100k points).
%
% Reference :
% [Besl 92]   Besl, P. and McKay, N. "A Method for Registration of 3-D Shapes", Trans. PAMI, Vol. 14, No. 2, 1992.
% [Chen 91]   Chen, Y. and Medioni, G. "Object Modeling by Registration of Multiple Range Images", Proc. IEEE Conf. on Robotics and Automation, 1991.
% [Fitz 03]   Andrew W. Fitzgibbon. "Robust registration of 2d and 3d point sets". Image and Vision Computing, 21(13-14):1145{1153, 2003.
% [Lag 98]    J.C. Lagarias, J. A. Reeds, M. H. Wright, and P. E. Wright, "Convergence Properties of the Nelder-Mead Simplex Method in Low Dimensions," SIAM Journal of Optimization, Vol. 9 Number 1, pp. 112-147, 1998.
% [Low 03]    K.-L. Low, A. Lastra, "Reliable and rapidly-converging ICP Algorithm using Multiresolution Smoothing."
% [Rus 01]    S. Rusinkiewicz, M. Leroy, 2001 "Effective Variants of the ICP Algorithm". 
% [Dorai 98]  Dorai, C.,Weng, J., and Jain, A. "Registration and Integration of Multiple Object Views for 3D Model Constrution", Trans. PAMI, Vol. 20, No. 1, 1998.
%
% Disclaimer :
% 1) This function comes with no warranty whatsoever. The responsability is 
%    upon the user to test thoroughly that it yields results consistent with expectations. 
%    Please signal any bug encountered.
%    Please feel free to adapt and distribute.
% 2) This function contains code freely adapted from (see license):
%    Martin Kjer and Jakob Wilm, DTU, 2010
%    D.Kroon, University of Twente, 2009
%    Respect the licenses.
%
%AUTHOR  : Neal Gauvin (at a3 dot epfl dot ch)
%VERSION : 2.7
%STATUS  : OK
%DATE    : 30 mai 2011

%% Validate input arguments.
ip = inputParser;

ip.addRequired( 'tpc', @(x)isa(x,'PointCloud') || isa(x,'MeshPointCloud') );
ip.addRequired( 'spc', @(x)isa(x,'PointCloud') || isa(x,'MeshPointCloud')  );
ip.addRequired( 'out', @(x)isa(x,'ICPVarOut') );

ip.addParamValue('NbIterMax', 30, @(x)x > 0 && x < 10^4);

ip.addParamValue('Tol', 0.01, @(x) (x > 0 && x < 1) || x < 0 );

ip.addParamValue('DistToMin', 2, @(x) (x >= 1 && x <= 2));

ip.addParamValue('RejectEdges', false, @(x)islogical(x));

ip.addParamValue('Extrapolation', false, @(x)islogical(x));

validMatching = {'bruteForce','Delaunay','GLtree','kDtree'};
ip.addParamValue('Matching', 'bruteForce', @(x)any(strcmpi(x,validMatching)));

ip.addParamValue('Minimise', {}, @(x)iscell(x));

validWeight = {'constant','dist','normal','curv'};
ip.addParamValue('Weight', 'constant', @(x)any(strcmpi(x,validWeight)));

ip.addParamValue('RejectNWorstPairs', 0, @(x)isscalar(x) && x >= 0 && x < 1);
ip.addParamValue('RejectDistPairs', 0, @(x)isscalar(x) && x >= 0);
ip.addParamValue('RejectRMSPairs', 0, @(x)isscalar(x) && x >= 0);
ip.addParamValue('RejectNormals', 0, @(x)isscalar(x) && x >= 0 && x <= pi/2);
ip.addParamValue('RejectIsolated', 0, @(x)isscalar(x) && x >= 0 && x < 1);

ip.addParamValue('Verbose', false, @(x)islogical(x));

ip.addParamValue('TruePos', true, @(x)islogical(x));

ip.addParamValue('TimeLimit', inf, @(x)isscalar(x) && x > 0 );

ip.parse(tpc,spc,out,varargin{:});
arg = ip.Results;

%% Sanity check 

% If MeshPointCloud are given, change them into PointCloud
if isa(tpc,'MeshPointCloud'), tpc = tpc.PointCloud; end
if isa(spc,'MeshPointCloud'), spc = spc.PointCloud; end

% True positions must be provided to enable TruePos
arg.TruePos = arg.TruePos && tpc.HasTrueP && spc.HasTrueP;

% Minimise
if size(arg.Minimise,2)<1
    arg.Minimise{1} = 'point';
else
    validMinimise = {'point','plane','lma','nm','bfgs'};
    if not(any(strcmpi(arg.Minimise{1},validMinimise)))
        error('ICP:WM','Unknown Minimisation option %s\n',arg.Minimise{1});
    end
end
global minit mscale % 
if strcmpi(arg.Minimise{1}, 'lma') || strcmpi(arg.Minimise{1}, 'nm')  || strcmpi(arg.Minimise{1}, 'bfgs')
    if not(strcmpi(arg.Weight,'constant'))
        % You cannot use weights with lma and nm minimisation
        msg = 'You cannot use weights with lma, nm and bfgs minimisation. Weight reset to constant.';
        warning('ICP:WM',msg);
        arg.Weight = 'constant';
    end
    if size(arg.Minimise,2)<2
        arg.Minimise{2}=1; %only rotations and translations
    elseif arg.Minimise{2}<0 || arg.Minimise{2}>3
        %test input values
        error('ICP:WM','Non valid Minimisation option %i\n',arg.Minimise{2});
    end
    % set initial and scaling parameters
    % Initial
    if size(arg.Minimise,2)<3 || isempty(arg.Minimise{3})
        switch arg.Minimise{2}
            case 0
                minit = [0 0 0];
            case 1
                minit = [0 0 0  0 0 0];
            case 2
                minit = [0 0 0  0 0 0  1 1 1];
            case 3
                minit = [0 0 0  0 0 0  1 1 1  0 0 0  0 0 0];
        end
    else
        minit = arg.Minimise{3};
    end
    % Scale
    if size(arg.Minimise,2)<4 || isempty(arg.Minimise{4})
        switch arg.Minimise{2}
            case 0
                mscale = ones(1,3);
            case 1
                mscale = ones(1,6);
            case 2
                mscale = ones(1,9);
            case 3
                mscale = ones(1,15);
        end
    else
        mscale = arg.Minimise{4};
    end
elseif strcmpi(arg.Minimise{1}, 'plane')    
    if size(arg.Minimise,2)<2
        arg.Minimise{2}=1; % rotations and translations
    elseif arg.Minimise{2}<0 || arg.Minimise{2}>1
        %test input values
        error('ICP:WM','Non valid Minimisation option %i\n',arg.Minimise{2});
    end    
end



if arg.Verbose
    fprintf('--------------------------------------------------\n');
    fprintf('ICP is starting with the following parameters :\n');
    arg                                                                    %#ok<NOPRT>
end
   



%% Internal parameters which can be set.
%Which distance to use ? (See def of ICPVarOut.Dist)
global dar
dar = false; % don't touch it here ! (but in Min_lma)

%Widht of the output text (fprintf)
tw = 65;

% Minimum nb of points required to perform the alignment
NbMinPoint = 2;

%% Start initialisation
% Start a stopwatch timer.
tic;

% Initialise temporary transform vector and matrix.
TM = eye(4,4);

% Common internal parameters.
global t s
global tPm sPm tTPm sTPm 
global s_idx t_idx 
global Dist din
global Weights
global TruePos
global TimeLimit
t = tpc; s = spc;
TruePos = arg.TruePos;
TimeLimit = arg.TimeLimit;
din = arg.DistToMin; 

%Ns = size(s.P,2); %number of source points.

scP = []; scTP = []; %Possible copy of the original point cloud

%If point to plane minimisation, normals are needed
if strcmpi(arg.Minimise{1}, 'plane')
    % If you want to change the nb of neighbours... set 'NbNeighbours',10
    t.ComputeOptimalNormals();
    % t.ComputeNormals();
end

% If Matching == 'Delaunay', a triangulation is needed
if strcmpi(arg.Matching, 'Delaunay')
    t.ComputeDelaunayTriangulation();
end

% If Matching == 'GLtree', a GL tree must be built
if strcmpi(arg.Matching, 'GLtree')
    t.ComputeGLTree();
end

% If Matching == 'kDtree', a kD tree must be built
if strcmpi(arg.Matching, 'kDtree')
    t.ComputeKDTree();
end

% Be sure edge vertices are provided
if arg.RejectEdges
    if isempty(t.Boundaries)
        msg = ['You requested edge rejection, but no boundary points have '...
            'been provided in the target (obj.Boundaries). ICP will stop'];
        warning('ICP:NoB',msg);
        Terminate(out);
    end
end

if arg.Extrapolation
    % You need to keep a copy of the original point cloud
    scP = s.P; 
    if arg.TruePos, scTP = s.TrueP; end
    % Initialise total transform vector (quaternion ; translation vec.)
    qq = [ones(1,arg.NbIterMax+1);zeros(6,arg.NbIterMax+1)];   
    % Allocate vector for direction change and change angle.
    dq = zeros(7,arg.NbIterMax+1);
    theta = zeros(1,arg.NbIterMax+1);
end

% Keep track of initialisation variables (transformation matrix, status,
% CPU time).
out.AddEntry( TM, uint32(ICPStatus.Preprocessing), toc );

%% Main iteration loop

k = 0; conv = false; % has converged ?
while( k < arg.NbIterMax && not(conv))
    % Restart the stopwatch timer.
    tic;
    
    k = k + 1;
    Dist = NaN(5,1);%zeros(5,1);

    if arg.Verbose
        fprintf('--------------------------------- iteration %u :\n', k );
    end
    
    % Do matching
    switch arg.Matching
        case 'bruteForce'
            [match mindist] = Match_bruteForce(t,s);
        case 'Delaunay'
            [match mindist] = Match_Delaunay(t,s);
        case 'GLtree'
            [match mindist] = Match_GLtree(t,s);
        case 'kDtree'
            [match mindist] = Match_kDtree(t,s);
    end
    TestElapsedTime(out);
    
    % Initialiase masks, only on non Nan source values.
    s_idx = true(1,length(match));
    t_idx = match;
    
    if arg.RejectIsolated
        % Menu
        NbNgb = 10; % Nb of closest neighbours to search
        NbInc = 5;  % Nb of tolerated inconsistent neighbours (< NbNgb)
        % Sanity checks
        %NbP = size(mindist,2); %nb of points to treat.
        NbP = length(t_idx);
        if NbNgb >= NbP
            NbNgb = NbP-1; NbInc = uint32(NbNgb/2);
        end
        % Compute a kD tree for the closest neighbours
        s.ComputeKDTree();        
        % find the closest neighbours and distances on the source (for each point with the neighbours)
        sPm = s.P(:,s_idx);
        %Ngb = transpose(knnsearch(s.KDTree, transpose(sPm), 'k', NbNgb+1));
        [Ngb,Ds] = knnsearch(s.KDTree, transpose(sPm), 'k', NbNgb+1);
        Ngb = Ngb'; Ds = Ds';
        tPm = t.P(:,t_idx);
        % an index to mask the inconsistent points
        idx = false(1,NbP);
        % loop on source points
        for is = 1:NbP
            % retrieve all neigbours of the target
            tNgb = tPm(:,Ngb(:,is)');
            %idc = match(Ngb(:,is)');
            %tNgb = t.P(:,idc);
            % we need to compute the distances wrt all its neighbours
            Dt = zeros(NbNgb,1);
            for it = 1:NbNgb
                Dt(it,1) = EuclDist( tNgb(:,1), tNgb(:,it+1) );
            end
            % Now, test the consistency
            Nbf = 0;
            for it = 1:NbNgb
                % Cross-checks
                absD = abs(Ds(it+1,is)-Dt(it,1));
                maxD = max(Ds(it+1,is),Dt(it,1));
                if absD > arg.RejectIsolated * maxD
                    %if( is < 11 ),fprintf('Neigbour is not consistent !\n');end
                    Nbf = Nbf + 1;
                end
                if Nbf > NbInc
                   % the point is inconsistent, drop it out !
                   %if( is < 11 ),fprintf('Source point is inconsistent !\n');end
                   idx(1,is) = true;
                   break
                end
            end           
        end % source points
        % Drop out inconsistent points
        pairs = find(s_idx);
        s_idx(pairs(idx)) = false;
        t_idx = match(s_idx);
    end % RejectIsolated
    TestElapsedTime(out);
    
    % Possibly reject matches to edge points
    if arg.RejectEdges
        s_idx = not(ismember(match,t.Boundaries ));
        t_idx = match(s_idx);
    end
    
    % Possibly reject worst matches
    if arg.RejectNWorstPairs
        %nb of elements to keep
        edge = round((1-arg.RejectNWorstPairs)*sum(s_idx)); 
        % Rejection will apply only on non-zero elements, create an index
        % on them
        pairs = find(s_idx);
        % Sort by increasing distances
        mindist2 = mindist(s_idx);
        [~, idx] = sort(mindist2);
        %update the index
        s_idx(pairs(idx(edge:end))) = false;
        t_idx = match(s_idx);
    end
    
    % Possibly reject matches with distances greater than a threshold
    if arg.RejectDistPairs
        % Index on non zero elements.
        pairs = find(s_idx);
        % logical index on elements greater than the threshold.
        mindist2 = mindist(s_idx);
        idx = mindist2 > arg.RejectDistPairs;
        s_idx(pairs(idx)) = false;
        t_idx = match(s_idx);
    end
    
    % Possibly reject matches with distances greater than the RMS
    if arg.RejectRMSPairs
        % Index on non zero elements.
        pairs = find(s_idx);
        %Compute RMS
        mindist2 = mindist(s_idx);        
        rms = sqrt(sum(mindist2.^2)/length(mindist2));        
        % logical index on elements greater than the threshold.
        idx = mindist2 > arg.RejectRMSPairs*rms;
        s_idx(pairs(idx)) = false;
        t_idx = match(s_idx);
    end    
    TestElapsedTime(out);
    
    if arg.RejectNormals
        % Index on non zero elements.
        pairs = find(s_idx);        
        % Compute the angles between all the normals
        t.ComputeNormals(); s.ComputeNormals();
        nt = t.Normals(:,t_idx); ns = s.Normals(:,s_idx);
        snt = size(nt,2);
        angles = zeros(1,snt);
        for in = 1:snt
            angles(in)=acos(abs(nt(:,in)'*ns(:,in)));
        end
        % logical index on elements greater than the threshold.
        idx = angles > arg.RejectNormals;
        s_idx(pairs(idx)) = false;
        t_idx = match(s_idx);
    end
    
    % Remove possible nans in source.
    nanm = isnan(s.P(1,:));
    if any(nanm)
        s_idx = logical(s_idx .* not(nanm));
        t_idx = match(s_idx);
    end
    
    mindist = mindist(s_idx);
    TestElapsedTime(out);
    
    % Compute weights
    switch arg.Weight
        case 'constant'
            Weights = ones(size(mindist));
        case 'dist'
            distmax = max(mindist);
            Weights = 1-mindist/distmax;
        case 'normal'
            t.ComputeNormals(); s.ComputeNormals();
            nt = t.Normals(:,t_idx); ns = s.Normals(:,s_idx);
            snt = size(nt,2);
            Weights = zeros(1,snt);
            for in = 1:snt
                Weights(in)=abs(nt(:,in)'*ns(:,in));
            end
        case 'curv'
            t.ComputeCurvature(); s.ComputeCurvature();
            ct = t.Curv(t_idx); cs = s.Curv(s_idx);
            ctm = max(ct); csm = max(cs);
            ct = ct ./ ctm; cs = cs ./ csm;
            Weights=exp(-(cs-ct).^2);
    end
    TestElapsedTime(out);

    % I hope we have enough points left for minimisation...
    if length(mindist) < NbMinPoint
        out.AddEntry( eye(4,4), uint32(ICPStatus.Failed), 0 );
        error('ICP:Stop','Insufficent nb of points to perform alignment.');
    end    
    
    if k == 1
        Dist(1) = mean(mindist);
        Dist(2) = sqrt(sum(mindist.^2)/length(mindist)); 
        % Il faudrait ici tout calculer...
        out.SetDist(Dist); % In the last entry = initialisation
        if arg.Verbose
            display('Initial distances between matched points :');
            out.PrintDist();
        end
    end
    
    % Set a copy of the current working cloud point in the global scope    
    tPm = t.P(:,t_idx); sPm = s.P(:,s_idx);                                %#ok<NASGU>
    if arg.TruePos 
        tTPm = t.TrueP(:,t_idx); sTPm = s.TrueP(:,s_idx);
    end
    
    if arg.Verbose
        str = 'Number of points considered for minimisation';
        fprintf('%-65s : %i\n', str, size(tPm,2) );
    end
    
    %Cross check : plot some intermediate point clouds
    %if k<3, Plot3DPoints(tPm,sPm); end
%     if k<3
%         figure; hold on;
%         tPNm = t.Normals(:,t_idx); sPNm = s.Normals(:,s_idx);
%         ht=quiver3(tPm(1,:),tPm(2,:),tPm(3,:),tPNm(1,:),tPNm(2,:),tPNm(3,:));
%         set(ht,'Color','r','LineWidth',2);
%         hs=quiver3(sPm(1,:),sPm(2,:),sPm(3,:),sPNm(1,:),sPNm(2,:),sPNm(3,:));
%         set(hs,'Color','b','LineWidth',2); hold off;
%     end
    
    switch arg.Minimise{1}
        case 'point'
            TM = Min_point();
        case 'plane'
            TM = Min_plane(arg.Minimise{2});
        case 'lma'
            TM = Min_lma();
        case 'nm'
            TM = Min_nm();
        case 'bfgs'
            TM = Min_bfgs();
    end
    TestElapsedTime(out);
    
    % Compute total transformation
    TTM = out.GetLastTMTransformed( TM, s.TMode );
    
    % Apply last transformation
    s.transform(TM);
    % Update your current copy of the working point cloud
    sPm = s.P(:,s_idx);
    if arg.TruePos, sTPm = s.TrueP(:,s_idx); end
    
    % Compute the objective distance
    %ComputeDistances(din);

    % Compute all distances
    ComputeDistances();
    
    % If Extrapolation, we might be able to move quicker
    % You may want to consider the modifications proposed in [Rus 01].
    if arg.Extrapolation
        qq(:,k+1) = [Rot2Quat(TTM(1:3,1:3));TTM(1:3,4)];
        dq(:,k+1) = qq(:,k+1) - qq(:,k);
        theta(k+1) = (180/pi)*acos(dot(dq(:,k),dq(:,k+1))/(norm(dq(:,k))*norm(dq(:,k+1))));
%         if arg.Verbose
%             disp(['Direction change ' num2str(theta(k+1)) ' degree in iteration ' num2str(k)]);
%         end
        if k>2 && theta(k+1) < 10 && theta(k) < 10
            dold = out.GetDist(); doldold = out.GetDist(-1);
            d = [Dist(din), dold(din), doldold(din)];
            v = [0, -norm(dq(:,k+1)), -norm(dq(:,k))-norm(dq(:,k+1))];
            vmax = 25 * norm(dq(:,k+1));
            dv = Extrapolate(v,d,vmax,arg.Verbose);
            if dv ~= 0
                q_mark = qq(:,k+1) + dv * dq(:,k+1)/norm(dq(:,k+1));
                q_mark(1:4) = q_mark(1:4)/norm(q_mark(1:4));
                qq(:,k+1) = q_mark;
                TTM(1:3,1:3) = Quat2Rot(qq(1:4,k+1));
                TTM(1:3,4) = qq(5:7,k+1);
                % Reapply total transformation
                s.P = AffinTransform(scP,TTM);
                if arg.TruePos, s.TrueP = AffinTransform(scTP,TTM); end
            end
        end
    end
    % Compute the remaining distances.
    %ComputeDistances(-din);
    
    % Has the alignment converged or not ?
    if arg.Tol > 0
        %When working with the true positions, the resulting distances
        %might be zero... Need to get protected from that.
        oldDist = out.GetDist();
        if( Dist(din) < 1e-6 || abs(Dist(din) - oldDist(din))/Dist(din) < arg.Tol )
            conv = true;
        end
    end
    
    % Status of the iteration
    status = uint32(ICPStatus.Success);
    if conv
        status = uint32(ICPStatus.Converged);
    elseif k == arg.NbIterMax
        status = uint32(ICPStatus.NotConverged);
    end
        
    % Keep track of transformation matrix, status, CPU time.
    te = toc;
    out.AddEntry( TTM, status, te, Dist );

    % Tell me your life
    if arg.Verbose
        str = 'Status of the iteration';
        fprintf('%-*s : %s\n', tw, str, ICPStatus(status).char);
        str = 'Elapsed time [s]';
        fprintf('%-*s : %.4f\n', tw, str, te );
        out.PrintDist(6);
    end
    
end %while loop
   
if arg.Verbose
    fprintf('--------------------------------------------------\n');
end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [match mindist] = Match_bruteForce(t, s)
% Matches source and target points by euclidean distance.

%works faster with that trick.
tar = t.P;
p = s.P;
m = size(p,2);
n = size(tar,2);
match = zeros(1,m);
mindist = zeros(1,m);
for k=1:m
    d=zeros(1,n);
    for l=1:3
        d=d+(tar(l,:)-p(l,k)).^2;
    end
    [mindist(k),match(k)]=min(d);
end

mindist = sqrt(mindist);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [match mindist] = Match_Delaunay(t, s)
% Matches source and target points by euclidean distance using Delaunay triangles.

tp = t.P; ts = s.P; %Gain half a second...
match = transpose(nearestNeighbor(t.DT, transpose(ts)));
mindist = sqrt(sum((ts-tp(:,match)).^2,1));
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [match mindist] = Match_GLtree(t, s)
% Matches source and target points by euclidean distance using GL trees.

    [match mindist] = KNNSearch3D(t.P,s.P,t.GLTree,1);
	match = transpose(match);
    mindist = transpose(mindist);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [match mindist] = Match_kDtree(t, s)
% Matches source and target points by euclidean distance using kD trees.
% Note : will work only with Toolbox version > 7.2
    [match mindist] = knnsearch(t.KDTree,transpose(s.P));
    match = transpose(match);
    mindist = transpose(mindist);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function TM = Min_point()
% Point to point minimisation using singular value decomposition.

global tPm sPm
global Weights

% q : target, p : source
m = size(sPm,2);
n = size(tPm,2);
TM = eye(4,4);

% normalise weights
Weights = Weights ./ sum(Weights);

% find data centroid and deviations from centroid
q_bar = tPm * transpose(Weights);
q_mark = tPm - repmat(q_bar, 1, n);
% Apply weights
q_mark = q_mark .* repmat(Weights, 3, 1);

% find data centroid and deviations from centroid
p_bar = sPm * transpose(Weights);
p_mark = sPm - repmat(p_bar, 1, m);

N = p_mark*transpose(q_mark); % taking points of q in matched order

[U,~,V] = svd(N); % singular value decomposition

% The rotation
R = V*transpose(U);
TM(1:3,1:3) = R;
% The translation
TM(1:3,4) = q_bar - R*p_bar;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function TM = Min_plane( mo )
% Point to plane minimisation. In brief, the idea is to bring the data points 
% close to the planes in which the target points reside. 
% Mathematically this can be done by minimising the dot products of the vectors 
% p_iq_i and normals n_i.
% The rotation matrix is linearised in order to have an analytical solution
% for the problem.
% Inpout : minimisation option mo. 0 : translation. 1 : translation +
% rotation (default).

global t
global tPm sPm
global t_idx 
global Weights

% get the normals and normalise them according to the weights
n = t.Normals(:,t_idx); 
n = n .* repmat(Weights,3,1);
st = sPm-tPm;

% Perform the computation
if mo == 1
    c = cross(sPm,n);
    cn = vertcat(c,n);
    C = cn*transpose(cn);    
    b = - [sum(sum(st.*repmat(cn(1,:),3,1).*n));
        sum(sum(st.*repmat(cn(2,:),3,1).*n));
        sum(sum(st.*repmat(cn(3,:),3,1).*n));
        sum(sum(st.*repmat(cn(4,:),3,1).*n));
        sum(sum(st.*repmat(cn(5,:),3,1).*n));
        sum(sum(st.*repmat(cn(6,:),3,1).*n))];    
    X = C\b;
    
    TM = TransformMatrix(X(1:3),X(4:6));
else
    C = n*transpose(n);
    b = - [sum(sum(st.*repmat(n(1,:),3,1).*n));
        sum(sum(st.*repmat(n(2,:),3,1).*n));
        sum(sum(st.*repmat(n(3,:),3,1).*n))];    
    X = C\b;
    
    TM = TransformMatrix([],X(1:3));
end

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function TM = Min_lma()
% Non-linear minimisation using the Levenberg-Marquardt algorithm.
% Note : you can provide upper and lower bounds in the optimisation.

global tPm sPm
global minit

% TolX don't seem to play any role here, keep the default.
TolX = 1e-6; % default 1e-6
% lsqcurvefit implements two different algorithms: trust region reflective and 
% Levenberg-Marquardt. Choose one via the option Algorithm: for instance, to 
% choose Levenberg-Marquardt, set OPTIONS = optimset('Algorithm','levenberg-marquardt'), 
% and then pass OPTIONS to LSQCURVEFIT. 
optim = optimset('Display','off','TolX',TolX,'Algorithm','levenberg-marquardt');
myfun = @(x, data)AffinTransform(data,ParVec2TransMat(x));
minit = lsqcurvefit(myfun, minit, sPm, tPm, [], [], optim);

%%%%%%% More...
% lsqcurvefit attempts to solve non-linear least squares problems of the form:
%   min  sum {(FUN(X,XDATA)-YDATA).^2}  where X, XDATA, YDATA and the
%    X                                  values returned by FUN can be 
%                                       vectors or matrices.
% You may want to try to compute the distance yourself :
% lsqnonlin attempts solve non-linear least squares problems of the form:
%   min  sum {FUN(X).^2}    where X and the values returned by FUN can be   
%    X                      vectors or matrices.
% global dar
% dar = true; % compute difference between point clouds
% optim = optimset('Display','off','TolX',TolX,'Algorithm','levenberg-marquardt');
% myfun = @(x)ObjectiveFunction(x);
% minit = lsqnonlin(myfun,minit,[],[],optim);

TM = ParVec2TransMat(minit);

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function TM = Min_nm()
% Multidimensional unconstrained nonlinear minimisation (Nelder-Mead)

global minit

% It is difficult to assess the impact of TolX. It seems to help converging
% faster when very small.
TolX = 1e-6; % default 1e-6
optim = optimset('Display','off','TolX',TolX);
myfun = @(x)ObjectiveFunction(x);
minit = fminsearch(myfun,minit,optim);

TM = ParVec2TransMat(minit);

end % Min_nm
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function TM = Min_bfgs()
% Fletcher Goldfarb Shanno (BFGS) optimisation methods

global minit

% Note : Broyden Fletcher Goldfarb Shanno optisation is used by default, 
%        when the number of unknowns is	larger then 3000 the function will 
%       switch to Limited memory BFGS, or if you set 'HessUpdate' to 'lbfgs'. 
%       When 'HessUpdate' is set to 'steepdesc', steepest decent optimization is used.
TolX = 1e-6; % default 1e-6
optim = struct('Display','off','TolX',TolX);
myfun = @(x)ObjectiveFunction(x);
minit = fminlbfgs(myfun,minit,optim);

TM = ParVec2TransMat(minit);

end % Min_lbfgs

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [dv] = Extrapolate(v,d,vmax,verb)

p1 = polyfit(v,d,1); % linear fit
p2 = polyfit(v,d,2); % parabolic fit
v1 = -p1(2)/p1(1); % linear zero crossing
v2 = -p2(2)/(2*p2(1)); % polynomial top point

if issorted([0 v2 v1 vmax]) || issorted([0 v2 vmax v1])
    if(verb), disp('Parabolic update!'); end
    dv = v2;
elseif issorted([0 v1 v2 vmax]) || issorted([0 v1 vmax v2])...
        || (v2 < 0 && issorted([0 v1 vmax]))
    if(verb), disp('Line based update!'); end
    dv = v1;
elseif v1 > vmax && v2 > vmax
    if(verb), disp('Maximum update!'); end
    dv = vmax;
else
    if(verb), disp('No extrapolation!'); end
    dv = 0;
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Terminate(out)
% Stops the program properly.
    out.AddEntry( eye(4,4), uint32(ICPStatus.Error), 0 );
    error('ICP:Stop','ICP stopped by errors.');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function ComputeDistances(din)
% COMPUTEDISTANCES Compute the distances as defined in ICPVarOut.Dist
% Case din = 0 : all distances
% Case din < 0 : all distances except abs(din)
% Case din > 0 : computes din.
%
% Available distances to compute :
%       (1),(2) Mean distance between measured matched positions (mean and rms).
%       (3) Mean distance between true matched positions.
%       (4) Mean distance between true positions irrespectively of any matching.
%          In a perfect world this should be close to 0, therefore giving an
%          idea of the "error" in the alignment.
%       (5)=(4)-(3) error of the matching.
        
    global tPm sPm tTPm sTPm
    global t_idx %s_idx
    global Dist
    global t s
    global TruePos
    
    dind = [false false false false false];
    if nargin == 0, din = 0; end

    if din <= 0
        dind = [true true true true true];
        if din < 0, dind(abs(din)) = false; end
    else
        dind(din) = true;
    end
    
    % If no true positions, do not work with them !
    if ~TruePos, dind(3:end) = false; end
    
    % Compute some common stuff
    if dind(1) || dind(2)
       dsq = sum(power(tPm - sPm, 2),1);
   end
   
   %Set Dist
   if dind(1), Dist(1) = mean(sqrt(dsq)); end %(1)
   if dind(2), Dist(2) = sqrt(mean(dsq)); end %(2)
   if dind(3) || dind(5)
       dsq3 = sum(power(tTPm - sTPm, 2),1);
       Dist(3) = mean(sqrt(dsq3)); %(3)
   end
   if dind(4) || dind(5)
       % a copy of the true clouds that truly match each other.
       if isempty(s.TrueM)
           sTPmm = s.TrueP;
       else
           sTPmm = s.TrueP(:,s.TrueM);
       end       
        % You need to make sure that that they have the same size
       if size(sTPmm,2) ~= size(t.TrueP,2);
           return;
       end
       dsq4 = sum(power(sTPmm - t.TrueP, 2),1);
       Dist(4) = mean(sqrt(dsq4));

       % To compute (5), we need to reorder (4) in the order of the matching.
       % Then (5)=(4)-(3)
       dsq5 = abs(dsq4(t_idx) - dsq3);
       Dist(5) = mean(sqrt(dsq5));
       
   end %(4) (5)
  
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function TestElapsedTime(out)
% TESTELAPSEDTIME causes the program to stop if a certain time is elapsed.
    global TimeLimit
    if(toc < TimeLimit ), return; end
    
    % Need to stop the program...
    out.AddEntry( eye(4,4), uint32(ICPStatus.Stuck), toc );
    error('ICP:Stop','ICP running beyond the limit of iteration time... Stopped !');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function TM=ParVec2TransMat(par)
% retreves the transform matrix from the set of parameters.

% Scale the input parameters
%global mscale
%par=par.*mscale;
switch(length(par))

    case 3  % Translation
        TM = TransformMatrix([],par(1:3));
    case 6  % Translation and Rotation
        TM = TransformMatrix(par(4:6),par(1:3));
    case 9  % Translation, Rotation and Resize
        TM = TransformMatrix(par(4:6),par(1:3),par(7:9));
    case 15 % Translation, Rotation, Resize and Shear
        TM = TransformMatrix(par(4:6),par(1:3),par(7:9),par(10:15));
        
end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function  [d, dgrad] = ObjectiveFunction(par)

% Calculate distance error
d = ComputeDistanceFromPar(par);

% If asked calculate finite difference error gradient
if(nargout>1)
    % Stepsize used for finite differences
    delta=1e-8;
    dgrad=zeros(1,length(par));
    for i=1:length(par)
        par2=par; par2(i)=par(i)+delta;
        dgrad(i)=ComputeDistanceFromPar(par2)/delta;
    end
end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function dist = ComputeDistanceFromPar(par)

global tPm sPm
global din dar

% Get current transformation matrix
M=ParVec2TransMat(par);

% Transform the points with the transformation matrix
sPmM = AffinTransform( sPm, M );

if dar
    dist = sPmM-tPm;
    return
end

% Calculate the squared distance between the points
dsq=sum((sPmM-tPm).^2,1);

%if din == 0
    % calculate the total distance
    %dist = sum(dsq);
if din == 1
    % mean distance between points
    dist = mean(sqrt(dsq));    
elseif din == 2
    % standard deviation
    dist = sqrt(mean(dsq));
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

