function MultiResoICP(tpc,spc,out,varargin)
%MULTIRESOICP Iteratively perform an evolutive smoothing of the given point clouds and calls the ICP algorithm upon the smoothed surfaces.
%
% General Idea :
% Smoothing the surface of the shape into multiple resolutions. These smoothed
% surfaces are used in place of the original surface in a coarse-to-fine
% manner during alignement, which allows the algorithm to avoid being
% trapped at local minima close to the global solution.
%
% More precisely :
% Nearby local minima usually occur in shapes that have many
% higher-frequency surface features that exert stronger constraints on the
% pose than those by the lower-frequency features [Sim 96]. When the distance
% between the two surfaces is more than the size of these high-frequency
% features, the alignement solution will not converge to the global
% minimum.
% However, Smoothing both surfaces such that the higher-frequency
% "spikes" are removed and the lower-frequency bumps are preserved, gives
% more chance to the ICP to converge.
%
% The ideal smoothing operator should possess the following criteria :
%
% 1) Invariant  to  rigid  motion. Since the two range images are initially 
% described in different 3D coordinate frames, the smoothing should not produce
% different results on the same surface. More specifically, if S is a surface, 
% T is a rigid motion, and F is the smoothing operator, then T(F(S)) = F(T(S)).
%
% 2) Invariant  to  surface  parameterisation  and sampling. 
% The same surface regions on the physical object can be sampled by the scanner 
% from different positions and orientations. 
% This results in the common surface regions having different sampling
% densities and parameterisations in the two range images. 
% Given that both sampling densities are sufficient to preserve all surface details, 
% the smoothing should produce similar surfaces from the two range images 
% regardless of the different sampling densities and parameterization. 
% However, this is usually hard to achieve, and as the amount of
% smoothing increases, the deviation between the shapes of the two surfaces becomes larger. 
% One way to deal with this problem is to use multiple smoothed
% versions of the surfaces, each with different amount
% of smoothing. 
% Using this multiresolution approach, the coarsest smoothed versions of the two surfaces
% are used for registration first, until the distance between them become 
% less than half the smallest feature size in the next finer smoothed versions.
% Then, the finer versions are used in turn. 
% This repeats until the original range images are used.
%
% 3) Preserves desired constraints on rigid motion. 
% The motivation to smooth the surface is to remove high-frequency
% features that could interfere with the current registration. 
% Equally important, lower-frequency features should be preserved as much as
% possible to maximise the constraints on the rigid motion imposed by these features. 
%
% You must be careful that smoothing should not be applied further if it will 
% remove the necessary constraints on the rigid motion. 
% You can use the constraint analysis method from [Sim 96] to test this.
% When switching from coarser smoothed surfaces to finer ones at the appropriate
% distance, additional constraints on the rigid motion are re-introduced into the
% registration. This can help to quicken the convergence.
% One obvious drawback of smoothing is that the smoothed surfaces become smaller. 
% This happens because the filtering should not include unknown data that are
% outside the surface boundaries, including boundaries of holes caused by self-occlusions. 
% The reduction in surface area may reduce rigid motion constraints.
%
% The smoothing solutions implemented so far should satisfy 1 and 3. 
% For the moment we have to assume that the two range images are quite similarly sampled 
% (in that the sampling densities at their common surface regions are similar).
%
% Ouput :
%   The CPU time consumed, the transformation matrices, mean distances and status in each
%   iteration are put in the given ICPVarOut object (out).
%
% Input : 
%   tpc : the target, a PointCloud object.
%   spc : the source, a PointCloud object.
%   Important note : 
%   1) the point cloud must be mappable in the form z = f(x,y).
%   2) MeshPointCloud will be turned into PointCloud objects.
%   3) The source is returned aligned.
%
% Additional parameters that can be set via their field name :
%
% Ratio
%   {1} | cell array of reals in [0,inf[
%   If Type 'GridFit', controls the ratio between the size of the given point clouds and the 
%   desired numbers of control points in the smoothed surface: ratio = original / smoothed.
%   If Type 'Box', controls the size of the desired box in x and y.
%
%   if Ratio is set to 0, no smoothing is performed on the point clouds.
%   This may be a good idea for the final step.
%   Note that you are allowed to make some oversampling, ie add more point
%   estimations in your samples, but this is not recommended.
%
% Type
%   {'GridFit'} | 'Box'
%   Type of the smoothing technology used.
%   Box option of SubSampling may be improved to welcome a customised
%   function for the boxing, ie a function that would eliminate points too
%   far from the rms in one box...
%
% ICPOpt
%   Place in ICPOpt a cell with all the options you want to pass to
%   the ICP.
%
% SmoothOpt
%   Place in ICPOpt a cell with all the options you want to pass to the
%   smoothing function. It depends on the choice of the smoothing technology used.
%
% Verbose
%       {0} | integers in [0,1,2]
%       Level of verbosity
%       0 : disabled
%       1 : print output summary in the Command Window. Overwrite ICP Verbose
%       as well.
%       2: Plot lignement results of intermediate steps.
%
% Avise :
% 1) This function might be seen as an example how to perform multi
%    resolution smoothing. Copy it as script and adapt it to your needs !
%    Check that every step is performing the right thing.
%
% 2) Read [Low 03]
%
%   It is desirable that the smoothed surface are as similar as possible,
%   especially in their overlapping region.
%
% Reference :
% [Low 03]  K.-L. Low, A. Lastra, "Reliable and rapidly-converging ICP Algorithm using Multiresolution Smoothing."
% [Sim 96]  D. A. Simon, "Fast and Accurate Shape-based Registration". PhD dissertation, Carnegie Mellon University, CMU-RI-TR-96-45, 1996.
%
% Disclaimer :
% 1) This function comes with no warranty whatsoever. The responsability is 
%    upon the user to test thoroughly that it yields results consistent with expectations. 
%    Please signal any bug encountered.
%    Please feel free to adapt and distribute.
%
% 2) If the content of this function has been inspired by [Low 03], it does
% not contain the same technology. [Low 03] presents two-dimentionnal cubic
% B-spline wavelet transform. While GridFit should yield similar results to B-splines, cubic interpolation is not yet implemented. 
% What is more the essence of procedure is not the same.
%
%AUTHOR  : Neal Gauvin (at a3 dot epfl dot ch)
%VERSION : 1.0
%STATUS  : OK
%DATE    : 24 august 2011

%% Validate input arguments.
ip = inputParser;

ip.addRequired( 'tpc', @(x)isa(x,'PointCloud') || isa(x,'MeshPointCloud')  );
ip.addRequired( 'spc', @(x)isa(x,'PointCloud') || isa(x,'MeshPointCloud')  );
ip.addRequired( 'out', @(x)isa(x,'ICPVarOut') );

ip.addParamValue('Ratio', {1}, @(x) iscell(x) && ~isempty(x) );
validType = {'GridFit','Box'};
ip.addParamValue('Type','GridFit', @(x)any(strcmpi(x,validType)));
ip.addParamValue('ICPOpt', {}, @(x) iscell(x) );
ip.addParamValue('SmoothOpt', {}, @(x) iscell(x) );
ip.addParamValue('Verbose', 0, @(x)isnumeric(x) && x >= 0 && x <= 2);

ip.parse(tpc,spc,out,varargin{:});
arg = ip.Results;

%% Sanity checks

% If MeshPointCloud are given, change them into PointCloud
if isa(tpc,'MeshPointCloud'), tpc = tpc.PointCloud; end
if isa(spc,'MeshPointCloud'), spc = spc.PointCloud; end

% Set some default for ICP
if isempty(arg.ICPOpt)
    arg.ICPOpt{end+1} = 'Matching'; arg.ICPOpt{end+1} = 'kDtree';
end
arg.ICPOpt{end+1} = 'Verbose'; arg.ICPOpt{end+1} = logical(arg.Verbose);

%% Let's go !

if arg.Verbose >= 1
    fprintf('######################################################################\n');
    fprintf('MultiResolICP is starting with the following parameters :\n');
    arg                                                                    %#ok<NOPRT>
end

% Make a copy of the source to start working on.
%spcc = PointCloud(spc.Name,spc);

% Loop on the nb of smoothing steps
nbit = length(arg.Ratio);
for it = 1:nbit
    % Start a stopwatch timer.
    tic;

    % check ratio
    ratio = arg.Ratio{it};
    if ~isnumeric(ratio) || ratio < 0 || isinf(ratio)
        error('MultiResolICP:WR','Ratio must be a positive real in [0,inf[ : %f .', ratio);
    end

    
    if arg.Verbose >= 1
        fprintf('######################################################################\n');
        fprintf('################################### MultiResolICP iteration %u :\n', it );
        str = 'Ratio / box size';
        fprintf('%-65s : %.2f\n', str, ratio );
    end
    
    if ratio == 0
        % make a copy of the point clouds, so that spc does not get
        % transformed before the very end.
        tspc = PointCloud(tpc.Name, tpc) ; sspc = PointCloud(spc.Name,spc);
    else            
        % Nb of desired points
        nbpts = spc.Size / ratio; nbptt = tpc.Size / ratio;
        
        % Smooth point clouds : Subsampling does the job you want
        if strcmpi(arg.Type,'GridFit')
            tspc = SubSampling(tpc,'Type','GridFit','Arg',{nbptt, arg.SmoothOpt});
            sspc = SubSampling(spc,'Type','GridFit','Arg',{nbpts, arg.SmoothOpt});
        elseif strcmpi(arg.Type,'Box')
            tspc = SubSampling(tpc,'Type','Box','Arg',{[ratio,ratio,inf]});
            sspc = SubSampling(spc,'Type','Box','Arg',{[ratio,ratio,inf]});            
        end
    end
    % Update names to differentiate.
    tspc.Name = [tpc.Name ' smoothed']; sspc.Name = [spc.Name ' smoothed'];

    % Fill an entry in out to mark the step.
    out.AddEntry( eye(4,4), uint32(ICPStatus.Smoothing), toc );
    
    % Perform alignment
    ICP(tspc, sspc, out,arg.ICPOpt{:});
    
    % Transform the source.
    spc.transform(out.LastTM);

    % If true position infos are available, plot distance between the point
    % clouds.
    if arg.Verbose >= 1 && spc.HasTrueP && tpc.HasTrueP
        TrueDist = spc.ComputeTrueDistance(tpc);
        % Mean distance between true positions irrespectively of any matching.
        % In a perfect world this should be close to 0, therefore giving an
        % idea of the "error" in the alignment.
        str = 'Mean distance between true positions';
        fprintf('%-65s : %.6f\n', str, TrueDist );
    end
    
    if arg.Verbose >= 2 
        % You may want to plot the results of the alignment/smoothing in each
        % steps
        tit = ['MultiResoICP ratio ' num2str(ratio,'%.2f') ];
        % On the smoothed point clouds
        PlotMultiPointClouds({tspc,sspc},'Title',tit, 'PlotOpt',{{'b*'} {'ro'}});
        % On the original point clouds
        tit = ['MultiResoICP result of ratio ' num2str(ratio,'%.2f') ];
        PlotMultiPointClouds({tpc,spc},'Title',tit, 'PlotOpt',{{'b*'} {'ro'}});
    end
end % main iteration

if arg.Verbose >= 1
    fprintf('######################################################################\n');
end

end

