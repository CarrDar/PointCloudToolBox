function [ Out, Idx ] = SubSampling( In,varargin )
%SUBSAMPLING Create a sub-sample of a given PointCloud object.
%
% The general philosophy is the following: when no point positions are
% found, the function looks in the true positions. If any, it will work on
% these and return a PointCloud with TrueP set accordingly, instead of P.
%
% INPUT
%   In : a PointCloud object.
%
% Additional parameters that can be set via their field name :
%
%   Type :
%       UseAll | {Uniform} | Random | Normal | Curv | Box | Color |
%       Intensity | GridFit | UsefulVar
%
%       In the following Arg{x} referes to the xth entry of the 'Arg' field
%       (see below). 
%       Here are described the various options 'Type' can be set to :
%
%       UseAll : keep all points, just make a copy of the given cloud.
%
%       Uniform : make an uniform subsampling by a given reduction factor (Arg{1}).
%       Random : make a random subsampling. The probability for a point to
%       be taken is given by Arg{1}.
%
%       Auto : automatic reduction to a number of points of the order of (Arg{1}).
%           if Arg{1} > 0, Uniform method will be used.
%           if Arg{1} < 0, Random method will be used.
%           Arg{1} = 1000 (D);
%
%       Normal : choose points such that the distribution of normals among selected points is as large as possible.
%           Arg{1} sets the nb of points you wish approximately to keep. Default = 1000.
%              if Arg{1} > 0, Uniform method will be used.
%              if Arg{1} < 0, Random method will be used.
%               Arg{1} = 1000 (D);
%           Arg{2} sets the nb of buckets. Arg{2} = 3 (D).
%
%       Curv : choose points such that the distribution of curvature among selected points is as large as possible.
%           Arg{1} sets the nb of points you wish approximately to keep.
%              if Arg{1} > 0, Uniform method will be used.
%              if Arg{1} < 0, Random method will be used.
%               Arg{1} = 1000 (D);
%           Arg{2} sets the nb of buckets. Arg{2} = 5 (D).
%           Method similar to Normal but in 1D, thus much faster.
%           If you wish to use curvatures different from the default ones,
%           compute them before calling this function.
%
%       Box : smoothing of point cloud by averaging boxes of given size.
%           Arg{1} sets the size of the boxes. Arg{1} = [1 1 1] (D);
%           Arg{2} sets the minimum nb of points in a box to consider it.
%           Arg{2} = 5 (D).
%           The size of the boxes can be set to inf. In this case, a single
%           box is taken. Useful case : when the point cloud can be mapped
%           such that z = f(x,y) with an unique z, set 'Arg',{[5,5,inf]} and for each couple
%           of x and y, a single cell in z will be created, therefore a
%           single evaluation in z will be made.
%           By default, properties of points in a box are averaged out.
%           In Arg{3}, you can pass a function handle that will acts on the
%           points in the individual boxes and perform the desired action.
%           The function gets a PointCloud object containing the points in
%           the box and must return a PointCloud object.
%           The first entry will be used to set the properties of the box.
%           If empty, the box is dropped.
%           You can for instance design a function that remove the points
%           far away from the mean and recompute the mean after this
%           operation.
%           Example
%               Compute the mean position in the box (similar to the default):
%               pc2 = @(pc1) PointCloud('',mean(pc1.P,2));
%               pcout = SubSampling(pcin,'Type','Box','Arg',{[5,5,inf],3,pc2});
%
%       Color : keep points with a color (rgb) between given bounds.
%           Arg{1} : min color, in [0,1] or [0,255]. Arg{1} = [0 0 0] (D).
%           Arg{2} : max color, in [0,1] or [0,255]. Arg{2} = [255 255 255] (D).
%
%       Intensity : keep points with an intensity between given bounds.
%           Arg{1} : min intensity, in [0,1] or [0,255]. Arg{1} = 0 (D).
%           Arg{2} : max intensity, in [0,1] or [0,255]. Arg{2} = 1 (D).
%
%       GridFit : over (!) or subsample the PointCloud using GridFit.
%           Arg{1} sets the nb of points you wish approximately to keep.
%               Arg{1} = 1000 (D).
%               Uniform griding will be performed, keeping the ratio
%               between the widths of x and y.
%           Arg{2} : put in a cell extra options to pass to GridFit.
%
%           Note :
%               Your PointCloud must meet the GridFit requirements.
%               call GridFit directly if you know the nodes (control
%               points).
%
%       (UsefulVar) in prep.: keep point with corresponding to defined UsefulVar
%           property. A unique value or a range of values.
%           Arg{1} : Correspond to which useful variable to use for
%           subsampling.
%           Arg{2} : min value.
%           Arg{3} : max value.
%
%   Arg :
%       {Arg{1} Arg{2} Arg{3}}
%       Place in a cell the various arguments you want to pass to the
%       option you have selected by setting the "Type" field.
%       Example :
%           pc2 = SubSampling(pc1,'Type','Curv','Arg',{-100});
%
%   Idx :
%       {[]} | vector of indices.
%       If a vector of indices is provided, it will be used to reduce the size of
%       the given PointCloud. The "Type" field becomes dummy.
%
%
% Output
%   Out : a PointCloud object.
%   Idx : a vector of indices to the original points. If it makes sense.
%
% Usage
%   pc2 = SubSampling(pc1,'Type','Curv','Arg',{-1000, 10});
%
%AUTHOR  : Neal Gauvin (at a3 dot epfl dot ch)
%VERSION : 2.17
%STATUS  : OK
%DATE    : 22 june 2011.

%% Validate input arguments.
ip = inputParser;
ip.addRequired( 'In', @(x)isa(x,'PointCloud') );
validtype = {'UseAll','Uniform','Random','Auto','Normal','Curv','Box','Color','Intensity','GridFit','UsefulVar'};
ip.addParamValue('Type', 'Uniform', @(x)any(strcmpi(x,validtype)));
ip.addParamValue('Arg', {}, @(x)iscell(x));
ip.addParamValue('Idx',[],@(x) islogical(x) || isnumeric(x) );
ip.parse(In,varargin{:});
arg = ip.Results;
Idx = arg.Idx;

%% We have to determine the size of the Point Cloud, and which one is non-zero.
b = [false false];
s = [];
if ~isempty(In.P)
    b(1) = true;
    s = size(In.P,2);
end
if ~isempty(In.TrueP)
    b(2) = true;
    if isempty(s), s = size(In.TrueP,2); end
end
if isempty(s), fprintf('No given points. Nothing to do.'); Out=In; return; end
s = uint32(s);

% Sanity check for option Idx
if isempty(Idx)
    Idx = 1:s;
else
    if ~isvector(Idx) || ( islogical(Idx) && size(Idx,2) ~= s )
        error('SubSampling:WArg','Idx must be a vector of length equal to the nb of points.');
    end
    % If Idx then reduce the point cloud... 
    Out = PointCloud(In.Name);
    Out.TMode = In.TMode;
    if b(1), Out.P = In.P(:,Idx); end
    if b(2), Out.TrueP = In.TrueP(:,Idx); end
    if ~isempty(In.Colors), Out.Colors = In.Colors(:,Idx); end
    if ~isempty(In.Intensities), Out.Intensities = In.Intensities(:,Idx); end
    return
end


%% UseAll
if strcmpi(arg.Type,'UseAll')
    Out = PointCloud(In.Name, In); %make a copy
    return

%% Uniform
elseif strcmpi(arg.Type,'Uniform')
    % Initialisation and sanity checks
    if isempty(arg.Arg), arg.Arg = {10}; 
    elseif arg.Arg{1} <= 0
        error('SubSampling:WArg','Provide Arg bigger than 0.');
    end
    
    Idx = 1:uint32(arg.Arg{1}):s;
    
%% Random    
elseif strcmpi(arg.Type,'Random')
    % Initialisation and sanity checks
    if isempty(arg.Arg), arg.Arg = {0.1}; 
    elseif arg.Arg{1} < 0 || arg.Arg{1} > 1
        error('SubSampling:WArg','Provide Arg between 0 and 1.');
    end    

    mask = random('bino',1,arg.Arg{1}, 1, s)>0;
    Idx = Idx(mask);

    %% Auto
elseif strcmpi(arg.Type,'Auto')
    % Initialisation and sanity checks
    if isempty(arg.Arg), arg.Arg = {1000}; end
    if s<abs(arg.Arg{1}) || arg.Arg{1} == 0
        Idx = 1:s; 
    elseif arg.Arg{1} > 0
        inc = uint32( s/arg.Arg{1} );
        Idx = 1:inc:s;
    else
        mask = random('bino',1,abs(arg.Arg{1}/double(s)), 1, s)>0;
        Idx = Idx(mask);
    end
    
    %% Normal
elseif strcmpi(arg.Type,'Normal')    
    % Initialisation and sanity checks
    nbarg = size(arg.Arg,2);
    if nbarg < 2
        arg.Arg{2} = 3;
        if nbarg < 1, arg.Arg{1} = 1000; end
    end
    if arg.Arg{2} < 1
        warning('SubSampling:WArg','Provide a positive integer for the bucket size %i', arg.Arg);
        fprintf('Bucket size reset to 3');
        arg.Arg{2} = 3;
    end
    
    
    if s<abs(arg.Arg{1}) || arg.Arg{1} == 0
        Idx = 1:s;
    else
        if arg.Arg{2}^3 > 0.8*s
            warning('SubSampling:WArg','Beware, the total number of bucket %i is close or greater than the number of points %i',arg.Arg{2}^3,s);
        end
        
        % reduction factor
        redfact = abs(double(s)/arg.Arg{1} );
        
        % if only the true positions are given, need to copy them to the
        % measured ones in order to compute the normals
        if ~b(1), In.copyTrue2MeasPos(); end
        
        % compute normals
        In.ComputeNormals();
        
        % angles formed by the normals
        angles = acos(In.Normals);
        
        % put the angles in buckets of size Arg{2}
        B = ceil( angles .* arg.Arg{2} / pi );
        % avoid zero values in B
        B( B==0) = 1;
        % let's reduce the problem in 1D
        B1 = B(1,:) + (B(2,:)-1)*arg.Arg{2} + (B(3,:)-1)*arg.Arg{2}^2;
        % and loop on all possible value
        Idx = [];
        % preallocating does not reduce the CPU time
        %Idx = NaN(1,s); Inc = 1;
        for ib = 1:arg.Arg{2}^3
            idxt = find( B1==ib );
            % size of the bucket
            sb = size(idxt,2);
            % either keep equally distant point
            if arg.Arg{1} > 0
                idxt = idxt(1:uint32(redfact):sb);
            else
                mask = random('bino',1,1.0/redfact, 1, sb)>0;
                idxt = idxt(mask);
            end
            Idx = [Idx idxt];                                              %#ok<AGROW>
            %sidxt = size(idxt,2);
            %Idx(Inc:(Inc+sidxt)) = idxt;
            %Inc = Inc + sidxt;
        end
        %Idx = Idx( Idx > 0 );
    end

    %% Curv
elseif strcmpi(arg.Type,'Curv')    
    % Initialisation and sanity checks
    nbarg = size(arg.Arg,2);
    if nbarg < 2
        arg.Arg{2} = 5;
        if nbarg < 1, arg.Arg{1} = 1000; end
    end
    if arg.Arg{2} < 1
        warning('SubSampling:WArg','Provide a positive integer for the bucket size %i', arg.Arg);
        fprintf('Bucket size reset to 5');
        arg.Arg{2} = 5;
    end
    
    
    if s<abs(arg.Arg{1}) || arg.Arg{1} == 0
        Idx = 1:s;
    else
        if arg.Arg{2} > 0.8*s
            warning('SubSampling:WArg','Beware, the total number of bucket %i is close or greater than the number of points %i',arg.Arg{2},s);
        end
        
        % reduction factor
        redfact = abs(double(s)/arg.Arg{1} );
        
        % if only the true positions are given, need to copy them to the
        % measured ones in order to compute the normals
        if ~b(1), In.copyTrue2MeasPos(); end

        % compute curvatures
        In.ComputeCurvature;
        % Find min, max and compute bucket length.
        MinC = min(In.Curv);
        MaxC = max(In.Curv); %so that MaxC is included in the last bucket.
        MaxC = MaxC + 0.1*MaxC;
        Bl = (MaxC - MinC)/arg.Arg{2};
        B = floor( (In.Curv-MinC)/Bl );
        Idx = [];
        for ib = 0:arg.Arg{2}
            idxt = find( B==ib );
            % size of the bucket
            sb = size(idxt,2);
            % either keep equally distant point
            if arg.Arg{1} > 0
                idxt = idxt(1:uint32(redfact):sb);
            else
                mask = random('bino',1,1.0/redfact, 1, sb)>0;
                idxt = idxt(mask);
            end
            Idx = [Idx idxt];                                              %#ok<AGROW>
        end        
        
    end % curvature
    
    %% Box
elseif strcmpi(arg.Type,'Box')
    % To add : average out the intensity and the colors ?
    % Initialisation and sanity checks
    nbarg = size(arg.Arg,2);
    if nbarg < 3
        arg.Arg{3} = [];
        if nbarg < 2
            arg.Arg{2} = 5;
            if nbarg < 1, arg.Arg{1} = [1 1 1]; end
        end
    end
    if size(arg.Arg{1},1) ~= 3 && ~isreal(arg.Arg{1})
        error('SubSampling:WArg','Provide the size of the boxes in the following form Arg{1} = [bx by bz].');
    end
    if arg.Arg{2} < 1
        warning('SubSampling:WArg','Provide a positive integer for the min nb of required points in a box : %i', arg.Arg);
        fprintf('Bucket size reset to 5');
        arg.Arg{2} = 5;
    end
    if ~isempty(arg.Arg{3}) && ~isa(arg.Arg{3}, 'function_handle')
        error('SubSampling:WArg','Provide a function handle in the third entry of Arg.');
    end
    bl = arg.Arg{1}; bl = bl';    
    %Minimum nb of points in a box to consider it.
    NbPMin = arg.Arg{2};
    % A possible function handle
    fh = arg.Arg{3};
    
    % if only the true positions are given, need to copy them to the
    % measured ones in order to perform the boxing
    if ~b(1), In.copyTrue2MeasPos(); end

    % Get the minimum and maximum coordinates of the points
    PMax = max(In.P,[],2);
    PMin = min(In.P,[],2);    
    NbSB = 0; %Nb of boxes with less than NbPMin points.
    MNbPB = 0.; %Mean nb of points in kept boxes.
    NbB = (PMax - PMin) ./ bl;
    NbB(NbB==0) = 1; % a division by inf leads to zero, set the result to 1.
    NbB = uint32( NbB ); %Nb of boxes
    NbBT = NbB(1)*NbB(2)*NbB(3); %Total nb of boxes to treat.    

    fprintf('--------------------------------------------------\n');
    fprintf('SubSampling... \n');
    fprintf('Warning : I hope you know what you are doing here... \n');
    msg = 'Point cloud size before sampling';
    fprintf('%-65s : %i \n',msg,s);
    msg = 'Nb of boxes to treat';
    fprintf('%-65s : %i x %i x %i = %i\n',msg,NbB(1),NbB(2),NbB(3),NbBT);    
   
    %Matrix of indices on the boxes.
    BI = zeros(size(In.P));
    for ib = 1:3, BI(ib,:) = (In.P(ib,:)-PMin(ib))./bl(ib); end 
    BI = uint32(floor(BI)); %goes from 0 to NbB-1
    %1D reduction
    Bi = BI(1,:) + BI(2,:).*NbB(1) + BI(3,:).*(NbB(2)*NbB(3));
    
    %Loop on all the boxes to compute the average point position
    P = NaN(3,NbBT); %preallocation of the future point cloud
    colb = ~isempty(In.Colors);
    Col = NaN(3,NbBT); %preallocation of possible colors
    itsb = ~isempty(In.Intensities);
    Its = NaN(1,NbBT); %preallocation of possible intensities
    for ib = 1:NbBT
        pid = find( Bi==ib ); % retrieve indices for box ib
        if isempty(pid), continue; end        
        p = In.P(:,pid); % retrieve points in box ib
        % Do not consider box if insufficient nb of points
        if size(p,2) < NbPMin, NbSB = NbSB+1; continue; end
        
        % Two regimes now, with and without a given function handle
        % Yes, I could have defined default function handle returning the
        % mean, but this is slower. Beg your pardon.
        if ~isempty(fh)
            % Create PointCloud with points in the box
            pc = SubSampling(In,'Idx',pid);
            pc = fh(pc);
            if pc.IsEmpty, NbSB = NbSB+1; continue; end
            P(:,ib) = pc.P(:,1);
            if ~isempty(pc.Colors), Col(:,ib) = pc.Colors(:,1); end
            if ~isempty(pc.Intensities), Its(:,ib) = pc.Intensities(:,1); end
        else 
            % Compute a center-of-mass for the box
            P(:,ib) = mean(p,2);
            % You can also compute the rms and get rid of points too far from
            % the mean... provide a function handle !
            % Mean intensity and mean color
            if colb
                col = In.Colors(:,pid);
                Col(:,ib) = mean(col,2);
            end
            if itsb
                its = In.Intensities(1,pid);
                Its(1,ib) = mean(its,2);
            end
        end  % with and without a given function handle 
        MNbPB = MNbPB + size(p,2);
        
    end %loop box

    % Get rid of NaN entries.
    Pid = not(isnan(P(1,:)));
    P = P(:,Pid);
    % Compute mean nb of points in kept boxes.
    MNbPB = MNbPB / size(P,2);
    Out = PointCloud(In.Name); Out.TMode = In.TMode;
    if b(1), Out.P = P; end
    % The true positions are lost when boxing on the simulated positions.
    if b(2) && ~b(1), Out.TrueP = P; end
    % Fill Colors and Intensities, if computed
    if colb
        Colid = not(isnan(Col(1,:)));
        Out.Colors = Col(:,Colid);
    end
    if itsb
        Itsid = not(isnan(Its(1,:)));
        Out.Intensities = Its(1,Itsid);
    end
    Idx = NaN;
    msg = ' ';
    fprintf('Nb of boxes with nb of points in [1,%i] %-26s : %i \n',NbPMin,msg,NbSB);
    msg = 'Mean nb of points in kept boxes';
    fprintf('%-65s : %.2f\n',msg,MNbPB);
    msg = 'Point cloud size after sampling';
    fprintf('%-65s : %i \n',msg,size(P,2));
    fprintf('--------------------------------------------------\n');
    return;

%% Color
elseif strcmpi(arg.Type,'Color')

    if isempty(In.Colors)
        error('SubSampling:NoC','No colors available for sampling...');
    end
    
    % Set the initial bounds and sanity checks
    nbarg = size(arg.Arg,2);
    if nbarg < 2
        arg.Arg{2} = [255 255 255];
        if nbarg < 1, arg.Arg{1} = [0 0 0]; end
    end
    if ~isequal( size(arg.Arg{1}), [1 3])
        error('SubSampling:WArg','Provide min color in the RGB format [r g b], %f', arg.Arg{1});
    end
    if ~isequal( size(arg.Arg{2}), [1 3])
        error('SubSampling:WArg','Provide max color in the RGB format [r g b], %f', arg.Arg{2});
    end
    % set everything within [0,1]
    if any(arg.Arg{1} > 1), arg.Arg{1} = arg.Arg{1} ./ 255; end
    if any(arg.Arg{2} > 1), arg.Arg{2} = arg.Arg{2} ./ 255; end
    Min = arg.Arg{1}; Max = arg.Arg{2};
    Col = In.Colors;
    if any(Col(1,:) > 1), Col = Col ./ 255; end
    % Apply filter
    idxi1 = Col(1,:) >= Min(1); idxa1 = Col(1,:) <= Max(1);
    idxi2 = Col(2,:) >= Min(2); idxa2 = Col(2,:) <= Max(2);
    idxi3 = Col(3,:) >= Min(3); idxa3 = Col(3,:) <= Max(3);
    Idx = logical(idxi1 .* idxa1 .* idxi2 .* idxa2 .* idxi3 .* idxa3);

%% Intensity
elseif strcmpi(arg.Type,'Intensity')

    if isempty(In.Intensities)
        error('SubSampling:NoC','No intensities available for sampling...');
    end
    
    % Set the initial bounds and sanity checks
    nbarg = size(arg.Arg,2);
    if nbarg < 2
        arg.Arg{2} = 1.;
        if nbarg < 1, arg.Arg{1} = 0.; end
    end
    if ~isequal( size(arg.Arg{1}), [1 1]) || arg.Arg{1} < 0.
        error('SubSampling:WArg','Provide a positive real for the min intensity, %f', arg.Arg{1});
    end
    if ~isequal( size(arg.Arg{2}), [1 1]) || arg.Arg{2} < 0.
        error('SubSampling:WArg','Provide a positive real for the max intensity, %f', arg.Arg{2});
    end
    % set everything within [0,1]
    Int = In.Intensities;
    IMax = max(Int);
    if any(arg.Arg{1} > 1), arg.Arg{1} = arg.Arg{1} ./ IMax; end
    if any(arg.Arg{2} > 1), arg.Arg{2} = arg.Arg{2} ./ IMax; end
    Min = arg.Arg{1}; Max = arg.Arg{2};
    if any(Int(1,:) > 1), Int = Int ./ IMax; end
    % Apply filter
    idxi1 = Int(1,:) >= Min; idxa1 = Int(1,:) <= Max;
    Idx = logical(idxi1 .* idxa1);    

%% GridFit
elseif strcmpi(arg.Type,'GridFit')

    nbarg = size(arg.Arg,2);
    if nbarg < 2
        arg.Arg{2} = {};
        if nbarg < 1, arg.Arg{1} = 1000; 
        elseif arg.Arg{1} < 0
            error('SubSampling:WArg','Provide a positive integer for the number of desired points.');            
        end
    end
    
    % Do we need to work on the TrueP ?
    TP = ~b(1);
    
    % From what is given in arguments, set the nodes accordingly.
    % To set the number of nodes nx in x and ny in y, I want to keep the
    % ratio nx/ny = Lx/Ly.
    if TP
        Lx = max(In.TrueP(1,:),2) - min(In.TrueP(1,:),2);
        Ly = max(In.TrueP(2,:),2) - min(In.TrueP(2,:),2);
    else
        Lx = max(In.P(1,:),2) - min(In.P(1,:),2);
        Ly = max(In.P(2,:),2) - min(In.P(2,:),2);
    end
    nx = sqrt( arg.Arg{1} * Lx / Ly );
    ny = arg.Arg{1} / nx;
    % Create the mesh
    opt = arg.Arg{2};
    if TP, opt{end+1} = 'TrueP'; opt{end+1} = true; end
    mpc = GridFit(In, nx, ny, opt{:} );
    % Return the new point cloud
    Out = mpc.PointCloud(TP); Out.TMode = In.TMode;
    Idx = NaN;
    return

%% UsefulVar -> in prep.
% elseif strcmpi(arg.Type,'UsefulVar')
% 
%     if isempty(In.UsefulVar)
%         error('SubSampling:NoC','No Useful Variables available for sampling...');
%     end
%     
%     % Set the initial bounds and sanity checks
%     nbarg = size(arg.Arg,2);
%      if nbarg < 1
%         arg.Arg{2} = 1.;
%         if nbarg < 1, arg.Arg{1} = 0.; end
%     end
%     if nbarg < 2
%         arg.Arg{2} = 1.;
%         if nbarg < 1, arg.Arg{1} = 0.; end
%     end
%     if ~isequal( size(arg.Arg{1}), [1 1]) || arg.Arg{1} < 0.
%         error('SubSampling:WArg','Provide a positive real for the min intensity, %f', arg.Arg{1});
%     end
%     if ~isequal( size(arg.Arg{2}), [1 1]) || arg.Arg{2} < 0.
%         error('SubSampling:WArg','Provide a positive real for the max intensity, %f', arg.Arg{2});
%     end
%     % set everything within [0,1]
%     Int = In.UsefulVar;
%     IMax = max(Int);
%     if any(arg.Arg{1} > 1), arg.Arg{1} = arg.Arg{1} ./ IMax; end
%     if any(arg.Arg{2} > 1), arg.Arg{2} = arg.Arg{2} ./ IMax; end
%     Min = arg.Arg{1}; Max = arg.Arg{2};
%     if any(Int(1,:) > 1), Int = Int ./ IMax; end
%     % Apply filter
%     idxi1 = Int(1,:) >= Min; idxa1 = Int(1,:) <= Max;
%     Idx = logical(idxi1 .* idxa1);    
       
else
    error('SubSampling:WType','Unknown Type %s', arg.Type);
end

Out = PointCloud(In.Name);
Out.TMode = In.TMode;
if b(1), Out.P = In.P(:,Idx); end
if b(2), Out.TrueP = In.TrueP(:,Idx); end
if b(1) && strcmp(arg.Type,'Normals') && ~isempty(In.Normals), Out.Normals = In.Normals(:,Idx); end
if b(1) && strcmp(arg.Type,'Curv') && ~isempty(In.Curv), Out.Curv = In.Curv(:,Idx); end
if ~isempty(In.Boundaries), Out.Boundaries = In.Boundaries(:,Idx); end
if ~isempty(In.UsefulVar), Out.UsefulVar = In.UsefulVar(:,Idx); end
if ~isempty(In.Colors), Out.Colors = In.Colors(:,Idx); end
if ~isempty(In.Intensities), Out.Intensities = In.Intensities(:,Idx); end

end

