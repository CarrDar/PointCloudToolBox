classdef PointCloud < handle
    %POINTCLOUD Class to define a cloud of 3D points.
    % Type doc PointCloud for a more detailed description.
    %
    % PointCloud Properties:
    %   Name         - The name of the cloud.
    %   P            - The true positions of the cloud, if a simulated one.
    %   TrueP        - The true positions of the cloud, if a simulated one.
    %   TrueM        - True match indices to another PointCloud.
    %   Colors       - Store colors for each points.
    %   Intensities  - Store intensity value associated to any point.
    %   Normals      - Normals to the points.
    %   Curv         - Curvature of the surface at the point positions.
    %   Boundaries   - Index to the border points.
    %   UsefulVar    - Keep useful variables for segmentation.
    %   DT           - Keep computed Delaunay triangulation.
    %   GLTree       - Keep computed GLTree.
    %   KDTree       - Keep computed kd tree.
    %   TMode        - The way transformations are applied to the cloud.
    %   TLSPos       - Give the position of the TLS of the cloud.
    %   TLSAttribute - Give each point an attribute according to TLSPos.
    %
    %
    % PointCloud Methods:
    %   PointCloud                   - The constructor.
    %   delete                       - Properly delete class.
    %   copyTrue2MeasPos             - Copy the "true" positions to the "measured" ones.
    %   plot3                        - Plot the point cloud positions.
    %   PlotPositionsWithColors      - Plot the point cloud with the Colors.
    %   PlotPositionsWithIntensities - Plot the point cloud with the Intensities.
    %   PlotPCLViewer                - Plot for large point cloud positions with Colors or Intensities.
    %   PlotNormals                  - Plot the computed normals.
    %   PlotCurvature                - Plot the computed Curvatures.
    %   transform                    - Transform the point cloud.
    %   MoveToCM                     - Move to the center-of-mass of another given cloud.
    %   addNoise                     - Add simulated noise to the true point positions.
    %   ComputeNormals               - Compute the least squares normal estimation of the points.
    %   ComputeOptimalNormals        - Compute the optimal normals based adaptative number of neighbours.
    %   NormalsOutTopo               - Correct Normals orientation to be out of the topography.
    %   ComputeCurvature             - Compute the curvatures at each point.
    %   ComputeBoundaries            - Compute the Boundary points.
    %   ComputeDelaunayTriangulation - Compute a Delaunay Triangulation.
    %   ComputeGLTree                - Compute a GLTree.
    %   ComputeKDTree                - Compute a k-d search tree.
    %   ComputeTrueDistance          - Compute the true distance between this and a given PointCloud.
    %   SaveInASCII                  - Save point cloud in ASCII format.
    %   SaveInPCD                    - Save point cloud in PCD format for PointCloud Library.
    %   ImportDataFromASCII          - Import data from an ASCII file.
    %   HasTrueP                     - Return true if the point cloud has true positions.
    %   Size                         - What is the size of the point cloud ?
    %   IsEmpty                      - Is the point cloud empty ?
    %   WhatColor                    - What is the color of the closest point ?
    %   WhatIntensity                - What is the intensity of the closest point ?
    %   GetMissingPropFromPC         - Complete properties of object by getting the missing ones.
    %   RemoveNans                   - Remove any nans in P and TrueP.
    %   MeshPointCloud               - Create a MeshPointCloud out of this PointCloud.
    %   Add                          - Add the content of a given point cloud to this one.
    %
    % Known problems/limitations:
    % - The depending variables like Boundaries and KDTree should not
    % always be reset when setting P, especially when the cloud is
    % transformed.
    %
    %AUTHOR  : Neal Gauvin (at a3 dot epfl dot ch)
    %VERSION : 2.5.9
    %STATUS  : OK
    %DATE    : 26 mai 2011
    %
    %
    %   UPDATED BY
    %
    %AUTHOR  : Dario Carrea (at unil dot ch)
    %VERSION : 3.2.2
    %STATUS  : OK
    %DATE    : 9 juillet 2013
    %
    % 3.2.1 -> Fix bug; add Boundaries, UsefulVar, TLSPos, TLSAttribute,
    %          ComputeOptimalNormals, SaveInPCD, PlotPCLViewer.
    % 3.2.2 -> skip unnecessary fields in ImportDataFromASCII using
    %          "Ignore".
    % 
    %
    %
    
    properties
        Name = ''; % The name of the cloud.
        P         % The cloud, a set of n 3D points represented by a 3xn matrix.
        TrueP     % The true positions of the cloud, if a simulated one.
        TrueM = [];% True match indices to another PointCloud.
        Colors = []; % Store colors for each points (3xn matrix), like rgb.
        Intensities = []; % Store intensity value associated to any point (1xn matrix).
        Normals = []; % Normals to the points.
        Curv = []; % Curvature of the surface at the point positions, or any scalar transformation-independant feature.
        Boundaries = []; % Index to the border points.
        UsefulVar = []; % Keep useful variables for segmentation.
        TLSPos = [];       % Give the position of the TLS of the cloud.
        TLSAttribute = []; % Give each point an attribute according to TLSPos.
        DT = [];  % Keep computed Delaunay triangulation.
        GLTree = []; % Keep computed GLTree.
        KDTree = []; % Keep computed kd tree.
        % Transformation can be performed in different way, by setting TMode :
        % 0 : wrst point (0,0,0).
        % 1 : wrst mass center of the point cloud.
        % [x,y,z] : wrst to the given point.
        TMode = 0;
    end
    
    methods
        
        function obj = PointCloud( name, P, TrueP )
            % The constructor
            if nargin > 0
                obj.Name = name;
                % copy constructor
                if nargin > 1 && isa(P,'PointCloud')
                    obj.P = P.P;
                    obj.TrueP = P.TrueP;
                    obj.TrueM = P.TrueM;
                    obj.Colors = P.Colors;
                    obj.Intensities = P.Intensities;
                    obj.Normals = P.Normals;
                    obj.Curv = P.Curv;
                    obj.Boundaries = P.Boundaries;
                    obj.UsefulVar = P.UsefulVar;
                    obj.TLSPos = P.TLSPos;
                    obj.TLSAttribute = P.TLSAttribute;
                    obj.DT = P.DT;
                    obj.KDTree = P.KDTree;
                    %obj.GLTree = P.GLTree; %dangerous to copy.
                    obj.TMode = P.TMode;

                    return
                end
                if nargin > 1
                    obj.P = P;
                    if nargin > 2
                        obj.TrueP = TrueP;
                    end
                end
            end
        end % constructor
        
        function delete(obj)
            % Properly delete class.
            obj.KillGLTree();
        end
        
        function obj = set.P( obj, P )                                     %#ok<*MCHV2>
            if(size(P,1) == 3), obj.P=P;
            elseif(size(P,2) == 3), obj.P=P';
            elseif isempty(P), obj.P=[];                              %#ok<*BDSCA>
            else error('Please provide a 3xn matrix !');
            end
            % P has changed, reset depending variables
            obj.Normals = [];
            obj.Curv = [];
            obj.Boundaries = [];
            obj.UsefulVar = [];
            obj.DT = [];
            obj.KDTree = [];
            obj.KillGLTree();
            %obj.TLSPos = [];
            obj.TLSAttribute = [];
        end
        function obj = set.TrueP( obj, P )
            if(size(P,1) == 3), obj.TrueP=P; return;
            elseif(size(P,2) == 3), obj.TrueP=P'; return;
            elseif isempty(P), obj.TrueP=[]; return;                      %#ok<*MCSUP>
            else error('Please provide a 3xn matrix !');
            end
        end  % set
        
        function obj = set.Colors( obj, C )
            if(size(C,1) == 3), obj.Colors=C; return;
            elseif(size(C,2) == 3), obj.Colors=C'; return;
            elseif isempty(C), obj.Colors=[]; return;                      %#ok<*MCSUP>
            else error('Please provide a 3xn matrix !');
            end
        end  % set
        
        function obj = set.Intensities( obj, I )
            if(size(I,1) == 1), obj.Intensities=I; return;
            elseif(size(I,2) == 1), obj.Intensities=I'; return;
            elseif isempty(I), obj.Intensities=[]; return;                      %#ok<*MCSUP>
            else error('Please provide a 1xn matrix !');
            end
        end  % set
        
        function obj = set.TMode( obj, tmode )
            s = size(tmode);
            
            if isequal(s,[1 1])
                if tmode < 0 || tmode > 1
                    error('Unknown TMode. Choose between 0 or 1.');
                end
                obj.TMode = tmode; return;
            elseif isequal(s,[3 1])
                obj.TMode = tmode; return;
            elseif isequal(s,[1 3])
                obj.TMode = tmode'; return;
            else
                error('Provide reference point as 3x1 or 1x3 array.');
            end
        end
        
        function A = saveobj(obj)
            % Allows to save PointCloud object by using the built-in save function.
            A.Name = obj.Name;
            A.P = obj.P;
            A.TrueP = obj.TrueP;
            A.Colors = obj.Colors;
            A.Intensities = obj.Intensities;
            A.Normals = obj.Normals;
            A.Curv = obj.Curv;
            A.Boundaries = obj.Boundaries;
            A.UsefulVar = obj.UsefulVar;
            A.TLSPos = obj.TLSPos;
            A.TLSAttribute = obj.TLSAttribute;
            A.DT = obj.DT;
            A.KDTree = obj.KDTree;
            % GLTree cannot be saved (pointer to Cpp object).
            %A.DT = obj.GLTree;
            A.TMode = obj.TMode;
        end
        
        function obj = KillGLTree( obj )
            % Properly delete a GLTree object.
            if ~isempty( obj.GLTree )
                try
                    DeleteGLTree3D(obj.GLTree);
                catch exception
                    throw(exception)
                end
                obj.GLTree = [];
            end
        end % KillGLTree
        
        function obj = copyTrue2MeasPos( obj )
            % Copy the "true" positions to the "measured" ones.
            if size(obj.TrueP,2)<1
                warning('PointCloud:NoTP',...
                    'You need to provide first some true points.');
                return
            end
            obj.P = obj.TrueP;
        end % copyTrue2MeasPos
        
        function h = plot3(obj,sim,varargin)
            % Plot the point cloud positions.
            % Overload the plot3 built-in method.
            %
            % Input:
            %   sim : bool to plot true position or not (default false).
            %   Any other argument will be passed the built-in plot3.
            
            if nargin < 3
                varargin = {'b.'};
                if nargin < 2
                    sim = false;
                end
            end %initialisation
            
            % sanity checks
            if sim == true && size(obj.TrueP,2)==0
                warning('PointCloud:NoTP',...
                    'I cannot plot the true positions if they are not given...');
                return
            end
            if sim == false && size(obj.P,2)==0
                warning('PointCloud:NoP',...
                    'I cannot plot the positions if they are not given...');
                return
            end % checks
            
            % plot !
            if sim==true
                h = plot3(obj.TrueP(1,:),obj.TrueP(2,:),obj.TrueP(3,:),varargin{:});
            else
                h = plot3(obj.P(1,:),obj.P(2,:),obj.P(3,:),varargin{:});
            end
        end % plot
        
        function h = PlotPositionsWithColors(obj,scale,varargin)
            % Plot the point cloud with the Colors (given in [0,255] or [0,1])
            %
            % Input :
            %   scale : a scale to pass to scatter3. Default 4.
            %   Any other given argument will be passed to scatter3.
            
            if isempty( obj.Colors )
                warning('PointCloud:NoC','No colors to plot...');
                h = obj.plot3(false,varargin);
                return
            end
            
            % Set scale to 4 by default.
            if nargin < 2
                scale=4;
            elseif scale <= 0
                warning('PointCloud:WS','Provide a positive scalar for the scale. Reset to 4.');
                scale = 4;
            end
            %Create figure with white background
            figure1 = figure('Color',[1 1 1]);
            % Create axes
            axes1 = axes('Parent',figure1);
            view(3);
            hold(axes1,'all');
            % Create title
            title('Point Positions with related Colors');
            hold on;
            C = transpose(obj.Colors);
            % Colors might be given as integers in [0,255] (rgb), Matlab
            % only accepts doubles in [0,1].
            if any(C(:,1)>1)
                C = C ./ 255;
            end
            
            h=scatter3(obj.P(1,:),obj.P(2,:),obj.P(3,:), scale, C, varargin{:} );
            hold off;
            
        end %PlotPositionsWithColors
        
        function h = PlotPositionsWithIntensities(obj,scale,varargin)
            % Plot the point cloud with the Intensities (given in [0,255] or [0,1])
            %
            % Input :
            %   scale : a scale to pass to scatter3. Default 4.
            %   Any other given argument will be passed to scatter3.
            
            if isempty( obj.Intensities )
                warning('PointCloud:NoC','No intensities to plot...');
                h = obj.plot3(false,varargin);
                return
            end
            
            % Set scale to 4 by default.
            if nargin < 2
                scale=4;
            elseif scale <= 0
                warning('PointCloud:WS','Provide a positive scalar for the scale. Reset to 4.');
                scale = 4;
            end
            %Create figure with white background
            figure1 = figure('Color',[1 1 1]);
            % Create axes
            axes1 = axes('Parent',figure1);
            view(3);
            hold(axes1,'all');
            % Create title
            title('Point Positions with related Intensities');
            hold on;
            % Intensities might be given as integers in [0,255], Matlab
            % only accepts doubles in [0,1].
            I = obj.Intensities;
            if any(I>1)
                I = I ./ 255;
            end
            % Quick and dirty way to convert grayscale into rgb for plotting, because the effective luminance you get depends on the actual luminance of the R, G and B subpixels of the device that you're using.
            C = zeros(obj.Size,3);
            for ic = 1:3, C(:,ic) = I; end
            h=scatter3(obj.P(1,:),obj.P(2,:),obj.P(3,:), scale, C, varargin{:} );
            hold off;
            
        end %PlotPositionsWithIntensities
        
        function h=PlotPCLViewer(obj,fields,args)
            
            %PlotPCLViewer View a point cloud using PCL
            %
            % PlotPCLViewer convert PointCloud class object into the point cloud P to a temporary file and invokes
            % the PCL point cloud viewer for fast display and visualization.  The columns of P
            % represent the 3D points.
            %
            % If M=3 then the rows are x, y, z.
            % If M=6 then the rows are x, y, z, R, G, B where R,G,B are in the range 0
            % to 1.
            %
            % PlotPCLViewe(P, ARGS) as above but the optional arguments ARGS are passed to the
            % PCL viewer.  For example:
            %
            %         pclviewer( rand(3,1000), '-ps 2 -ax 1' )
            %
            % Notes::
            % - Only the "x y z" and "x y z rgb" field formats are currently supported.
            % - The file is written in ascii format.
            % - When viewing colored point clouds in pcl_viewer remember to toggle to
            %
            %  Update for IGAR_pctb by Dario Carrea from an original code
            %  written by Peter I. Corke 2013
            %
            % TODO
            % - add color
            % change the next line to suit your operating system
            
            warning('You need the PCL 1.6.0 or higher install on your computer.');
            viewer = 'C:\MATLAB\PCL_1.6.0\bin\pcd_viewer_release.exe';
            
            pointfile = [tempname '.pcd'];
            
            if nargin < 3
                args = '-ps 2 -ax 1';
            end
            
            % Need a translation to avoid large coordinates that may cause
            % errors
            
            TT(1)=-(mean(obj.P(1,:)));
            TT(2)=-(mean(obj.P(2,:)));
            TT(3)=-(mean(obj.P(3,:)));
            
            TM = eye(4);
            TM(1:3,4) = TT;
            % Apply transformation
            h=obj.transform(TM);
            
            SaveInPCD(obj,pointfile,fields);
            
            system(sprintf('head -20 %s', pointfile));
            
            system(sprintf('%s %s %s &', ...
                viewer, pointfile, args));
            
            delete(pointfile);
            
            TM = eye(4);
            TM(1:3,4) = -(TT);
            obj.transform(TM);
        end %PlotPCLViewer
        
        function h = PlotNormals(obj,varargin)
            % Plot the computed normals, placed in the Normals field.
            %
            % Any given argument will be passed to the called quiver3 plot
            % function.
            
            % Sanity check
            if isempty(obj.Normals)
                warning('PointCloud:NoNorm','No Normals to plot. Compute them first !');
                return
            end
            % Set scale to o by default.
            if nargin < 2, varargin{1}=0; end
            
            h=quiver3(obj.P(1,:),obj.P(2,:),obj.P(3,:),obj.Normals(1,:),obj.Normals(2,:),obj.Normals(3,:),varargin{:});
            
        end % PlotNormals
        
        function h = PlotCurvature(obj,scale,varargin)
            % Plot the computed Curvatures.
            %
            % Input :
            %   scale : a scale to pass to scatter3. Default 4.
            %   Any other given argument will be passed to scatter3.
            
            % Sanity check
            if isempty(obj.Curv)
                warning('PointCloud:NoCurv','No Curvatures to plot. Compute them first !');
                return
            end
            
            % Set scale to 4 by default.
            if nargin < 2
                scale=4;
            elseif scale <= 0
                warning('PointCloud:WS','Provide a positive scalar for the scale. Reset to 4.');
                scale = 4;
            end
            %Create figure with white background
            figure1 = figure('Color',[1 1 1]);
            % Create axes
            axes1 = axes('Parent',figure1);
            view(3);
            hold(axes1,'all');
            % Create title
            title('Curvature');
            hold on;
            h=scatter3(obj.P(1,:),obj.P(2,:),obj.P(3,:), scale, obj.Curv, varargin{:} );
            colorbar('peer',axes1);
            hold off;
            
        end % PlotCurvature
        
        function h = PlotBoundaries(obj,varargin)
            % Plot the computed Boundary points.
            %
            % Input :
            %   scale : a scale to pass to scatter3. Default 4.
            %   Any other given argument will be passed to scatter3.
            
            % Sanity check
            if isempty(obj.Boundaries)
                warning('PointCloud:NoBoundaries','No Boundaries to plot. Change angle in ComputeBoundaries !');
                return
            end
            %initialisation            
            if nargin < 3
                varargin = {'k.'};
            end
            
            %Create figure with white background
            figure1 = figure('Color',[1 1 1]);
            % Create axes
            axes1 = axes('Parent',figure1);
            view(3);
            hold(axes1,'all');
            % Create title
            title('Boundary Points');
            hold on;
            h=scatter3(obj.P(1,obj.Boundaries(1,:)),obj.P(2,obj.Boundaries(1,:)),obj.P(3,obj.Boundaries(1,:)), varargin{:} );
            axis equal
            hold off
            
        end % PlotBoundaries
        
        function obj = transform(obj,AT)
            % TRANSFORM Transform the point cloud.
            %
            % Input :
            %   AT : a transform matrix.
            
            if isempty(obj.P) && isempty(obj.TrueP), return; end
            
            % preallocation
            TP = zeros(size(obj.P));
            N = [];
            % Eventually save the curvatures.
            C = obj.Curv;
            P = [];
            % computation
            % start with the normals as they get erased in set P.
            if ~isempty(obj.Normals)
                N = zeros(size(obj.Normals));
                N(1,:) = obj.Normals(1,:)*AT(1,1)+obj.Normals(2,:)*AT(1,2)+obj.Normals(3,:)*AT(1,3);
                N(2,:) = obj.Normals(1,:)*AT(2,1)+obj.Normals(2,:)*AT(2,2)+obj.Normals(3,:)*AT(2,3);
                N(3,:) = obj.Normals(1,:)*AT(3,1)+obj.Normals(2,:)*AT(3,2)+obj.Normals(3,:)*AT(3,3);
            end
            % Transform the point cloud position according to set TMode.
            if ~isequal(obj.TMode,0)
                if isempty( obj.P )
                    refp = mean(obj.TrueP,2);
                else
                    refp = mean(obj.P,2);
                end
                if size(obj.TMode,1)==3
                    refp = refp + obj.TMode;
                end
                refpT = repmat(refp,1,obj.Size);
                if ~isempty(obj.P), obj.P = obj.P - refpT; end
                if ~isempty(obj.TrueP), obj.TrueP = obj.TrueP - refpT; end
                AT(1:3,4) = AT(1:3,4) + refp;
            end
            if ~isempty(obj.P)
                TP(1,:) = obj.P(1,:)*AT(1,1)+obj.P(2,:)*AT(1,2)+obj.P(3,:)*AT(1,3)+AT(1,4);
                TP(2,:) = obj.P(1,:)*AT(2,1)+obj.P(2,:)*AT(2,2)+obj.P(3,:)*AT(2,3)+AT(2,4);
                TP(3,:) = obj.P(1,:)*AT(3,1)+obj.P(2,:)*AT(3,2)+obj.P(3,:)*AT(3,3)+AT(3,4);
                obj.P = TP;
            end
            if ~isempty(obj.TrueP)
                TP(1,:) = obj.TrueP(1,:)*AT(1,1)+obj.TrueP(2,:)*AT(1,2)+obj.TrueP(3,:)*AT(1,3)+AT(1,4);
                TP(2,:) = obj.TrueP(1,:)*AT(2,1)+obj.TrueP(2,:)*AT(2,2)+obj.TrueP(3,:)*AT(2,3)+AT(2,4);
                TP(3,:) = obj.TrueP(1,:)*AT(3,1)+obj.TrueP(2,:)*AT(3,2)+obj.TrueP(3,:)*AT(3,3)+AT(3,4);
                obj.TrueP = TP;
            end
            if ~isempty(obj.TLSPos)
                P = zeros(size(obj.TLSPos));
                P(1,1) = obj.TLSPos(1,1)*AT(1,1)+obj.TLSPos(2,1)*AT(1,2)+obj.TLSPos(3,1)*AT(1,3)+AT(1,4);
                P(2,1) = obj.TLSPos(1,1)*AT(2,1)+obj.TLSPos(2,1)*AT(2,2)+obj.TLSPos(3,1)*AT(2,3)+AT(2,4);
                P(3,1) = obj.TLSPos(1,1)*AT(3,1)+obj.TLSPos(2,1)*AT(3,2)+obj.TLSPos(3,1)*AT(3,3)+AT(3,4);
            end
            obj.Normals = N;
            obj.Curv = C;
            obj.TLSPos = P;
        end % transform
        
        function obj = MoveToCM( obj, PC, Out )
            % MOVETOCM Move to the center-of-mass (CM) of another given cloud.
            %
            % Input :
            %   PC : a PointCloud object.
            %   Out : an ICPVarOut object to store details of the operation (optional).
            
            % Start a stopwatch timer.
            tic;
            % Sanity check
            if nargin < 2 || ~isa(PC,'PointCloud')
                error('PointCloud:NoPC','Provide a PointCloud class. You gave %s.',class(PC));
            end
            
            % Compute the mean position of the source and the target.
            mps = mean(obj.P,2); mpt = mean(PC.P,2);
            
            TM = eye(4);
            TT = mpt - mps; % Compute translation
            TM(1:3,4) = TT;
            
            % Apply transformation
            obj.transform(TM);
            
            % Save the parameters
            t = toc;
            if nargin >= 3 && isa(Out,'ICPVarOut')
                Out.AddEntry( TM, uint32(ICPStatus.Preprocessing), t, NaN(5,1) );
            end
        end % MoveToCM
        
        function obj = addNoise( obj, varargin )
            %ADDNOISE Add simulated noise to the true point positions.
            %
            % Possible noise to simulate :
            % - Gaussian position smearing.
            % - Outliers : simulate completely wrong position.
            % - Drop out some points : they haven't be measured.
            %
            % Additional parameters that can be set via their field name :
            %
            % OutlierProb : probability for a point to be an outlier.
            %           Real between 0 and 1 {0}
            %           Outlier points are moved anywhere in a box defined
            %           by the minimum and maximum values of the point
            %           clouds.
            % DropOutProb : probability for a point to be a dropout.
            %           Real between 0 and 1 {0}
            %           The dropped out points are replaced by nans.
            % GaussSmear  : Gaussian smearing in 3D of point positions.
            %         vector of 2 real, the sigma and norm parameters
            %         ([0.01,1]) of the gaussian distribution.
            %
            % Beware : do not assume this function alyways does something with a
            % physical meaning !
            %
            % Usage :
            % pc.addNoise('OutlierProb',0.001, 'GaussSmear', [0.1 1],'DropOutProb', 0.01 );
            
            % sanity check
            if size(obj.TrueP,2)<1
                warning('PointCloud:NoTP',...
                    'You need to provide first some true points.');
                return
            end
            
            ip = inputParser;
            ip.addParamValue('DropOutProb', 0, @(x)isreal(x) && x>=0 && x<=1 );
            ip.addParamValue('OutlierProb', 0, @(x)isreal(x) && x>=0 && x<=1 );
            ip.addParamValue('GaussSmear', [0.01 1], @(x)isreal(x) && size(x,2)==2 );
            ip.parse(varargin{:}); arg = ip.Results;
            
            Np = size(obj.TrueP);
            
            % Add error on point position (gaussian)
            m=0; sig=arg.GaussSmear(1); norm=arg.GaussSmear(2);
            obj.P = obj.TrueP + norm.*random('Normal',m, sig, Np);
            
            % Add some outliers and dropouts
            Pmin = min(obj.TrueP, [], 2);
            Pmax = max(obj.TrueP, [], 2);
            
            if arg.OutlierProb > 0
                for k = find(random('bino',1,arg.OutlierProb, 1, Np(2))>0)
                    r = random('unif',Pmin,Pmax);
                    obj.P(:,k) = r(:);
                end
            end
            if arg.DropOutProb > 0
                k = random('bino',1,arg.DropOutProb, 1, Np(2))>0;
                obj.P(:,k) = NaN;
                %obj.RemoveNans; % Not advised here.
            end
            
        end % addNoise
        
        function obj = ComputeNormals( obj, varargin )
            % COMPUTENORMALS Compute the least squares normal estimation of the points.
            %
            % Additional parameters that can be set via their field name :
            %
            % Force
            %   {false} | true
            %   Force the computing of the curvature even if already
            %   computed.
            %
            % NbNeighbours
            %   {6} | scalar in [1,inf]
            %  Set the number of neighbours to be used in the computation.
            %  A value of nbn=24 is 5x5NN can be an optimal compromise using ILRIS Optech scanner.
            %  A. Abellan, M. Jaboyedoff, T. Oppikofer, and J.M. Vilaplana
            %  Detection of millimetric deformation using a terrestrial
            %  laser scanner: experiment and application to a rockfall event.
            %  In Nat. Hazards Earth Syst. Sci., 9, pp. 365-372, 2009.
            
            % Validate input arguments.
            ip = inputParser;
            ip.addParamValue('Force', false, @(x)islogical(x));
            ip.addParamValue('NbNeighbours',6,@(x)isscalar(x)&&x>0);
            ip.parse(varargin{:});
            arg = ip.Results;
            
            % No need to do it again if already done
            if not(arg.Force) && ~isempty( obj.Normals ), return; end
            
            %As ComputeCurvature also computes the normals, let's do
            %everything together.
            obj.ComputeCurvature('Type','estcurv','Force',true,'NbNeighbours',arg.NbNeighbours);
            
        end % ComputeNormals
      
        function obj = ComputeOptimalNormals( obj, varargin )
            % COMPUTE_OPTIMAL_NORMALS is a script using Lalonde et al. (2005) algorithm to compute
            % adaptative normals based on neighbour size and research radius. A first
            % loop define the optimal number of neighbours for each point. A seconde
            % loop compute the optimal normal and curvature for each point.
            %
            % Additional parameters that can be set via their field name :
            %
            % Iteration
            %   {15} | scalar in [>0,inf]
            %   Number of iteration defined experimentally by Lalonde et al. 2005
            %
            % Threshold
            %   {100} | scalar in [3,inf]
            %   Number of neighbours upper threshold. Above this number of
            %   neighbours iteration is stopped.
            %
            % D1
            %   {1} | scalar in [>0,inf]
            %   Constant defined experimentally by Lalonde et al. 2005
            %
            % D2
            %   {4} | scalar in [>0,inf]
            %   Constant defined experimentally by Lalonde et al. 2005
            %
            % epsilon
            %   {0.1} | scalar in [>0,inf]
            %   Indicate a variation of density ---> could be calculated if
            %   really needed!
            %
            % sigma
            %   {1.4} | scalar in [>0,inf]
            %   Instrumental error defined by for ILRIS Optech laser scanner.
            %  A. Abellan, M. Jaboyedoff, T. Oppikofer, and J.M. Vilaplana
            %  Detection of millimetric deformation using a terrestrial
            %  laser scanner: experiment and application to a rockfall event.
            %  In Nat. Hazards Earth Syst. Sci., 9, pp. 365-372, 2009.
            %
            % gamma
            %   {0.3} | scalar in [>0,inf]
            %   Correction factor defined experimentally by Lalonde et al. 2005
            %
            % Remarks:
            % - COMPUTE_OPTIMAL_NORMALS is optimized for TLS data from Optech ILRIS.
            %
            
            % Validate input arguments.
            ip = inputParser;
            ip.addParamValue('Iteration',15,@(x)isscalar(x)&&x>=3);
            ip.addParamValue('Threshold',100,@(x)isscalar(x)&&x>=3);
            ip.addParamValue('D1',1,@(x)isscalar(x)&&x>0);
            ip.addParamValue('D2',4,@(x)isscalar(x)&&x>0);
            ip.addParamValue('epsilon',0.1,@(x)isscalar(x)&&x>0);
            ip.addParamValue('sigma',1.4,@(x)isscalar(x)&&x>0);
            ip.addParamValue('gamma',0.3,@(x)isscalar(x)&&x>0);
            ip.parse(varargin{:});
            arg = ip.Results;
            iteration = arg.Iteration;
            k_threshold =arg.Threshold;
            d1 = arg.D1;
            d2 = arg.D2;
            epsilon = arg.epsilon;
            sigma = arg.sigma;
            gamma = arg.gamma;
            
            if ~isempty( obj.Normals ), return; end
            
            m = size(obj.P,2);
            obj.ComputeKDTree;
            
            Neighbours = transpose(knnsearch(obj.KDTree,transpose(obj.P), 'k', k_threshold+1));
            % Loop on all points
            obj.Curv = zeros(1,m);
            obj.Normals = zeros(3,m);
            Best_radius = zeros(1,m);
            Best_neighbours= zeros(1,m);
            knbn = zeros(iteration,m); % matrix build to follow evolution of neighbour size during the iterations.
            
            for i = 1:m
                nbn = 3;
                for j = 1:iteration
                    
                    % skip nan values
                    if any(isnan(obj.P(:,m)))
                        continue;
                    end
                    %nearest neighbours
                    P = obj.P(:,Neighbours(2:nbn+1, i));                         %#ok<*PROP>
                    % if the data contains some nans, knn might return nan as
                    % closest neighbours. Strange, but true. Remove them
                    nanm = not(isnan(P(1,:)));
                    P = P(:,nanm);
                    nbn = size(P,2);
                    % Find centroid.
                    Pc = sum(P,2)./nbn;
                    %Give it the same size as P.
                    Pbar = zeros(3,k_threshold);
                    for ir = 1:nbn
                        Pbar(:,ir) = Pc;
                    end
                    % Using repmat takes more time !!
                    % Find deviation from centroid.
                    R = P - Pbar(:,1:nbn);
                    % Compute covariance matrix;
                    Cov = R * transpose(R);
                    % Solve the eigenvector problem.
                    [V,D] = eig(Cov);
                    % sort the eigenvalues
                    [E, idx] = sort(diag(D),'descend'); %#ok<ASGLU>
                    % compute the normal
                    V = V(:,idx);
                    obj.Normals(:,i) = V(:,3);
                    % vector point centroid.
                    Pi = obj.P(:,i); % my point.
                    PiPc = Pi-Pc;
                    % The normal
                    N = obj.Normals(:,i);
                    %Dist to plane
                    dc = abs(transpose(PiPc)*N);
                    % mean distances to the neighbours
                    dm = zeros(1,nbn);
                    for in = 1:nbn
                        dm(in) = EuclDist(Pi, P(:,in));
                    end
                    d = mean(dm);
                    % compute curvature estimate.
                    obj.Curv(:,i) = 2*dc /(d^2);
                    
                    
                    %%%%% COMPUTING NORMALS BASED ON LALONDE ET AL. (2005)
                    rrn_old = dm(nbn);
                    % calculating first density.
                    rho = nbn/(pi*(rrn_old^2));
                    % evaluating research radius.
                    rrn = (1/(obj.Curv(:,i))*((d1*sigma/sqrt(epsilon*rho))+d2*(sigma^2)))^1/3;
                    % correction factor for the research radius.
                    rrn_new = rrn*gamma + (1 - gamma)*rrn_old;
                    % optimal neighbour size base on local point density.
                    knbn(j,i) = pi*rho*(rrn_new^2);
                    % uppermost threshold: if the neighbour size is mors than a
                    % value => stop and save the last neighbour  size before
                    % thershold.
                    if knbn(j,i) >= k_threshold
                        break;
                    else
                        nbn = round(knbn(j,i)); % because need an integer number of neighbours.
                    end                         % this value of nbn is use for the next iteration
                    
                end
                %%%
                % save in a matrix (1,nb of point) the optimal research radius.
                Best_radius(1,i) = rrn_new;
                % save in a matrix (1,nb of point) the optimal research radius.
                Best_neighbours(1,i)= nbn;
                % Sometimes the best solution is a nbn less than 3, but at least 3
                % neighbours are needed to compute normal.
                if Best_neighbours(1,i) <3
                    Best_neighbours(1,i) =3;
                end
                
            end
            
            
            obj.ComputeKDTree;
            m = size(obj.P,2);
            obj.Curv    = zeros(1,m);
            obj.Normals = zeros(3,m);
           
            for i = 1:m
                
                nbn= Best_neighbours(1,i);
                if any(isnan(obj.P(:,m)))
                    continue;
                end
                %nearest neighbours
                P = obj.P(:,Neighbours(2:nbn+1, i));                         %#ok<*PROP>
                % if the data contains some nans, knn might return nan as
                % closest neighbours. Strange, but true. Remove them
                nanm = not(isnan(P(1,:)));
                P = P(:,nanm);
                nbn = size(P,2);
                % Find centroid.
                Pc = sum(P,2)./nbn;
                %Give it the same size as P.
                Pbar = zeros(3,k_threshold);
                for ir = 1:nbn
                    Pbar(:,ir) = Pc;
                end
                % Using repmat takes more time !!
                % Find deviation from centroid.
                R = P - Pbar(:,1:nbn);
                % Compute covariance matrix;
                Cov = R * transpose(R);
                % Solve the eigenvector problem.
                [V,D] = eig(Cov);
                % sort the eigenvalues
                [E, idx] = sort(diag(D),'descend');
                % compute the normal
                V = V(:,idx);
                obj.Normals(:,i) = V(:,3);
                % vector point centroid.
                Pi = obj.P(:,i); % my point.
                PiPc = Pi-Pc;
                % The normal
                N = obj.Normals(:,i);
                %Dist to plane
                dc = abs(transpose(PiPc)*N);
                % mean distances to the neighbours
                dm = zeros(1,nbn);
                for in = 1:nbn
                    dm(in) = EuclDist(Pi, P(:,in));
                end
                d = mean(dm);
                % compute curvature estimate.
                obj.Curv(:,i) = 2*dc /(d^2);
            end
            
        end % ComputeOptimalNormals
                
        function obj = NormalsOutTopo( obj )
            
            %Correction of normals direction to get out of topographic surface.
            if isempty(obj.TLSPos)
                error('Please provide the TLS Position');
            end
            
            for i=1:length(obj.P)
                Vector(1:3,i) = obj.TLSPos - obj.P(1:3,i);
            end
            Vector_norm(1:3,:)=Vector(1:3,:)./norm(Vector(1:3,:));
            Incidence(1,:) = dot(obj.Normals(1:3,:),Vector_norm(1:3,:));
            idx=Incidence<0;
            obj.Normals(1:3,idx) = -1.*obj.Normals(1:3,idx);
            
        end % NormalsOutTopo
        
        function obj = ComputeCurvature( obj, varargin )
            % COMPUTECURVATURE Compute curvatures or any scalar surface feature, invariant under spatial transformations.
            %
            % Additional parameters that can be set via their field name :
            %
            % Type
            %   {'estcurv'} | 'surfvar'
            %   estcurv : estimation of the curvature based on
            %       Stefan Gumhold, Xinlong Wang, and Rob Macleod.
            %       Feature extraction from point clouds.
            %       In Proceedings of the 10 th International Meshing Roundtable, pages 293{305, 2001.
            %   surfvar : variation of the surface from correlation of point clouds.
            %       Mark Pauly, Markus Gross, and Leif P. Kobbelt.
            %       Efficient simplification of point-sampled surfaces.
            %       In VIS '02: Proceedings of the conference on Visualization '02, pages 163{170, Washington, DC, USA,2002. IEEE Computer Society.
            %       Points lying in a plane will have c = 0 and for isotropically
            %       distributed points c = 1/3.
            %       Note : highly dependent on the nb of neighbours and local
            %       point density. Works well for sampling, but not for surface
            %       feature recognition.
            %
            % Force
            %   {false} | true
            %   Force the computing of the curvature even if already
            %   computed.
            %
            % NbNeighbours
            %   {15} | scalar in [1,inf]
            %   Set the number of neighbours to be used in the computation.
            %   A value of 24 is 5x5NN can be an optimal compromise using ILRIS Optech scanner.
            %   A. Abellan, M. Jaboyedoff, T. Oppikofer, and J.M. Vilaplana
            %   Detection of millimetric deformation using a terrestrial
            %   laser scanner: experiment and application to a rockfall event.
            %   In Nat. Hazards Earth Syst. Sci., 9, pp. 365-372, 2009.
            
            % Validate input arguments.
            ip = inputParser;
            ip.addParamValue('Force', false, @(x)islogical(x));
            ip.addParamValue('NbNeighbours',15,@(x)isscalar(x)&&x>0);
            validType = {'surfvar','estcurv'};
            ip.addParamValue('Type','estcurv',@(x)any(strcmpi(x,validType)));
            ip.parse(varargin{:});
            arg = ip.Results;
            nbn = arg.NbNeighbours;
            
            % No need to do it again if already done
            if not(arg.Force) && ~isempty( obj.Curv ), return; end
            % Sanity check
            m = size(obj.P,2);
            if m == 0
                warning('PointCloud:NoP',...
                    'I cannot compute the curvature if the positions are not given...');
                return;
            end
            
            % Remove Nans
            %tP = transpose(obj.P);
            %tP = tP(not(isnan(tP(:,1))),:);
            %m = size(tP,1);
            
            % Perform significatly faster when Statistics Toolbox version >
            % 7.2 is installed.
            v = ver('stats');
            if str2double(v.Version) >= 7.3
                obj.ComputeKDTree();
                Neighbours = transpose(knnsearch(obj.KDTree, transpose(obj.P), 'k', nbn+1));
                %Neighbours = transpose(knnsearch(obj.KDTree, tP, 'k', nbn+1));
            else
                Error('PointCloud:WV','You need Statistics Toolbox version >= 7.3 to search for the nearest neighbours.');
                %Neighbours = kNearestNeighbors(obj.P, obj.P, nbn+1);
            end
            
            % Loop on all points
            obj.Curv    = zeros(1,m);
            obj.Normals = zeros(3,m);
%             S_linear    = zeros(3,m);
%             S_surface   = zeros(3,m);
%             S_LINEAR    = zeros(1,m);
%             S_SURFACE   = zeros(1,m);
%             S_SCATTER   = zeros(1,m);
%             S_Det       = zeros(1,m);
%             S_Vol       = zeros(1,m);
%             Pbar        = zeros(3,nbn);
            for i = 1:m
                % skip nan values
                if any(isnan(obj.P(:,m)))
                    continue;
                end
                %nearest neighbours
                P = obj.P(:,Neighbours(2:end, i));                          %#ok<*PROP>
                % if the data contains some nans, knn might return nan as
                % closest neighbours. Strange, but true. Remove them
                nanm = not(isnan(P(1,:)));
                P = P(:,nanm);
                nbn = size(P,2);
                % Find centroid.
                Pc = sum(P,2)./nbn;
                %Give it the same size as P.
                for ir = 1:nbn
                    Pbar(:,ir) = Pc;
                end
                % Using repmat takes more time !!
                % Find deviation from centroid.
                R = P - Pbar(:,1:nbn);
                % Compute covariance matrix;
                Cov = R * transpose(R);
                % Solve the eigenvector problem.
                [V,D] = eig(Cov);
                % sort the eigenvalues
                [E, idx] = sort(diag(D),'descend');
                % compute the normal
                V = V(:,idx);
                obj.Normals(:,i) = V(:,3);
                
                switch arg.Type
                    case 'surfvar'
                        
                        %E(1) quantitatively describes the variation along the surface normal,
                        % i.e. estimates how much the points deviate from the tangent plane.
                        % compute the surface variation :
                        obj.Curv(:,i) = E(1)/(E(1)+E(2)+E(3));
                        
                    case 'estcurv'
                        % vector point centroid.
                        Pi = obj.P(:,i); % my point.
                        PiPc = Pi-Pc;
                        % The normal
                        N = V(:,idx(1));
                        %Dist to plane
                        dc = abs(transpose(PiPc)*N);
                        % mean distances to the neighbours
                        dm = zeros(1,nbn);
                        for in = 1:nbn
                            dm(in) = EuclDist(Pi, P(:,in));
                        end
                        d = mean(dm);
                        % compute curvature estimate.
                        obj.Curv(:,i) = 2*dc /(d^2);
                        
                end
                
%                 S_linear(1:3,i)    = (E(1)-E(2))*V(:,1); % OK
%                 S_surface(1:3,i)   = (E(2)-E(3))*V(:,3); % OK
%                 S_LINEAR(1,i)      = S_linear(1,i)*S_linear(2,i)*S_linear(3,i); % Could be a solution
%                 S_SURFACE(1,i)     = S_surface(1,i)*S_surface(2,i)*S_surface(3,i); % Could be a solution
%                 S_SCATTER(1,i)     =  E(1); %OK
%                 S_Det(1,i)         = prod(E); %OK
                % to compute volume at least 4 neighbours are needed: if nbn is < 3
                % => nbn become 4.
% %                 if nbn <=3
% %                     nbn = 4;
% %                     P = obj.P(:,Neighbours(2:nbn+1, i));
% %                 end
% %                 [out1 PP] = mapstd(P);
% %                 ssnorm = out1';
% %                 [t Vol] = convhull(ssnorm);
% %                 S_Vol(1,i)  = Vol;
            end
            
%             obj.UsefulVar(3,:) = S_Det;
%             obj.UsefulVar(4,:) = S_SCATTER;
%             obj.UsefulVar(5,:) = S_SURFACE;
%             obj.UsefulVar(6,:) = S_LINEAR;
%             obj.UsefulVar(7,:) = S_Vol;

        end % ComputeCurvatures
        
        function obj = ComputeBoundaries( obj,  varargin )
            
            % Find the border points.
            warning('Provide still unstable results!')
            % No need to do it again if already done or provided
            ip = inputParser;
            ip.addParamValue('radius',Inf,@(x)isscalar(x));
            ip.addParamValue('filterangle',pi/6,@(x)isscalar(x));
            ip.parse(varargin{:});
            arg = ip.Results;
            rrc = arg.radius;
            delta = arg.filterangle;
            
            % Unique points
            %[X,~] = unique((obj.P)','rows');
            
            % Need to compute Delaunay triangulation
            obj.ComputeDelaunayTriangulation;
            % Remove zero volume tetrahedra since
            % these can be of arbitrary large circumradius
            % Limit circumradius of simplices
            if rrc==Inf
                [~,rc] = obj.DT.circumcenters;
                rcc = max(rc);
            else
                rcc=rrc;
            end
            [~,S] = AlphaBoundary((obj.P)',rcc,0);
            
            tr = TriRep(S.bnd,obj.P');
            
            fe = tr.featureEdges(delta)';
            
            Boundary = horzcat(fe(1,:),fe(2,:));
            
            obj.Boundaries = Boundary;
        end % ComputeBoundaries
        
        function obj = ComputeDelaunayTriangulation( obj )
            % Compute a Delaunay Triangulation.
            % The created DelaunayTri object is stored in the DT property.
            %
            % Usage philosophy :
            % Call ComputeDelaunayTriangulation(pc) in each function/script
            % where you need a triangulation for pc.
            % If the triangulation is already computed, the computation is
            % not redone and the call becomes dummy.
            %
            % Status :
            % Not tested.
            %
            % Advice :
            % Do not use for complexe point clouds like landslide survey.
            % Prefer a home implementation that you are welcome to store in
            % the DT property. Just adapt this fonction to call your implementation.
            
            % No need to do it again if already done
            if ~isempty( obj.DT )
                return;
            else
                obj.DT = DelaunayTri(transpose(obj.P));
            end
        end % ComputeDelaunayTriangulation
        
        function obj = ComputeGLTree( obj )
            % COMPUTEGLTREE Compute a GL search tree (GLTree object).
            % The pointer on the created GLTree C++ object is stored in the GLTree field.
            %
            % Usage philosophy :
            % Call ComputeGLTree(pc) in each function/script
            % where you need a GL search tree for pc.
            % If the tree is already computed, the computation is
            % not redone and the call becomes dummy.
            %
            % Requirement :
            % To compute the GLtree you need to install GLTreePro
            % (Matlab Central File ID: #29115).
            %
            % Note :
            %   You can use the GLTree field to store any pointer (C/C++)
            %   to a tree object. Just adapt this function to make the
            %   appropriate call. Eventually also adapt the KillGLTree
            %   function.
            
            % No need to do it again if already done
            if ~isempty( obj.GLTree ), return; end
            
            if isempty(obj.P)
                warning('PointCloud:NoP','You need some points to build a tree...');
                return;
            end
            nanm = isnan(obj.P(1,:));
            if any(nanm)
                error('PointCloud:NoGLTree', 'GLTreePro cannot work with nans. Remove them first.' );
            end
            
            try
                obj.GLTree = BuildGLTree3D(obj.P);
            catch exception
                msg = 'To build the GLtree you need to install GLTreePro (Matlab Central File ID: #29115).';
                error('PointCloud:NoGLTree', msg);
                %throw(exception);
            end
            
        end % ComputeGLTree
        
        function obj = ComputeKDTree( obj )
            % Compute a k-d search tree, using Matlab's KDTreeSearcher.
            % The resulting structure is saved in the field KDTree.
            %
            % Usage philosophy :
            % Call ComputeKDTree(pc) in each function/script
            % where you need a search tree for pc.
            % If the tree is already computed, the computation is
            % not redone and the call becomes dummy.
            
            % No need to do it again if already done
            if ~isempty( obj.KDTree ), return; end
            
            if isempty( obj.P ), error('PointCloud:NoP','No positions to build kd-tree upon.'); end
            
            % Build the k-d tree.
            obj.KDTree = KDTreeSearcher(transpose(obj.P));
            
        end % ComputeKDTree
        
        function [mtd,rmstd] = ComputeTrueDistance( obj, pc )
            % COMPUTETRUEDISTANCE Compute the mean and root mean squared distances between this and a given PointCloud true positions.
            % Note : TrueM must be fixed if obj needs to be transmuted to match pc.
            %
            % Input :
            %   pc : a PointCloud object.
            %
            % Ouput :
            %   mtd : the mean (first).
            %   rmstd : the standard deviation (second).
            
            mtd = nan; rmstd = nan;
            if isempty(obj.TrueP)
                warning('PointCloud:NoTP','PointCloud has no true positions to compare.'); return
            end
            if ~pc.HasTrueP
                warning('PointCloud:NoTP','Given PointCloud has no true positions to compare.'); return
            end
            if isempty(obj.TrueM),
                TTrueP = obj.TrueP(:,obj.TrueM);
            else
                TTrueP = obj.TrueP;
            end
            
            if size(TTrueP,2) ~= pc.Size
                error('PointCloud:WS','PointCloud TrueP size %f does not match given TrueP size %f !', size(TTrueP,2), pc.Size);
            end
            
            % Compute the distance !
            dist = sum(power(TTrueP - pc.TrueP, 2),1);
            mtd = mean(sqrt(dist));
            rmstd = sqrt(mean(dist));
            
        end % ComputeTrueDistance
               
        function SaveInASCII(obj, file, fields)
            % SAVEINASCII Save point cloud in ASCII format.
            %
            % Input :
            %   file : output file name, with possible full path.
            %   fields : cell of property names to save. Default {'P'}.
            
            if nargin < 2
                error('PointCloud:NoFile','Please provide a file name.');
            end
            pcsize = obj.Size;
            if isempty(pcsize)
                fprintf('No positions to save !\n');
                return;
            end
            
            if nargin < 3, fields = {'P'}; end
            
            % open file with write permission
            fid = fopen(file, 'w');
            % if you print a header, beware, Matlab, as well as many other
            % program like PolyWorks won't be able to read it back and will
            % crash.
            %print header
            %             fprintf(fid,'#');
            %             for f = fields
            %                 str = char(f);
            %                 fprintf(fid,'%s\t',str);
            %             end
            %             fprintf(fid,'\n');
            %print data
            for ip = 1:pcsize
                for f = fields
                    str = char(f);
                    %fprintf(fid,'%f %f %f\t',obj.(str)(1,ip),obj.(str)(2,ip),obj.(str)(3,ip));
                    fprintf(fid,'%f\t',obj.(str)(:,ip));
                end
                fprintf(fid,'\n');
            end
            fclose(fid);
            
        end % SaveInASCII
        
        function SaveInPCD(obj, file, fields)
                        
            % Santiy Check
            if nargin < 2
                error('PointCloud:NoFile','Please provide a file name.');
            end
            
            if nargin < 3, fields = {'P','',''}; end
            
            % You need a 1x3 cell for next steps
            if length(fields)==1
                fields={char(fields),'',''};
                nfields =3;
            elseif length(fields)==2
                fields={char(fields{1}),char(fields{2}),''};
                nfields =6;
            else
                nfields =7;
            end
            % Check if the variables are well attibuted.
            if strfind(fields{1,1},'P')
            else
                error('PointCloud:NoFile','Please provide first the position P.')
            end
            if ~isempty(fields{1,2})
                if  strfind(fields{1,2},'Intensities')
                elseif strfind(fields{1,2},'Colors')
                else
                    error('PointCloud:NoFile','Please provide in second the Colors or the Intensities.')
                end
            end
            if ~isempty(fields{1,3})
                if strfind(fields{1,3},'UsefulVar')
                else
                    error('PointCloud:NoFile','Please provide in third a scalar as UsefulVar.')
                end
            end
            % open file with write permission
            fid = fopen(file, 'w');
            
            npoints = obj.Size;
            width = npoints;
            height  = 1;
            
            % write the PCD file header
            switch nfields
                case 3
                    Fields = 'x y z';
                    count = '1 1 1';
                    typ = 'F F F';
                    siz = '4 4 4';
                case 6
                    Fields = 'x y z rgb';
                    count = '1 1 1 1';
                    typ = 'F F F I';
                    siz = '4 4 4 4';
                case 7
                    Fields = 'x y z rgb a';
                    count = '1 1 1 1 1';
                    typ = 'F F F I I';
                    siz = '4 4 4 4 4';
            end
            
            fprintf(fid, '# .PCD v.7 - Point Cloud Data file format\n');
            fprintf(fid, 'VERSION .7\n');
            
            fprintf(fid, 'FIELDS %s\n', Fields);
            fprintf(fid, 'SIZE %s\n', siz);
            fprintf(fid, 'TYPE %s\n', typ);
            fprintf(fid, 'COUNT %s\n', count);
            
            fprintf(fid, 'WIDTH %d\n', width);
            fprintf(fid, 'HEIGHT %d\n', height);
            fprintf(fid, 'POINTS %d\n', npoints);
            
            
            
            switch nfields
                case 3
                    % point data
                    points = [ obj.P ];
                    
                case 6
                    % RGB or Int data
                    if strfind(fields{1,2},'Colors')
                        RGB = uint32(obj.Colors);
                        rgb = (RGB(1,:)*256+RGB(2,:))*256+RGB(3,:);
                        points = [ obj.P; double(rgb)];
                    elseif strfind(fields{1,2},'Intensities')
                        intensities = [obj.Intensities; obj.Intensities; obj.Intensities];
                        INT = uint32(intensities);
                        int = (INT(1,:)*256+INT(2,:))*256+INT(3,:);
                        points = [ obj.P; double(int) ];
                    end
                    
                case 7
                    % RGBA data
                    if strfind(fields{1,2},'Colors')
                        RGB = uint32(obj.Colors);
                        rgb = (RGB(1,:)*256+RGB(2,:))*256+RGB(3,:);
                        a = obj.UsefulVar(1,:);
                        points = [ obj.P; double(rgb); double(a) ];
                    elseif strfind(fields{1,2},'Intensities')
                        intensities = [obj.Intensities; obj.Intensities; obj.Intensities];
                        INT = uint32(intensities);
                        int = (INT(1,:)*256+INT(2,:))*256+INT(3,:);
                        a = obj.UsefulVar;
                        points = [ obj.P; double(int); double(a) ];
                    end
            end
            
            % Write ASCII format data
            fprintf(fid, 'DATA ascii\n');
            
            if nfields == 3
                % uncolored points
                fprintf(fid, '%f %f %f\n', points);
                
            elseif nfields == 6
                % colored points
                fprintf(fid, '%f %f %f %d\n', points);
                
            else
                % colored points and scalar DON'T WORK YET
                fprintf(fid, '%f %f %f %d\n %d\n', points);                
            end
            
            fclose(fid);
        end % SaveInPCD
        
        function obj = ImportDataFromASCII(obj, file, fields)
            % IMPORTDATAFROMASCII Import data from an ASCII file.
            %
            % Input :
            %   file : input file name, with possible full path
            %   fields : cell of property names to import. Default {'P'}.
            
            
            if nargin < 2
                error('PointCloud:NoFile','Please provide a file name.');
            end
            % One assumes that the data contains a mxn matrix, with m>=3.
            % Check if file exists
            if ~exist(file, 'file')
                warning('PointCloud:NoFex','Given data file does not exist !');
                return
            end
            if nargin < 3, fields = {}; end
            
            A=importdata(file);
            if ~isnumeric(A)
                warning('PointCloud:NoDReal','Input data is not numbers.');
                return
            end
            if isempty(fields)
                obj.P = A(:,1:3);
            else
                if ~iscellstr(fields)
                    error('PointClouds:WF','The name of the properties to load must be given in a cell.');
                end
                inc = 1; s = 0;                                             %#ok<NASGU>
                for f = fields
                    str = char(f); %convert from cell to string
                    D1 = {'Intensities','Curv','TLSPos','UsefulVar','TLSAttribute','Ignore'};
                    D3 = {'P','TrueP','Normals','Colors'};
                    if any(strcmpi(str,D3))
                        s = 2;
                    elseif any(strcmpi(str,D1))
                        s = 0;
                    else
                        error('PointClouds:WF','Inknown properties %s\n',str);
                    end
                    if any(strcmpi(str,'Ignore'))
                        inc = inc + s + 1;
                    else
                        obj.(str) = transpose( A(:,inc:inc+s) );
                        inc = inc + s + 1;
                    end
                end
            end
        end % ImportDataFromASCII
        
        function bool = HasTrueP( obj )
            % Return true if the point cloud has true positions.
            bool = ~isempty( obj.TrueP );
        end % HasTrueP
        
        function pcsize = Size( obj )
            % What is the size of the point cloud ?
            % Return the number of points.
            pcsize = 0;
            if ~isempty(obj.P)
                pcsize = size(obj.P,2); return
            end
            if ~isempty(obj.TrueP)
                pcsize = size(obj.TrueP,2);
            end
        end % PC size
        
        function bool = IsEmpty( obj )
            % Is the point cloud empty ? Return true if no entries.
            bool = (obj.Size == 0);
        end % IsEmpty
        
        function col = WhatColor( obj, p )
            % What is the color of the closest point ?
            %
            % Input :
            %   p : a vector of x, y, z positions.
            %
            % Output :
            %   col : an entry in the field Colors, ie an array of 3
            %   doubles either in [0 1], or in [0 255].
            %
            % Usage :
            %   pc.WhatColors( [0.234 4.67 123.6] )
            % ans =
            %       34      78      250
            
            % Sanity check
            if isempty(obj.Colors)
                error('PointCloud:NoC','The PointCloud does not contain any colors.');
            end
            
            % Get a kd tree
            obj.ComputeKDTree();
            % Get nearest neighbour
            nb = knnsearch(obj.KDTree, p, 'k', 1);
            % Return its color
            col = obj.Colors(:,nb);
            col = col';
        end
        
        function inty = WhatIntensity( obj, p )
            % What is the intensity of the closest point ?
            %
            % Input :
            %   p : a vector of x, y, z positions.
            %
            % Output :
            %   inty : an entry in the field Intensities, ie a
            %   double either in [0 1], or in [0 255].
            %
            % Usage :
            %   pc.WhatIntensity( [0.234 4.67 123.6] )
            % ans =
            %       0.33
            
            
            
            % Sanity check
            if isempty(obj.Intensities)
                error('PointCloud:NoI','The PointCloud does not contain any intensities.');
            end
            
            % Get a kd tree
            obj.ComputeKDTree();
            % Get nearest neighbour
            nb = knnsearch(obj.KDTree, p, 'k', 1);
            % Return its color
            inty = obj.Intensities(:,nb);
            inty = inty';
        end
        
        function obj = GetMissingPropFromPC( obj, PC )
            % Complete properties of object by getting the missing ones in PC.
            % Very useful in the case you have a created a subsample of
            % PC containg only the point positions (for example using the
            % data selection tool from the Matlab plot viewer) and would
            % like to retrieve the corresponding colors/intensities of
            % those points from PC.
            % Note : works on measured positions only (obj.P)
            %
            % Input :
            %   PC : a PointCloud object.
            
            if ~isa(PC,'PointCloud')
                error('PointCloud:NoPC','The provided object is not a PointCloud !');
            end
            % Get a kd tree
            PC.ComputeKDTree();
            % Get nearest neighbour
            if ~isempty(obj.P)
                nb = transpose(knnsearch(PC.KDTree, transpose(obj.P), 'k', 1));
            elseif ~isempty(obj.TrueP)
                nb = transpose(knnsearch(PC.KDTree, transpose(obj.TrueP), 'k', 1));
            else
                error('PointCloud:NoP','Obj has P and TrueP fields empty !');
            end
            % Set missing properties
            if ~isempty(PC.Colors) && isempty(obj.Colors)
                obj.Colors = PC.Colors(1:3,nb);
                fprintf('I copied the Colors.\n');
            end
            if ~isempty(PC.Intensities) && isempty(obj.Intensities)
                obj.Intensities = PC.Intensities(nb);
                fprintf('I copied the Intensities.\n');
            end
            if ~isempty(PC.Normals) && isempty(obj.Normals)
                obj.Normals = PC.Normals(1:3,nb);
                fprintf('I copied the Normals.\n');
            end     
            if ~isempty(PC.Curv) && isempty(obj.Curv)
                obj.Curv = PC.Curv(nb);
                fprintf('I copied the Curvature.\n');
            end  
            if ~isempty(PC.UsefulVar) && isempty(obj.UsefulVar)
                obj.UsefulVar = PC.UsefulVar(:,nb);
                fprintf('I copied the Useful Variables.\n');
            end
            if ~isempty(PC.TLSPos) && isempty(obj.TLSPos)
                obj.TLSPos = PC.TLSPos(:,1);
                fprintf('I copied the TLS Position.\n');
            end
            if ~isempty(PC.TLSAttribute) && isempty(obj.TLSAttribute)
                obj.TLSAttribute = PC.TLSAttribute(nb);
                fprintf('I copied the TLS Attribute.\n');
            end
        end
        
        function obj = RemoveNans(obj)
            % Remove any nans in P and TrueP.
            % Corresponding entries in all fields are deleted.
            % Use it very carefuly.
            
            mn = false(3,obj.Size);
            if ~isempty(obj.P), mn = mn | isnan(obj.P); end
            if ~isempty(obj.TrueP), mn = mn | isnan(obj.TrueP); end
            ma = any(mn);
            if not(any(ma)), return; end
            m = not(ma);
            if ~isempty(obj.P), obj.P = obj.P(:,m); end
            if ~isempty(obj.TrueP), obj.TrueP = obj.TrueP(:,m); end
            %Better to erase it.
            %if ~isempty(obj.TrueM), obj.TrueM = obj.TrueM(m); end
            if ~isempty(obj.TrueM)
                warning('PointCloud:TM','Beware, the content of TrueM has been erased.');
            end
            if ~isempty(obj.Colors), obj.Colors = obj.Colors(:,m); end
            if ~isempty(obj.Intensities), obj.Intensities = obj.Intensities(m); end
            
        end % RemoveNans
        
        function mpc = MeshPointCloud(obj, varargin )
            % MESHPOINTCLOUD Create a MeshPointCloud out of this PointCloud.
            %
            % Input : options to pass to GridFit.
            %
            % Output : a MeshPointCloud object.
            %
            % Presently, only GridFit is available to meshify.
            % Note : to work on TrueP, set 'TrueP' to true in the GridFit
            % options.
            mpc = GridFit( obj, varargin{:} );
        end
        
        function obj = Add(obj, PC )
            % Add the content of a given point cloud to this one.
            % Beware : they must have the same fields (properties) filled.
            %
            % Input :
            %   PC : a PointCloud.
            %
            % Usage :
            %   pc1.Add(pc2);
            %   Add(pc1,pc2);
            
            % Compute sum of sizes to check compatibility.
            totsize = obj.Size + PC.Size;
            totP = size(obj.P,2)+size(PC.P,2);
            totTP = size(obj.TrueP,2)+size(PC.TrueP,2);
            totCol = size(obj.Colors,2)+size(PC.Colors,2);
            totInt = size(obj.Intensities,2)+size(PC.Intensities,2);
            %totNorm = size(obj.Normals,2)+size(PC.Normals,2);
            totTLSAtt = size(obj.TLSAttribute,2)+size(PC.TLSAttribute,2);
            %totTLSpos = size(obj.TLSpos,2)+size(PC.TLSpos,2);
            if totP > 0 && totP < totsize ||...
                    totTP > 0 && totTP < totsize ||...
                    totCol > 0 && totCol < totsize ||...
                    totTLSAtt > 0 && totTLSAtt < totsize ||...  
                    totInt > 0 && totInt < totsize %||...
                    %totNorm > 0 && totNorm < totsize
                    %totTLSpos > 0 && totInt < totsize||...
                error('PointCloud:NoComp','Point clouds are incompatible !');
            end
            
            obj.P = horzcat(obj.P,PC.P);
            obj.TrueP = horzcat(obj.TrueP,PC.TrueP);
            obj.Colors = horzcat(obj.Colors,PC.Colors);
            obj.Intensities = horzcat(obj.Intensities,PC.Intensities);
            %obj.Normals = horzcat(obj.Normals,PC.Normals);
            obj.TLSAttribute = horzcat(obj.TLSAttribute,PC.TLSAttribute);
            obj.TLSPos = horzcat(obj.TLSPos,PC.TLSPos);
        end % Add
        
        
    end % methods
    
    methods (Static)
        function obj = loadobj(A)
            % Allows to load a PointCloud object by using the load built-in function.
            obj = PointCloud(A.Name, A.P, A.TrueP);
            obj.TMode = A.TMode;
            if ~isempty(A.Colors), obj.Colors=A.Colors; end
            if ~isempty(A.Intensities), obj.Intensities=A.Intensities; end
            if ~isempty(A.Normals), obj.Normals=A.Normals; end
            if ~isempty(A.Curv), obj.Curv=A.Curv; end
            if ~isempty(A.UsefulVar), obj.UsefulVar=A.UsefulVar; end
            if ~isempty(A.DT), obj.DT=A.DT; end
            if ~isempty(A.KDTree), obj.KDTree=A.KDTree; end
            if ~isempty(A.Boundaries), obj.Boundaries=A.Boundaries; end
            % GLTree cannot be saved (pointer to Cpp object).
            %if ~isempty(A.GLTree), obj.GLTree=A.GLTree; end
            if ~isempty(A.TLSAttribute), obj.TLSAttribute=A.TLSAttribute; end
            if ~isempty(A.TLSPos), obj.TLSPos=A.TLSPos; end
        end % loadobj
    end % static methods
    
end

