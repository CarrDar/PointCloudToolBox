classdef Rockfall < PointCloud
    % ROCKFALL Subclass of PointCloud.
    % Type doc Rockfall for a more detailed description.
    %
    % Rockfall Properties:
    %   Time1PC            - The point cloud at time step #1.
    %   Time2PC            - The point cloud at time step #2.
    %   IndexPC1           - Store indexing of point at time step #1.
    %   IndexPC2           - Store indexing point cloud at time step #2.
    %   Volumes            - Store estimated volume for each indexed rockfall.
    %   Position           - Store position of center of mass for each indexed rockfall.
    %   PrincipComps       - Length of the three main componant for each indexed rockfall.
    %
    %   [0,1,...,n] with zero for unuse points and one to n for indexed
    %   rockfall event.
    % is a estimation of rockfall volume with multi-temporal point cloud
    % acquisition using ICP algorithm, Euclidian distance to classify points to create a new point cloud
    % representing the fallen block and alpha-shapehull to compute the volume of the block pointcloud.
    %
    % INPUT: - a (Mesh)PointCloud object for post-failure surface Pc1.
    %        - a (Mesh)PointCloud object for pre-failure surface Pc2.
    %        - T1 first threshold
    %        - T2 second threshold
    %        - k number of objects in a neighborhood of an object
    %        - Eps neighborhood radius, if not known avoid this parameter or put []
    %        - R radius of reserch alpha-shape
    %        - fig if you want to plot the histograms or not
    %
    % OUTPUT: - a PointCloud object for block of fallen rocks.
    %         - the estimated volume.
    %
    % Remarks:
    % - ESTIMATION VOLUME works with libraries of PointCloudToolBox developed at
    %   the Institute of Geomatics and Analysis of Risk - University of Lausanne
    %   (IGAR-UNIL) by Neal Gauvin and Dario Carrea. All the PointCloudToolBox is
    %   propriety of IGAR-UNIL.
    %
    % Update:
    %   1.1: For faster computing, replace line knnsearch (KDTree) by nearestNeighbor(Delaunay Tri).
    %   1.2: fonction(DBSCAN) for clustering data & multi measurement blocks.
    %   2.0: Implemented for (Mesh)PointCloud object. Faster computing.
    %        Possibility to use Point-to-Surface ('P2S') comaprison for a better
    %        segementation of fallen block but increase computing time.
    %        ( => see Comparison()).
    %
    %   References:
    %   [1] D. Carrea, A. Abellán, M.-H. Derron, M. Jaboyedoff. Automatic rockfalls volume
    %       estimation based on terrestrial laser scanning data, In  Engineering Geology
    %       for Society and Territory - Vol. 2, Eds. G. Lollino, D. Giordan, G.B. Crosta, J.Corominas,
    %       R. Azzam, J. Wasowski, N. Sciarra, Springer International Publishing, 2015.
    %   [2] M. Tonini & A.,Abellán  Rockfall detection from terrestrial
    %       LiDAR point clouds: a clustering approach using R. Journal of Spatial
    %       Information Science, 2014.
    %
    % AUTHOR  : Dario Carrea (at unil dot ch)
    % VERSION : 2.0
    % STATUS  : OK
    % DATE    : 14 December 2014
    
    properties
        Time1PC            %- The point cloud at time step #1.
        Time2PC            %- The point cloud at time step #2.
        IndexPC1=[];           %- Store indexing of point at time step #1.
        IndexPC2=[];           %- Store indexing point cloud at time step #2.
        Volumes=[];            %- Store estimated volume for each indexed rockfall.
        Position=[];           %- Store position of center of mass for each indexed rockfall.
        PrincipComps=[];       %- Length of the three main componant for each indexed rockfall.
    end
    
    methods
        
        function obj = set.Time1PC(obj,Time1PC)
            
            if  isa(Time1PC,'PointCloud')
                % copy point cloud
                    obj.Time1PC.P = Time1PC.P;
                    obj.Time1PC.TrueP = Time1PC.TrueP;
                    obj.Time1PC.TrueM = Time1PC.TrueM;
                    obj.Time1PC.Colors = Time1PC.Colors;
                    obj.Time1PC.Intensities = Time1PC.Intensities;
                    obj.Time1PC.Normals = Time1PC.Normals;
                    obj.Time1PC.Curv = Time1PC.Curv;
                    obj.Time1PC.Boundaries = Time1PC.Boundaries;
                    obj.Time1PC.UsefulVar = Time1PC.UsefulVar;
                    obj.Time1PC.TLSPos = Time1PC.TLSPos;
                    obj.Time1PC.TLSAttribute = Time1PC.TLSAttribute;
                    obj.Time1PC.DT = Time1PC.DT;
                    obj.Time1PC.KDTree = Time1PC.KDTree;
                    obj.Time1PC.TMode = Time1PC.TMode;
            else error('Please provide a PointCloud class object!'); 
            end
        end
        
        function obj = set.Time2PC(obj,Time2PC)
            
            if  isa(Time2PC,'PointCloud')
                % copy point cloud&& isa(AfterPC,'PointCloud')
                obj.Time2PC.P = Time2PC.P;
                obj.Time2PC.TrueP = Time2PC.TrueP;
                obj.Time2PC.TrueM = Time2PC.TrueM;
                obj.Time2PC.Colors = Time2PC.Colors;
                obj.Time2PC.Intensities = Time2PC.Intensities;
                obj.Time2PC.Normals = Time2PC.Normals;
                obj.Time2PC.Curv = Time2PC.Curv;
                obj.Time2PC.Boundaries = Time2PC.Boundaries;
                obj.Time2PC.UsefulVar = Time2PC.UsefulVar;
                obj.Time2PC.TLSPos = Time2PC.TLSPos;
                obj.Time2PC.TLSAttribute = Time2PC.TLSAttribute;
                obj.Time2PC.DT = Time2PC.DT;
                obj.Time2PC.KDTree = Time2PC.KDTree;
                obj.Time2PC.TMode = Time2PC.TMode;
            else error('Please provide a PointCloud class object!');
            end
        end
    end % methods
        methods (Static)
        function obj = loadobj(A,B)
            % Allows to load a PointCloud object by using the load built-in function.
            obj = Rockfall(A,B);
        end % loadobj
    end % static methods
end
