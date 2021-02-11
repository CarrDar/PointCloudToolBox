classdef MeshPointCloud < handle
    %MESHPOINTCLOUD Class to hold mesh grids as created by functions like GridFit.
    %
    % Note : 
    %       Only optimised for surface mesh (2.5D) for now.
    %
    %       Matlab 2011 provides tools like GridFit in the CurveFitting toolbox, to be purchased separately.
    %       Try doc tpaps. Some GUI for splines are also praised.
    %       Have a look in the future releases of Matlab.
    %
    %       No function for spatial transformations, like in PointCloud ?
    %       Very tempting, but this would destroy the whole sense of the
    %       grid. Turn it into a PointCloud.
    %
    % MeshPointCloud Properties:
    %   Name - The name of the object.
    %   xgrid - The x matrix of the mesh.
    %   ygrid - The y matrix of the mesh.
    %   zgrid - The z matrix of the mesh.
    %   Colors - Color of each vertex of facet.
    %
    % MeshPointCloud Methods:
    %   MeshPointCloud - The construtor.
    %   surf - Plot the surface.
    %   surf - Plot the surface with a contour plot.
    %   Size - Return the size of the mesh, ie number of points.
    %   PointCloud - Create a PointCloud object from the mesh.
    %
    %
    %AUTHOR  : Neal Gauvin (at a3 dot epfl dot ch)
    %VERSION : 1.2
    %STATUS  : OK
    %DATE    : 22 august 2011


    properties
        Name = ''; % The name of the object.
        xgrid = []; % The x matrix of the mesh.
        ygrid = []; % The y matrix of the mesh.
        zgrid = []; % The z matrix of the mesh.
        Colors = []; % Might be for each i,j vertex, or each facet. See doc surf for example.
    end
    
    methods
        
       function obj = MeshPointCloud( x, y, z )
           % The construtor.
           %
           % Input : 
           %    x : mesh matrix for x.
           %    y : mesh matrix for y.
           %    z : mesh matrix for z.

	   % copy constructor
           if nargin > 1 && isa(x,'MeshPointCloud')
               obj.Name = x.Name;
               obj.xgrid = x.xgrid;
               obj.ygrid = x.ygrid;
               obj.zgrid = x.zgrid;
               obj.Colors = x.Colors;
               return
           end
           % Start filling variables
           % No coherance tests. We suppose the functions using
           % meshpointcloud provide clean stuff.           
           if nargin == 1
               if size(x,1)==1 || size(x,2)==1
                   [obj.xgrid, obj.ygrid] = meshgrid(x);
               else
                   obj.xgrid = x;
               end
           elseif nargin >= 2
               if size(x,1)==1 && size(y,1)==1 || size(x,2)==1 && size(y,2)==1
                   [obj.xgrid,obj.ygrid] = meshgrid(x,y);
               elseif isequal(size(x),size(y))
                   obj.xgrid = x;
                   obj.ygrid = y;
               else
                   error('MeshPointCloud:WA','Provide arrays or a nxn matrices of equal size for x and y argument.');
               end
               if nargin >= 3
                   if isequal(size(obj.xgrid),size(z))
                       obj.zgrid = z;
                   else
                       error('MeshPointCloud:WA','Provide a nxn matrix of the same size as x for the z argument.');
                   end
                       
               end
           end
       end % constructor        
        
       function A = saveobj(obj)
	   % Allows to use the built-in save command.
           A.Name = obj.Name;
           A.xgrid = obj.xgrid;
           A.ygrid = obj.ygrid;
           A.zgrid = obj.zgrid;
           A.Colors = obj.Colors;
       end

       function h = surf(obj,varargin)
           % Plot the surface.
           % Any given argument will be given in turn to built-in surf.
           %
           % Output : a handle on the created surf object (optional).

           % For a nice plot, you may add
           % colormap(hot(256));
           % %colormap(jet(256));
           % %colormap(hsv(256));
           % camlight right;
           % lighting phong;
           % shading interp
           if isempty(obj.zgrid)
               warning('MeshPointCloud:NoP','No data to plot.');
               return
           end
           
           % is there any colors ?
           if ~isempty(obj.Colors);
               h = surf(obj.xgrid, obj.ygrid, obj.zgrid, obj.Colors, varargin{:});
           else
               h = surf(obj.xgrid, obj.ygrid, obj.zgrid, varargin{:}); 
           end
       end

       function h = surfc(obj,varargin)
           % Plot the surface with a contour plot.
           % Any given argument will be given in turn to built-in surfc.
           %
           % Output : a handle on the created surf object (optional).

           if isempty(obj.zgrid)
               warning('MeshPointCloud:NoP','No data to plot.');
               return
           end
           
           % is there any colors ?
           if ~isempty(obj.Colors);
               h = surfc(obj.xgrid, obj.ygrid, obj.zgrid, obj.Colors, varargin{:});
           else
               h = surfc(obj.xgrid, obj.ygrid, obj.zgrid, varargin{:}); 
           end
       end       

       function msize = Size(obj)
           % Return the size of the mesh, ie number of points.

           if ~isempty(obj.zgrid)
               msize = size(obj.zgrid); return
           elseif ~isempty(obj.ygrid)
               msize = size(obj.ygrid); return
           elseif ~isempty(obj.xgrid)
               msize = size(obj.xgrid); return
           else
               msize = 0;
           end
       end
       
       function pc = PointCloud(obj,TrueP)
           % Create a PointCloud object from the mesh. 
           %
           % Input :
           %    TrueP : if set to true, the TrueP field will be filled.
           %    False by default.

           pc = PointCloud(obj.Name);
           
           if isempty(obj.zgrid)
              warning('MeshPointCloud:NoP','No mesh values. Creating empty PointCloud.');
              return
           end
           
           if nargin < 2, TrueP = false; end
           
           % Initialise position matrix
           s = obj.Size;
           P = zeros(3,s(1)*s(2));
           % Fill it !
           for in = 1:s(1)              
               for im = 1:s(2)
                  ip = im + (in-1) * s(2);
                  P(:,ip) = [obj.xgrid(in,im), obj.ygrid(in,im), obj.zgrid(in,im)]';                   
               end
           end
           if TrueP
               pc.TrueP = P;
           else
               pc.P = P;
           end
           
           % The order in which the points come could be conserved in the
           % form of a Delaunay triangulation...
           
           % Find a way to convert the colors ?
           
       end
       
       
      
    end % end methods

               
    methods (Static)
        function obj = loadobj(A)
	   % Allows to use the built-in load command.
            obj = MeshPointCloud();
            obj.Name = A.Name;
            obj.xgrid = A.xgrid;
            obj.ygrid = A.ygrid;
            obj.zgrid = A.zgrid;
            obj.Colors=A.Colors;
        end % loadobj
    end % static methods
    
    
end

