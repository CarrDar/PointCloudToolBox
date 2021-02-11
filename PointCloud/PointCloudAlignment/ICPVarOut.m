classdef ICPVarOut < handle
    %ICPVAROUT A class to store efficiently the output variables of ICP algorithms,
    % such as the transformation matrices, CPU time consumed, status in each
    % iteration. Type doc PointCloud for a more detailed description.
    % Please add here all variables you may want to keep for further study.
    % As well as the useful functions to display/study them.
    %
    % ICPVarOut allows to store values in the following properties :
    %   TM : transform matrices.
    %   CPU : CPU times.
    %   Status : iteration statuses.
    %   Dist : distances between point clouds.
    %   See below for more details about the properties.
    %   Each of these fields gets the same dimension as the value to store 
    %   plus an extra dimension specifying the iteration number, or called here "entry". 
    %
    % ICPVarOut works the following way. 
    % It first allocate in advance some memory for the iterative properties
    % when the class is instantiated. That is to say, a certain number of
    % entries are filled with nans. This number can be set via the Step
    % field directly, or already in the constructor. But always before the 
    % filling process begins. Default is 100. 
    %
    % Now, in the iterative process, when you want to store some values,
    % just call the AddEntry function. It will set the iterative properties
    % with the provided values in a new entry and will increment the number
    % of accessible entries NbEntries. Therefore NbEntries gives you always
    % the latest entry filled. Note the the class will never let you access
    % a field with an entry bigger than NbEntries.
    % However, you are free to go back in time and modify any stored value. 
    %
    % Finally, many public functions have been defined to acces the
    % iterative properties, either for all entries - when you want to plot
    % the CPU values with respect to the iteration step for instance -
    % either for a desired entry.
    % A last set of functions allows you to display the stored values.
    %
    % Usage :
    %
    % In the global script, instantiate an ICPVarOut object :
    % out = ICPVarOut();
    % Somewhere in an iteration, fill in the iterative properties :
    % out.AddEntry( aTM, uint32(ICPStatus.Preprocessing), toc );
    % out.AddEntry( eye(4,4), uint32(ICPStatus.Failed), 0 );
    % out.AddEntry( TM, aStatus, aCPU, aDist );
    % And so on...
    %
    % At the end of the iterative process. 
    % Display the CPU time with respect to the nb of iterations :
    % out.PlotCPU;
    % Or retrieve some properties like the last transform matrix :
    % out.LastTM
    %
    %AUTHOR  : Neal Gauvin (at a3 dot epfl dot ch)
    %VERSION : 1.23
    %STATUS  : OK
    %DATE    : 31 mai 2011    
    
    properties
        TM          % a 4x4 x NbEntries matrix to store the transformation matrices
        CPU         % a 1xNbEntries matrix to store CPU times.
        Status      % a 1xNbEntries matrix of integers to store ICPStatus of iterations. 
        % Keep the following distances :
        %       (1),(2) Mean distance between measured matched positions (mean and rms).
        %       (3) Mean distance between true matched positions.
        %       (4) Mean distance between true positions irrespectively of any matching.
        %          In a perfect world this should be close to 0, therefore giving an
        %          idea of the "error" in the alignment.
        %       (5)=(4)-(3) error of the matching.
        Dist        
        Step        % size of a block of initialized data.    
    end

    properties(SetAccess=private)

        NbEntries   % actual nb of entries in the stored data.        
        
    end % private properties
 
    methods
        
        function obj = ICPVarOut( n )
            % The constructor.
            % Input :
            %   n : an initialisation value for Step. Default 100. 
            if nargin < 1, n = 100; end
            obj.Step = n;
            obj.TM = zeros(4,4,obj.Step);
            obj.Dist = zeros(5,obj.Step);
            obj.CPU = zeros(1,obj.Step);
            obj.Status = zeros(1,obj.Step);
            obj.NbEntries = 0;
            
        end % constructor
        
        function obj = set.Step( obj, n )                                  %#ok<*MCHV2>
            n = int8(n); % be sure n is an integer
            if n < 1
                warning('ICPVarOut:IntStep','You need to provide a positive integer for the default data length.'); return
            end
            if obj.Step > 0
                warning('ICPVarOut:StepCh',...
                    'You are not allowed to change the value of Step once initialised.');
                return;
            end
            obj.Step = n;
        end % set Step

        function obj = set.TM( obj, TM )
            if size(TM,1)~=4 || size(TM,2)~=4 || ~isreal(TM)
                warning('ICPVarOut:WTM','Only 4x4 matrices of real can be stored');
                return
            end
            obj.TM = TM;
        end % set TM
 
        function obj = set.CPU( obj, CPU )
            if ~isreal(CPU)
                warning('ICPVarOut:Real','Only real can be stored');
                return
            end
            obj.CPU = CPU;
        end %set CPU
        
        function obj = set.Status( obj, Status )
            if ~isreal(Status)
                warning('ICPVarOut:Real','Only real can be stored');
                return
            end
            obj.Status = Status;
        end % Set Status
 
        function obj = set.Dist( obj, Dist )
            if ~isreal(Dist) || ~isequal(size(Dist,1),5)
               error('ICPVarOut:WDist','Please provide a vector of size 5 of reals for Dist.'); 
            end
            obj.Dist = Dist;
        end % set.Dist
        
        function obj = SetDist( obj, Dist, n )
            % Set Dist for a given entry.
            %
            % Input :
            %   Dist : the value to set Dist to, for the given entry nb.
            %   n : an entry number. It must be non zero and inferior to
            %   the total nb of entries NbEntries. By default it is set to
            %   NbEntries, that is to say the last one.

            if nargin < 3
                n = obj.NbEntries;
            end
            if n == 0 || n > obj.NbEntries
               error('ICPVarOut:WEnt','Entry nb must be non zero and inferior to the total nb of entries.');
            end               
            obj.Dist(:,n) = Dist;
            
        end % SetDist

        function Dist = GetDist( obj, n )
            % Get Dist for a given entry.
            %
            % Input :
            %   n : an entry number. It must be non zero and inferior to
            %   the total nb of entries NbEntries. 
            %   It can be also inferior to zero. In this case, the entry is
            %   taken as NbEntries minus the absolute value of n.
            %   By default it is set to NbEntries, that is to say the last one.
            
            if nargin < 2
                n = obj.NbEntries;
            end
            if n < 0
                n = obj.NbEntries + n;
            end
            if n <= 0 || n > obj.NbEntries
               error('ICPVarOut:WEnt','Entry nb must be non zero and inferior to the total nb of entries.');
            end               
            Dist = obj.Dist(:,n);            
            
        end % GetDist
        
        function TM = GetTM( obj )
            %Get transformation matrices for all available entries.
            TM = obj.TM(:,:,1:obj.NbEntries);
        end % get TM
        
        function TM = GetLastTM( obj )
            % Get the last available transformation matrix.
            TM = obj.LastTM;
        end        
        
        function TM = LastTM( obj )
            % Get the last available transformation matrix.
            TM = obj.TM(:,:,obj.NbEntries);
        end
        function R = LastRot( obj )
            % Get the last available rotation matrix.
            %
            % Output : a 3x3 matrix. 
            R = obj.TM(1:3,1:3,obj.NbEntries);
        end
        function T = LastTransl( obj )
            % Get the last available translation vector.
            %
            % Output : a 3x1 vector.
            T = obj.TM(1:3,4,obj.NbEntries);
        end 

        function TM = GetLastTMTransformed( obj, TMin, TMode )
            % GETLASTTMTRANSFORMED Get the last transform matrix transformed by matrix given in argument. 
            %
            % Input:
            %   TMin : a transform matrix.
            %   TMode : set the transformation mode. Type doc PointCloud
            %           for more infos. By default TMode = 0.
            %
            if ~isequal(size(TMin),[4 4])
                msg = 'Please provide a 4x4 matrix for the transform matrix.';
                error('ICPVarOut:WTM', msg);
            end
            if nargin < 3, TMode = 0; end
            TM = eye(4,4);
            LTM = obj.LastTM;
            TM(1:3,1:3) = TMin(1:3,1:3)*LTM(1:3,1:3);
            if TMode == 0
                TM(1:3,4) = TMin(1:3,1:3)*LTM(1:3,4) + TMin(1:3,4);
            elseif TMode == 1
                TM(1:3,4) = LTM(1:3,4) + TMin(1:3,4);
            elseif size(TMode,1)==3
                warning('ICPVarOut:WMode','Unsupported transform mode. Work with 0 and 1.\n'); PointCloud
            else
                warning('ICPVarOut:WMode','Unsupported transform mode. Work with 0 and 1.\n'); PointCloud
            end
        end % GetLastTMTransformed
        
        function CPU = GetCPU(obj)
            % Get all available CPU values in an array.
            CPU = obj.CPU(1:obj.NbEntries);
        end % get CPU

        function CPU = GetLastCPU(obj)
            % Get the last available CPU entry.
            CPU = obj.CPU(obj.NbEntries);
        end % get last CPU
        
        function Status = GetStatus(obj)
            % Get all available Statuses in an array.
            Status = obj.Status(1:obj.NbEntries);
        end
        
        
        function obj = AddEntry( obj, TM, Status, CPU, Dist )
            % ADDENTRY Add a new entry.
            %
            % When AddEntry is called, a new entry is created for all
            % properties. Give in argument the values for the properties to be set to. 
            % By default, new entries are filled with nans.
            %
            % Input :
            %   TM : a transform matrix to store in TM.
            %       Provide at least an identity matrix.
            %   Status : a valid integer to store in Status (optional).
            %
            %   CPU : a CPU value (optional).
            %
            %   Dist : a matrix to fill in Dist (optional).
            %
            % Usage :
            %
            % In the global script
            % out = ICPVarOut();
            % Somewhere in an iteration
            % out.AddEntry( aTM, uint32(ICPStatus.Preprocessing), toc );
            % out.AddEntry( eye(4,4), uint32(ICPStatus.Failed), 0 );
            % out.AddEntry( TM, aStatus, aCPU, aDist );
            % And so on...
            
            if obj.NbEntries>1 && mod(obj.NbEntries,obj.Step) == 0
                obj.AddBlock(); 
            end
            if nargin < 2
                warning('ICPVarOut:NoD',...
                    'You need to provide some data to store...'); return
            end
            if nargin >= 5
                obj.Dist(:,obj.NbEntries+1) = Dist;
            end
            if nargin >= 4
                    obj.CPU(obj.NbEntries+1) = CPU;
            end
            if nargin >= 3
                obj.Status(obj.NbEntries+1) = Status;
            end
            if nargin >= 2
                obj.TM(:,:,obj.NbEntries+1) = TM;                    
            end
            %At the end, because if a set fails, NbEntries mustn't be
            %incremented.
            obj.NbEntries = obj.NbEntries + 1;
           
        end % AddEntry
        
        function PlotCPU( obj, varargin )
            % PLOTCPU Plots the CPU time with respect to the iteration number.
            %
            % Additional parameters that can be set via their field name :
            %
            %   Accu
            %       Set Accu = -1 to turn on the accumulation mode.
            %       or give in an array step statuses that reset the
            %       accumulation.
            %
            %   Filter
            %       array of status of steps you want to plot.
            %   Note : for both Filter and Accu, you can provide integers,
            %   as well as ICPStatus.
            %
            %   Title
            %       title of the plot.
            %
            % Usage :
            %   Output.PlotCPU('Filter',[1 2 6]);
            %   Output.PlotCPU('Accu',[1 2]);

            ip = inputParser;
            ip.addParamValue('Accu', [], @(x)isnumeric(x));
            ip.addParamValue('Filter', [], @(x)isnumeric(x));
            ip.addParamValue('Title','3D Point Cloud Alignement CPU consumption', @(x)ischar(x));
            ip.parse(varargin{:});
            arg = ip.Results;
          
            figure1 = figure('Color',[1 1 1]);
            % Create axes
            axes1 = axes('Parent',figure1);
            box(axes1,'on');
            hold(axes1,'all');

            T = obj.GetCPU;
            X = 1:length(T);
            C = obj.GetStatus;
            
            % Loop on the steps
            ttot = 0;
            for it = 1:length(T)
               %Filter !
                if ~isempty(arg.Filter)
                   if ~any( C(it)==arg.Filter )
                       X(it)=nan; continue;
                   end
                end
               % Accumulate
               if ~isempty(arg.Accu)                   
                   if any( C(it)==arg.Accu )
                       ttot = 0;
                   end
                   ttot = ttot + T(it);
                   T(it) = ttot;
               end
            end
            
            % Create scatter plot
            scatter(X,T,20,C);
            
            % Create ylabel
            ylabel({'CPU time [s]'});            
            % Create xlabel
            xlabel({'Nb of iterations'});           
            % Create title
            title(arg.Title);
            % Create colorbar
            colorbar('peer',axes1,'LineWidth',1);
            
            % Print out the meaning of the Status code.
            obj.PrintStatusMeaning
            
        end        
        
        function PlotDist( obj, varargin )
            % Plot the Dist stored values in a nice way, with respect to the iteration number.
            %
            % Additional parameters that can be set via their field name :
            %
            %   Log : 
            %       {true} | false
            %       Set log display mode for y axis.
            %
            %   DisplStatus :
            %       {false} | true
            %       Display the iteration statuses with colors
            
            ip = inputParser;
            %validLog = {'log','linear'};
            %ip.addParamValue('Log', 'log', @(x)any(strcmpi(x,validLog)));
            ip.addParamValue('Log', true, @(x)islogical(x));
            ip.addParamValue('DisplStatus', false, @(x)islogical(x));
            ip.parse(varargin{:});
            arg = ip.Results;
            
            % Set log scale according to given input.
            if arg.Log, arg.Log = 'log';
            else arg.Log = 'linear'; end
            
            fig1 = figure('Color',[1 1 1]);
            % Create title
            title('Minimum Distances between 2 Point Clouds');

            ttle = {'Mean dist. between measured matched positions',...
                'RMS dist. between measured matched positions',...
                'Mean dist. between matched true positions',...
                'Mean dist. between true positions irresp. of any matching',...
                'Error of the matching'};
            
            D = obj.Dist(:,1:obj.NbEntries);
            X = 1:size(D,2);
            C = obj.GetStatus;
            
            for ip = 1:size(obj.Dist,1)
                s1 = subplot(3,2,ip,'Parent',fig1,'YScale',arg.Log);
                box(s1,'on');
                hold(s1,'all');
                if arg.DisplStatus
                    % Create scatter plot
                    scatter(X,D(ip,:),20,C);
                    % Create colorbar
                    colorbar('peer',s1,'LineWidth',1);
                 else
                    %semilogy(obj.Dist(ip,1:obj.NbEntries),'Parent',s1);% Create semilogy
                    plot(X,D(ip,:));
                end % DisplStatus
                xlabel('Nb iterations');
                ylabel('Distance');
                title( ttle{ip} )
            end % loop on distances
            
            % Print out the meaning of the Status code.
            if arg.DisplStatus, obj.PrintStatusMeaning; end
            
        end % PlotDist
        
        function PrintDist( obj, p, n )  
            % Nice print out of the Dist stored value for a given entry.
            %
            % Input :
            %   p : the desired precision. Default : 2.
            %   n : the entry to print out. Default : the last one.
            
            s1 = 'Mean distance between measured matched positions';
            s2 = 'RMS distance between measured matched positions';
            s3 = 'Mean distance between true matched positions';
            s4 = 'Mean distance between true positions irresp. of any matching';
            s5 = 'Error of the matching'; % =4-3
            if nargin > 2
                dist = obj.GetDist(n);
            else
                dist = obj.GetDist();
            end
            if nargin < 2
                p = 2;
            end
            fprintf('%-65s : %.*f\n', s1, p, dist(1));
            fprintf('%-65s : %.*f\n', s2, p, dist(2));
            if ~isnan(dist(3))
                fprintf('%-65s : %.*f\n', s3, p, dist(3));
                fprintf('%-65s : %.*f\n', s4, p, dist(4));
                fprintf('%-65s : %.*f\n', s5, p, dist(5));
            end
            
        end % PrintDist        
        
    end
    
    methods(Access=private,Hidden=true)
        
        function obj = AddBlock( obj )
            % Internal function to add a block of memory.
            obj.TM = cat( 3, obj.TM, zeros(4,4,obj.Step) );
            obj.CPU = [ obj.CPU zeros(1,obj.Step)];
            obj.Status = [ obj.Status zeros(1,obj.Step) ];
            obj.Dist = [ obj.Dist zeros(5,obj.Step) ];
        end % AddBlock
        
    end % private and hidden methods
    
    methods(Static=true)

        function PrintStatusMeaning
            % Print out the meaning of the Status code from the ICPStatus enumeration class.
            
            fprintf('----- Meaning of the Status code -----\n');
            is = 0;
            while true
                try
                    stat = ICPStatus(is).char;
                catch                                                       %#ok<CTCH>
                    break;
                end
                fprintf('%i : %s \n', is, stat);                
                is = is + 1;
            end
            fprintf('--------------------------------------\n');
        end % PrintStatusSign        
        
    end % static methods
    
    
end

