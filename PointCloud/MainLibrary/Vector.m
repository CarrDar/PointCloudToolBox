classdef Vector < handle
    %VECTOR A class to store efficiently data.
    %   
    % Vector implements a class to store data in the C++ vector class fashion.
    % The following data formats have been implemented :
    % - single double.
    % - arrays, both column- and row-wise.
    % - matrices of size nxm.
    %
    %
    % To be implemented :
    %   - add various operation on the vector, like mean, find, ...
    %   - other data formats.
    %   - A vector should be able to add itself with AddVector.
    %   - The Data property should become private. In this case, a Set
    %     method must be implemented.
    %
    % Vector works the following way. 
    % It first allocate in advance some memory for the data to push back
    % when the class is instantiated. That is to say, a certain number of
    % entries are filled with zeros. This number can be set via the Step
    % field directly, or already in the constructor. But always before the 
    % filling process begins. Default is 100. 
    %
    % Now, in the iterative process, when you want to store some values,
    % just call the AddEntry function. It will add the given value
    % in a new entry and will increment the number
    % of accessible entries NbEntries. Therefore NbEntries gives you always
    % the latest entry filled. Note the the class will never let you access
    % a field with an entry bigger than NbEntries.
    % However, you are free to go back in time and modify any stored value. 
    %
    % Finally, many public functions have been defined to acces the
    % data content, either for all entries, either for a desired entry.
    %
    %
    % Usage :
    %   Double values, dimension 1, for instance CPU times.
    %   vec1 = Vector(1);
    %   or
    %   vec1 = Vector([1 1]);
    %   3x3 matrices, for instance rotation matrices.
    %   vec33 = Vector([3 3]);
    %   If you already have an idea of what final length will be your
    %   vector, you can provide it to the constructor
    %   vec1 = Vector([1 1], 320);
    %   Later in your script. Fill in !
    %   vec1.AddEntry( 3.56 );
    %   vec33.AddEntry( magic(3) );
    %   To copy a Vector object, vec1 for instance :
    %   vcop = Vector(vec1);
    %
    % Check the various functions to retrieve the data entries.
    %
    % Important note :
    %   - If you now in advance the size of you data to store, you better
    %   work directly with a [] pre-initialised to the final size.
    %   Unless you are interested in working with a handle.
    %
    % Advice :
    %   - If you want to define methods for a specific use, create a new
    %   class with all your defined functions and make this class inherit
    %   from this one : 
    %       classdef StatVector < Vector
    %       ...
    %
    %AUTHOR  : Neal Gauvin (at a3 dot epfl dot ch)
    %VERSION : 1.0
    %STATUS  : OK
    %DATE    : 13 Sept 2011    
    
    properties
        Step        % size of a block of initialized data.
        Data        % Where the data is stored
    end
    
    properties(SetAccess=private)        
        NbEntries   % actual nb of entries in the stored data.
        Size        % size of the Data to store
        
    end % private properties
    
    methods
        
        function obj = Vector( S, n )
            % The constructor.
            %
            % Input :
            %   S : the size of the values to be stored in Data. Default
            %   double of size 1.
            %   n : an initialisation value for Step. Default 100. 
            %
            %   You can also provide a Vector object in S. In this case,
            %   the constructor will act as a copy constructor.

            if nargin < 2
                n = 100; 
                if nargin < 1
                    S = [1 1];
                end
            end
            % Copy constructor
            if isa(S,'Vector')
                obj.Data = S.Data;
                obj.Step = S.Step;
                obj.NbEntries = S.NbEntries;
                obj.Size = S.Size;
                return
            end
            % Beware, the initialisation of the properties can be made in
            % any order by Matlab.
            S = Vector.CheckSize( S );
            obj.Step = n;
            obj.Size = S;
            obj.Data = zeros([S n]);
            obj.NbEntries = 0;
            
        end % constructor
        
        function obj = set.Step( obj, n )                                  %#ok<*MCHV2>
            n = int32(n); % be sure n is an integer
            if n < 1
                warning('Vector:IntStep','You need to provide a positive integer for the default data length.'); return
            end
            if obj.Step > 0
                warning('Vector:StepCh',...
                    'You are not allowed to change the value of Step once initialised.');
                return;
            end
            obj.Step = n;
        end % set Step
                
        function AddVector( obj, V )
            % Add the content of a given Vector to this one.
            %
            % Input : 
            %   V : a Vector object that must be different this one.
            %
            % Usage :
            %   v1.AddVector(v2);

            % Check compatibility of vectors
            if ~isequal(obj.Size, V.Size)
                error('Given vector must have the same data content size to be added.');
            end
            
            if obj == V
                error('A vector cannot be added to itself yet.');
            end
            
            % Nothing to do !
            if V.NbEntries == 0, return; end
            
            % Faster this way...
            
            % Future number of entries
            FNbEntries = obj.NbEntries + V.NbEntries;
            
            % Free entries
            FE = obj.Step - obj.NbEntries;
            % if more entries to add than available free entries, we need
            % to add more block of preallocated memory.
            if V.NbEntries > FE
                % How many blocks to add ?
                NE = V.NbEntries - FE;
                NbBlocs = ceil(double(NE)/double(obj.Step));
                obj.AddBlock(NbBlocs);
            end
            
            % Add data content
            obj.Data(:,:,(obj.NbEntries+1):FNbEntries ) = V.Data(:,:,1:V.NbEntries);            
            
            % Adapt number of entries
            obj.NbEntries = FNbEntries;                       
           
        end
 
        function Data = GetData( obj, n )
            % Get data content for a given entry.
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
               error('Vector:WEnt','Entry nb must be non zero and inferior to the total nb of entries.');
            end 
            Data = obj.Data(:,:,n);            
            
        end 
        
        function Data = GetAllData( obj )
            %Get data content for all available entries.
            
            % For user comfort, suppress unused rows.
            if isequal( obj.Size, [1 1] )
                Data(1:obj.NbEntries) = obj.Data(1,1,1:obj.NbEntries);
            elseif isequal( obj.Size, [3 1] )
                Data(:,1:obj.NbEntries) = obj.Data(:,1,1:obj.NbEntries);
            else
                Data = obj.Data(:,:,1:obj.NbEntries);
            end
        end % get Data
        
        function Data = GetLastData( obj )
            %Get data content for the last available entries.
            Data = obj.Data(:,:,obj.NbEntries);
        end % get Data
        
        function obj = AddEntry( obj, Data )
            % ADDENTRY Add a new data entry in the vector.
            %
            % When AddEntry is called, a new entry is created for the Data
            % property. Give in argument the value for the property to be set to.
            %
            % Input :
            %   Data : the data element to store
            %
            % Usage :
            %
            % In the global script
            % myvect = Vector(1);
            % Somewhere...
            % myvect.AddEntry( 2.56 );
            % And so on...
            
            if obj.NbEntries>1 && mod(obj.NbEntries,obj.Step) == 0
                obj.AddBlock();
            end
            if nargin < 2
                warning('Vector:NoD',...
                    'You need to provide some data to store...'); return
            end
            % Check if data is compatible.
            s = size(Data);
            if ~isequal(s,obj.Size)
                error('Vector:WS','You must provide data with the same size as the one the Vector has been initialised to.');
            end
            obj.Data(:,:,obj.NbEntries+1) = Data;
            %At the end, because if a set fails, NbEntries mustn't be
            %incremented.
            obj.NbEntries = obj.NbEntries + 1;
            
        end % AddEntry
        
    end % methods
    
    methods(Access=private,Hidden=true)
        
        function obj = AddBlock( obj, NbBlocs )
            % Internal function to add blocks of memory of size Step.
            % NbBlocs : number of blocks to add. Default 1.
            if nargin < 2, NbBlocs = 1; end
            if NbBlocs < 1, return; end
            TotStep = NbBlocs * obj.Step;
            
            obj.Data(:,:,(end+1):(end+TotStep) ) = zeros([obj.Size TotStep]);

        end % AddBlock
        
    end % private and hidden methods
 
    methods(Access=private,Hidden=true, Static=true)
        function S = CheckSize( S )
            if ~isnumeric(S)
                error('Vector:WS','You need to provide numerical values for the data size.');
            end
            s = size(S);
            if isequal(s,[1 1])
                S = [1 S];
            elseif ~isequal(s,[1 2])
                error('Vector:WS','Sorry only data size equals to [n m] are implemented for now.');
            end
        end
    end % private hidden static methods
        
end

