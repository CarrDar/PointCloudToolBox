classdef ICPStatus < uint32
    %ICPSTATUS Enumeration class to store status of an iteration of an ICP algorithm or any related iterative algorithm.
    %
    % Call PrintStatusMeaning to print out all the status and their
    % relative conversion as uint32.
    %
    % Usage :
    %   From a given uint32 get the status :
    %   ICPStatus(8)
    %   From a given status get the corresponding uint32 :
    %   uint32(ICPStatus.Preprocessing)
    %   From a given status get the corresponding string of character :
    %   char(ICPStatus.Preprocessing)
    %   or
    %   t = ICPStatus.Preprocessing;
    %   t.char
    %
    %AUTHOR  : Neal Gauvin (at a3 dot epfl dot ch)
    %VERSION : 1.7
    %STATUS  : OK
    %DATE    : 1 juin 2011
    
    enumeration
        Dummy         (0)
        Preprocessing (1) % Also for initialisation
        Smoothing     (2) % The point cloud has been smoothed/griddified.
        Error         (3) % Processing error, abandon of the program.
        Stuck         (4) % Iteration caught in an endless process
        Failed        (5) % Iteration failed to get an alignment
        NotConverged  (6) % When the nb of max iteration is reached.
        Success       (7) % Iteration succeeded
        Converged     (8) % When the tolerance is reached.


        
    end
    
end

