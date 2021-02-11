function PC = ImportPointCloudFromASCII( file, name, fields )
% IMPORTPOINTCLOUDFROMASCII Create a PointCloud handle from imput data in an ASCII file. 
%
% Input : 
%   file   : file name containing a possible path.
%   name   : name of PointCloud (optional)
%   fields : name of properties to set in PointCloud from the file, in the
%         given order. Default {'P'}.
%
%AUTHOR  : Neal Gauvin (at a3 dot epfl dot ch)
%VERSION : 1.3
%STATUS  : OK
%DATE    : 7 juin 2011

    if nargin < 3
        fields = {};
        if nargin < 1
            error('ImportPointCloudFromASCII:NoF','You need to provide a file name');
        end
    end
    PC = PointCloud;
    PC.ImportDataFromASCII(file,fields); %the consistency of file is check in there.
    if nargin > 1
        PC.Name = name;
    end
    
end

