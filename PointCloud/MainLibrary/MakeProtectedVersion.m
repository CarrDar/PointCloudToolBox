function MakeProtectedVersion( version, folderpath)
% MAKEPROTECTEDVERSION Create an encrypted (protected) copy of MatlabPointCloudToolBox.
%
% The MakeProtectedVersion first create a copy of the MatlabPointCloudToolBox 
% directory (whatever is its name) found in the Matlab path.
% The new name will be the same as the old one plus the extension specified in argument.
% Then all m files are encrypted using pcode, except the Contents.m files.
% Finally, all remaining .asv, .m and .m~ are deleted, as well as possible
% .svn files and folders.
%
% The folders to protect are specified in the begining of the function,
% for now they are :
% - PointCloud/lib
% - PointCloud/PointCloudAlignment
% - PointCloud/ShapeSimulator
%
% Certain files can escape protection, like the demo files. Specify
% patterns in PatAvoid at the begining of the function and of course put
% that pattern in the file names.
%
% Input :
%   version : string name to add as extension to the copy of the
%   MatlabPointCloudToolBox directory. Default '_copy.
%   Note that a '_' is automatically added between the name and the
%   extension.
%
%   folderpath : string of name plus full path of the folder to protect.
%   If none is provided, MakeProtectedVersion will look in the Matlab path
%   if it can find a folder containing a 'PointCloud' repertory.
%   If none or more than 1 is found, the function stops.
%
% Usage :
%   MakeProtectedVersion('v2r5');
%   MakeProtectedVersion('v2r5','/Data/Matlab/MatlabPointCloudToolbox_v2r4');
%
%
%AUTHOR  : Neal Gauvin (at a3 dot epfl dot ch)
%VERSION : 1.1
%STATUS  : Ok
%DATE    : 12 september 2011

% Menu
% List of dir to protect.
DirToProt = {'_PointCloud/MainLibrary','_PointCloud/PointCloudAlignment','_PointCloud/PointCloudComparison','_PointCloud/ShapeSimulator'};
% A pattern in file name to avoid protection
PatAvoid = 'demo';

% Initialisation
if nargin < 1
    version = 'copy';
end

version = ['_' version];
PatAvoid = lower(PatAvoid); % work with lowercase.

if nargin < 2
    % Find the root of the MatlabPointCloudToolbox in the Matlab path.
    R = what('PointCloud');
    % It might happen that  there are numerous folders in the Matlab path that
    % contain a PointCloud...
    if length(R) ~= 1
        fprintf('Numerous folders have been found which contain a PointCloud folder :\n');
        R.path
        fprintf('Rerun MakeProtectedVersion, giving it the name with the full path of the folder you wish to protect.\n');
        return;
    end
    % Retrieve the name of the MatlabPointCloudToolbox folder.
    folderpath = R.path(1:(end-11));
end

% Remind from where you start.
home = pwd;

% Be sure the doc is updated
GenerateDoc;

% Go on top of it.
cd([folderpath '/..']);

% Create a copy and move into it
folderpathv = [folderpath version];
copyfile(folderpath, folderpathv); 
cd(folderpathv);

% Loop on dir to protect.
for d = DirToProt
    cd(d{:});

    % Loop on all files in it.
    D = what(d{:});
    mf = D.m;
    for imf = mf'
        % Protect all m-files excepts Contents.m        
        if strcmpi(imf{:},'Contents.m'), continue; end
        
        % Avoid a certain pattern
        k = strfind(lower(imf{:}), PatAvoid);
        if ~isempty(k), continue; end
        
        pcode(imf{:});

        % Remove the undesired files.
        delete(imf{:});

    end % files
    cd(folderpathv);
    
end % folders to protext

% Remove remaining undesired files and folders
RemoveFilesAndFolders( folderpathv, {'*.asv','*.m~'},{'.svn'});

fprintf('Created %s from %s.\n', folderpathv, folderpath);

% Go back home home.
cd(home);

end




