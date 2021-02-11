function RemoveFilesAndFolders( dirhead, flist, replist )
% REMOVEFILESANDFOLDERS remove all files and folders.
%
% RemoveFilesAndFolders loops recursively into a given folder and removes
% files or folders with a name corresponding to a given list.
%
% Input :
%   dirhead : the head directory to start from.
%   flist : a list of files to remove. 
%   replist : a list of folders to remove. 
%   Strings may contain wildcards.
%
% Usage :
%   RemoveFilesAndFolders( '/Data/Matlab/MatlabPointCloudToolbox_v2r4', {'*.asv','*.m~'},{'*.svn'});
%
%
%AUTHOR  : Neal Gauvin (at a3 dot epfl dot ch)
%VERSION : 1.0
%STATUS  : Ok
%DATE    : 12 september 2011

% Sanity checks
if nargin < 3
    error('RemoveFilesAndFolders:WArg','You need to provide three arguments. Type help RemoveFilesAndFolders.');
end

% Let's go !
cd(dirhead);

% Remove files.
for f = flist
    delete(f{:});
end

% Remove folders
for f = replist
    try rmdir(f{:},'s'); end                                               %#ok<*TRYNC>
end

% Let's move recursively
d = dir(dirhead);
for id = 1:length(d)
    % Avoid certain dangerous destinations
    if strcmpi(d(id).name,'.'), continue; end
    if strcmpi(d(id).name,'..'), continue; end
    if d(id).isdir
        cd(d(id).name);
        RemoveFilesAndFolders(pwd, flist, replist);
        cd(dirhead);
    end
end
    

end

