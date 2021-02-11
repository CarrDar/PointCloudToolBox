function Error = Error_PolyWorks( filename )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
[ ~,~,~,err_X,err_Y,err_Z,err_sign ] = Import_txt_Polyworks(filename);

Vector(:,1)=(err_X);
Vector(:,2)=(err_Y);
Vector(:,3)=(err_Z);

Error=nan(size(Vector,1),1);
for i=1:size(Vector,1)
    Error(i,1)=norm(Vector(i,1:3));
    if char(err_sign{i,1})==char('-');
        Error(i,1)=-1*Error(i,1);
    end
end
end

function [ X,Y,Z,err_x,err_y,err_z,sign ] = Import_txt_Polyworks(filename)
%% Import data from text file.
%% Initialize variables.
delimiter = ' ';
%% Format string for each line of text:
%   column1: double (%f)
%	column2: double (%f)
%   column3: double (%f)
%	column4: double (%f)
%   column5: double (%f)
%	column6: double (%f)
%   column7: text (%s)
% For more information, see the TEXTSCAN documentation.
formatSpec = '%f%f%f%f%f%f%s%[^\n\r]';
%% Open the text file.
file = fopen(filename,'r');
%% Read columns of data according to format string.
dataArray = textscan(file, formatSpec, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true, 'EmptyValue' ,NaN, 'ReturnOnError', true);
%% Close the text file.
fclose(file);
%% Post processing for unimportable data.
%% Allocate imported array to column variable names
X = dataArray{:,1};
Y = dataArray{:,2};
Z = dataArray{:,3};
err_x = dataArray{:,4};
err_y = dataArray{:,5};
err_z = dataArray{:,6};
sign = dataArray{:,7};
end