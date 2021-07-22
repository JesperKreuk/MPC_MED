%{
This file is based on a file simalarly named by Nathan Timmers.
The script adds specified folders to the path such that all .m files can be
run.
%}

% Find main path
outputstruct = dir('mainfolder') ;
dots = '';
idx = 1;
while isempty(outputstruct) && idx < 5
    dots = ['../',dots];
    outputstruct = dir(strcat(dots,'mainfolder'));
    idx = idx + 1;
end

mainfolderpath = outputstruct.folder;

% Add path of certain subfolders
addpath(mainfolderpath);
addpath(genpath(strcat(mainfolderpath,[filesep 'Functions'])));
addpath(strcat(mainfolderpath,[filesep 'Results']));
addpath(strcat(mainfolderpath,[filesep 'CompassGaitBiped']));
addpath(strcat(mainfolderpath,[filesep 'Parameters']));
addpath(strcat(mainfolderpath,[filesep 'NMSModelsSimulink']));

% This is only possible if gurobi is installed! This is the default path of
% the gurobi installation
addpath(genpath('C:\gurobi911'))
