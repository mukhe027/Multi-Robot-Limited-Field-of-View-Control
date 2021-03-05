function [] = setWorkingFolders(folder)
%% Script to remove all subfolders from path and add the one selected

% Suppress rmpath warning
warning('off','MATLAB:rmpath:DirNotFound');

% Find current path
current_path = pwd;
% Elaborate folders and subfolders path
folders = genpath(current_path);
% Remove them
rmpath(folders);

% Find new path
new_path = genpath(folder);
addpath(new_path);