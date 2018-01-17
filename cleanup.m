%% Root directory of this running .m file:
projectRootDir = fileparts(mfilename('fullpath'));

%% Remove project directories from path:
% rmpath(fullfile(projectRootDir,'data'));
rmpath(fullfile(projectRootDir,'functions'));
% rmpath(fullfile(projectRootDir,'models'));
% rmpath(fullfile(projectRootDir,'scripts'));
% rmpath(fullfile(projectRootDir,'system identification'));
% rmpath(fullfile(projectRootDir,'trajectory'));
rmpath(fullfile(projectRootDir,'work'));

%% Reset the loction of Simulink-generated files:
Simulink.fileGenControl('reset');

%% leave no trace...
clear projectRootDir;
clear;