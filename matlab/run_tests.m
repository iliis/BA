% run all unittests in unit_tests folder

% add main directory and all subfolders to path
addpath(genpath(pwd));

import matlab.unittest.TestSuite;
run(TestSuite.fromFolder(fullfile(pwd, 'unit_tests')));