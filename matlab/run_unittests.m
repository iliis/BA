% run all unittests in unit_tests folder

% add main directory and all subfolders to path
addpath(genpath(pwd));

import matlab.unittest.TestSuite;
import matlab.unittest.constraints.IsEqualTo;
import matlab.unittest.constraints.AbsoluteTolerance;

run(TestSuite.fromFolder(fullfile(pwd, 'unit_tests'), 'IncludingSubfolders', true));

% to run a single test suite:
%run(TestSuite.fromClass(?jacobi_tests))