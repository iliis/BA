% execute all scripts in test_scripts to check if they run without error

clear all;

scripts = dir(fullfile('test_scripts/*.m'));

for script_idx = 1:size(scripts,1)
    filename = fullfile('test_scripts', scripts(script_idx).name);
    
    disp(['executing "' filename '"']);
    
    run(filename);
    
    % clean up after script
    close all;
    % clear all; % cannot do this: would also clear variables in this script :P
    clearvars -except scripts script_idx;
end