function plots = find_errorplots( foldername, scale )
% looks for errorplot images in plots/errplots that fit given name and scale

plots = {};
if isdir(fullfile('plots', 'errplots', foldername))
    figs = dir(fullfile('plots', 'errplots', foldername, ['scale' num2str(scale) '*.fig']));
    plots = cell(numel(figs),2);
    for i = 1:numel(figs)
        plots{i, 1} = fullfile('plots', 'errplots', foldername, figs(i).name);
        plots{i, 2} = figs(i).name;
    end
end

end

