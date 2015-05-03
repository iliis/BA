function test_minimization_gui

f = figure('visible', 'off', 'Position', [0,0,1200,700]);

% main gui elements

himg_source_text = uicontrol('Style', 'text', ...
    'String', 'image pair', ...
    'Position', [10 600 100 25]);

himg_source = uicontrol('Style', 'popupmenu', ...
    'String', find_img_source_directories(), ...
    'Callback', @dropdown_img_source_callback, ...
    'Position', [10 575 300 25]);

hscale_text = uicontrol('Style', 'text', ...
    'String', 'scale', ...
    'Position', [10 540 100 25]);

hscale = uicontrol('Style', 'popupmenu', ...
    'String', {1,2,3,4,5,6}, ...
    'Callback', @dropdown_img_source_callback, ... % update list of plots
    'Position', [120 540 190 25]);

himg_plot = uicontrol('Style', 'popupmenu', ...
    'String', 'choose img pair first ...', ...
    'Callback', @dropdown_img_plot_callback, ...
    'Position', [10 500 300 25]);

aplot_axes = axes('Units', 'pixels', 'Position', [330 30 860 660]);




% change units to normaliezd so components resize automatically
f.Units = 'normalized';
himg_source_text.Units = 'normalized';
himg_plot.Units = 'normalized';
himg_source.Units = 'normalized';
aplot_axes.Units = 'normalized';
hscale_text.Units = 'normalized';
hscale.Units = 'normalized';

current_plot_path = '';


% update listing
dropdown_img_source_callback();


f.Visible = 'on';


    function dropdown_img_source_callback(source, ~)
        [~, foldername] = fileparts(himg_source.String{himg_source.Value});
        plots = find_errorplots(foldername, hscale.Value);
        if size(plots,1) > 0
            himg_plot.String = plots(:,2);
        else
            himg_plot.String = {'no plots found!'};
        end
        
        if himg_plot.Value > numel(himg_plot.String)
            himg_plot.Value = 1;
        end
    end

    function dropdown_img_plot_callback(source, ~)
        [~, foldername] = fileparts(himg_source.String{himg_source.Value});
        plots = find_errorplots(foldername, 4);
        
        % clear current plot
        cla(aplot_axes);
        
        tmpfig = openfig(plots{himg_plot.Value, 1}, 'invisible');
        
        % copy graph (axes) from figure
        copyobj(allchild(tmpfig.Children(2)), aplot_axes);
        
        xlim(aplot_axes, tmpfig.Children(2).XLim);
        ylim(aplot_axes, tmpfig.Children(2).YLim);
    end


end