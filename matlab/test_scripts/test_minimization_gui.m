function test_minimization_gui

f = figure('visible', 'off', 'Position', [0,0,1200,700]);

global minimization_running;
minimization_running = false;

min_methods = {'Gradient Descent', @gradient_descent; ...
    'Gauss-Newton', @(D1,I1,I2,T,r,~) gauss_newton(D1,I1,I2,T,r); ...
    'Levenberg-Marquardt', @(D1,I1,I2,T,r,~) levenberg_marquardt(D1,I1,I2,T,r)};
% TODO: lsqnonlin

min_init = [1.1 0 0 0 0 0];


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
    ... %'Callback', @dropdown_img_plot_callback, ...
    'Position', [10 500 300 25]);

himg_show = uicontrol('Style', 'pushbutton', ...
    'String', '[re]load error surface', ...
    'Callback', @show_errorsurface, ...
    'Position', [10 460 300 25]);

hmin_method = uicontrol('Style', 'popupmenu', ...
    'String', min_methods(:,1), ...
    'Position', [10 350 300 25]);

hmin_restrict2D = uicontrol('Style', 'checkbox', ...
    'String', 'restrict to X and Y', ...
    'Position', [10 320 300 25]);

hmin_step_size_label = uicontrol('Style', 'text', ...
    'String', 'step size:', ...
    'Position', [10 290 100 25]);

hmin_step_size = uicontrol('Style', 'edit', ...
    'String', 0.0001, ...
    'Position', [110 290 190 25]);

hmin_init_text = uicontrol('Style', 'text', ...
    'String', {'initial T: ', num2str(min_init')}, ...
    'Position', [10 140 300 140]);

hchoose_min_init = uicontrol('Style', 'pushbutton', ...
    'String', 'choose initial T', ...
    'Callback', @choose_initial_T, ...
    'Position', [10 130 300 25]);

hmin_start_button = uicontrol('Style', 'pushbutton', ...
    'String', 'START', ...
    'Callback', @start_minimization, ...
    'Position', [10 10 300 50]);

aplot_axes = axes('Units', 'pixels', 'Position', [340 30 850 660]);




% change units to normaliezd so components resize automatically
f.Units = 'normalized';
himg_source_text.Units = 'normalized';
himg_plot.Units = 'normalized';
himg_source.Units = 'normalized';
aplot_axes.Units = 'normalized';
hscale_text.Units = 'normalized';
hscale.Units = 'normalized';
himg_show.Units = 'normalized';
hmin_method.Units = 'normalized';
hmin_restrict2D.Units = 'normalized';
hmin_init_text.Units = 'normalized';
hchoose_min_init.Units = 'normalized';
hmin_start_button.Units = 'normalized';
hmin_step_size_label.Units = 'normalized';
hmin_step_size.Units = 'normalized';

% update listing
dropdown_img_source_callback();


f.Visible = 'on';


    function dropdown_img_source_callback(~, ~)
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

    function show_errorsurface(~, ~)
        [~, foldername] = fileparts(himg_source.String{himg_source.Value});
        plots = find_errorplots(foldername, hscale.Value);
        
        tmpfig = openfig(plots{himg_plot.Value, 1}, 'invisible');
        
        % clear current plot
        cla(aplot_axes);
        
        % copy graph (axes) from figure
        copyobj(allchild(tmpfig.Children(2)), aplot_axes);
        
        xlim(aplot_axes, tmpfig.Children(2).XLim);
        ylim(aplot_axes, tmpfig.Children(2).YLim);
    end

    function choose_initial_T(~, ~)
        [x, y] = ginput(1);
        min_init(1) = x;
        min_init(2) = y;
        hmin_init_text.String = {'initial T: ', num2str(min_init')};
        hold on; plot(min_init(1),min_init(2),'xw'); hold off;
    end

    function start_minimization(~, ~)
        if (~minimization_running)
            % not running, start simulation

            image_path  = fullfile(himg_source.String{himg_source.Value}); %'unit_tests/linear_translation';

            minimization_running = true;
            hmin_start_button.String = 'STOP';
            do_global_minimization(image_path, hscale.Value, min_methods{hmin_method.Value,2}, hmin_restrict2D.Value, str2num(hmin_step_size.String), min_init);
        else
            % abort minimization currently in progress
            minimization_running = false;
            hmin_start_button.String = 'START';
        end
    end

end