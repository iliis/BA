% basic test for new core functions
% loads an image pair and warps the keyframe onto the current frame

%testscene = Scene('input/testscene1');
%testscene = Scene('input/testscene2');
%testscene = Scene('input/trajectory1');
%testscene = Scene('input/courtyard/lux');
%testscene = Scene('input/testscene_rotonly');
testscene = Scene('../presentations/final/media/smallscene');

%testscene = testscene.scale_down(3);

%for k = 1:testscene.step_count

k = 1;
    
    step = testscene.getStep(k);
    T = [-3 3 0 0 0 0];
    %T = step.ground_truth;
    %T = [ -0.26418     0.52966      1.6856    0.017505    0.059513  -0.0054971 ]';
    
    %image(step.D1);
    %title(['frame ' num2str(k)]);
    %drawnow;

    % do the actual calculations
    tic;
    %err = camera_warp(step, T, true);
    [err, J] = camera_warp(step, T, true);
    toc;

    disp(['total error: ' num2str(norm(err))]);
    %whos J;

    %print(fullfile(testscene.source_path, 'warp_anim', ['frame' num2str(k) '.png']), '-dpng');
    
    %disp(['step ' num2str(k) ' of ' num2str(testscene.step_count)]);
%end


