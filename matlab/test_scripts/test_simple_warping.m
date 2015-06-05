% basic test for new core functions
% loads an image pair and warps the keyframe onto the current frame

%testscene = Scene('input/testscene1');
%testscene = Scene('input/testscene2');
testscene = Scene('input/trajectory1');
%testscene = Scene('input/testscene_rotonly');

%testscene = testscene.scale_down(3);

for k = 1:testscene.step_count
    
    step = testscene.getStep(k);

    T = step.ground_truth;
    
    %image(step.D1);
    %title(['frame ' num2str(k)]);
    %drawnow;

    % do the actual calculations
    err = camera_warp(step, T, true);
    %[err, J] 

    disp(['total error: ' num2str(norm(err))]);
    %whos J;

    print(fullfile(testscene.source_path, 'warp_anim', ['frame' num2str(k) '.png']), '-dpng');
    
    disp(['step ' num2str(k) ' of ' num2str(testscene.step_count)]);
end