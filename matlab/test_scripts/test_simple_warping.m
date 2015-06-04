% basic test for new core functions
% loads an image pair and warps the keyframe onto the current frame

%testscene = Scene('input/testscene1');
testscene = Scene('input/testscene2_rotonly');

T = testscene.ground_truth;


%testscene = testscene.scale_down(3);


% do the actual calculations
err = camera_warp(testscene,T,true);
%[err, J] 

disp(['total error: ' num2str(norm(err))]);
%whos J;