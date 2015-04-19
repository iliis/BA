%% global (constant) parameters: 'magic' constants used troughout the project


%% image properties
% depth images are 16bit grayscale, color images 8bit RGB

DEPTH_IMAGE_MAXVAL = 65535;
COLOR_IMAGE_MAXVAL = 255;

% TODO: find a clean way to implement this variable at run-time for the image pyramid
IMAGE_WIDTH  = 256; W = IMAGE_WIDTH;
IMAGE_HEIGHT = 128; H = IMAGE_HEIGHT;


%% properties of camera
% in Blender Units / meters

CAMERA_FOCAL = 0.035; % 35mm
CAMERA_WIDTH = 0.032; % sensor size = 32mm

CAMERA_CLIPPING = [0.1 100];



%% error term parameters

% empiric penalty for empty pixels
NONMATCHED_PIXEL_PENALTY = 0.03;