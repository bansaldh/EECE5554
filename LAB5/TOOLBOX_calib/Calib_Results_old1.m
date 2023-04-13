% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 3085.248551933716499 ; 3082.436998887652862 ];

%-- Principal point:
cc = [ 2002.804934045813980 ; 1511.813150540330753 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.066695678851387 ; -0.169779986401853 ; -0.001040336972764 ; -0.002792560626658 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 14.119185715323896 ; 15.488963797485102 ];

%-- Principal point uncertainty:
cc_error = [ 13.030310348519388 ; 9.494465949800972 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.012228804521283 ; 0.036170817536403 ; 0.001169322500099 ; 0.001556279352438 ; 0.000000000000000 ];

%-- Image size:
nx = 4032;
ny = 3024;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 6;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ -2.020889e+00 ; -2.068283e+00 ; -5.848421e-01 ];
Tc_1  = [ -1.192043e+02 ; -5.255776e+01 ; 2.236042e+02 ];
omc_error_1 = [ 2.301362e-03 ; 3.387217e-03 ; 6.085713e-03 ];
Tc_error_1  = [ 9.625213e-01 ; 7.300815e-01 ; 1.113558e+00 ];

%-- Image #2:
omc_2 = [ 2.077098e+00 ; 2.121123e+00 ; 1.307704e-01 ];
Tc_2  = [ -1.193423e+02 ; -8.726884e+01 ; 3.131052e+02 ];
omc_error_2 = [ 3.141934e-03 ; 3.160637e-03 ; 6.218948e-03 ];
Tc_error_2  = [ 1.363227e+00 ; 9.916067e-01 ; 1.588952e+00 ];

%-- Image #3:
omc_3 = [ 1.960496e+00 ; 1.922009e+00 ; -2.314128e-01 ];
Tc_3  = [ -1.428720e+02 ; -1.341754e+02 ; 3.368227e+02 ];
omc_error_3 = [ 2.478746e-03 ; 3.843507e-03 ; 5.399943e-03 ];
Tc_error_3  = [ 1.465253e+00 ; 1.067629e+00 ; 1.556690e+00 ];

%-- Image #4:
omc_4 = [ 1.772505e+00 ; 2.118228e+00 ; -6.392353e-01 ];
Tc_4  = [ -1.294780e+02 ; -1.272600e+02 ; 3.958128e+02 ];
omc_error_4 = [ 1.928093e-03 ; 4.077272e-03 ; 5.265891e-03 ];
Tc_error_4  = [ 1.674913e+00 ; 1.240728e+00 ; 1.533293e+00 ];

%-- Image #5:
omc_5 = [ 2.056727e+00 ; 2.006286e+00 ; 5.321741e-01 ];
Tc_5  = [ -8.021956e+01 ; -8.269592e+01 ; 2.496267e+02 ];
omc_error_5 = [ 3.622614e-03 ; 2.700569e-03 ; 5.650953e-03 ];
Tc_error_5  = [ 1.089928e+00 ; 7.845820e-01 ; 1.322243e+00 ];

%-- Image #6:
omc_6 = [ -1.865391e+00 ; -1.908259e+00 ; -8.066322e-01 ];
Tc_6  = [ -1.331791e+02 ; -4.117477e+01 ; 2.542026e+02 ];
omc_error_6 = [ 2.470862e-03 ; 3.471059e-03 ; 5.678154e-03 ];
Tc_error_6  = [ 1.088239e+00 ; 8.280198e-01 ; 1.320850e+00 ];

