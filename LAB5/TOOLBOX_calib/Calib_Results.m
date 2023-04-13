% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 3075.069061268479345 ; 3071.611219459076892 ];

%-- Principal point:
cc = [ 1994.040474860434415 ; 1510.873264201474058 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.076420642411524 ; -0.193538101071458 ; -0.000747397095208 ; -0.003736652635861 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 2.373633079483471 ; 2.604153337573966 ];

%-- Principal point uncertainty:
cc_error = [ 2.160897313925091 ; 1.580527653195410 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.002089955025708 ; 0.006215090908834 ; 0.000197535780694 ; 0.000260220634459 ; 0.000000000000000 ];

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
omc_1 = [ -2.020929e+00 ; -2.066198e+00 ; -5.802202e-01 ];
Tc_1  = [ -1.186371e+02 ; -5.247581e+01 ; 2.234609e+02 ];
omc_error_1 = [ 3.868035e-04 ; 5.677420e-04 ; 1.016554e-03 ];
Tc_error_1  = [ 1.599353e-01 ; 1.216412e-01 ; 1.876740e-01 ];

%-- Image #2:
omc_2 = [ 2.078653e+00 ; 2.122943e+00 ; 1.298760e-01 ];
Tc_2  = [ -1.184079e+02 ; -8.718140e+01 ; 3.123910e+02 ];
omc_error_2 = [ 5.269053e-04 ; 5.283737e-04 ; 1.042861e-03 ];
Tc_error_2  = [ 2.261853e-01 ; 1.651109e-01 ; 2.675681e-01 ];

%-- Image #3:
omc_3 = [ 1.961502e+00 ; 1.924492e+00 ; -2.340829e-01 ];
Tc_3  = [ -1.418724e+02 ; -1.341537e+02 ; 3.363711e+02 ];
omc_error_3 = [ 4.157313e-04 ; 6.417531e-04 ; 9.049724e-04 ];
Tc_error_3  = [ 2.432925e-01 ; 1.778891e-01 ; 2.621534e-01 ];

%-- Image #4:
omc_4 = [ 1.773171e+00 ; 2.120701e+00 ; -6.398679e-01 ];
Tc_4  = [ -1.283916e+02 ; -1.273168e+02 ; 3.952136e+02 ];
omc_error_4 = [ 3.242866e-04 ; 6.799339e-04 ; 8.796115e-04 ];
Tc_error_4  = [ 2.780848e-01 ; 2.065712e-01 ; 2.583507e-01 ];

%-- Image #5:
omc_5 = [ 2.058372e+00 ; 2.007945e+00 ; 5.293332e-01 ];
Tc_5  = [ -7.947174e+01 ; -8.264385e+01 ; 2.491042e+02 ];
omc_error_5 = [ 6.059592e-04 ; 4.519285e-04 ; 9.469666e-04 ];
Tc_error_5  = [ 1.808773e-01 ; 1.305709e-01 ; 2.226939e-01 ];

%-- Image #6:
omc_6 = [ -1.865720e+00 ; -1.906640e+00 ; -8.040754e-01 ];
Tc_6  = [ -1.325002e+02 ; -4.103535e+01 ; 2.536197e+02 ];
omc_error_6 = [ 4.135340e-04 ; 5.794775e-04 ; 9.463324e-04 ];
Tc_error_6  = [ 1.806244e-01 ; 1.377990e-01 ; 2.224381e-01 ];

