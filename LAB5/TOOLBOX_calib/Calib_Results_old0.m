% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 3059.237254332925204 ; 3038.604365071535540 ];

%-- Principal point:
cc = [ 1990.715330430368795 ; 1515.234320820607763 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.038742947138378 ; -0.121071825924009 ; -0.002098366446213 ; -0.002076113108875 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 23.597081315170520 ; 25.661026303328303 ];

%-- Principal point uncertainty:
cc_error = [ 21.754051255361158 ; 15.771808670525564 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.020150093623530 ; 0.058342646137203 ; 0.001905834144004 ; 0.002513711619750 ; 0.000000000000000 ];

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
omc_1 = [ -2.029140e+00 ; -2.075083e+00 ; -5.759545e-01 ];
Tc_1  = [ -1.182269e+02 ; -5.313054e+01 ; 2.209536e+02 ];
omc_error_1 = [ 3.853573e-03 ; 5.688125e-03 ; 1.031259e-02 ];
Tc_error_1  = [ 1.603002e+00 ; 1.218630e+00 ; 1.851758e+00 ];

%-- Image #2:
omc_2 = [ 2.075462e+00 ; 2.124133e+00 ; 1.321246e-01 ];
Tc_2  = [ -1.178543e+02 ; -8.790054e+01 ; 3.096588e+02 ];
omc_error_2 = [ 5.287563e-03 ; 5.310252e-03 ; 1.048187e-02 ];
Tc_error_2  = [ 2.271841e+00 ; 1.654643e+00 ; 2.641147e+00 ];

%-- Image #3:
omc_3 = [ 1.963937e+00 ; 1.928588e+00 ; -2.262462e-01 ];
Tc_3  = [ -1.415302e+02 ; -1.350430e+02 ; 3.325268e+02 ];
omc_error_3 = [ 4.191851e-03 ; 6.474168e-03 ; 9.093255e-03 ];
Tc_error_3  = [ 2.440367e+00 ; 1.778936e+00 ; 2.592344e+00 ];

%-- Image #4:
omc_4 = [ 1.770639e+00 ; 2.118081e+00 ; -6.505280e-01 ];
Tc_4  = [ -1.279080e+02 ; -1.269016e+02 ; 3.944171e+02 ];
omc_error_4 = [ 3.255817e-03 ; 6.890144e-03 ; 8.837654e-03 ];
Tc_error_4  = [ 2.811398e+00 ; 2.087328e+00 ; 2.553924e+00 ];

%-- Image #5:
omc_5 = [ 2.055784e+00 ; 2.004342e+00 ; 5.300765e-01 ];
Tc_5  = [ -7.907160e+01 ; -8.300425e+01 ; 2.453416e+02 ];
omc_error_5 = [ 6.092670e-03 ; 4.533407e-03 ; 9.533711e-03 ];
Tc_error_5  = [ 1.804469e+00 ; 1.301599e+00 ; 2.185330e+00 ];

%-- Image #6:
omc_6 = [ -1.869211e+00 ; -1.908936e+00 ; -8.007887e-01 ];
Tc_6  = [ -1.323894e+02 ; -4.148764e+01 ; 2.504101e+02 ];
omc_error_6 = [ 4.151150e-03 ; 5.854365e-03 ; 9.602143e-03 ];
Tc_error_6  = [ 1.805300e+00 ; 1.377840e+00 ; 2.187523e+00 ];

