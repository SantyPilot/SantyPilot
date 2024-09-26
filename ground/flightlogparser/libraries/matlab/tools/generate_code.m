clear, clc, close all

% Folder pointing towards the matlab function
% opt.matlab_code_folder = 'matlab_code/example_for_readme';
opt.matlab_code_folder = '../ukf';

% The function name to generate to c
% opt.matlab_function = 'Addition';
% opt.matlab_function = 'ukf_predict2';
opt.matlab_function = 'ukf_update2';

% Here you specify the shape of your inputs
% coder.typeof( int/bool/dobule, [dim_x dim_y], [dynamic_x dynamic_y])
% the dynamic_x, dynamic_y 0/1 depending on the input dimension can change
% For more info read up on the Matlab Codegen docs
opt.arguments = {coder.typeof(double(1),[13 1], [0 0]),...
                 coder.typeof(double(1),[13 13], [0 0]),...
                 coder.typeof(double(1),[10 1], [0 0]),...
                 coder.typeof(double(1),[10 10], [0 0]),...
                 coder.typeof(double(1),[1 3], [0 0])...
                 coder.typeof(double(1),[1 1], [0 0])...
                 coder.typeof(double(1),[1 1], [0 0])...
                 coder.typeof(double(1),[1 1], [0 0])...
                 coder.typeof(double(1),[1 1], [0 0])};

             % Xpre, Ppre, raIn(:,i), R, hparam, alpha, beta, kappa, mat
% Settings for the Matlab coder
cfg = coder.config('lib');
cfg.TargetLang = 'C++';
cfg.Toolchain = 'MinGW64 v4.x | gmake (64-bit Windows)';

% Remove unecessary extra stuff that will help increase readability in the
% code
cfg.SaturateOnIntegerOverflow = false;
cfg.SupportNonFinite = false; % false
cfg.FilePartitionMethod = 'MapMFileToCFile'; % Easier with 'SingleFile'


% Start the code generation
addpath(genpath(opt.matlab_code_folder));

codegen('-config',cfg,opt.matlab_function,'-args',opt.arguments,'-v','-d',['./' opt.matlab_function '/'])