function demo1
%DEMO1  Demo for usage of DIFFERENTIALEVOLUTION.
%   DEMO1 starts searching the minimum of Rosenbrock's saddle as a demo.
%   Modify this function for your first optimization.
%
%   <a href="differentialevolution.html">differentialevolution.html</a>
%   <a href="https://www.mathworks.com/matlabcentral/fileexchange/18593-differential-evolution">File Exchange</a>
%   <a href="https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=KAECWD2H7EJFN">Donate via PayPal</a>
%
%   Markus Buehren
%   Last modified 31.08.2014
%
%   See also DIFFERENTIALEVOLUTION, ROSENBROCKSADDLE.

% Set title
optimInfo.title = 'Demo 1 (Rosenbrock''s saddle)';

% Specify objective function
objFctHandle = @rosenbrocksaddle;

% Define parameter names, ranges and quantization:

% 1. column: parameter names
% 2. column: parameter ranges
% 3. column: parameter quantizations
% 4. column: initial values (optional)

paramDefCell = {
	'parameter1', [-3 3], 0.01
	'parameter2', [-3 3], 0.01
};

% Set initial parameter values in struct objFctParams 
objFctParams.parameter1 =  -2;
objFctParams.parameter2 = 2.5;

% Set single additional function parameter
objFctSettings = 100;

% Get default DE parameters
DEParams = getdefaultparams;

% Set number of population members (often 10*D is suggested) 
DEParams.NP = 20;

% Do not use slave processes here. If you want to, set feedSlaveProc to 1 and
% run startmulticoreslave.m in at least one additional Matlab session.
DEParams.feedSlaveProc = 0;

% Set times
DEParams.maxiter  = 20;
DEParams.maxtime  = 30; % in seconds
DEParams.maxclock = [];

% Set display options
DEParams.infoIterations = 1;
DEParams.infoPeriod     = 10; % in seconds

% Do not send E-mails
emailParams = [];

% Set random state in order to always use the same population members here
setrandomseed(1);

% Start differential evolution
[bestmem, bestval, bestFctParams, nrOfIterations, resultFileName] = differentialevolution(...
	DEParams, paramDefCell, objFctHandle, objFctSettings, objFctParams, emailParams, optimInfo); %#ok

disp(' ');
disp('Best parameter set returned by function differentialevolution:');
disp(bestFctParams);

% Continue optimization by loading result file
if DEParams.saveHistory
  
  disp(' ');
  disp(textwrap2(sprintf(...
    'Now continuing optimization by loading result file %s.', resultFileName)));
  disp(' ');
  
  DEParams.maxiter = 100;
  DEParams.maxtime = 60; % in seconds

  [bestmem, bestval, bestFctParams] = differentialevolution(...
    DEParams, paramDefCell, objFctHandle, objFctSettings, objFctParams, emailParams, optimInfo, ...
    resultFileName); %#ok
  
  disp(' ');
  disp('Best parameter set returned by function differentialevolution:');
  disp(bestFctParams);
end

