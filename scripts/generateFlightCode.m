function generateFlightCode()
% GENERATEFLIGHTCODE generates code for deployment in the PX4
% drone. It requires Embedded Coder (R) and Simulink Coder (TM).

% Copyright 2026 The MathWorks, Inc.
assignin('base', 'Simulation', false);

model = 'flightControlSystem';
open_system(model);
%Check for capability of code generation
set_param(model, 'GenCodeOnly','on');
%Disable the report generation
set_param(model, 'GenerateReport', 'Off');
rtwbuild(model);
%Reset to off and enable direct deployment on hardware
set_param(model, 'GenCodeOnly','off');
