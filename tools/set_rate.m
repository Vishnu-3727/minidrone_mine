function set_rate(dt)
%SET_RATE  Camera + guidance period of the models, s (0.1 = 10 Hz, 0.2 = 5 Hz).
%   Must match FRAME_DT in perception/trackGuidance.m. Changes:
%     top model   : Simulation 3D Camera Get sample time (sim camera)
%     perception  : Rate Transition into the guidance block, Raspberry Pi camera capture (deploy)
%   The flight controller already reads the setpoint at 10 Hz (Rate Transition 1/10).
if nargin < 1, dt = 0.1; end
v = num2str(dt);
top = 'PX4MultiCopterPerceptionAndControl'; pg = 'PerceptionAndGuidance';
load_system(top); load_system(pg);
fo = {'LookUnderMasks', 'all', 'FollowLinks', 'on', 'MatchFilter', @Simulink.match.allVariants};
b = find_system(top, fo{:}, 'Name', 'Simulation 3D Camera Get');
for i = 1:numel(b), set_param(b{i}, 'SampleTime', v); fprintf('%s  SampleTime = %s\n', b{i}, v); end
b = find_system(pg, fo{:}, 'BlockType', 'RateTransition');
for i = 1:numel(b)
    if any(strcmp(get_param(b{i}, 'OutPortSampleTime'), {'0.2', '0.1', '1/5', '1/10'}))
        set_param(b{i}, 'OutPortSampleTime', v); fprintf('%s  OutPortSampleTime = %s\n', b{i}, v);
    end
end
b = find_system(pg, fo{:}, 'Name', 'Raspberry Pi Camera Video Capture');
for i = 1:numel(b)
    try, set_param(b{i}, 'SampleTime', v); fprintf('%s  SampleTime = %s\n', b{i}, v);
    catch e, fprintf('skip %s: %s\n', b{i}, e.message); end
end
save_system(pg); save_system(top);
end
