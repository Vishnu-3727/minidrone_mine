function set_drone_ip(ip, username, password)
%SET_DRONE_IP  Point the whole project at the Raspberry Pi on the drone.
%   set_drone_ip('192.168.0.204')               % Drone B on the TP-Link router
%   set_drone_ip('192.168.137.57')              % example: Pi on the laptop hotspot
%   set_drone_ip(ip, 'rpi', 'rpib')             % also change the login
%
%   Updates:
%     1. PerceptionAndGuidance model: Configuration Parameters >
%        Hardware board settings > Device address / Username / Password / Build dir
%     2. MATLAB's saved Raspberry Pi login (used by Step_1_Deploy_Controller
%        to copy the Pixhawk firmware through the Pi)
%   Then saves the model and pings the Pi.

if nargin < 2 || isempty(username), username = 'rpi'; end
if nargin < 3 || isempty(password), password = 'rpib'; end
ip = char(ip); username = char(username); password = char(password);

model = 'PerceptionAndGuidance';
load_system(model);
cs = getActiveConfigSet(model);
codertarget.data.setParameterValue(cs, 'BoardParameters.DeviceAddress', ip);
codertarget.data.setParameterValue(cs, 'BoardParameters.Username', username);
codertarget.data.setParameterValue(cs, 'BoardParameters.Password', password);
codertarget.data.setParameterValue(cs, 'BoardParameters.BuildDir', ['/home/' username]);
codertarget.data.setParameterValue(cs, 'BoardParameters.OnlineBuildDir', ['/home/' username]);
save_system(model);

for board = {'RaspberryPi', 'RaspberryPi64bit'}
    try
        hb = rpiutils.linux.remotebuild.BoardParameters(board{1});
        hb.setParam('hostname', ip);
        hb.setParam('username', username);
        hb.setParam('password', password);
    catch
    end
end

fprintf('Drone target set to %s@%s (model saved).\n', username, ip);
[status, out] = system(['ping -n 2 -w 1500 ' ip]);
if status == 0 && contains(out, 'TTL=')
    fprintf('Ping OK: the Pi answers.\n');
else
    fprintf(2, 'Ping FAILED: laptop cannot reach %s. Check Wi-Fi network and the Pi power.\n', ip);
end
end
