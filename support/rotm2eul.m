function eul = rotm2eul(R, seq)
%ROTM2EUL  Rotation matrix -> Euler angles [z y x] (ZYX), drop-in for the UAV/Robotics
%   Toolbox function used by the flight controller's thrustToAttitude block.
%   Only the 'ZYX' sequence (the default) is needed by this project. Code-generation safe.
%#codegen
if nargin > 1
    assert(strcmpi(seq, 'ZYX'), 'rotm2eul shim: only ZYX supported');
end
sy  = -R(3,1);
sy  = min(max(sy, -1), 1);
eul = zeros(1, 3, 'like', R);
eul(2) = asin(sy);                              % pitch (Y)
if abs(sy) < 1 - 1e-9
    eul(1) = atan2(R(2,1), R(1,1));             % yaw   (Z)
    eul(3) = atan2(R(3,2), R(3,3));             % roll  (X)
else                                            % gimbal lock: put all rotation in yaw
    eul(1) = atan2(-R(1,2), R(2,2));
    eul(3) = 0;
end
end
