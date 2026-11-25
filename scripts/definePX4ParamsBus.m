% definePX4ParamsBus.m
% Defines PX4_Params_Bus and PX4_Params_Default for flightControlSystem.
% Called from model PreLoadFcn (after InitVars/setupParameters).
%
%   Copyright 2026 The MathWorks, Inc.

%% Bus Definition — 27 elements, all double (matches model signal types)
elems = Simulink.BusElement.empty(0,1);
idx = 0;

idx=idx+1; elems(idx)=Simulink.BusElement; elems(idx).Name='MPC_POS_P';           elems(idx).Dimensions=[3 1]; elems(idx).DataType='double';
idx=idx+1; elems(idx)=Simulink.BusElement; elems(idx).Name='MPC_VEL_P';           elems(idx).Dimensions=[3 1]; elems(idx).DataType='double';
idx=idx+1; elems(idx)=Simulink.BusElement; elems(idx).Name='MPC_VEL_I';           elems(idx).Dimensions=[3 1]; elems(idx).DataType='double';
idx=idx+1; elems(idx)=Simulink.BusElement; elems(idx).Name='MPC_VEL_D';           elems(idx).Dimensions=[3 1]; elems(idx).DataType='double';
idx=idx+1; elems(idx)=Simulink.BusElement; elems(idx).Name='MPC_THR_HOVER';       elems(idx).Dimensions=1;     elems(idx).DataType='double';
idx=idx+1; elems(idx)=Simulink.BusElement; elems(idx).Name='MPC_THR_MAX';         elems(idx).Dimensions=1;     elems(idx).DataType='double';
idx=idx+1; elems(idx)=Simulink.BusElement; elems(idx).Name='MPC_THR_MIN';         elems(idx).Dimensions=1;     elems(idx).DataType='double';
idx=idx+1; elems(idx)=Simulink.BusElement; elems(idx).Name='MPC_THR_XY_MARG';     elems(idx).Dimensions=1;     elems(idx).DataType='double';
idx=idx+1; elems(idx)=Simulink.BusElement; elems(idx).Name='MPC_TILTMAX_AIR';     elems(idx).Dimensions=1;     elems(idx).DataType='double';
idx=idx+1; elems(idx)=Simulink.BusElement; elems(idx).Name='MPC_XY_VEL_MAX';      elems(idx).Dimensions=1;     elems(idx).DataType='double';
idx=idx+1; elems(idx)=Simulink.BusElement; elems(idx).Name='MPC_Z_VEL_MAX_UP';    elems(idx).Dimensions=1;     elems(idx).DataType='double';
idx=idx+1; elems(idx)=Simulink.BusElement; elems(idx).Name='MPC_Z_VEL_MAX_DN';    elems(idx).Dimensions=1;     elems(idx).DataType='double';
idx=idx+1; elems(idx)=Simulink.BusElement; elems(idx).Name='MPC_ACC_DECOUPLE';    elems(idx).Dimensions=1;     elems(idx).DataType='double';
idx=idx+1; elems(idx)=Simulink.BusElement; elems(idx).Name='MC_ATT_P';            elems(idx).Dimensions=[3 1]; elems(idx).DataType='double';
idx=idx+1; elems(idx)=Simulink.BusElement; elems(idx).Name='MC_YAW_WEIGHT';       elems(idx).Dimensions=1;     elems(idx).DataType='double';
idx=idx+1; elems(idx)=Simulink.BusElement; elems(idx).Name='MC_RATE_P';           elems(idx).Dimensions=[3 1]; elems(idx).DataType='double';
idx=idx+1; elems(idx)=Simulink.BusElement; elems(idx).Name='MC_RATE_I';           elems(idx).Dimensions=[3 1]; elems(idx).DataType='double';
idx=idx+1; elems(idx)=Simulink.BusElement; elems(idx).Name='MC_RATE_D';           elems(idx).Dimensions=[3 1]; elems(idx).DataType='double';
idx=idx+1; elems(idx)=Simulink.BusElement; elems(idx).Name='MC_RATE_FF';          elems(idx).Dimensions=[3 1]; elems(idx).DataType='double';
idx=idx+1; elems(idx)=Simulink.BusElement; elems(idx).Name='MC_RATE_INT_LIM';     elems(idx).Dimensions=[3 1]; elems(idx).DataType='double';
idx=idx+1; elems(idx)=Simulink.BusElement; elems(idx).Name='MC_YAW_TQ_CUTOFF';    elems(idx).Dimensions=1;     elems(idx).DataType='double';
idx=idx+1; elems(idx)=Simulink.BusElement; elems(idx).Name='MPC_LAND_ALT1';       elems(idx).Dimensions=1;     elems(idx).DataType='double';
idx=idx+1; elems(idx)=Simulink.BusElement; elems(idx).Name='MPC_LAND_ALT2';       elems(idx).Dimensions=1;     elems(idx).DataType='double';
idx=idx+1; elems(idx)=Simulink.BusElement; elems(idx).Name='MPC_LAND_ALT3';       elems(idx).Dimensions=1;     elems(idx).DataType='double';
idx=idx+1; elems(idx)=Simulink.BusElement; elems(idx).Name='MPC_LAND_SPEED';      elems(idx).Dimensions=1;     elems(idx).DataType='double';
idx=idx+1; elems(idx)=Simulink.BusElement; elems(idx).Name='MPC_LAND_CRWL';       elems(idx).Dimensions=1;     elems(idx).DataType='double';
idx=idx+1; elems(idx)=Simulink.BusElement; elems(idx).Name='MPC_Z_VEL_MAX_DN_LAND'; elems(idx).Dimensions=1;   elems(idx).DataType='double';
idx=idx+1; elems(idx)=Simulink.BusElement; elems(idx).Name='MPC_VELD_LP';          elems(idx).Dimensions=1;   elems(idx).DataType='double';
idx=idx+1; elems(idx)=Simulink.BusElement; elems(idx).Name='EST_NF_FRQ';           elems(idx).Dimensions=1;   elems(idx).DataType='double';
idx=idx+1; elems(idx)=Simulink.BusElement; elems(idx).Name='EST_NF_BW';            elems(idx).Dimensions=1;   elems(idx).DataType='double';

PX4_Params_Bus = Simulink.Bus;
PX4_Params_Bus.Elements = elems;

%% Default Values (sourced from setupParameters.m workspace variables)
PX4_Params_Default.MPC_POS_P           = [MPC_XY_P; MPC_XY_P; MPC_Z_P];
PX4_Params_Default.MPC_VEL_P           = [MPC_XY_VEL_P_ACC; MPC_XY_VEL_P_ACC; MPC_Z_VEL_P_ACC];
PX4_Params_Default.MPC_VEL_I           = [MPC_XY_VEL_I_ACC; MPC_XY_VEL_I_ACC; MPC_Z_VEL_I_ACC];
PX4_Params_Default.MPC_VEL_D           = [MPC_XY_VEL_D_ACC; MPC_XY_VEL_D_ACC; MPC_Z_VEL_D_ACC];
PX4_Params_Default.MPC_THR_HOVER       = MPC_THR_HOVER;
PX4_Params_Default.MPC_THR_MAX         = MPC_THR_MAX;
PX4_Params_Default.MPC_THR_MIN         = MPC_THR_MIN;
PX4_Params_Default.MPC_THR_XY_MARG     = MPC_THR_XY_MARG;
PX4_Params_Default.MPC_TILTMAX_AIR     = MPC_TILTMAX_AIR;
PX4_Params_Default.MPC_XY_VEL_MAX      = MPC_XY_VEL_MAX;
PX4_Params_Default.MPC_Z_VEL_MAX_UP    = MPC_Z_VEL_MAX_UP;
PX4_Params_Default.MPC_Z_VEL_MAX_DN    = MPC_Z_VEL_MAX_DN;
PX4_Params_Default.MPC_ACC_DECOUPLE    = MPC_ACC_DECOUPLE;
PX4_Params_Default.MC_ATT_P            = [MC_ROLL_P; MC_PITCH_P; MC_YAW_P];
PX4_Params_Default.MC_YAW_WEIGHT       = MC_YAW_WEIGHT;
PX4_Params_Default.MC_RATE_P           = [MC_ROLLRATE_P; MC_PITCHRATE_P; MC_YAWRATE_P];
PX4_Params_Default.MC_RATE_I           = [MC_ROLLRATE_I; MC_PITCHRATE_I; MC_YAWRATE_I];
PX4_Params_Default.MC_RATE_D           = [MC_ROLLRATE_D; MC_PITCHRATE_D; MC_YAWRATE_D];
PX4_Params_Default.MC_RATE_FF          = [MC_ROLLRATE_FF; MC_PITCHRATE_FF; MC_YAWRATE_FF];
PX4_Params_Default.MC_RATE_INT_LIM     = [MC_RR_INT_LIM; MC_PR_INT_LIM; MC_YR_INT_LIM];
PX4_Params_Default.MC_YAW_TQ_CUTOFF    = MC_YAW_TQ_CUTOFF;
PX4_Params_Default.MPC_LAND_ALT1       = MPC_LAND_ALT1;
PX4_Params_Default.MPC_LAND_ALT2       = MPC_LAND_ALT2;
PX4_Params_Default.MPC_LAND_ALT3       = MPC_LAND_ALT3;
PX4_Params_Default.MPC_LAND_SPEED      = MPC_LAND_SPEED;
PX4_Params_Default.MPC_LAND_CRWL       = MPC_LAND_CRWL;
PX4_Params_Default.MPC_Z_VEL_MAX_DN_LAND = MPC_Z_VEL_MAX_DN;
PX4_Params_Default.MPC_VELD_LP        = MPC_VELD_LP;
PX4_Params_Default.EST_NF_FRQ         = 30;
PX4_Params_Default.EST_NF_BW          = 20;

%% Cleanup temporary variables to avoid polluting base workspace
% (InitVars.m reuses 'elems' without clearing — stale entries cause UAVStatesBus corruption)
clear elems idx;
