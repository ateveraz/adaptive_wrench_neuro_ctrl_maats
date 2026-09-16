function initBus_quadrotor_state() 
% INITBUS_QUADROTOR_STATE initializes a set of bus objects in the MATLAB base workspace 

% Bus object: quadrotor_state 
clear elems;
% elems(1) = Simulink.BusElement;
% elems(1).Name = 'linear_acceleration';
% elems(1).Dimensions = 3;
% elems(1).DimensionsMode = 'Fixed';
% elems(1).DataType = 'double';
% elems(1).Complexity = 'real';
% elems(1).Min = [];
% elems(1).Max = [];
% elems(1).DocUnits = '';
% elems(1).Description = '';

elems(1) = Simulink.BusElement;
elems(1).Name = 'linear_velocity';
elems(1).Dimensions = 3;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).Complexity = 'real';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'position';
elems(2).Dimensions = 3;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).Complexity = 'real';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'angular_velocity';
elems(3).Dimensions = 3;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).Complexity = 'real';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'quaternion';
elems(4).Dimensions = 4;
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'double';
elems(4).Complexity = 'real';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

quadrotor_state = Simulink.Bus;
quadrotor_state.HeaderFile = '';
quadrotor_state.Description = '';
quadrotor_state.DataScope = 'Auto';
quadrotor_state.Alignment = -1;
quadrotor_state.PreserveElementDimensions = 0;
quadrotor_state.Elements = elems;
clear elems;
assignin('base','quadrotor_state', quadrotor_state);

