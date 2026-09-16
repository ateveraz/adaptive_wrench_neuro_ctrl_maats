function initBus_payload_state() 
% INITBUS_PAYLOAD_STATE initializes a set of bus objects in the MATLAB base workspace 

clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'wrench_ctrl';
elems(1).Dimensions = 6;
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
elems(3).Name = 'linear_velocity';
elems(3).Dimensions = 3;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).Complexity = 'real';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'angular_velocity';
elems(4).Dimensions = 3;
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'double';
elems(4).Complexity = 'real';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

elems(5) = Simulink.BusElement;
elems(5).Name = 'quaternion';
elems(5).Dimensions = 4;
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'double';
elems(5).Complexity = 'real';
elems(5).Min = [];
elems(5).Max = [];
elems(5).DocUnits = '';
elems(5).Description = '';

payload_state = Simulink.Bus;
payload_state.HeaderFile = '';
payload_state.Description = '';
payload_state.DataScope = 'Auto';
payload_state.Alignment = -1;
payload_state.PreserveElementDimensions = 0;
payload_state.Elements = elems;
clear elems;
assignin('base','payload_state', payload_state);

