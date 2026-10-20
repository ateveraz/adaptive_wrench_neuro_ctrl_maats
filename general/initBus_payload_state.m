function initBus_payload_state() 
% INITBUS_PAYLOAD_STATE initializes a set of bus objects in the MATLAB base workspace 

clear elems;

names = {'position','linear_velocity','angular_velocity','quaternion'};

dims = {3,3,3,4};


for i = 1:numel(names)
    elems(i) = Simulink.BusElement;
    elems(i).Name = names{i};
    elems(i).Dimensions = dims{i};
    elems(i).DimensionsMode = 'Fixed';
    elems(i).DataType = 'double';
    elems(i).Complexity = 'real';
end

payload_state = Simulink.Bus;
payload_state.Elements = elems;
clear elems;
assignin('base','payload_state', payload_state);