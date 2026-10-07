function initBus_payload_reference()
% INITBUS_PAYLOAD_REFERENCE Desired-payload trajectory bus.
% This bus is intentionally distinct from payload_state. It contains no
% wrench, tension, or acceleration produced by the physical plant.

clear elems;

names = {'position','linear_velocity','linear_acceleration', ...
         'quaternion','angular_velocity'};
dims = {3,3,3,4,3};

for i = 1:numel(names)
    elems(i) = Simulink.BusElement;
    elems(i).Name = names{i};
    elems(i).Dimensions = dims{i};
    elems(i).DimensionsMode = 'Fixed';
    elems(i).DataType = 'double';
    elems(i).Complexity = 'real';
end

payload_reference = Simulink.Bus;
payload_reference.Elements = elems;
assignin('base','payload_reference',payload_reference);
end
