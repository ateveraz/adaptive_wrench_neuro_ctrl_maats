function visualizer(simData, varargin)
%VISUALIZE_MAATS Simple scalable 3-D MAATS animation from Simulink logs.
%
% Usage examples:
%   visualize_maats(out.logsout, 'Rattach', payload.Rattach);
%   visualize_maats(out, 'Rattach', payload.Rattach, ...
%                   'ShowCables', false, 'FrameStride', 10);
%
% Expected logged signals:
%   payload : bus/struct with position (Nx3) and quaternion (Nx4)
%   uav1, uav2, ..., uavN : buses/structs with position (Nx3)
%
% Frame convention: +z points downward. The function reverses the displayed
% z-axis so that larger z values visually appear lower.

    p = inputParser;
    addParameter(p, 'Rattach', []);
    addParameter(p, 'ShowCables', true, @(x)islogical(x) || isnumeric(x));
    addParameter(p, 'ShowTrajectories', false, @(x)islogical(x) || isnumeric(x));
    addParameter(p, 'FrameStride', 10, @(x)isnumeric(x) && isscalar(x) && x >= 1);
    addParameter(p, 'PlaybackRate', 1.0, @(x)isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'DroneSize', 70, @(x)isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'PayloadLineWidth', 5, @(x)isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'AxisPadding', 0.25, @(x)isnumeric(x) && isscalar(x) && x >= 0);
    addParameter(p, 'FigureHandle', []);
    parse(p, varargin{:});
    opt = p.Results;

    logs = resolve_logs(simData);
    names = logs.getElementNames;

    payloadIndex = find(strcmpi(names, 'payload'), 1);
    assert(~isempty(payloadIndex), ...
        'A logged signal named "payload" was not found.');

    uavIndices = find(startsWith(lower(names), 'uav'));
    assert(~isempty(uavIndices), ...
        'No logged UAV signals were found. Expected names such as uav1, uav2, ...');

    [~, order] = sort(lower(names(uavIndices)));
    uavIndices = uavIndices(order);
    N = numel(uavIndices);

    payloadValues = logs{payloadIndex}.Values;
    [t, xp] = read_timeseries_field(payloadValues, 'position');
    [~, qp] = read_timeseries_field(payloadValues, 'quaternion');

    xp = as_rows(xp, 3);
    qp = as_rows(qp, 4);
    nt = numel(t);

    Xuav = zeros(nt, 3, N);
    for i = 1:N
        uavValues = logs{uavIndices(i)}.Values;
        [tu, xi] = read_timeseries_field(uavValues, 'position');
        xi = as_rows(xi, 3);
        if numel(tu) ~= nt || any(abs(tu(:)-t(:)) > 1e-10)
            xi = interp1(tu(:), xi, t(:), 'linear', 'extrap');
        end
        Xuav(:,:,i) = xi;
    end

    Rattach = opt.Rattach;
    if isempty(Rattach)
        Rattach = default_attachments(N);
    end
    assert(size(Rattach,1) == 3, 'Rattach must have size 3xM.');

    if isempty(opt.FigureHandle) || ~isvalid(opt.FigureHandle)
        fig = figure('Name', 'MAATS visualization', 'Color', 'w');
    else
        fig = figure(opt.FigureHandle);
        clf(fig);
    end
    ax = axes(fig);
    hold(ax, 'on');
    grid(ax, 'on');
    axis(ax, 'equal');
    view(ax, 35, 24);
    xlabel(ax, '$x_I$ [m]', 'Interpreter', 'latex');
    ylabel(ax, '$y_I$ [m]', 'Interpreter', 'latex');
    zlabel(ax, '$z_I$ [m]', 'Interpreter', 'latex');
    set(ax, 'ZDir', 'reverse');
    title(ax, 'Multi-agent aerial transportation system');

    configure_axes(ax, xp, Xuav, opt.AxisPadding);
    colors = lines(N);

    if opt.ShowTrajectories
        plot3(ax, xp(:,1), xp(:,2), xp(:,3), 'k:', 'LineWidth', 1.2, ...
            'DisplayName', 'payload path');
        for i = 1:N
            plot3(ax, Xuav(:,1,i), Xuav(:,2,i), Xuav(:,3,i), ':', ...
                'Color', colors(i,:), 'LineWidth', 1.0, ...
                'HandleVisibility', 'off');
        end
    end

    payloadHandle = plot3(ax, nan, nan, nan, 'k-', ...
        'LineWidth', opt.PayloadLineWidth, 'DisplayName', 'payload');
    payloadMarker = plot3(ax, nan, nan, nan, 'ks', ...
        'MarkerFaceColor', 'r', 'MarkerSize', 10, 'HandleVisibility', 'off');

    uavHandle = gobjects(N,1);
    cableHandle = gobjects(N,1);
    for i = 1:N
        uavHandle(i) = plot3(ax, nan, nan, nan, 'o', ...
            'MarkerSize', sqrt(opt.DroneSize)/2, ...
            'MarkerFaceColor', colors(i,:), ...
            'MarkerEdgeColor', colors(i,:), ...
            'DisplayName', sprintf('UAV %d', i));
        cableHandle(i) = plot3(ax, nan, nan, nan, '--', ...
            'Color', colors(i,:), 'LineWidth', 1.2, ...
            'HandleVisibility', 'off');
    end
    timeHandle = text(ax, 0, 0, 0, '', 'FontWeight', 'bold', ...
        'BackgroundColor', 'w', 'Margin', 4);
    legend(ax, 'Location', 'bestoutside');

    indices = 1:round(opt.FrameStride):nt;
    if indices(end) ~= nt
        indices(end+1) = nt;
    end

    for k = 1:numel(indices)
        j = indices(k);
        R = quat_to_rot(qp(j,:).');
        attachmentI = xp(j,:).' + R*Rattach;

        if size(Rattach,2) == 1
            set(payloadHandle, 'XData', xp(j,1), 'YData', xp(j,2), 'ZData', xp(j,3));
        else
            payloadPolyline = [attachmentI, attachmentI(:,1)];
            set(payloadHandle, 'XData', payloadPolyline(1,:), ...
                'YData', payloadPolyline(2,:), ...
                'ZData', payloadPolyline(3,:));
        end
        set(payloadMarker, 'XData', xp(j,1), 'YData', xp(j,2), 'ZData', xp(j,3));

        for i = 1:N
            xi = squeeze(Xuav(j,:,i)).';
            set(uavHandle(i), 'XData', xi(1), 'YData', xi(2), 'ZData', xi(3));

            if opt.ShowCables
                ai = attachmentI(:, min(i, size(attachmentI,2)));
                set(cableHandle(i), 'Visible', 'on', ...
                    'XData', [xi(1), ai(1)], ...
                    'YData', [xi(2), ai(2)], ...
                    'ZData', [xi(3), ai(3)]);
            else
                set(cableHandle(i), 'Visible', 'off');
            end
        end

        xl = xlim(ax); yl = ylim(ax); zl = zlim(ax);
        set(timeHandle, 'Position', [xl(1), yl(1), zl(1)], ...
            'String', sprintf('t = %.2f s', t(j)));
        drawnow limitrate;

        if k < numel(indices)
            dt = t(indices(k+1)) - t(j);
            pause(max(0, dt/opt.PlaybackRate));
        end
    end
end

function logs = resolve_logs(simData)
    if isa(simData, 'Simulink.SimulationOutput')
        if simData.has('logsout')
            logs = simData.logsout;
        else
            error('SimulationOutput does not contain logsout.');
        end
    elseif isa(simData, 'Simulink.SimulationData.Dataset')
        logs = simData;
    else
        error('Input must be a SimulationOutput object or a Dataset such as out.logsout.');
    end
end

function [t, data] = read_timeseries_field(values, fieldName)
    assert(isfield(values, fieldName), ...
        'Logged signal does not contain field "%s".', fieldName);
    ts = values.(fieldName);
    t = ts.Time(:);
    data = ts.Data;
end

function X = as_rows(X, ncol)
    X = squeeze(X);
    if size(X,2) == ncol
        return
    elseif size(X,1) == ncol
        X = X.';
    else
        error('Expected a signal with %d components.', ncol);
    end
end

function Rattach = default_attachments(N)
    if N == 1
        Rattach = zeros(3,1);
    else
        x = linspace(-0.5, 0.5, N);
        Rattach = [x; zeros(2,N)];
    end
end

function R = quat_to_rot(q)
    q = q(:)/max(norm(q), 1e-12);
    q0 = q(1);
    qv = q(2:4);
    S = [0 -qv(3) qv(2); qv(3) 0 -qv(1); -qv(2) qv(1) 0];
    R = (q0^2-qv.'*qv)*eye(3) + 2*(qv*qv.') + 2*q0*S;
end
