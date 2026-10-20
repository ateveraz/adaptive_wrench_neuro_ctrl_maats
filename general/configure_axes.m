function configure_axes(ax, xp, Xuav, padding)
%CONFIGURE_AXES Set fixed 3-D limits from payload and UAV trajectories.
% xp is Nt-by-3. Xuav is Nt-by-3-by-N.

    Xall = xp;
    N = size(Xuav, 3);

    for i = 1:N
        Xall = [Xall; squeeze(Xuav(:,:,i))]; %#ok<AGROW>
    end

    lo = min(Xall, [], 1);
    hi = max(Xall, [], 1);
    span = hi - lo;

    % Keep a useful axis extent for hover or planar trajectories.
    span(span < 1e-3) = 1.0;

    lo = lo - padding*span;
    hi = hi + padding*span;

    xlim(ax, [lo(1), hi(1)]);
    ylim(ax, [lo(2), hi(2)]);
    zlim(ax, [lo(3), hi(3)]);
end
