function [dampingCmd, ctrlState] = ctrl_vertical(varargin)
%CTRL_VERTICAL Hybrid skyhook/groundhook continuous damping controller.
%   Supports both the assignment interface
%       ctrl_vertical(suspState, ctrlState, CTRL, dt)
%   and the legacy standalone runner interface
%       ctrl_vertical(suspVel, bodyVel, ay, roll, CTRL).

    if nargin == 4 && isstruct(varargin{1})
        suspState = varargin{1};
        ctrlState = varargin{2};
        CTRL = varargin{3};
        if isfield(suspState, 'zs_dot')
            zsDot = suspState.zs_dot(:);
        else
            zsDot = zeros(4, 1);
        end
        if isfield(suspState, 'zu_dot')
            zuDot = suspState.zu_dot(:);
        else
            zuDot = zeros(4, 1);
        end
    elseif nargin >= 5
        suspVel = varargin{1};
        bodyVel = varargin{2};
        CTRL = varargin{5};
        ctrlState = struct();
        zsDot = bodyVel(:);
        zuDot = bodyVel(:) - suspVel(:);
    else
        CTRL = struct();
        ctrlState = struct();
        zsDot = zeros(4, 1);
        zuDot = zeros(4, 1);
    end

    if numel(zsDot) == 1; zsDot = zsDot * ones(4, 1); end
    if numel(zuDot) == 1; zuDot = zuDot * ones(4, 1); end
    zsDot = zsDot(1:4);
    zuDot = zuDot(1:4);

    cMin = local_get(CTRL.VER, 'cMin', 500);
    cMax = local_get(CTRL.VER, 'cMax', 5000);
    cNom = 0.45 * cMin + 0.55 * cMax;
    relVel = zsDot - zuDot;

    dampingCmd = cNom * ones(4, 1);
    for i = 1:4
        skyActive = zsDot(i) * relVel(i) > 0;
        groundActive = zuDot(i) * relVel(i) < 0;
        if skyActive && abs(zsDot(i)) > 0.015
            dampingCmd(i) = cMax;
        elseif groundActive && abs(zuDot(i)) > 0.04
            dampingCmd(i) = 0.65 * cMax + 0.35 * cNom;
        else
            dampingCmd(i) = cMin;
        end
    end

    dampingCmd = local_clamp_vec(dampingCmd, cMin, cMax);
end

function v = local_get(s, name, value)
    if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
        v = s.(name);
    else
        v = value;
    end
end

function y = local_clamp_vec(x, lo, hi)
    y = min(max(x, lo), hi);
end
