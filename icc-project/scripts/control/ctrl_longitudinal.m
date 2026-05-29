function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
%CTRL_LONGITUDINAL PI speed controller with jerk-limited brake request.
%   The benchmark scenarios inject their own brake commands, but this module
%   is complete for standalone ICC operation and follows the assignment
%   interface: speed tracking, anti-windup, and a conservative ABS surrogate.

    if nargin < 8 || isempty(dt); dt = 0.001; end
    dt = max(dt, 1e-4);
    if ~isstruct(ctrlState); ctrlState = struct(); end
    ctrlState = local_default(ctrlState, 'intError', 0);
    ctrlState = local_default(ctrlState, 'prevForce', 0);

    vx = max(vx, 0);
    err = vxRef - vx;

    Kp = local_get(CTRL.LON, 'Kp', 0.5) * 1500;
    Ki = local_get(CTRL.LON, 'Ki', 0.05) * 1500;
    intMax = local_get(CTRL.LON, 'intMax', 2000) / max(Ki, 1);

    forceNoI = Kp * err;
    forceTrial = forceNoI + Ki * ctrlState.intError;
    maxForce = local_get(LIM, 'MAX_AX', 10.0) * 1500;

    if abs(forceTrial) < maxForce || sign(err) ~= sign(forceTrial)
        ctrlState.intError = ctrlState.intError + err * dt;
        ctrlState.intError = local_clamp(ctrlState.intError, -intMax, intMax);
    end

    Fx = forceNoI + Ki * ctrlState.intError;
    Fx = local_clamp(Fx, -maxForce, maxForce);

    % ABS surrogate when wheel-slip data are unavailable: if deceleration is
    % already near the tire limit, release part of additional braking.
    if Fx < 0 && ax < -0.85 * local_get(LIM, 'MAX_AX', 10.0)
        Fx = 0.65 * Fx;
    end

    maxJerk = local_get(LIM, 'MAX_JERK', 50.0);
    maxForceRate = maxJerk * 1500;
    Fx = ctrlState.prevForce + local_clamp(Fx - ctrlState.prevForce, ...
                                           -maxForceRate * dt, maxForceRate * dt);

    forceCmd.Fx_total = Fx;
    forceCmd.brakeRatio = local_clamp(-Fx / max(maxForce, 1), 0, 1);
    ctrlState.prevForce = Fx;
end

function s = local_default(s, name, value)
    needsDefault = ~isfield(s, name) || isempty(s.(name));
    if ~needsDefault && isnumeric(s.(name)) && isscalar(s.(name))
        needsDefault = ~isfinite(s.(name));
    end
    if needsDefault
        s.(name) = value;
    end
end

function v = local_get(s, name, value)
    if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
        v = s.(name);
    else
        v = value;
    end
end

function y = local_clamp(x, lo, hi)
    y = min(max(x, lo), hi);
end
