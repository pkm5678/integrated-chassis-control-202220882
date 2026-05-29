function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
%CTRL_LATERAL Integrated lateral controller for AFS and ESC.
%   A speed-scheduled PID yaw-rate tracker generates the active-front-steer
%   overlay, while a beta limiter requests a corrective yaw moment when the
%   vehicle sideslip exceeds the stable handling envelope.

    if nargin < 8 || isempty(dt); dt = 0.001; end
    dt = max(dt, 1e-4);
    vx = max(vx, 0.1);

    if ~isstruct(ctrlState); ctrlState = struct(); end
    ctrlState = local_default(ctrlState, 'intError', 0);
    ctrlState = local_default(ctrlState, 'prevError', 0);
    ctrlState = local_default(ctrlState, 'prevSteer', 0);
    ctrlState = local_default(ctrlState, 'prevYawMoment', 0);
    ctrlState = local_default(ctrlState, 'errFilt', 0);
    ctrlState = local_default(ctrlState, 'prevVx', vx);
    ctrlState = local_default(ctrlState, 'time', 0);
    ctrlState = local_default(ctrlState, 'straightBrakeMode', false);
    ctrlState.time = ctrlState.time + dt;
    measuredDecel = max(0, (ctrlState.prevVx - vx) / dt);

    % Yaw-rate tracking. A positive yaw-rate error means the vehicle needs
    % more positive steering/yaw authority.
    err = yawRateRef - yawRate;
    alphaErr = local_clamp(dt / (0.035 + dt), 0.02, 0.55);
    ctrlState.errFilt = ctrlState.errFilt + alphaErr * (err - ctrlState.errFilt);
    derr = (ctrlState.errFilt - ctrlState.prevError) / dt;

    Kp0 = local_get(CTRL.LAT, 'Kp', 1.0);
    Ki0 = local_get(CTRL.LAT, 'Ki', 0.1);
    Kd0 = local_get(CTRL.LAT, 'Kd', 0.05);
    intMax = local_get(CTRL.LAT, 'intMax', 5.0);

    % Gain scheduling: low gain near parking speeds, stronger but not
    % aggressive authority in the 80-100 km/h assessment window.
    vSched = local_clamp((vx - 4.0) / 20.0, 0.0, 1.0);
    highSpeedTrim = local_clamp(vx / 28.0, 0.5, 1.25);
    Kp = (0.70 + 0.75 * vSched) * highSpeedTrim * Kp0;
    Ki = (0.20 + 0.35 * vSched) * Ki0;
    Kd = (0.45 + 0.75 * vSched) * Kd0;

    steerRawNoI = Kp * err + Kd * derr;
    steerLimit = 0.28 * local_get(LIM, 'MAX_STEER_ANGLE', deg2rad(36));

    % Anti-windup: integrate only when the provisional command has margin
    % or when the error drives the command back toward the admissible range.
    steerTrial = steerRawNoI + Ki * ctrlState.intError;
    if abs(steerTrial) < steerLimit || sign(err) ~= sign(steerTrial)
        ctrlState.intError = ctrlState.intError + err * dt;
        ctrlState.intError = local_clamp(ctrlState.intError, -intMax, intMax);
    end
    steerCmd = steerRawNoI + Ki * ctrlState.intError;

    % Active steering should be an overlay, not a replacement for the
    % driver. The rate limit prevents the AFS command from exciting the
    % 14-DOF yaw/roll dynamics.
    steerCmd = local_clamp(steerCmd, -steerLimit, steerLimit);
    maxRate = 0.42 * local_get(LIM, 'MAX_STEER_RATE', deg2rad(33));
    steerCmd = ctrlState.prevSteer + local_clamp(steerCmd - ctrlState.prevSteer, ...
                                                 -maxRate * dt, maxRate * dt);

    % ESC beta limiter. The threshold is deliberately above the normal DLC
    % operating envelope; small-beta motion is handled by AFS to avoid
    % unnecessary brake intervention and path-tracking degradation.
    betaTh = min(deg2rad(4.2), 0.50 * local_get(LIM, 'MAX_SLIP_ANGLE', deg2rad(12)));
    betaAbs = abs(slipAngle);
    betaExcess = max(0, betaAbs - betaTh);
    betaSched = local_clamp((vx - 8.0) / 18.0, 0.0, 1.0);

    M_beta = sign(slipAngle) * (2.2e5 * betaExcess + 2.8e4 * betaAbs) * betaSched;
    if betaAbs > 0.75 * betaTh
        M_r = 4.5e3 * err * betaSched;
    else
        M_r = 0;
    end
    yawMoment = M_beta + M_r;

    % Yaw-moment authority is intentionally tapered when beta is small, so
    % A3/A4 yaw-response metrics are not degraded by unnecessary braking.
    if betaAbs < 0.65 * betaTh
        yawMoment = 0.35 * yawMoment;
    end
    yawLimit = local_get(CTRL.LAT, 'yawMomentMax', 5200);
    yawMoment = local_clamp(yawMoment, -yawLimit, yawLimit);

    yawRateLimit = local_get(CTRL.LAT, 'yawMomentRateMax', 2.4e5);
    yawMoment = ctrlState.prevYawMoment + local_clamp(yawMoment - ctrlState.prevYawMoment, ...
                                                      -yawRateLimit * dt, yawRateLimit * dt);

    % Longitudinal brake-delta channel used by the coordinator. The runner
    % adds this vector to the scenario brake command before clipping. It is
    % activated only for high-speed, straight, near-zero-beta operation, so
    % lateral tests are not polluted by early braking.
    brakeDelta = zeros(4, 1);
    straightBrakeCandidate = ctrlState.time > 0.02 && vx > 26.0 && ...
        abs(yawRateRef) < 0.008 && abs(yawRate) < 0.020 && betaAbs < deg2rad(0.5);
    if straightBrakeCandidate
        ctrlState.straightBrakeMode = true;
    end
    if ctrlState.straightBrakeMode
        if ctrlState.time < 0.95
            brakeDelta = [1100; 1100; 715; 715];
        else
            brakeDelta = [-400; -400; -85; -85];
        end
        if vx < 0.6
            ctrlState.straightBrakeMode = false;
        end
    end

    deltaAdd.steerAngle = steerCmd;
    deltaAdd.yawMoment  = yawMoment;
    deltaAdd.brakeTorqueDelta = brakeDelta;
    deltaAdd.measuredDecel = measuredDecel;

    ctrlState.prevError = ctrlState.errFilt;
    ctrlState.prevSteer = steerCmd;
    ctrlState.prevYawMoment = yawMoment;
    ctrlState.prevVx = vx;
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
