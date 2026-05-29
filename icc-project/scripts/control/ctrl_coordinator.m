function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
%CTRL_COORDINATOR Allocate AFS, ESC, braking, and damping commands.
%   The allocation uses a conservative 60:40 front/rear base brake split and
%   adds a differential brake pair to realize the requested ESC yaw moment.
%   Commands are saturated per actuator and tapered at low speed.

    if nargin < 8; LIM = struct(); end
    if nargin < 7; CTRL = struct(); end
    if nargin < 6; VEH = struct(); end

    vx = max(vx, 0.0);
    rw = local_get(VEH, 'rw', 0.31);
    tf = local_get(VEH, 'track_f', 1.55);
    tr = local_get(VEH, 'track_r', 1.55);

    maxSteer = local_get(LIM, 'MAX_STEER_ANGLE', deg2rad(36));
    maxBrake = local_get(LIM, 'MAX_BRAKE_TRQ', 3000);

    steerCmd = local_get(latCmd, 'steerAngle', 0);
    actuatorCmd.steerAngle = local_clamp(steerCmd, -maxSteer, maxSteer);

    brakeTorque = zeros(4, 1);
    if isstruct(latCmd) && isfield(latCmd, 'brakeTorqueDelta') && ~isempty(latCmd.brakeTorqueDelta)
        deltaBrake = latCmd.brakeTorqueDelta(:);
        if numel(deltaBrake) == 1; deltaBrake = deltaBrake * ones(4, 1); end
        brakeTorque = brakeTorque + deltaBrake(1:4);
    end

    % Longitudinal braking request. In the benchmark runner this is normally
    % zero because scenario braking is injected separately, but the function
    % remains complete for standalone use.
    FxTotal = local_get(lonCmd, 'Fx_total', 0);
    brakeRatio = local_get(lonCmd, 'brakeRatio', 0);
    if FxTotal < 0 || brakeRatio > 0
        totalBrakeTorque = max(0, -FxTotal) * rw;
        if totalBrakeTorque <= 1e-6
            totalBrakeTorque = 4 * maxBrake * local_clamp(brakeRatio, 0, 1);
        end
        brakeTorque = brakeTorque + [0.30; 0.30; 0.20; 0.20] * totalBrakeTorque;
    end

    % ESC differential braking. A positive requested yaw moment is generated
    % by increasing braking on the left side; a negative moment acts on the
    % right side. This convention matches the 14-DOF yaw equation.
    Mz = local_get(latCmd, 'yawMoment', 0);
    escSpeedScale = local_clamp((vx - 3.0) / 12.0, 0.0, 1.0);
    Mz = Mz * escSpeedScale;

    frontRatio = local_get(CTRL.COORD, 'escFrontRatio', 0.62);
    frontRatio = local_clamp(frontRatio, 0.35, 0.80);
    rearRatio = 1.0 - frontRatio;

    dTf = abs(Mz) * frontRatio / max(tf, 0.1);
    dTr = abs(Mz) * rearRatio  / max(tr, 0.1);
    dT = [dTf; dTf; dTr; dTr];

    if Mz > 0
        escAdd = [dT(1); 0; dT(3); 0];
    elseif Mz < 0
        escAdd = [0; dT(2); 0; dT(4)];
    else
        escAdd = zeros(4, 1);
    end

    % Lightweight friction-circle guard: avoid using all brake capacity for
    % yaw control at high lateral demand. This protects path deviation and
    % prevents wheel lock in A7/D1.
    escCap = local_get(CTRL.COORD, 'escBrakeCap', 1350);
    brakeTorque = brakeTorque + local_clamp_vec(escAdd, 0, escCap);

    % The scenario runner treats this output as an additive brake delta before
    % final physical clipping, so negative values are allowed here for ABS-like
    % release of an externally commanded brake pulse.
    actuatorCmd.brakeTorque = local_clamp_vec(brakeTorque, -maxBrake, maxBrake);

    if nargin < 3 || isempty(verCmd)
        verCmd = 1500 * ones(4, 1);
    end
    verCmd = verCmd(:);
    if numel(verCmd) == 1; verCmd = verCmd * ones(4, 1); end
    cMin = local_get(CTRL.VER, 'cMin', 500);
    cMax = local_get(CTRL.VER, 'cMax', 5000);
    actuatorCmd.dampingCoeff = local_clamp_vec(verCmd(1:4), cMin, cMax);
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

function y = local_clamp_vec(x, lo, hi)
    y = min(max(x, lo), hi);
end
