local robot = {_TYPE='module', _NAME='robot', _VERSION='0.1'}
local mat   = require("matrix")

function robot.genScurve(Pini, Pend, params)
    -- 7 stage Scurve
    -- ============== Input ==============
    -- Pini : (1, Axis) Initial Position
    -- Pend : (1, Axis) Goal Position
    -- params
    --   Amax : (1, Axis) Max Angular Acceleration
    --   Aavg : (1, Axis) Average Angular Acceleration ((0.5 ~ 1)*Amax)
    --   Vmax : (1, Axis) Max Angular Velocity
    --   Axis : Number of Axis

    -- ============= Output ==============
    -- Cmd
    --   P : (tf/sampT, Axis) Position Command
    --   V : (tf/sampT, Axis) Velocity Command
    --   A : (tf/sampT, Axis) Acceration Command
    --   (option) J : (tf/sampT, Axis) Jerk Command

    print("Pini : ", Pini)
    print("Pend : ", Pend)

    -- Parameters
    local Amax  = params.Amax
    local aavg  = params.aavg
    local Aavg  = {}
    local Vmax  = params.Vmax
    local Axis  = params.Axis
    local sampT = params.sampT

    local Ta = mat{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    local Tb = mat{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    local Tc = mat{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    local Ts = mat{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    local S  = mat{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}

    local ret = 0  -- check if successful (1)
    local Cmd = {
        P = {},
        V = {}, 
        A = {},
        J = {}
    }

    -- Time Interval Length
    for i = 1, Axis do
        Aavg[i] = aavg[i] * Amax[i]
        Ta[i]   = Vmax[i] / Aavg[i]
        Tb[i]   = (2 * Vmax[i] / Amax[i]) - Ta[i]
        Tc[i]   = (Ta[i] - Tb[i]) / 2
        S[i]    = math.abs(Pend[i] - Pini[i])
        Ts[i]   = (S[i] - Vmax[i] * Ta[i]) / Vmax[i]
    end

    -- Check Validation (Ts > 0)
    local minTs = math.min(table.unpack(Ts))
    while (minTs < 0) do
        print("Invalid Scurve!!")
        print("Ts is", minTs, "(s), try a smaller Vmax")
        for i = 1, Axis do
            Vmax[i] = math.sqrt(S[i]*Aavg[i])
        end
        for i = 1, Axis do
            Ta[i] = Vmax[i] / Aavg[i]
            Tb[i] = 2 * Vmax[i] / Amax[i] - Ta[i]
            Tc[i] = (Ta[i] - Tb[i]) / 2
            S[i] = math.abs(Pend[i] - Pini[i])
            Ts[i] = (S[i] - Vmax[i] * Ta[i]) / Vmax[i]
        end
        
        -- If it's still invalid, then return fail 
        minTs = math.min(table.unpack(Ts))
        if (minTs < 0) then
            return ret, Cmd
        end
    end

    -- Find the shortest time
    local t_idx = 1
    for i = 2, #Ts do
        if (2 * Ta[i] + Ts[i]) > (2 * Ta[t_idx] + Ts[t_idx]) then
            t_idx = i
        end
    end

    -- Time node
    local t1 = Tc[t_idx]
    local t2 = Tc[t_idx] + Tb[t_idx]
    local t3 = Ta[t_idx]
    local t4 = Ta[t_idx] + Ts[t_idx]
    local t5 = Ta[t_idx] + Ts[t_idx] + Tc[t_idx]
    local t6 = Ta[t_idx] + Ts[t_idx] + Tc[t_idx] + Tb[t_idx]
    local t7 = Ta[t_idx] + Ts[t_idx] + Ta[t_idx]

    -- check Axis to determine Jerk of each joint
    local Jerk = mat{}
    if Axis == 1 then
        Jerk = Amax * robot.sign((Pend - Pini)) / Tc[t_idx]
    else
        for i = 1, Axis do
            Jerk[i] = (Pend[i] - Pini[i]) / (1/6 * t1^3 + 1/6 * t2^3 + 1/3 * t3^3 + 1/6 * t4^3 - 1/6 * t5^3 - 1/6 * t6^3 - 1/2 * t1 * t3^2 - 1/2 * t2 * t3^2
                    + t7 * (-1/2 * t1^2 - 1/2 * t2^2 - 1/2 * t3^2 - 1/2 * t4^2 + 1/2 * t5^2 + 1/2 * t6^2 + t1 * t3 + t2 * t3)
                    + 1/2 * t7^2 * (t4 - t5 - t6) + 1/6 * t7^3)
        end
    end

    -- Generate Scurve
    local t = 0
    local ACmd_i, VCmd_i, PCmd_i = {}, {}, {}
    for i = 1, math.floor(t7 / sampT) do
        ACmd_i[i], VCmd_i[i], PCmd_i[i] = {}, {}, {}

        if t < t1 then
            for axis = 1, Axis do
                ACmd_i[i][axis] = Jerk[axis] * t
                VCmd_i[i][axis] = Jerk[axis] * (1/2 * t^2)
                PCmd_i[i][axis] = Pini[axis] + Jerk[axis] * (1/6 * t^3)
            end
        elseif t1 <= t and t < t2 then
            for axis = 1, Axis do
                ACmd_i[i][axis] = Jerk[axis] * t1
                VCmd_i[i][axis] = Jerk[axis] * (-1/2 * t1^2 + t1 * t)
                PCmd_i[i][axis] = Pini[axis] + Jerk[axis] * (1/6 * t1^3 - 1/2 * t1^2 * t + 1/2 * t1 * t^2)
            end
        elseif t2 <= t and t < t3 then
            for axis = 1, Axis do
                ACmd_i[i][axis] = Jerk[axis] * (-t + t1 + t2)
                VCmd_i[i][axis] = Jerk[axis] * (-1/2 * t1^2 - 1/2 * t2^2 + t1 * t + t2 * t - 1/2 * t^2)
                PCmd_i[i][axis] = Pini[axis] + Jerk[axis] * (1/6 * t1^3 + 1/6 * t2^3 + (-1/2 * t1^2 - 1/2 * t2^2) * t + 1/2 * (t1 + t2) * t^2 - 1/6 * t^3)
            end
        elseif t3 <= t and t < t4 then
            for axis = 1, Axis do
                ACmd_i[i][axis] = 0
                VCmd_i[i][axis] = Jerk[axis] * (-1/2 * t1^2 - 1/2 * t2^2 - 1/2 * t3^2 + t1 * t3 + t2 * t3)
                PCmd_i[i][axis] = Pini[axis] + Jerk[axis] * (1/6 * t1^3 + 1/6 * t2^3 + 1/3 * t3^3 - 1/2 * t1 * t3^2 - 1/2 * t2 * t3^2
                                        + (-1/2 * t1^2 - 1/2 * t2^2 - 1/2 * t3^2 + t1 * t3 + t2 * t3) * t)
            end
        elseif t4 <= t and t < t5 then
            for axis = 1, Axis do
                ACmd_i[i][axis] = Jerk[axis] * (-t + t4)
                VCmd_i[i][axis] = Jerk[axis] * (-1/2 * t1^2 - 1/2 * t2^2 - 1/2 * t3^2 - 1/2 * t4^2 + t1 * t3 + t2 * t3 + t4 * t - 1/2 * t^2)
                PCmd_i[i][axis] = Pini[axis] + Jerk[axis] * (1/6 * t1^3 + 1/6 * t2^3 + 1/3 * t3^3 + 1/6 * t4^3 - 1/2 * t1 * t3^2 - 1/2 * t2 * t3^2
                                        + (-1/2 * t1^2 - 1/2 * t2^2 - 1/2 * t3^2 - 1/2 * t4^2 + t1 * t3 + t2 * t3) * t
                                        + 1/2 * t4 * t^2 - 1/6 * t^3)
            end
        elseif t5 <= t and t < t6 then
            for axis = 1, Axis do
                ACmd_i[i][axis] = Jerk[axis] * (t4 - t5)
                VCmd_i[i][axis] = Jerk[axis] * (-1/2 * t1^2 - 1/2 * t2^2 - 1/2 * t3^2 - 1/2 * t4^2 + 1/2 * t5^2 + t1 * t3 + t2 * t3 + t4 * t - t5 * t)
                PCmd_i[i][axis] = Pini[axis] + Jerk[axis] * (1/6 * t1^3 + 1/6 * t2^3 + 1/3 * t3^3 + 1/6 * t4^3 - 1/6 * t5^3 - 1/2 * t1 * t3^2 - 1/2 * t2 * t3^2
                                        + (-1/2 * t1^2 - 1/2 * t2^2 - 1/2 * t3^2 - 1/2 * t4^2 + 1/2 * t5^2 + t1 * t3 + t2 * t3) * t
                                        + 1/2 * (t4 - t5) * t^2)
            end
        elseif t6 <= t and t <= t7 then
            for axis = 1, Axis do
                ACmd_i[i][axis] = Jerk[axis] * (t + t4 - t5 - t6)
                VCmd_i[i][axis] = Jerk[axis] * (-1/2 * t1^2 - 1/2 * t2^2 - 1/2 * t3^2 - 1/2 * t4^2 + 1/2 * t5^2 + 1/2 * t6^2
                                        + t1 * t3 + t2 * t3 + t4 * t - t5 * t - t6 * t + 1/2 * t^2)
                PCmd_i[i][axis] = Pini[axis] + Jerk[axis] * (1/6 * t1^3 + 1/6 * t2^3 + 1/3 * t3^3 + 1/6 * t4^3 - 1/6 * t5^3 - 1/6 * t6^3
                                        - 1/2 * t1 * t3^2 - 1/2 * t2 * t3^2
                                        + (-1/2 * t1^2 - 1/2 * t2^2 - 1/2 * t3^2 - 1/2 * t4^2 + 1/2 * t5^2 + 1/2 * t6^2 + t1 * t3 + t2 * t3) * t
                                        + 1/2 * (t4 - t5 - t6) * t^2 + 1/6 * t^3)
            end
        end

        t = t + sampT
    end

    ret   = 1
    Cmd.P = PCmd_i
    Cmd.V = VCmd_i
    Cmd.A = ACmd_i

    return ret, Cmd
end

function robot.sign(x)
    if x >= 0 then
        return 1
    else
        return 0
    end
end



return robot