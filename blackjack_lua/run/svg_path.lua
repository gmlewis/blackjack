local P = require("params")
local NodeLibrary = require("node_library")
local V = require("vector_math")

local parse_parameters = function(d)
    local params = {}
    while string.len(d) > 0 do
        local _, j, m = string.find(d, "^%s*([%d%.%-]+)%s*,*")
        if m ~= nil then
            table.insert(params, 0+m)  -- coerce m to a number.
            d = string.sub(d, j+1)
            continue
        end

        -- Should not reach here.
        print("programming error - parse_parameters")
        return params
    end
    return params
end

local parse_path = function(d)
    local path_steps = {}
    while string.len(d) > 0 do
        local i, j = string.find(d, "^%s+")
        if i ~= nil then
            d = string.sub(d, j+1)
        end

        local _, j, m = string.find(d, "^(z)%s*")
        if m ~= nil then
            table.insert(path_steps, {C="z"})
            d = string.sub(d, j+1)
            continue
        end

        local _, j, m, n = string.find(d, "^([mlhvcsqtaMLHVCSQTA])([%d%.%-,%s]*)")
        if m ~= nil then
            local params = parse_parameters(n)
            table.insert(path_steps, {C=m, P=params})
            d = string.sub(d, j+1)
            continue
        end

        -- Should not reach here, as this is an unsupported SVG command.
        return path_steps
    end
    return path_steps
end

local cmd_close_path = function(state)
    if #state.points > 0 then
        if state.mesh ~= nil then
            Ops.merge(state.mesh, Primitives.line_from_points(state.points))
        else
            state.mesh = Primitives.line_from_points(state.points)
        end
        state.points = {}
    end
end

local cmd_move_to_abs = function(state, params)
    cmd_close_path(state)
    -- print("pos=",state.pos,", params[1]=",params[1],", right=", state.right,", params[2]=",params[2],", normal=",state.normal)
    state.current_pos = state.pos + params[1] * state.right + params[2] * state.normal
    table.insert(state.points, state.current_pos)
    return state
end

local cmd_move_to = function(state, params)
    cmd_close_path(state)
    state.current_pos = state.current_pos + params[1] * state.right + params[2] * state.normal
    table.insert(state.points, state.current_pos)
    return state
end

local cmd_line_to_abs = function(state, params)
    state.current_pos = state.pos + params[1] * state.right + params[2] * state.normal
    table.insert(state.points, state.current_pos)
    return state
end

local cmd_line_to = function(state, params)
    for i = 1, #params, 2 do
        state.current_pos = state.current_pos + params[i] * state.right + params[i+1] * state.normal
        table.insert(state.points, state.current_pos)
    end
    return state
end

local cmd_line_horizontal_abs = function(state, params)
    for i = 1, #params do
        state.current_pos = state.pos + params[i] * state.right
        table.insert(state.points, state.current_pos)
    end
    return state
end

local cmd_line_horizontal = function(state, params)
    for i = 1, #params do
        state.current_pos = state.current_pos + params[i] * state.right
        table.insert(state.points, state.current_pos)
    end
    return state
end

local cmd_line_vertical_abs = function(state, params)
    for i = 1, #params do
        state.current_pos = state.pos + params[i] * state.normal
        table.insert(state.points, state.current_pos)
    end
    return state
end

local cmd_line_vertical = function(state, params)
    for i = 1, #params do
        state.current_pos = state.current_pos + params[i] * state.normal
        table.insert(state.points, state.current_pos)
    end
    return state
end

local cubic_to = function(state, p0, p1, p2, p3)
    for i = 1, state.segments do
        local t = i / state.segments
        local t1 = 1.0 - t
        local f = t1 * t1 * t1
        state.current_pos = p0 * f
        f = 3.0 * t1 * t1 * t
        state.current_pos = state.current_pos + p1 * f
        f = 3.0 * t1 * t * t
        state.current_pos = state.current_pos + p2 * f
        f = t * t * t
        state.current_pos = state.current_pos + p3 * f
        table.insert(state.points, state.current_pos)
    end
    return state
end

local cmd_cubic_bezier_curve_abs = function(state, params)
    for i = 1, #params, 6 do
        local p0 = state.current_pos
        local p1 = state.pos + params[i  ] * state.right + params[i+1] * state.normal
        local p2 = state.pos + params[i+2] * state.right + params[i+3] * state.normal
        local ep = state.pos + params[i+4] * state.right + params[i+5] * state.normal
        state = cubic_to(state, p0, p1, p2, ep)
        state.last_p2 = p2
        state.last_p3 = ep
    end
    return state
end

local cmd_cubic_bezier_curve = function(state, params)
    for i = 1, #params, 6 do
        local p0 = state.current_pos
        local p1 = state.current_pos + params[i  ] * state.right + params[i+1] * state.normal
        local p2 = state.current_pos + params[i+2] * state.right + params[i+3] * state.normal
        local ep = state.current_pos + params[i+4] * state.right + params[i+5] * state.normal
        state = cubic_to(state, p0, p1, p2, ep)
        state.last_p2 = p2
        state.last_p3 = ep
    end
    return state
end

local cmd_smooth_cubic_bezier_curve_abs = function(state, params)
    for i = 1, #params, 4 do
        local p0 = state.current_pos
        local p1 = state.pos
        if state.last_cmd == "C" or state.last_cmd == "c" or state.last_cmd == "S" or state.last_cmd == "s" then
            p1 = state.pos + state.last_p3 - state.last_p2
        end
        local p2 = state.pos + params[i  ] * state.right + params[i+1] * state.normal
        local ep = state.pos + params[i+2] * state.right + params[i+3] * state.normal
        state = cubic_to(state, p0, p1, p2, ep)
        state.last_p2 = p2
        state.last_p3 = ep
    end
    return state
end

local cmd_smooth_cubic_bezier_curve = function(state, params)
    for i = 1, #params, 4 do
        local p0 = state.current_pos
        local p1 = state.current_pos
        if state.last_cmd == "C" or state.last_cmd == "c" or state.last_cmd == "S" or state.last_cmd == "s" then
            p1 = state.current_pos + state.last_p3 - state.last_p2
        end
        local p2 = state.current_pos + params[i  ] * state.right + params[i+1] * state.normal
        local ep = state.current_pos + params[i+2] * state.right + params[i+3] * state.normal
        state = cubic_to(state, p0, p1, p2, ep)
        state.last_p2 = p2
        state.last_p3 = ep
    end
    return state
end

local quadratic_to = function(state, p0, p1, p2)
    for i = 1, state.segments do
        local t = i / state.segments
        local t1 = 1.0 - t
        local f = t1 * t1
        state.current_pos = p0 * f
        f = 2.0 * t1 * t
        state.current_pos = state.current_pos + p1 * f
        f = t * t
        state.current_pos = state.current_pos + p2 * f
        table.insert(state.points, state.current_pos)
    end
    return state
end

local cmd_quadratic_bezier_curve_abs = function(state, params)
    for i = 1, #params, 4 do
        local p0 = state.current_pos
        local p1 = state.pos + params[i  ] * state.right + params[i+1] * state.normal
        local p2 = state.pos + params[i+2] * state.right + params[i+3] * state.normal
        state = quadratic_to(state, p0, p1, p2)
        state.last_p1 = p1
        state.last_p2 = p2
    end
    return state
end

local cmd_quadratic_bezier_curve = function(state, params)
    for i = 1, #params, 4 do
        local p0 = state.current_pos
        local p1 = state.current_pos + params[i  ] * state.right + params[i+1] * state.normal
        local p2 = state.current_pos + params[i+2] * state.right + params[i+3] * state.normal
        state = quadratic_to(state, p0, p1, p2)
        state.last_p1 = p1
        state.last_p2 = p2
    end
    return state
end

local cmd_smooth_quadratic_bezier_curve_abs = function(state, params)
    for i = 1, #params, 2 do
        local p0 = state.current_pos
        local p1 = state.pos
        if state.last_cmd == "Q" or state.last_cmd == "q" or state.last_cmd == "T" or state.last_cmd == "t" then
            p1 = state.pos + state.last_p2 - state.last_p1
        end
        local p2 = state.pos + params[i  ] * state.right + params[i+1] * state.normal
        state = quadratic_to(state, p0, p1, p2)
        state.last_p1 = p1
        state.last_p2 = p2
    end
    return state
end

local cmd_smooth_quadratic_bezier_curve = function(state, params)
    for i = 1, #params, 2 do
        local p0 = state.current_pos
        local p1 = state.current_pos
        if state.last_cmd == "Q" or state.last_cmd == "q" or state.last_cmd == "T" or state.last_cmd == "t" then
            p1 = state.current_pos + state.last_p2 - state.last_p1
        end
        local p2 = state.current_pos + params[i  ] * state.right + params[i+1] * state.normal
        state = quadratic_to(state, p0, p1, p2)
        state.last_p1 = p1
        state.last_p2 = p2
    end
    return state
end

-- based on: https://ericeastwood.com/blog/curves-and-arcs-quadratic-cubic-elliptical-svg-implementations/
local elliptic_arc_to = function(state, rx, ry, angle, large_arc_flag, sweep_flag, p0, p1)
    if radius == vector(0,0,0) then  -- zero radius degrades to straight line
        state.current_pos = p1
        table.insert(state.points, state.current_pos)
        return
    end

    local tp = V.rotate_around_axis((p0 - p1) / 2, state.forward, angle)
    local tpx = V.dot(tp, state.right)
    local tpy = V.dot(tp, state.normal)
    -- print("tp=",tp,", tpx=",tpx,", tpy=",tpy)
    local radii_check = math.pow(tpx,2)/math.pow(rx,2) + math.pow(tpy,2)/math.pow(ry,2)
    if radii_check > 1 then
        rx = math.sqrt(radii_check)*rx
        ry = math.sqrt(radii_check)*ry
    end
    -- print("rx=",rx,", ry=",ry)

    local sn = math.pow(rx,2)*math.pow(ry,2) - math.pow(rx,2)*math.pow(tpy,2) - math.pow(ry,2)*math.pow(tpx,2)
    local srd = math.pow(rx,2)*math.pow(tpy,2) + math.pow(ry,2)*math.pow(tpx,2)
    local radicand = sn/srd
    if radicand < 0 then
        radicand = 0
    end
    -- print("sn=",sn,", srd=",srd,", radicand=",radicand)
    local coef = large_arc_flag == sweep_flag and -math.sqrt(radicand) or math.sqrt(radicand)
    local tcx = coef*((rx*tpy)/ry)
    local tcy = -coef*(-(ry*tpx)/rx)
    local tc = tcx * state.right + tcy * state.normal

    local center = V.rotate_around_axis(tc, state.forward, angle) + ((p0 + p1) / 2)

    local start_vector = vector((tpx - tcx)/rx, (tpy - tcy)/ry, 0)
    local cos_theta = V.dot(state.right, start_vector) / V.length(start_vector)
    local start_angle = math.acos(cos_theta)

    local end_vector = vector((-tpx - tcx)/rx, (-tpy - tcy)/ry, 0)
    local cos_theta = V.dot(start_vector, end_vector) / (V.length(start_vector) * V.length(end_vector))
    local sweep_angle = math.acos(cos_theta)

    if sweep_flag == 0 and sweep_angle > 0 then
        sweep_angle = sweep_angle - 2*math.pi
    elseif swap_flag ~= 0 and sweep_angle < 0 then
        sweep_angle = sweep_angle + 2*math.pi
    end
    sweep_angle = sweep_angle % (2*math.pi)

    for i = 1, state.segments do
        local t = i / state.segments

        local a = start_angle + (sweep_angle * t)
        local ecx = rx * math.cos(a)
        local ecy = ry * math.sin(a)

        local ec = ecx * state.right + ecy * state.normal
        state.current_pos = V.rotate_around_axis(ec, state.forward, angle) + center
        table.insert(state.points, state.current_pos)
    end
    return state
end

local cmd_elliptic_arc_curve_abs = function(state, params)
    for i = 1, #params, 7 do
        local rx = math.abs(params[i])
        local ry = math.abs(params[i+1])
        local angle = (params[i+2] % 360) * 180 / math.pi
        local large_arc_flag = params[i+3]
        local sweep_flag = params[i+4]
        local p0 = state.current_pos
        local p1 = state.pos + params[i+5] * state.right + params[i+6] * state.normal
        if p0 == p1 then
            continue
        end
        state = elliptic_arc_to(state, rx, ry, angle, large_arc_flag, sweep_flag, p0, p1)
    end
    return state
end

local cmd_elliptic_arc_curve = function(state, params)
    for i = 1, #params, 7 do
        local rx = math.abs(params[i])
        local ry = math.abs(params[i+1])
        local angle = (params[i+2] % 360) * 180 / math.pi
        local large_arc_flag = params[i+3]
        local sweep_flag = params[i+4]
        local p0 = state.current_pos
        local p1 = state.current_pos + params[i+5] * state.right + params[i+6] * state.normal
        if p0 == p1 then
            continue
        end
        state = elliptic_arc_to(state, rx, ry, angle, large_arc_flag, sweep_flag, p0, p1)
    end
    return state
end

local all_commands = {
    Z = cmd_close_path,
    z = cmd_close_path,
    M = cmd_move_to_abs,
    m = cmd_move_to,
    L = cmd_line_to_abs,
    l = cmd_line_to,
    H = cmd_line_horizontal_abs,
    h = cmd_line_horizontal,
    V = cmd_line_vertical_abs,
    v = cmd_line_vertical,
    C = cmd_cubic_bezier_curve_abs,
    c = cmd_cubic_bezier_curve,
    S = cmd_smooth_cubic_bezier_curve_abs,
    s = cmd_smooth_cubic_bezier_curve,
    Q = cmd_quadratic_bezier_curve_abs,
    q = cmd_quadratic_bezier_curve,
    T = cmd_smooth_quadratic_bezier_curve_abs,
    t = cmd_smooth_quadratic_bezier_curve,
    A = cmd_elliptic_arc_curve_abs,
    a = cmd_elliptic_arc_curve,
}

-- A path_step represents a single path step.
--
-- There are 20 possible commands, broken up into 6 types,
-- with each command having an "absolute" (upper case) and
-- a "relative" (lower case) version.
--
-- ClosePath: Z, z
-- MoveTo: M, m
-- LineTo: L, l, H, h, V, v
-- Cubic Bézier Curve: C, c, S, s
-- Quadratic Bézier Curve: Q, q, T, t
-- Elliptical Arc Curve: A, a
--
-- The 'C' field is the command, and the 'P' field is the numeric parameters.
local process_path_step = function(state, path_step)
    local cmd = all_commands[path_step.C]
    if cmd == nil then
        return state  -- error - command not found
    end
    state = cmd(state, path_step.P)
    return state
end

NodeLibrary:addNodes(
    {
        SVGPath = {
            label = "SVG Path",
            op = function(inputs)
                local normal = V.normalize(inputs.normal)
                local right = V.normalize(inputs.right)
                local forward = V.cross(normal, right)
                local state = {
                    normal = normal,
                    right = right,
                    forward = forward,
                    pos = inputs.pos,
                    current_pos = inputs.pos,
                    segments = inputs.segments,
                    size = inputs.size,
                    points = {},
                    mesh = nil,
                    last_cmd = nil,
                    last_p1 = nil,
                    last_p2 = nil,
                    last_p3 = nil,
                }

                if inputs.segments < 1 then
                    return {
                        out_mesh = state.mesh
                    }
                end

                local path_steps = parse_path(inputs.path)
                for _, path_step in path_steps do
                    process_path_step(state, path_step)
                    state.last_cmd = path_step.C  -- used for smooth curves
                end

                cmd_close_path(state)

                return {
                    out_mesh = state.mesh
                }
            end,
            inputs = {
                P.lua_str("path"),
                P.v3("pos", vector(0, 0, 0)),
                P.v3("normal", vector(0, 1, 0)),
                P.v3("right", vector(1, 0, 0)),
                P.v3("size", vector(1, 1, 1)),
                P.scalar_int("segments", {default = 36, min = 0, soft_max = 360}),
            },
            outputs = {P.mesh("out_mesh")},
            returns = "out_mesh"
        }
    }
)
