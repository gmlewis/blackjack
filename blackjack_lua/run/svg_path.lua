local P = require("params")
local NodeLibrary = require("node_library")
local V = require("vector_math")

local signed_mod = function(a, b)
    if a > b then
        return a % b
    elseif a < -b then
        return -(-a % b)
    end
    return a
end

local rotate_around_axis = function(angle, x, y)
    return vector(
        math.cos(angle)*x - math.sin(angle)*y,
        math.sin(angle)*x + math.cos(angle)*y,
        0)
end

local parse_parameters = function(d)
    local params = {}
    while string.len(d) > 0 do
        local _, j, m = string.find(d, "^%s*([%d%.%-]+)%s*,*")
        if m ~= nil then
            table.insert(params, 0+m)  -- coerce m to a number.
            d = string.sub(d, j+1)
        else
            -- Should not reach here.
            print("parse_parameters - bad path:", d)
            return {}
        end
    end
    return params
end

local parse_path = function(d)
    local path_steps = {}
    while string.len(d) > 0 do
        local i, j = string.find(d, "^%s+")
        if i ~= nil then
            d = string.sub(d, j+1) -- strip leading whitespace
        end

        local _, j, m = string.find(d, "^([zZ])%s*")
        if m ~= nil then
            table.insert(path_steps, {C="z"})
            d = string.sub(d, j+1)
        else
            local _, j, m, n = string.find(d, "^([mlhvcsqtaMLHVCSQTA])([%d%.%-,%s]*)")
            if m ~= nil then
                local params = parse_parameters(n)
                table.insert(path_steps, {C=m, P=params})
                d = string.sub(d, j+1)
            else
                -- Should not reach here, as this is an unsupported SVG command.
                print("unsupported SVG path command:", d)
                return {}
            end
        end
    end
    return path_steps
end

local cmd_close_path = function(state)
    if #state.points > 0 then
        table.insert(state.paths, {
                         primitive_func = Primitives.polygon,
                         points = state.points,
        })
        state.points = {}
    end
end

local terminate_path = function(state)
    if #state.points > 0 then
        table.insert(state.paths, {
                         primitive_func = Primitives.line_from_points,
                         points = state.points,
        })
        state.points = {}
    end
end

local merge_mbb_vert = function(mbb, vert)
    if vert.x < mbb.min.x then
        mbb.min = vector(vert.x, mbb.min.y, mbb.min.z)
    end
    if vert.y < mbb.min.y then
        mbb.min = vector(mbb.min.x, vert.y, mbb.min.z)
    end
    if vert.z < mbb.min.z then
        mbb.min = vector(mbb.min.x, mbb.min.y, vert.z)
    end
    if vert.x > mbb.max.x then
        mbb.max = vector(vert.x, mbb.max.y, mbb.max.z)
    end
    if vert.y > mbb.max.y then
        mbb.max = vector(mbb.max.x, vert.y, mbb.max.z)
    end
    if vert.z > mbb.max.z then
        mbb.max = vector(mbb.max.x, mbb.max.y, vert.z)
    end
end

local calculate_svg_mbb = function(path)
    local mbb = { min = vector(0,0,0), max = vector(0,0,0) }
    for i, vert in pairs(path.points) do
        if i == 1 then
            mbb = { min = vert, max = vert }
        else
            merge_mbb_vert(mbb, vert)
        end
    end
    path.mbb = mbb
end

local generate_final_mesh = function(state)
    -- first pass - calculate each path's minimum SVG bounding box
    local mbb = { min = vector(0,0,0), max = vector(0,0,0) }
    for i, path in pairs(state.paths) do
        calculate_svg_mbb(path)
        if i == 1 then
            mbb = path.mbb
        else
            merge_mbb_vert(mbb, path.mbb.min)
            merge_mbb_vert(mbb, path.mbb.max)
        end
    end

    if state.u_path == nil and state.v_path == nil and state.preserve_aspect_ratio then
        local diff = mbb.max - mbb.min
        if diff.x > 0 and diff.x > diff.y then
            state.u_path = Primitives.line_from_points({vector(0,0,0), vector(state.max_width,0,0)})
            local max_height = state.max_width * diff.y / diff.x
            state.v_path = Primitives.line_from_points({vector(0,0,0), vector(0,max_height,0)})
            print("wider than tall", state.max_width, max_height)
        elseif diff.y > 0 then
            state.v_path = Primitives.line_from_points({vector(0,0,0), vector(0,state.max_height,0)})
            local max_width = state.max_height * diff.x / diff.y
            state.u_path = Primitives.line_from_points({vector(0,0,0), vector(max_width,0,0)})
            print("taller than wide", max_width, state.max_height)
        else
            return -- nothing to draw
        end
    else
        if state.u_path == nil then
            state.u_path = Primitives.line_from_points({vector(0,0,0), vector(state.max_width,0,0)})
        end
        if state.v_path == nil then
            state.v_path = Primitives.line_from_points({vector(0,0,0), vector(0,state.max_height,0)})
        end
    end

    -- second pass - generate output mesh
    for i, path in pairs(state.paths) do
        if i == 1 then
            state.out_mesh = path.primitive_func(path.points)
        else
            Ops.merge(state.out_mesh, path.primitive_func(path.points))
        end
    end
end

local insert_current_pos = function(state)
    table.insert(state.points, (state.current_pos.x * state.right + state.current_pos.y * state.normal)+vector(0,0,state.pos.z))
end

local cmd_move_to_abs = function(state, params)
    if #params < 2 or #params % 2 ~= 0 then
        print("cmd 'M': requires even number of params")
        return state
    end
    terminate_path(state)
    for i = 1, #params, 2 do
        state.current_pos = state.pos + vector(params[i], params[i+1], 0)
        insert_current_pos(state)
    end
    return state
end

local cmd_move_to = function(state, params)
    if #params < 2 or #params % 2 ~= 0 then
        print("cmd 'm': requires even number of params")
        return state
    end
    terminate_path(state)
    for i = 1, #params, 2 do
        state.current_pos = state.current_pos + vector(params[i], params[i+1], 0)
        insert_current_pos(state)
    end
    return state
end

local line_to = function(state, p0, p1)
    for i = 1, state.segments do
        local t = i / state.segments
        local t1 = 1.0 - t
        state.current_pos = p0 * t1 + p1 * t
        insert_current_pos(state)
    end
    return state
end

local cmd_line_to_abs = function(state, params)
    if #params < 2 or #params % 2 ~= 0 then
        print("cmd 'L': requires even number of params")
        return state
    end
    for i = 1, #params, 2 do
        local p0 = state.current_pos
        local p1 = state.pos + vector(params[i], params[i+1], 0)
        state = line_to(state, p0, p1)
    end
    return state
end

local cmd_line_to = function(state, params)
    if #params < 2 or #params % 2 ~= 0 then
        print("cmd 'l': requires even number of params")
        return state
    end
    for i = 1, #params, 2 do
        local p0 = state.current_pos
        local p1 = state.current_pos + vector(params[i], params[i+1], 0)
        state = line_to(state, p0, p1)
    end
    return state
end

local cmd_line_horizontal_abs = function(state, params)
    if #params < 1 then
        print("cmd 'H': requires one or more params")
        return state
    end
    for i = 1, #params do
        local p0 = state.current_pos
        local p1 = state.current_pos*vector(0,1,0) + vector(params[i],0,0)
        state = line_to(state, p0, p1)
    end
    return state
end

local cmd_line_horizontal = function(state, params)
    if #params < 1 then
        print("cmd 'h': requires one or more params")
        return state
    end
    for i = 1, #params do
        local p0 = state.current_pos
        local p1 = state.current_pos + vector(params[i],0,0)
        state = line_to(state, p0, p1)
    end
    return state
end

local cmd_line_vertical_abs = function(state, params)
    if #params < 1 then
        print("cmd 'V': requires one or more params")
        return state
    end
    for i = 1, #params do
        local p0 = state.current_pos
        local p1 = state.current_pos*vector(1,0,0) + vector(0, params[i], 0)
        state = line_to(state, p0, p1)
    end
    return state
end

local cmd_line_vertical = function(state, params)
    if #params < 1 then
        print("cmd 'v': requires one or more params")
        return state
    end
    for i = 1, #params do
        local p0 = state.current_pos
        local p1 = state.current_pos + vector(0, params[i], 0)
        state = line_to(state, p0, p1)
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
        insert_current_pos(state)
    end
    return state
end

local cmd_cubic_bezier_curve_abs = function(state, params)
    if #params < 6 or #params % 6 ~= 0 then
        print("cmd 'C': requires multiple of 6 params")
        return state
    end
    for i = 1, #params, 6 do
        local p0 = state.current_pos
        local p1 = state.pos + vector(params[i  ], params[i+1], 0)
        local p2 = state.pos + vector(params[i+2], params[i+3], 0)
        local ep = state.pos + vector(params[i+4], params[i+5], 0)
        state = cubic_to(state, p0, p1, p2, ep)
        state.last_p2 = p2
        state.last_p3 = ep
    end
    return state
end

local cmd_cubic_bezier_curve = function(state, params)
    if #params < 6 or #params % 6 ~= 0 then
        print("cmd 'c': requires multiple of 6 params")
        return state
    end
    for i = 1, #params, 6 do
        local p0 = state.current_pos
        local p1 = state.current_pos + vector(params[i  ], params[i+1], 0)
        local p2 = state.current_pos + vector(params[i+2], params[i+3], 0)
        local ep = state.current_pos + vector(params[i+4], params[i+5], 0)
        state = cubic_to(state, p0, p1, p2, ep)
        state.last_p2 = p2
        state.last_p3 = ep
    end
    return state
end

local cmd_smooth_cubic_bezier_curve_abs = function(state, params)
    if #params < 4 or #params % 4 ~= 0 then
        print("cmd 'S': requires multiple of 4 params")
        return state
    end
    for i = 1, #params, 4 do
        local p0 = state.current_pos
        local p1 = state.pos
        if state.last_cmd == "C" or state.last_cmd == "c" or state.last_cmd == "S" or state.last_cmd == "s" then
            p1 = state.current_pos + state.last_p3 - state.last_p2
        end
        local p2 = state.pos + vector(params[i  ], params[i+1], 0)
        local ep = state.pos + vector(params[i+2], params[i+3], 0)
        state = cubic_to(state, p0, p1, p2, ep)
        state.last_p2 = p2
        state.last_p3 = ep
    end
    return state
end

local cmd_smooth_cubic_bezier_curve = function(state, params)
    if #params < 4 or #params % 4 ~= 0 then
        print("cmd 's': requires multiple of 4 params")
        return state
    end
    for i = 1, #params, 4 do
        local p0 = state.current_pos
        local p1 = state.current_pos
        if state.last_cmd == "C" or state.last_cmd == "c" or state.last_cmd == "S" or state.last_cmd == "s" then
            p1 = state.current_pos + state.last_p3 - state.last_p2
        end
        local p2 = state.current_pos + vector(params[i  ], params[i+1], 0)
        local ep = state.current_pos + vector(params[i+2], params[i+3], 0)
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
        insert_current_pos(state)
    end
    return state
end

local cmd_quadratic_bezier_curve_abs = function(state, params)
    if #params < 4 or #params % 4 ~= 0 then
        print("cmd 'Q': requires multiple of 4 params")
        return state
    end
    for i = 1, #params, 4 do
        local p0 = state.current_pos
        local p1 = state.pos + vector(params[i  ], params[i+1], 0)
        local p2 = state.pos + vector(params[i+2], params[i+3], 0)
        state = quadratic_to(state, p0, p1, p2)
        state.last_p1 = p1
        state.last_p2 = p2
    end
    return state
end

local cmd_quadratic_bezier_curve = function(state, params)
    if #params < 4 or #params % 4 ~= 0 then
        print("cmd 'q': requires multiple of 4 params")
        return state
    end
    for i = 1, #params, 4 do
        local p0 = state.current_pos
        local p1 = state.current_pos + vector(params[i  ], params[i+1], 0)
        local p2 = state.current_pos + vector(params[i+2], params[i+3], 0)
        state = quadratic_to(state, p0, p1, p2)
        state.last_p1 = p1
        state.last_p2 = p2
    end
    return state
end

local cmd_smooth_quadratic_bezier_curve_abs = function(state, params)
    if #params < 2 or #params % 2 ~= 0 then
        print("cmd 'T': requires multiple of 2 params")
        return state
    end
    for i = 1, #params, 2 do
        local p0 = state.current_pos
        local p1 = state.current_pos
        if state.last_cmd == "Q" or state.last_cmd == "q" or state.last_cmd == "T" or state.last_cmd == "t" then
            p1 = state.current_pos + state.last_p2 - state.last_p1
        end
        local p2 = state.pos + vector(params[i  ], params[i+1], 0)
        state = quadratic_to(state, p0, p1, p2)
        state.last_p1 = p1
        state.last_p2 = p2
    end
    return state
end

local cmd_smooth_quadratic_bezier_curve = function(state, params)
    if #params < 2 or #params % 2 ~= 0 then
        print("cmd 't': requires multiple of 2 params")
        return state
    end
    for i = 1, #params, 2 do
        local p0 = state.current_pos
        local p1 = state.current_pos
        if state.last_cmd == "Q" or state.last_cmd == "q" or state.last_cmd == "T" or state.last_cmd == "t" then
            p1 = state.current_pos + state.last_p2 - state.last_p1
        end
        local p2 = state.current_pos + vector(params[i  ], params[i+1], 0)
        state = quadratic_to(state, p0, p1, p2)
        state.last_p1 = p1
        state.last_p2 = p2
    end
    return state
end

local angle_between = function(v0, v1)
    local p =  v0.x*v1.x + v0.y*v1.y
    local n = math.sqrt((math.pow(v0.x, 2)+math.pow(v0.y, 2)) * (math.pow(v1.x, 2)+math.pow(v1.y, 2)))
    local sign = v0.x*v1.y - v0.y*v1.x < 0 and -1 or 1
    local angle = sign*math.acos(p/n)
    return angle
end

-- based on: https://ericeastwood.com/blog/curves-and-arcs-quadratic-cubic-elliptical-svg-implementations/
local elliptic_arc_to = function(state, rx, ry, angle, large_arc_flag, sweep_flag, p0, p1)
    if radius == vector(0,0,0) then  -- zero radius degrades to straight line
        state.current_pos = p1
        insert_current_pos(state)
        return
    end

    local d = 0.5*(p0 - p1)
    local tpx = math.cos(angle)*d.x + math.sin(angle)*d.y
    local tpy = -math.sin(angle)*d.x + math.cos(angle)*d.y
    local radii_check = math.pow(tpx,2)/math.pow(rx,2) + math.pow(tpy,2)/math.pow(ry,2)
    if radii_check > 1 then
        rx = math.sqrt(radii_check)*rx
        ry = math.sqrt(radii_check)*ry
    end

    local sn = math.pow(rx,2)*math.pow(ry,2) - math.pow(rx,2)*math.pow(tpy,2) - math.pow(ry,2)*math.pow(tpx,2)
    local srd = math.pow(rx,2)*math.pow(tpy,2) + math.pow(ry,2)*math.pow(tpx,2)
    local radicand = sn/srd
    if radicand < 0 then
        radicand = 0
    end

    local coef = large_arc_flag ~= sweep_flag and math.sqrt(radicand) or -math.sqrt(radicand)
    local tcx = coef*((rx*tpy)/ry)
    local tcy = coef*(-(ry*tpx)/rx)
    local tc = vector(tcx, tcy, 0)

    local center = rotate_around_axis(angle, tcx, tcy) + (0.5*(p0 + p1))

    local start_vector = vector((tpx - tcx)/rx, (tpy - tcy)/ry, 0)
    local start_angle = angle_between(vector(1,0,0), start_vector)

    local end_vector = vector((-tpx - tcx)/rx, (-tpy - tcy)/ry, 0)
    local sweep_angle = angle_between(start_vector, end_vector)

    if sweep_flag == 0 and sweep_angle > 0 then
        sweep_angle = sweep_angle - 2*math.pi
    elseif sweep_flag ~= 0 and sweep_angle < 0 then
        sweep_angle = sweep_angle + 2*math.pi
    end
    sweep_angle = signed_mod(sweep_angle, 2*math.pi)

    for i = 1, state.segments do
        local t = i / state.segments

        local a = start_angle + (sweep_angle * t)
        local ecx = rx * math.cos(a)
        local ecy = ry * math.sin(a)

        state.current_pos = rotate_around_axis(angle, ecx, ecy) + center
        insert_current_pos(state)
    end
    return state
end

local cmd_elliptic_arc_curve_abs = function(state, params)
    if #params < 7 or #params % 7 ~= 0 then
        print("cmd 'A': requires multiple of 7 params")
        return state
    end
    for i = 1, #params, 7 do
        local rx = math.abs(params[i])
        local ry = math.abs(params[i+1])
        local angle = signed_mod(params[i+2], 360) * math.pi / 180
        local large_arc_flag = params[i+3]
        local sweep_flag = params[i+4]
        local p0 = state.current_pos
        local p1 = state.pos + vector(params[i+5], params[i+6], 0)
        if p0 ~= p1 then
            state = elliptic_arc_to(state, rx, ry, angle, large_arc_flag, sweep_flag, p0, p1)
        end
    end
    return state
end

local cmd_elliptic_arc_curve = function(state, params)
    if #params < 7 or #params % 7 ~= 0 then
        print("cmd 'a': requires multiple of 7 params")
        return state
    end
    for i = 1, #params, 7 do
        local rx = math.abs(params[i])
        local ry = math.abs(params[i+1])
        local angle = signed_mod(params[i+2], 360) * math.pi / 180
        local large_arc_flag = params[i+3]
        local sweep_flag = params[i+4]
        local p0 = state.current_pos
        local p1 = state.current_pos + vector(params[i+5], params[i+6], 0)
        if p0 ~= p1 then
            state = elliptic_arc_to(state, rx, ry, angle, large_arc_flag, sweep_flag, p0, p1)
        end
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
                local state = {
                    normal = normal,
                    right = right,
                    pos = inputs.pos,
                    current_pos = inputs.pos,
                    segments = inputs.segments,
                    max_width = inputs.max_width,
                    max_height = inputs.max_height,
                    preserve_aspect_ratio = inputs.preserve_aspect_ratio ~= 0,
                    u_path = inputs.u_path,
                    v_path = inputs.v_path,
                    out_mesh = nil,
                    last_cmd = nil,
                    last_p1 = nil,
                    last_p2 = nil,
                    last_p3 = nil,
                    points = {}, -- used to build up the current path
                    paths = {}, -- used to build up collection of paths
                }

                if inputs.segments < 1 or inputs.d == "" then
                    return {
                        out_mesh = Primitives.line_from_points({})
                    }
                end

                local path_steps = parse_path(inputs.d)
                for _, path_step in pairs(path_steps) do
                    process_path_step(state, path_step)
                    state.last_cmd = path_step.C  -- used for smooth curves
                end

                terminate_path(state)

                generate_final_mesh(state)

                if state.out_mesh == nil then
                    return {
                        out_mesh = Primitives.line_from_points({})
                    }
                end

                return {
                    out_mesh = state.out_mesh
                }
            end,
            inputs = {
                P.lua_str("d"),
                P.v3("pos", vector(0, 0, 0)),
                P.v3("normal", vector(0, 1, 0)),
                P.v3("right", vector(1, 0, 0)),
                P.scalar("max_width", {default = 10, min = 0, soft_max = 100}),
                P.scalar("max_height", {default = 10, min = 0, soft_max = 100}),
                P.scalar_int("preserve_aspect_ratio", {default = 1, min = 0, max = 1}),
                P.scalar_int("segments", {default = 10, min = 0, soft_max = 360}),
                P.mesh("u_path"),  -- u_path overrides the "max_width" and "preserve_aspect_ratio" settings.
                P.mesh("v_path"),  -- v_path overrides the "max_height" and "preserve_aspect_ratio" settings.
            },
            outputs = {P.mesh("out_mesh")},
            returns = "out_mesh"
        }
    }
)
