local P = require("params")
local NodeLibrary = require("node_library")
local V = require("vector_math")

local parse_parameters = function(d)
    local params = {}
    while string.len(d) > 0 do
        local _, j, m = string.find(d, "^%s*([%d%.%-]+)%s*")
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
                    pos = inputs.pos,
                    current_pos = inputs.pos,
                    right = right,
                    segments = inputs.segments,
                    size = inputs.size,
                    points = {},
                    mesh = nil,
                }

                if inputs.segments < 1 then
                    return {
                        out_mesh = state.mesh
                    }
                end

                local path_steps = parse_path(inputs.path)
                for _, path_step in path_steps do
                    process_path_step(state, path_step)
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
