local P = require("params")
local NodeLibrary = require("node_library")
local V = require("vector_math")

NodeLibrary:addNodes(
    {
        BFEMCage = {
            label = "Bifilar Electromagnet Cage",
            op = function(inputs)
                local all_faces_selection = SelectionExpression.new("*")
                local segments_per_rail = math.ceil(inputs.segments / inputs.num_pairs / 2)
                if segments_per_rail < 2 then
                    segments_per_rail = 2
                end

                local lower_y = inputs.pos.y - inputs.wire_width/2
                -- extrude_height is flush with the bottom (front) and rises (width+gap) above for wiring purposes on the top (back):
                local extrude_height = 2 * inputs.turns * (inputs.wire_width + inputs.wire_gap) + inputs.wire_width + inputs.wire_gap + inputs.wire_width

                local backbone = Primitives.line(vector(0,0,0), vector(0,1,0), 1)

                local rail_angle_delta = math.pi / inputs.num_pairs
                local inner_radius = inputs.size.x + inputs.wire_width/2 + inputs.wire_gap
                local outer_radius = inner_radius + inputs.thickness

                local inner_dtheta = math.asin(inputs.wire_gap / (2 * inner_radius))
                local outer_dtheta = math.asin(inputs.wire_gap / (2 * outer_radius))

                local delta_y = (inputs.wire_gap + inputs.wire_width) / (inputs.num_pairs - 1)

                local out_mesh = {}
                for i = 0, 2*inputs.num_pairs-1 do
                    local inner_start_angle = i * rail_angle_delta
                    local inner_end_angle = (i + 1) * rail_angle_delta - 2*inner_dtheta
                    local inner_angle_delta = (inner_end_angle - inner_start_angle) / segments_per_rail

                    local outer_start_angle = i * rail_angle_delta
                    local outer_end_angle = (i + 1) * rail_angle_delta - 2*outer_dtheta
                    local outer_angle_delta = (outer_end_angle - outer_start_angle) / segments_per_rail

                    if inner_angle_delta <= 0 or outer_angle_delta <= 0 then
                        error("wire_gap too large with given num_pairs and size")
                        return
                    end

                    local y = lower_y + ((inputs.num_pairs-i) % inputs.num_pairs) * delta_y

                    local function gen_points(dir)
                        local points = {}
                        local function new_point(j, start_angle, angle_delta, r)
                            local angle = start_angle + j * angle_delta
                            local x = inputs.pos.x + r * math.cos(angle)
                            local z = inputs.pos.z + r * math.sin(angle)
                            local point = vector(x, y, z) -- y is "up"
                            table.insert(points, point)
                        end

                        -- inner points:
                        for j = 0, segments_per_rail do
                            new_point(j, inner_start_angle, inner_angle_delta, inner_radius)
                        end

                        -- outer points:
                        for j = segments_per_rail, 0, -1 do
                            new_point(j, outer_start_angle, outer_angle_delta, outer_radius)
                        end

                        if dir < 0 then
                            local rev = {}
                            for i=#points, 1, -1 do
                                rev[#rev+1] = points[i]
                            end
                            return rev
                        end

                        return points
                    end

                    local new_mesh = Primitives.polygon(gen_points(1))
                    Ops.extrude(all_faces_selection, extrude_height, new_mesh)

                    -- Now generate the extrusion caps by going the other direction
                    local face_mesh = Primitives.polygon(gen_points(-1))
                    Ops.merge(new_mesh, face_mesh)

                    if i == 0 then
                        out_mesh = new_mesh
                    else
                        Ops.merge(out_mesh, new_mesh)
                    end
                end

                -- Pre-calculate important values for all coil connectors
                local rotations = {}
                local line_lengths = {}
                local ys = {}
                for i = 1, inputs.num_pairs do
                    ys[i] = lower_y + (i-1) * delta_y
                    -- Create the connectors to the ends of the coils, based on the shift_mixer setting
                    -- which determines how far each coil pair has rotated:
                    rotations[i] = -inputs.shift_mixer * math.pi * (i-1) / inputs.num_pairs
                    line_lengths[i] = (inputs.num_pairs - i + 1) * (inputs.wire_width + inputs.wire_gap)
                end

                local function gen_points(y, inner_start_angle, inner_angle_delta, dir)
                    local points = {}
                    local function new_point(j, start_angle, angle_delta, r)
                        local angle = start_angle + j * angle_delta
                        local x = inputs.pos.x + r * math.cos(angle)
                        local z = inputs.pos.z + r * math.sin(angle)
                        local point = vector(x, y, z) -- y is "up"
                        table.insert(points, point)
                    end

                    -- inner points:
                    for j = 0, segments_per_rail do
                        new_point(j, inner_start_angle, inner_angle_delta, inner_radius)
                    end

                    if dir < 0 then
                        local rev = {}
                        for i=#points, 1, -1 do
                            rev[#rev+1] = points[i]
                        end
                        return rev
                    end

                    return points
                end

                for i = 1, inputs.num_pairs do
                    local y = ys[i] + inputs.wire_width
                    local rotation = rotations[i]
                    local line_length = line_lengths[i]
                    local inner_start_angle = -(i-1) * rail_angle_delta
                    local inner_end_angle = -(i-2) * rail_angle_delta - 2*inner_dtheta
                    local inner_angle_delta = (inner_end_angle - inner_start_angle) / segments_per_rail
                    local points = gen_points(y, inner_start_angle, inner_angle_delta, 1)
                    local cap_points = gen_points(y, inner_start_angle, inner_angle_delta, -1)

                    local connector_radius = inner_radius - line_length
                    local connector_dtheta = math.asin(inputs.wire_gap / (2 * connector_radius))
                    local sx = inputs.pos.x + connector_radius * math.cos(rotation)
                    local sz = inputs.pos.z + connector_radius * math.sin(rotation)

                    -- calculate the final point that will be wire_width away from the next wire
                    local t_angle = inner_end_angle
                    if i > 1 then
                        t_angle = rotations[i-1] - connector_dtheta
                    end
                    local tx = inputs.pos.x + connector_radius * math.cos(t_angle)
                    local tz = inputs.pos.z + connector_radius * math.sin(t_angle)
                    table.insert(points, vector(tx, y, tz))
                    table.insert(points, vector(sx, y, sz))

                    table.insert(cap_points, vector(sx, y, sz))
                    table.insert(cap_points, vector(tx, y, tz))

                    local face = Primitives.polygon(points)
                    Ops.extrude(all_faces_selection, inputs.wire_width, face)
                    Ops.merge(out_mesh, face)
                    local cap_face = Primitives.polygon(cap_points)
                    Ops.merge(out_mesh, cap_face)

                    -- second connection for pair directly opposite first connection
                    local points = gen_points(y, inner_start_angle + math.pi, inner_angle_delta, 1)
                    local cap_points = gen_points(y, inner_start_angle + math.pi, inner_angle_delta, -1)
                    local sx = inputs.pos.x + connector_radius * math.cos(rotation + math.pi)
                    local sz = inputs.pos.z + connector_radius * math.sin(rotation + math.pi)
                    local tx = inputs.pos.x + connector_radius * math.cos(t_angle + math.pi)
                    local tz = inputs.pos.z + connector_radius * math.sin(t_angle + math.pi)
                    table.insert(points, vector(tx, y, tz))
                    table.insert(points, vector(sx, y, sz))

                    table.insert(cap_points, vector(sx, y, sz))
                    table.insert(cap_points, vector(tx, y, tz))

                    local face = Primitives.polygon(points)
                    Ops.extrude(all_faces_selection, inputs.wire_width, face)
                    Ops.merge(out_mesh, face)
                    local cap_face = Primitives.polygon(cap_points)
                    Ops.merge(out_mesh, cap_face)
                end

                return {
                    out_mesh = out_mesh
                }
            end,
            inputs = {
                P.v3("pos", vector(0, 0, 0)),  -- pos is lowered by wire_width/2
                P.v3("size", vector(1, 0, 1)),  -- wire_width/2 + wire_gap is added to size.
                P.scalar("shift_mixer", {default = 1, min = -1, soft_max = 1}),
                P.scalar("thickness", {default = 1, min = 0, soft_max = 10}),
                P.scalar("turns", {default = 1, min = 0, soft_max = 10}),
                P.scalar("wire_gap", {default = 0.1, min = 0, soft_max = 10}),
                P.scalar("wire_width", {default = 1, min = 0, soft_max = 10}),
                P.scalar_int("segments", {default = 36, min = 1, soft_max = 360}),
                P.scalar_int("num_pairs", {default = 5, min = 1, soft_max = 33}),
            },
            outputs = {P.mesh("out_mesh")},
            returns = "out_mesh"
        }
    }
)
