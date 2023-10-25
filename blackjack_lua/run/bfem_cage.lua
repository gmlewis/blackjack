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

                local backbone = Primitives.line(vector(0,0,0), vector(0,1,0), 1)

                local direction = inputs.direction == "Clockwise" and -1 or 1
                local rail_angle_delta = math.pi / inputs.num_pairs
                local inner_radius = inputs.size.x
                local outer_radius = inner_radius + inputs.thickness

                local inner_dtheta = math.asin(inputs.wire_gap / (2 * inner_radius))
                local outer_dtheta = math.asin(inputs.wire_gap / (2 * outer_radius))

                local out_mesh = {} -- P.mesh("out_mesh")
                for i = 0, 2*inputs.num_pairs-1 do
                    local inner_start_angle = i * rail_angle_delta + inner_dtheta
                    local inner_end_angle = (i + 1) * rail_angle_delta - inner_dtheta
                    local inner_angle_delta = (inner_end_angle - inner_start_angle) / segments_per_rail

                    local outer_start_angle = i * rail_angle_delta + outer_dtheta
                    local outer_end_angle = (i + 1) * rail_angle_delta - outer_dtheta
                    local outer_angle_delta = (outer_end_angle - outer_start_angle) / segments_per_rail

                    if inner_angle_delta <= 0 or outer_angle_delta <= 0 then
                        error("wire_gap too large with given num_pairs and size")
                        return
                    end

                    local function gen_points(dir)
                        local points = {}
                        local function new_point(j, start_angle, angle_delta, rx, rz)
                            local angle = dir * (start_angle + j * angle_delta)
                            local x = inputs.pos.x + rx * math.cos(angle)
                            local z = inputs.pos.z + rz * math.sin(angle)
                            local point = vector(x, inputs.pos.y, z) -- y is "up"
                            table.insert(points, point)
                        end

                        -- inner points:
                        local rx = inputs.size.x
                        local rz = inputs.size.z
                        for j = 0, segments_per_rail do
                            new_point(j, inner_start_angle, inner_angle_delta, rx, rz)
                        end

                        -- outer points:
                        rx = inputs.size.x + inputs.thickness
                        rz = inputs.size.z + inputs.thickness
                        for j = segments_per_rail, 0, -1 do
                            new_point(j, outer_start_angle, outer_angle_delta, rx, rz)
                        end

                        return points
                    end

                    local new_mesh = Primitives.polygon(gen_points(direction))
                    Ops.extrude(all_faces_selection, 1, new_mesh)

                    -- Now generate the extrusion caps by going the other direction
                    local face_mesh = Primitives.polygon(gen_points(-direction))
                    Ops.merge(new_mesh, face_mesh)

                    if i == 0 then
                        out_mesh = new_mesh
                    else
                        Ops.merge(out_mesh, new_mesh)
                    end
                end

                return {
                    out_mesh = out_mesh
                }
            end,
            inputs = {
                P.v3("pos", vector(0, 0, 0)),
                P.v3("size", vector(1, 0, 1)),
                P.scalar("thickness", {default = 1, min = 0, soft_max = 10}),
                P.scalar("turns", {default = 1, min = 0, soft_max = 10}),
                P.scalar("wire_gap", {default = 0.1, min = 0, soft_max = 10}),
                P.scalar_int("segments", {default = 36, min = 1, soft_max = 360}),
                P.scalar_int("num_pairs", {default = 5, min = 1, soft_max = 33}),
                P.enum("direction", {"Clockwise", "Counter-Clockwise"}, 0)
            },
            outputs = {P.mesh("out_mesh")},
            returns = "out_mesh"
        }
    }
)
