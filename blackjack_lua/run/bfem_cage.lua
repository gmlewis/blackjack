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
                local coil_height = 2 * inputs.turns * (inputs.wire_width + inputs.wire_gap) + inputs.wire_width
                local extrude_height = coil_height + inputs.wire_gap + inputs.wire_width

                local backbone = Primitives.line(vector(0,0,0), vector(0,1,0), 1)

                local rail_angle_delta = math.pi / inputs.num_pairs
                local inner_radius = inputs.size.x + inputs.wire_width/2 + inputs.wire_gap
                local outer_radius = inner_radius + inputs.radial_thickness

                local inner_dtheta = math.asin(inputs.wire_gap / (2 * inner_radius))
                local outer_dtheta = math.asin(inputs.wire_gap / (2 * outer_radius))

                local delta_y = inputs.num_pairs > 1 and
                    (inputs.wire_gap + inputs.wire_width) / (inputs.num_pairs - 1) or 0

                local function reverse(points)
                    local rev = {}
                    for i=#points, 1, -1 do
                        rev[#rev+1] = points[i]
                    end
                    return rev
                end

                local function new_point(j, start_angle, angle_delta, r, y, points)
                    local angle = start_angle + j * angle_delta
                    local x = inputs.pos.x + r * math.cos(angle)
                    local z = inputs.pos.z + r * math.sin(angle)
                    local point = vector(x, y, z) -- y is "up"
                    table.insert(points, point)
                end

                -- first, generate the long axial connectors
                local axial_connector_top_ys = {}
                local max_axial_connector_top_ys = extrude_height + lower_y + (inputs.num_pairs-1) * delta_y
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
                    axial_connector_top_ys[i+1] = y + extrude_height

                    local function gen_points()
                        local points = {}
                        -- inner points:
                        for j = 0, segments_per_rail do
                            -- new_point(j, inner_start_angle, inner_angle_delta, inner_radius, y, points)
                            new_point(j, inner_start_angle, inner_angle_delta, inner_radius, lower_y - inputs.front_thickness, points)
                        end
                        -- outer points:
                        for j = segments_per_rail, 0, -1 do
                            -- new_point(j, outer_start_angle, outer_angle_delta, outer_radius, y, points)
                            new_point(j, outer_start_angle, outer_angle_delta, outer_radius, lower_y - inputs.front_thickness, points)
                        end
                        return points
                    end

                    local new_mesh = Primitives.polygon(gen_points())  -- this is the axial connector itself

                    if i == 0 then
                        -- final output connector of coil 1 (at outer edge)
                        Ops.extrude_with_caps(all_faces_selection, extrude_height + inputs.connector_length + inputs.front_thickness + inputs.back_thickness, new_mesh)
                        out_mesh = new_mesh
                    else
                        -- Ops.extrude_with_caps(all_faces_selection, max_axial_connector_top_ys-y, new_mesh)
                        Ops.extrude_with_caps(all_faces_selection, max_axial_connector_top_ys-lower_y + inputs.front_thickness + inputs.back_thickness, new_mesh)
                        Ops.merge(out_mesh, new_mesh)
                    end
                end

                -- Pre-calculate important values for all coil connectors
                local rotations = {}
                local line_lengths = {}
                local ys = {}
                for i = 1, inputs.num_pairs do
                    ys[i] = lower_y + (i-1) * delta_y
                    -- Create the connectors to the ends of the coils:
                    rotations[i] = -math.pi * (i-1) / inputs.num_pairs
                    line_lengths[i] = (inputs.num_pairs - i + 1) * (inputs.wire_width + inputs.wire_gap)
                end

                local function gen_points(y, inner_start_angle, inner_angle_delta)
                    local points = {}
                    -- inner points:
                    for j = 0, segments_per_rail do
                        new_point(j, inner_start_angle, inner_angle_delta, inner_radius, y, points)
                    end
                    return points
                end

                -- now generate both front/bottom and back/top connectors to the helices
                for i = 1, inputs.num_pairs do
                    local y = ys[i] + inputs.wire_width
                    local rotation = rotations[i]
                    local line_length = line_lengths[i]
                    local inner_start_angle = -(i-1) * rail_angle_delta
                    local inner_end_angle = -(i-2) * rail_angle_delta - 2*inner_dtheta
                    local inner_angle_delta = (inner_end_angle - inner_start_angle) / segments_per_rail
                    local points = gen_points(y, inner_start_angle, inner_angle_delta)

                    local connector_radius = inner_radius - line_length
                    local connector_dtheta = math.asin(inputs.wire_gap / (2 * connector_radius))
                    local sx = inputs.pos.x + connector_radius * math.cos(rotation)
                    local sz = inputs.pos.z + connector_radius * math.sin(rotation)

                    -- calculate the final point that will be wire_width away from the next wire
                    -- this is the front/bottom of the design

                    local t_angle = rotation + math.asin(inputs.wire_width / connector_radius)
                    local tx = inputs.pos.x + connector_radius * math.cos(t_angle)
                    local tz = inputs.pos.z + connector_radius * math.sin(t_angle)
                    table.insert(points, vector(tx, y, tz))
                    table.insert(points, vector(sx, y, sz))

                    -- this connects the bottom/front helix to the axial connector
                    local bottom_thickness = ys[i] - lower_y + inputs.wire_width + inputs.front_thickness
                    local face = Primitives.polygon(points)
                    Ops.extrude_with_caps(all_faces_selection, bottom_thickness, face)
                    Ops.merge(out_mesh, face)

                    -- add a vertical post to the back/top of the design for connecting the coils
                    -- this is the back/top of the design
                    -- these are the odd coils (1, 3, 5, etc.)

                    local top_angle = rotation
                    if i > 1 then
                        -- half wire-width at outer radius in radians
                        local top_hww = 0.5 * math.asin(inputs.wire_width / (2 * outer_radius))
                        top_angle = rotation - rail_angle_delta/2 + top_hww
                    end
                    local top_helix_y = coil_height - inputs.wire_width + ys[i]
                    -- local top_helix_y = coil_height - (max_axial_connector_top_ys-axial_connector_top_ys[i]) + ys[i]
                    local sx4 = inputs.pos.x + connector_radius * math.cos(top_angle)
                    local sz4 = inputs.pos.z + connector_radius * math.sin(top_angle)
                    local sx1 = sx4 + inputs.wire_width * math.cos(top_angle - math.pi/2)
                    local sz1 = sz4 + inputs.wire_width * math.sin(top_angle - math.pi/2)
                    local sx2 = inputs.pos.x + (connector_radius + inputs.wire_width) * math.cos(top_angle)
                    local sz2 = inputs.pos.z + (connector_radius + inputs.wire_width) * math.sin(top_angle)
                    local sx3 = sx2 + inputs.wire_width * math.cos(top_angle - math.pi/2)
                    local sz3 = sz2 + inputs.wire_width * math.sin(top_angle - math.pi/2)

                    -- this is the "up" part of the "up-and-over" connector on the back/top of the design for the odd coils:
                    -- note the the "up" part for the last odd coil is simply the wire width.
                    local points = {
                        vector(sx1, top_helix_y, sz1),
                        vector(sx4, top_helix_y, sz4),
                        vector(sx2, top_helix_y, sz2),
                        vector(sx3, top_helix_y, sz3),
                    }
                    local face = Primitives.polygon(points)
                    -- local over_height = axial_connector_top_ys[(inputs.num_pairs - i) % inputs.num_pairs + 1] - top_helix_y
                    local over_height = max_axial_connector_top_ys - top_helix_y + inputs.back_thickness  -- flat top/back (connector side)
                    Ops.extrude_with_caps(all_faces_selection, over_height, face)
                    Ops.merge(out_mesh, face)
                    -- this is the "over" part of the "up-and-over" connector on the back/top of the design for the odd coils:
                    local over_start_angle = -(i) * rail_angle_delta
                    local points = gen_points(top_helix_y + over_height, over_start_angle, inner_angle_delta)
                    table.insert(points, vector(sx2, top_helix_y + over_height, sz2))
                    table.insert(points, vector(sx3, top_helix_y + over_height, sz3))
                    local face = Primitives.polygon(points)
                    local top_thickness = over_height-inputs.wire_width-inputs.wire_gap
                    if i == inputs.num_pairs then
                        top_thickness = over_height
                    end
                    Ops.extrude_with_caps(all_faces_selection, top_thickness, face)
                    Ops.merge(out_mesh, face)

                    -- for all but the coil 1, the helix needs to be connected to the shifted back/top "up-and-over" connector
                    if i > 1 then
                        local sx5 = inputs.pos.x + connector_radius * math.cos(rotation)
                        local sz5 = inputs.pos.z + connector_radius * math.sin(rotation)
                        local sx6 = inputs.pos.x + (connector_radius + inputs.wire_width) * math.cos(rotation)
                        local sz6 = inputs.pos.z + (connector_radius + inputs.wire_width) * math.sin(rotation)
                        local points = {
                            vector(sx5, top_helix_y, sz5),
                            vector(sx6, top_helix_y, sz6),
                            vector(sx2, top_helix_y, sz2),
                            vector(sx4, top_helix_y, sz4),
                        }
                        local face = Primitives.polygon(points)
                        Ops.extrude_with_caps(all_faces_selection, inputs.wire_width, face)
                        Ops.merge(out_mesh, face)
                    end

                    -- second connection for pair directly opposite first connection
                    -- this is the front/bottom of the design

                    local points = gen_points(y, inner_start_angle + math.pi, inner_angle_delta, 1)
                    local sx = inputs.pos.x + connector_radius * math.cos(rotation + math.pi)
                    local sz = inputs.pos.z + connector_radius * math.sin(rotation + math.pi)
                    local tx = inputs.pos.x + connector_radius * math.cos(t_angle + math.pi)
                    local tz = inputs.pos.z + connector_radius * math.sin(t_angle + math.pi)
                    table.insert(points, vector(tx, y, tz))
                    table.insert(points, vector(sx, y, sz))

                    local face = Primitives.polygon(points)
                    Ops.extrude_with_caps(all_faces_selection, bottom_thickness, face)
                    Ops.merge(out_mesh, face)

                    -- add a vertical post to the back/top of the design for connecting the coils
                    -- this is the back/top of the design
                    -- these are the even coils (2, 4, 6, etc.)

                    if i >= inputs.num_pairs then  -- final output connector of last coil
                        -- wire-width at outer radius in radians
                        local top_ww = math.asin(inputs.wire_width / (2 * outer_radius))
                        top_angle = rotation - top_ww
                    end
                    local top_helix_y = coil_height - inputs.wire_width + ys[i]
                    local sx4 = inputs.pos.x + connector_radius * math.cos(top_angle + math.pi)
                    local sz4 = inputs.pos.z + connector_radius * math.sin(top_angle + math.pi)
                    local sx1 = sx4 + inputs.wire_width * math.cos(top_angle + math.pi/2)
                    local sz1 = sz4 + inputs.wire_width * math.sin(top_angle + math.pi/2)
                    local sx2 = inputs.pos.x + (connector_radius + inputs.wire_width) * math.cos(top_angle + math.pi)
                    local sz2 = inputs.pos.z + (connector_radius + inputs.wire_width) * math.sin(top_angle + math.pi)
                    local sx3 = sx2 + inputs.wire_width * math.cos(top_angle + math.pi/2)
                    local sz3 = sz2 + inputs.wire_width * math.sin(top_angle + math.pi/2)
                    if i >= inputs.num_pairs then  -- final output connector of last coil
                        local angle_diff = rail_angle_delta - 2*outer_dtheta
                        sx1 = inputs.pos.x + connector_radius * math.cos(top_angle + math.pi - angle_diff)
                        sz1 = inputs.pos.z + connector_radius * math.sin(top_angle + math.pi - angle_diff)
                        sx3 = inputs.pos.x + (connector_radius + inputs.wire_width) * math.cos(top_angle + math.pi - angle_diff)
                        sz3 = inputs.pos.z + (connector_radius + inputs.wire_width) * math.sin(top_angle + math.pi - angle_diff)
                    end
                    local points = {
                        vector(sx1, top_helix_y, sz1),
                        vector(sx4, top_helix_y, sz4),
                        vector(sx2, top_helix_y, sz2),
                        vector(sx3, top_helix_y, sz3),
                    }
                    local face = Primitives.polygon(points)
                    if i >= inputs.num_pairs then  -- final output connector of last coil (closer to center)
                        Ops.extrude_with_caps(all_faces_selection, inputs.wire_width, face)
                        Ops.merge(out_mesh, face)
                        -- generate inner final output connector of last coil with same radial thickness as other connector
                        local angle_diff = rail_angle_delta - 2*outer_dtheta
                        local sx2 = inputs.pos.x + (connector_radius - inputs.radial_thickness) * math.cos(top_angle + math.pi)
                        local sz2 = inputs.pos.z + (connector_radius - inputs.radial_thickness) * math.sin(top_angle + math.pi)
                        local sx3 = inputs.pos.x + (connector_radius - inputs.radial_thickness) * math.cos(top_angle + math.pi - angle_diff)
                        local sz3 = inputs.pos.z + (connector_radius - inputs.radial_thickness) * math.sin(top_angle + math.pi - angle_diff)
                        local points = {
                            vector(sx4, top_helix_y, sz4),
                            vector(sx1, top_helix_y, sz1),
                            vector(sx3, top_helix_y, sz3),
                            vector(sx2, top_helix_y, sz2),
                        }
                        local face = Primitives.polygon(points)
                        Ops.extrude_with_caps(all_faces_selection, inputs.wire_width + inputs.connector_length + inputs.back_thickness, face)
                        Ops.merge(out_mesh, face)
                    else
                        local extrude_amount = 2 * inputs.wire_width + inputs.wire_gap
                        Ops.extrude_with_caps(all_faces_selection, extrude_amount + inputs.back_thickness, face)
                        Ops.merge(out_mesh, face)
                    end
                    if i < inputs.num_pairs then
                        -- this is the "up" part of the "up-and-over" connector on the back/top of the design for the even coils:
                        local points = {
                            vector(sx1, top_helix_y, sz1),
                            vector(sx4, top_helix_y, sz4),
                            vector(sx2, top_helix_y, sz2),
                            vector(sx3, top_helix_y, sz3),
                        }
                        local face = Primitives.polygon(points)
                        -- local over_height = axial_connector_top_ys[(inputs.num_pairs - i) % inputs.num_pairs + 1] - top_helix_y
                        local over_height = max_axial_connector_top_ys - top_helix_y + inputs.back_thickness  -- flat top/back (connector side)
                        Ops.extrude_with_caps(all_faces_selection, over_height, face)
                        Ops.merge(out_mesh, face)
                        -- this is the "over" part of the "up-and-over" connector on the back/top of the design for the even coils:
                        local over_start_angle = -(i) * rail_angle_delta + math.pi
                        local points = gen_points(top_helix_y + over_height, over_start_angle, inner_angle_delta)
                        table.insert(points, vector(sx2, top_helix_y + over_height, sz2))
                        table.insert(points, vector(sx3, top_helix_y + over_height, sz3))
                        local face = Primitives.polygon(points)
                        Ops.extrude_with_caps(all_faces_selection, top_thickness, face)
                        Ops.merge(out_mesh, face)
                    end
                    -- for all but the first connector, the helix needs to be connected to the shifted connector
                    if i > 1 then
                        local sx5 = inputs.pos.x + connector_radius * math.cos(rotation + math.pi)
                        local sz5 = inputs.pos.z + connector_radius * math.sin(rotation + math.pi)
                        local sx6 = inputs.pos.x + (connector_radius + inputs.wire_width) * math.cos(rotation + math.pi)
                        local sz6 = inputs.pos.z + (connector_radius + inputs.wire_width) * math.sin(rotation + math.pi)
                        local points = {
                            vector(sx5, top_helix_y, sz5),
                            vector(sx6, top_helix_y, sz6),
                            vector(sx2, top_helix_y, sz2),
                            vector(sx4, top_helix_y, sz4),
                        }
                        local face = Primitives.polygon(points)
                        Ops.extrude_with_caps(all_faces_selection, inputs.wire_width, face)
                        Ops.merge(out_mesh, face)
                    end
                end

                return {
                    out_mesh = out_mesh
                }
            end,
            inputs = {
                P.v3("pos", vector(0, 0, 0)),  -- pos is lowered by wire_width/2
                P.v3("size", vector(10, 0, 10)),  -- wire_width/2 + wire_gap is added to size.
                P.scalar("connector_length", {default=10, min=0, soft_max = 33}),
                P.scalar("front_thickness", {default = 1, min = 0, soft_max = 10}),
                P.scalar("back_thickness", {default = 1, min = 0, soft_max = 10}),
                P.scalar("radial_thickness", {default = 2, min = 0, soft_max = 10}),
                P.scalar("turns", {default = 1, min = 0, soft_max = 10}),
                P.scalar("wire_gap", {default = 1, min = 0, soft_max = 10}),
                P.scalar("wire_width", {default = 1, min = 0, soft_max = 10}),
                P.scalar_int("segments", {default = 36, min = 1, soft_max = 360}),
                P.scalar_int("num_pairs", {default = 3, min = 1, soft_max = 33}),
            },
            outputs = {P.mesh("out_mesh")},
            returns = "out_mesh"
        }
    }
)
