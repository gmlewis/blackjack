local P = require("params")
local NodeLibrary = require("node_library")
local V = require("vector_math")

local POINTS_IN_INVOLUTE_CURVE = 20

local rotate_point = function(pnt, theta)
    local c = math.cos(theta)
    local s = math.sin(theta)
    local x = c * pnt.x - s * pnt.z
    local z = s * pnt.x + c * pnt.z
    return vector(x, pnt.y, z)
end

local involute_xy = function(radius, theta)
    local c = math.cos(theta)
    local s = math.sin(theta)
    local x = radius * (c + theta * s)
    local z = radius * (s - theta * c)
    return vector(x, 0, z)
end

local involute_theta = function(radius, distance)
   local x = distance / radius
   return math.sqrt(x*x - 1)
end

local animate_gear = function(gear_angle, params)
   if params.speed_units == "TeethPerSecond" then
      params.rpm = params.speed
      return gear_angle + params.direction * 2 * math.pi * params.speed * os.clock() / params.num_teeth
   end
   -- RotationsPerMinute
   params.rpm = params.speed
   return gear_angle + params.direction * 2 * math.pi * params.speed * os.clock() / 60
end

local gen_pivot_center = function(params)
   local gear_angle = math.rad(params.pivot_rotation + params.gear_rotation)
   if params.speed ~= 0 then
       gear_angle = animate_gear(gear_angle, params)
   end
   return function(face)
      local new_face = {}
      for i = 1, #face do
         local pt = rotate_point(face[i], gear_angle)
         table.insert(new_face, params.pos + pt)
      end
      return new_face
   end
end

local gen_pivot_at_radius = function(params, radius)
   local gear_angle = math.rad(params.gear_rotation)
   local pivot_angle = math.rad(params.pivot_rotation)
   if params.speed ~= 0 then
       gear_angle = animate_gear(gear_angle, params)
   end
   return function(face)
      local new_face = {}
      for i = 1, #face do
         -- rotate the gear
         local pt = rotate_point(face[i], gear_angle)
         -- translate to pivot point
         pt = pt - vector(-radius,0,0)
         -- rotate around pivot point
         pt = rotate_point(pt, pivot_angle)
         table.insert(new_face, params.pos + pt)
      end
      return new_face
   end
end

local pivot_lookup_func = {
    Center=gen_pivot_center,
    RootRadius=function(params) return gen_pivot_at_radius(params, params.root_radius) end,
    BaseRadius=function(params) return gen_pivot_at_radius(params, params.base_radius) end,
    PitchRadius=function(params) return gen_pivot_at_radius(params, params.pitch_radius) end,
    OuterRadius=function(params) return gen_pivot_at_radius(params, params.outer_radius) end,
}

local gen_rotate_final_face_func = function(params)
    return pivot_lookup_func[params.pivot](params)
end

local generate_involute_verts = function(params)
    local pressure_angle = math.rad(params.pressure_angle)
    local pitch_radius = params.module * params.num_teeth / 2
    local base_radius = pitch_radius * math.cos(pressure_angle)
    local root_radius = pitch_radius - 1.25 * params.module
    local outer_radius = pitch_radius + params.module

    -- work out the angular extent of the tooth on the base radius
    local backlash = (0.25 * math.pi * math.cos(pressure_angle)) / (params.num_teeth * params.module)
    local pitch_point = involute_xy(base_radius, involute_theta(base_radius, pitch_radius))
    local face_angle = math.atan2(pitch_point.z, pitch_point.x)
    local backlash_angle = backlash / (2 * pitch_radius)
    local center_angle = math.pi/(2*params.num_teeth) + face_angle - backlash_angle

    -- work out the angles over which the involute will be used
    local start_angle = involute_theta(base_radius, math.max(base_radius, root_radius))
    local stop_angle = involute_theta(base_radius, outer_radius)
    local d_theta = (stop_angle - start_angle) / POINTS_IN_INVOLUTE_CURVE

    local involute = {}
    -- start of tooth
    if (root_radius < base_radius) then
       local pt = rotate_point(vector(root_radius, 0, 0), -center_angle)
       table.insert(involute, pt)
    end

    -- lower tooth face
    local angle = start_angle
    for i = 0, POINTS_IN_INVOLUTE_CURVE do
       local pt = rotate_point(involute_xy(base_radius, angle), -center_angle)
       table.insert(involute, pt)
       angle = angle + d_theta
    end
    -- Note the number of facets up to this point before creating the outer edge arc
    local num_facets = #involute

    -- create arc across outer tooth edge
    local arc_tip = involute[#involute]
    local arc_tip_angle = math.abs(math.atan2(arc_tip.z, arc_tip.x))
    for i = 1, params.resolution-1 do
       local phi = -arc_tip_angle + 2*arc_tip_angle*i/params.resolution
       table.insert(involute, rotate_point(vector(outer_radius, 0, 0), phi))
    end

    -- upper tooth face (mirror the lower point)
    for i = 1, num_facets do
       local pt = involute[num_facets-i+1]
       table.insert(involute, vector(pt.x, 0, -pt.z))
    end

    params.involute = involute
    params.base_radius = base_radius
    params.pitch_radius = pitch_radius
    params.root_radius = root_radius
    params.outer_radius = outer_radius
    params.rotate_final_face = gen_rotate_final_face_func(params)
end

local generate_side_of_tooth = function(faces, last_side_verts, new_side_verts, first_iteration, theta, pt, params)
    local total_segments = (params.num_elbows + 1) * params.vertical_resolution
    for segment = 0, total_segments do
        local y = params.gear_length * segment / total_segments -- 0..top (max at midpoint)
        local t = (segment/params.vertical_resolution) % 2
        local helix_rotation = params.max_helix_rotation - math.abs(params.max_helix_rotation * (t - 1))
        local vert = vector(0, y, 0) + rotate_point(pt, -theta - params.direction*helix_rotation)
        table.insert(new_side_verts, vert)

        if segment > 0 and not first_iteration then
           -- create two faces
           table.insert(faces, params.rotate_final_face({
              last_side_verts[segment], -- 1-indexed
              new_side_verts[segment],
              new_side_verts[segment + 1],
           }))
           table.insert(faces, params.rotate_final_face({
              last_side_verts[segment],
              new_side_verts[segment + 1],
              last_side_verts[segment + 1],
           }))
        end
    end
end

-- note that hole_generator_outline does not rotate the outline into its final place
-- since the vertices are reused for positioning of the hole.
local hole_generator_outline = function(faces, top_tooth, bottom_tooth, last_side_verts, params)
   local tpt1 = top_tooth[1]
   local tpt2 = top_tooth[#top_tooth]
   local top_face = { tpt1, tpt2 }
   local bpt1 = bottom_tooth[#bottom_tooth]
   local bpt2 = bottom_tooth[1]
   local bot_face = { bpt2, bpt1 }
   for j = 0, params.resolution-1 do
      local tnext2 = rotate_point(tpt2, -params.gap_delta)
      table.insert(top_face, tnext2)
      tpt2 = tnext2
      local bnext2 = rotate_point(bpt2, -params.gap_delta)
      table.insert(bot_face, 1, bnext2)  -- reversed to that normal is correct
      bpt2 = bnext2
   end
   return top_face, bot_face
end

local hole_generator_none = function(faces, top_tooth, bottom_tooth, last_side_verts, params)
   local top_face, bot_face = hole_generator_outline(faces, top_tooth, bottom_tooth, last_side_verts, params)
   table.insert(top_face, params.top)
   table.insert(bot_face, vector(0,0,0))

   table.insert(faces, params.rotate_final_face(top_face))
   table.insert(faces, params.rotate_final_face(bot_face))
end

local hole_generator_with_hole = function(faces, top_tooth, bottom_tooth, last_side_verts, params, hole_outline_func)
   local top_face, bot_face = hole_generator_outline(faces, top_tooth, bottom_tooth, last_side_verts, params)

   local top = params.top
   local tpt1 = top_tooth[1]
   local start_theta = math.atan2(tpt1.z, tpt1.x)
   local tpt2 = top_tooth[#top_tooth]
   local gap_arc_length = params.resolution * params.gap_delta
   local end_theta = math.atan2(tpt2.z, tpt2.x) - gap_arc_length

   while end_theta > start_theta do
      end_theta = end_theta - 2*math.pi
   end

   for j = 0, params.resolution do
      local t = j / params.resolution -- t = 0..1
      local theta1 = start_theta + (1 - t) * (end_theta - start_theta)
      table.insert(top_face, top + rotate_point(hole_outline_func(-theta1), theta1))
      local theta2 = start_theta + t * (end_theta - start_theta)
      table.insert(bot_face, rotate_point(hole_outline_func(-theta2), theta2))
   end

   for j = 0, params.resolution-1 do
      -- add a new quad face for the inner circular hole
      local v1 = top_face[#top_face-params.resolution+j+1]
      local v2 = top_face[#top_face-params.resolution+j]
      local v3 = bot_face[#bot_face-j]
      local v4 = bot_face[#bot_face-j-1]
      table.insert(faces, params.rotate_final_face({v1, v2, v3, v4}))
   end

   table.insert(faces, params.rotate_final_face(top_face))
   table.insert(faces, params.rotate_final_face(bot_face))
end

local hole_generator_circular = function(faces, top_tooth, bottom_tooth, last_side_verts, params)
   local r = params.hole_radius
   local hole_outline_func = function(theta) return vector(r,0,0) end
   hole_generator_with_hole(faces, top_tooth, bottom_tooth, last_side_verts, params, hole_outline_func)
end

local hole_generator_squared = function(faces, top_tooth, bottom_tooth, last_side_verts, params)
   local r = params.hole_radius
   local pi_over_4 = math.pi/4
   local pi_over_2 = math.pi/2
   local hole_outline_func = function(theta)
      local phi = theta % pi_over_2 - pi_over_4
      return vector(r/math.cos(phi),0,0)
   end
   hole_generator_with_hole(faces, top_tooth, bottom_tooth, last_side_verts, params, hole_outline_func)
end

local hole_generator_hexagonal = function(faces, top_tooth, bottom_tooth, last_side_verts, params)
   local r = params.hole_radius
   local pi_over_6 = math.pi/6
   local pi_over_3 = math.pi/3
   local hole_outline_func = function(theta)
      local phi = theta % pi_over_3 - pi_over_6
      return vector(r/math.cos(phi),0,0)
   end
   hole_generator_with_hole(faces, top_tooth, bottom_tooth, last_side_verts, params, hole_outline_func)
end

local hole_generator_octagonal = function(faces, top_tooth, bottom_tooth, last_side_verts, params)
   local r = params.hole_radius
   local pi_over_8 = math.pi/8
   local pi_over_4 = math.pi/4
   local hole_outline_func = function(theta)
      local phi = theta % pi_over_4 - pi_over_8
      return vector(r/math.cos(phi),0,0)
   end
   hole_generator_with_hole(faces, top_tooth, bottom_tooth, last_side_verts, params, hole_outline_func)
end

local hole_generator = {
   None=hole_generator_none,
   Hollow=function() end,
   Squared=hole_generator_squared,
   Hexagonal=hole_generator_hexagonal,
   Octagonal=hole_generator_octagonal,
   Circular=hole_generator_circular,
}

local generate_teeth = function(params)
    local helix_length = params.gear_length / 2
    params.top = vector(0, params.gear_length, 0)
    local helix_ratio = math.tan(math.rad(params.helix_angle))
    params.max_helix_rotation = helix_ratio * helix_length / params.outer_radius
    -- The top segment is at (num_elbows + 1) * vertical_resolution, so its helix
    -- phase is (num_elbows + 1) % 2. For an even number of elbows the top ends at
    -- the maximum helix rotation, and the top cap must be rotated to match the
    -- teeth where they actually ended up.
    params.top_helix_rotation = params.max_helix_rotation * ((params.num_elbows + 1) % 2)

    local involute = params.involute
    local involute_start_angle = math.atan2(involute[1].z, involute[1].x)
    local involute_end_angle = math.atan2(involute[#involute].z, involute[#involute].x)
    local involute_arc_angle = math.abs(involute_end_angle - involute_start_angle)
    local gap_arc_angle = 2 * math.pi / params.num_teeth - involute_arc_angle
    params.gap_delta = gap_arc_angle / params.resolution

    local hole_func = hole_generator[params.hole_type]
    params.vertical_resolution = 2 * params.resolution

    local last_side_verts = {}
    local faces = {}
    for i = 0, params.num_teeth - 1 do
        local theta = i * 2 * math.pi / params.num_teeth
        -- note that top_tooth and bottom_tooth are not rotated into final position
        -- since they are reused to create hole positions.
        local top_tooth = {}
        local bottom_tooth = {}

        -- create the tooth
        for j = 1, #involute do
            local rev_pt = involute[#involute - j + 1]
            table.insert(top_tooth, params.top + rotate_point(rev_pt, -theta - params.direction*params.top_helix_rotation))
            local pt = involute[j]
            table.insert(bottom_tooth, rotate_point(pt, -theta))
            -- generate one side of the tooth
            local new_side_verts = {}
            generate_side_of_tooth(faces, last_side_verts, new_side_verts, j==1, theta, rev_pt, params)
            last_side_verts = new_side_verts
        end

        if params.hole_type ~= "Hollow" then
           table.insert(faces, params.rotate_final_face(top_tooth))
           table.insert(faces, params.rotate_final_face(bottom_tooth))
        end

        -- create arc at root_radius across inner tooth edge
        for segment = 1, #last_side_verts-1 do
           local last_pt1 = last_side_verts[segment]
           local last_pt2 = last_side_verts[segment + 1]
           for j = 1, params.resolution do
              local pt1 = rotate_point(last_pt1, -params.gap_delta)
              local pt2 = rotate_point(last_pt2, -params.gap_delta)
              table.insert(faces, params.rotate_final_face({ last_pt1, pt1, pt2 }))
              table.insert(faces, params.rotate_final_face({ last_pt1, pt2, last_pt2 }))
              last_pt1 = pt1
              last_pt2 = pt2
           end
        end

        -- create the top and bottom caps
        hole_func(faces, top_tooth, bottom_tooth, last_side_verts, params)

    end
    return Primitives.mesh_from_faces(faces)
end

-- ---------------------------------------------------------------------------
-- Internal (ring) gear support
-- ---------------------------------------------------------------------------
-- An internal gear cannot be made by simply mirroring an external gear. The
-- ring's inner boundary is the outer boundary of a phantom external gear with
-- the ring tooth count and the internal addendum/dedendum convention (tooth
-- tip 1.25*module inside the pitch circle, root 1.0*module outside it). The
-- phantom's tooth centers are phase-aligned with the mating planet's teeth.
-- The requested clearance is applied as a smooth radial offset. The helpers
-- below build that profile and extrude it with the same herringbone twist used
-- by the external teeth.

-- external_gear_boundary returns the complete clockwise outer boundary of an
-- external involute gear (tooth flanks, tip arcs, and root arcs).
local external_gear_boundary = function(module, pressure_angle_deg, num_teeth, root_radius, outer_radius, resolution)
    local pressure_angle = math.rad(pressure_angle_deg)
    local pitch_radius = module * num_teeth / 2
    local base_radius = pitch_radius * math.cos(pressure_angle)
    local face_angle = math.tan(pressure_angle) - pressure_angle
    local backlash = (0.25 * math.pi * math.cos(pressure_angle)) / (num_teeth * module)
    local backlash_angle = backlash / (2 * pitch_radius)
    local center_angle = math.pi / (2 * num_teeth) + face_angle - backlash_angle

    local start_angle = involute_theta(base_radius, math.max(base_radius, root_radius))
    local stop_angle = involute_theta(base_radius, outer_radius)
    local d_theta = (stop_angle - start_angle) / POINTS_IN_INVOLUTE_CURVE

    local involute = {}
    if root_radius < base_radius then
        table.insert(involute, rotate_point(vector(root_radius, 0, 0), -center_angle))
    end
    local angle = start_angle
    for i = 0, POINTS_IN_INVOLUTE_CURVE do
        table.insert(involute, rotate_point(involute_xy(base_radius, angle), -center_angle))
        angle = angle + d_theta
    end

    local num_facets = #involute
    local arc_tip = involute[#involute]
    local arc_tip_angle = math.abs(math.atan2(arc_tip.z, arc_tip.x))
    for i = 1, resolution - 1 do
        local phi = -arc_tip_angle + 2 * arc_tip_angle * i / resolution
        table.insert(involute, rotate_point(vector(outer_radius, 0, 0), phi))
    end
    for i = 1, num_facets do
        local pt = involute[num_facets - i + 1]
        table.insert(involute, vector(pt.x, 0, -pt.z))
    end

    local start_profile_angle = math.atan2(involute[1].z, involute[1].x)
    local end_profile_angle = math.atan2(involute[#involute].z, involute[#involute].x)
    local involute_arc_angle = math.abs(end_profile_angle - start_profile_angle)
    local gap_arc_angle = 2 * math.pi / num_teeth - involute_arc_angle
    local gap_delta = gap_arc_angle / resolution

    local profile = {}
    for tooth = 0, num_teeth - 1 do
        local tooth_angle = -2 * math.pi * tooth / num_teeth
        -- Walk clockwise: upper root through the tip to the lower root.
        for k = #involute, 1, -1 do
            table.insert(profile, rotate_point(involute[k], tooth_angle))
        end
        local lower_root = rotate_point(involute[1], tooth_angle)
        for j = 1, resolution - 1 do
            table.insert(profile, rotate_point(lower_root, -j * gap_delta))
        end
    end
    return profile, pitch_radius, base_radius
end

-- internal_gear_boundary builds the ring's inner boundary as the outer
-- boundary of a phantom external gear with the ring tooth count. For an
-- internal gear the addendum and dedendum are reversed, so the phantom's
-- root radius is the ring tooth tip and its tip radius is the ring notch.
local internal_gear_boundary = function(params)
    local module = params.module
    local pa = params.pressure_angle
    local num_teeth = params.num_teeth
    local mate_teeth = params.mate_num_teeth
    if mate_teeth < 6 then
        mate_teeth = num_teeth
    end

    local ring_pitch = module * num_teeth / 2
    local ring_base = ring_pitch * math.cos(math.rad(pa))
    local mate_pitch = module * mate_teeth / 2
    local mate_outer = mate_pitch + module

    -- Compensate for an increased ring/planet center distance.
    local nominal_center = ring_pitch - mate_pitch
    local center = params.center_distance
    if center <= 0 then
        center = nominal_center
    end
    local center_delta = math.max(0, center - nominal_center)

    -- Internal gear addendum/dedendum: the tooth tip is 1.25*module inside
    -- the pitch circle and the root/notch is 1.0*module outside it.
    local phantom_root = ring_pitch - 1.25 * module
    local phantom_tip = ring_pitch + module
    local profile = external_gear_boundary(module, pa, num_teeth, phantom_root, phantom_tip, params.resolution)

    local min_r, max_r = math.huge, 0
    for i = 1, #profile do
        local r = math.sqrt(profile[i].x * profile[i].x + profile[i].z * profile[i].z)
        if r < min_r then min_r = r end
        if r > max_r then max_r = r end
    end

    -- Align the phantom's tooth centers with the mating planet's tooth
    -- centers at the ring/planet pitch point. The ring's material teeth are
    -- the phantom's gaps, so they end up half a ring pitch away.
    local mate_pitch_angle = 2 * math.pi / mate_teeth
    local ring_pitch_angle = 2 * math.pi / num_teeth
    local mate_rotation = math.rad(params.mate_rotation)
    local phase = ring_pitch_angle * math.fmod(mate_rotation / mate_pitch_angle, 1)
    if phase < 0 then
        phase = phase + ring_pitch_angle
    end

    return profile, min_r, max_r, ring_pitch, ring_base, mate_pitch, mate_outer, phase, center_delta
end

local generate_internal_teeth = function(params)
    local profile, min_r, max_r, ring_pitch, ring_base, mate_pitch, mate_outer, phase, center_delta = internal_gear_boundary(params)

    -- Apply the clearance as a smooth radial offset. A radial offset of d
    -- produces a normal flank clearance of d*sin(pressure_angle), so use
    -- clearance/sin(pressure_angle) for the requested flank clearance. The
    -- center_delta term compensates for the increased ring/planet center
    -- distance. A radial offset is smooth by construction and avoids the
    -- spikes/notches that polygon-offset joins produce at the sharp tooth
    -- corners.
    local radial_clearance = params.clearance / math.sin(math.rad(params.pressure_angle))
    local radial_offset = center_delta + radial_clearance
    for i = 1, #profile do
        local p = profile[i]
        local r = math.sqrt(p.x * p.x + p.z * p.z)
        if r > 0 then
            local s = (r + radial_offset) / r
            profile[i] = vector(p.x * s, 0, p.z * s)
        end
    end
    local outer_radius = max_r + radial_offset + params.outer_thickness

    local vertical_resolution = 2 * params.resolution
    local total_segments = (params.num_elbows + 1) * vertical_resolution
    local max_helix_mate = math.tan(math.rad(params.helix_angle)) * (params.gear_length / 2) / mate_outer
    local ring_scale = mate_pitch / ring_pitch

    params.rotate_final_face = gen_rotate_final_face_func(params)

    local inner = {}
    local outer = {}
    local num_points = #profile
    for segment = 0, total_segments do
        local y = params.gear_length * segment / total_segments
        local t = (segment / vertical_resolution) % 2
        local helix_rotation = max_helix_mate - math.abs(max_helix_mate * (t - 1))
        local rot = phase - params.direction * ring_scale * helix_rotation
        inner[segment] = {}
        outer[segment] = {}
        for i = 1, num_points do
            local p = profile[i]
            local r = math.sqrt(p.x * p.x + p.z * p.z)
            local pi = rotate_point(p, rot)
            local s = 1
            if r > 0 then s = outer_radius / r end
            local po = rotate_point(vector(p.x * s, 0, p.z * s), rot)
            inner[segment][i] = vector(pi.x, y, pi.z)
            outer[segment][i] = vector(po.x, y, po.z)
        end
    end

    local faces = {}
    for segment = 0, total_segments - 1 do
        for i = 1, num_points do
            local j = (i % num_points) + 1
            table.insert(faces, params.rotate_final_face({
                inner[segment][i], inner[segment + 1][i], inner[segment + 1][j], inner[segment][j],
            }))
            table.insert(faces, params.rotate_final_face({
                outer[segment][i], outer[segment][j], outer[segment + 1][j], outer[segment + 1][i],
            }))
        end
    end
    for i = 1, num_points do
        local j = (i % num_points) + 1
        table.insert(faces, params.rotate_final_face({
            inner[0][i], inner[0][j], outer[0][j], outer[0][i],
        }))
    end
    for i = 1, num_points do
        local j = (i % num_points) + 1
        table.insert(faces, params.rotate_final_face({
            inner[total_segments][i], outer[total_segments][i], outer[total_segments][j], inner[total_segments][j],
        }))
    end

    return Primitives.mesh_from_faces(faces), ring_base, ring_pitch, min_r + radial_offset, outer_radius
end

NodeLibrary:addNodes(
    {
        -- This is a herringbone gear model, inspired by:
        -- https://www.stlgears.com/generators/3dprint
        -- https://geargenerator.com/
        -- https://geargenerator.com/beta/
        -- https://github.com/jamesgregson/gear_generator/blob/master/templates/gears.js
        HerringboneGear = {
            label = "Herringbone Gear",
            op = function(params)
                if params.helix_angle < 0 then
                   params.helix_angle = 0
                end
                if params.helix_angle > 45 then
                   params.helix_angle = 45
                end
                params.rpm = 0
                params.direction = params.direction == "Clockwise" and -1 or 1

                if params.gear_side == "Internal" then
                    if params.num_teeth < 6 then
                        print("invalid internal gear: num_teeth=", params.num_teeth)
                        return {
                            out_mesh = {},
                            base_radius = 0,
                            pitch_radius = 0,
                            outer_radius = 0,
                            root_radius = 0,
                            rpm = params.rpm,
                        }
                    end
                    local out_mesh, base_radius, pitch_radius, outer_radius, root_radius = generate_internal_teeth(params)
                    return {
                        out_mesh = out_mesh,
                        base_radius = base_radius,
                        pitch_radius = pitch_radius,
                        outer_radius = outer_radius,
                        root_radius = root_radius,
                        rpm = params.rpm,
                    }
                end

                generate_involute_verts(params)
                if params.root_radius <= 0 or (params.hole_type ~= "None" and params.hole_type ~= "Hollow" and params.hole_radius >= params.root_radius) then
                   print("invalid gear: root_radius=", params.root_radius, "hole_radius=", params.hole_radius)
                   return {
                      out_mesh = {},
                      base_radius = params.base_radius,
                      pitch_radius = params.pitch_radius,
                      outer_radius = params.outer_radius,
                      root_radius = params.root_radius,
                      rpm = params.rpm
                   }
                end
                local out_mesh = generate_teeth(params)
                return {
                    out_mesh = out_mesh,
                    base_radius = params.base_radius,
                    pitch_radius = params.pitch_radius,
                    outer_radius = params.outer_radius,
                    root_radius = params.root_radius,
                    rpm = params.rpm
                }
            end,
            inputs = {
                -- pos sets the pivot point in 3D space.
                P.v3("pos", vector(0, 0, 0)),
                -- A non-zero speed causes the gear to rotate in Blackjack with the "speed_units" units.
                P.scalar("speed", {default = 0}),
                -- "speed_units" represents the units that "speed" uses for animating rotation of the gear.
                P.enum("speed_units", {"TeethPerSecond", "RotationsPerMinute"}, 0),
                -- Use the "PitchRadius" for meshing with other gears.
                -- "Center" is the center of the gear.
                -- "RootRadius" is the inner-most radius of each tooth.
                -- "BaseRadius" is the start of the involute curve of each tooth.
                -- "PitchRadius" is the perfect meshing distance of the gear from the center.
                -- "OuterRadius" is the extreme outer edge radius of each tooth.
                P.enum("pivot", {"Center", "RootRadius", "BaseRadius", "PitchRadius", "OuterRadius"}, 3),
                P.scalar("pivot_rotation", {default = 0, soft_max = 360}),
                P.scalar("gear_rotation", {default = 0, soft_max = 360}),
                -- According to: https://www.stlgears.com/theory#module
                -- "The module is a crucial factor in gear design as it determines the overall size of the gear.
                --  The module affects the size of the gear teeth, which is represented by the distance between
                --  the pitch radius and the tip of the tooth (addendum radius)."
                -- Common values are:
                -- 75,70,65,60,55,50,45,42,39,36,33,30,27,24,22,20,18,16,15,14,13,12,11,10,9,8,7,
                -- 6.5,6,5.5,5,4.5,4,3.75,3.5,3.25,3,2.75,2.5,2.25,2,1.75,1.5,1.25,1,0.9,0.8,0.7,
                -- 0.6,0.5,0.4,0.3
                -- According to: https://github.com/chrisspen/gears, common values are:
                -- 60,50,40,32,25,20,16,12,10,8,6,5,4,3,2.5,2,1.5,1.25,1,0.9,0.8,0.7,0.6,0.5,0.4,
                -- 0.3,0.25,0.20,0.16,0.12,0.10,0.08,0.06,0.05
                P.scalar("module", {default = 3, min = 0.01, soft_max = 75}),
                P.enum("direction", {"Clockwise", "Counter-Clockwise"}, 0),
                P.scalar_int("num_teeth", {default = 13, min = 6, soft_max = 150}),
                P.scalar_int("num_elbows", {default = 1, min = 1, soft_max = 10}),
                P.scalar("helix_angle", {default = 30, min = 0, soft_max = 45}),
                -- "resolution" controls the number of points in each radial curved section.
                P.scalar_int("resolution", {default = 1, min = 1, soft_max = 100}),
                -- "pressure_angle" is in degrees:
                P.scalar("pressure_angle", {default = 20, min = 1, soft_max = 35}),
                P.scalar("gear_length", {default = 30, min = 0.01, soft_max = 100}),
                P.enum("hole_type", {"None", "Hollow", "Squared", "Hexagonal", "Octagonal", "Circular"}, 0),
                -- The following are only useful for hole_types: "Squared", "Hexagonal", "Octagonal", "Circular":
                -- For "Squared", "Hexagonal", and "Octagonal", "hole_radius" refers the inner radius, not the polygon's edge length.
                P.scalar("hole_radius", {default = 0, min = 0, soft_max = 100}),
                -- Internal (ring) gear support. With gear_side="Internal", the
                -- mesh is an inward-facing herringbone ring gear whose inner
                -- profile is generated as the envelope of the mating planet.
                P.enum("gear_side", {"External", "Internal"}, 0),
                -- Number of teeth on the mating planet gear (used only when internal).
                P.scalar_int("mate_num_teeth", {default = 0, min = 0, soft_max = 150}),
                -- Rotation of the mating planet at y=0, in degrees (used only when internal).
                P.scalar("mate_rotation", {default = 0, soft_max = 360}),
                -- Center distance between the ring and the mating planet (0 = pitch radii difference).
                P.scalar("center_distance", {default = 0, min = 0, soft_max = 500}),
                -- Radial clearance added to the internal profile (used only when internal).
                P.scalar("clearance", {default = 0.25, min = 0, soft_max = 10}),
                -- Radial wall thickness from the deepest notch to the smooth outer cylinder.
                P.scalar("outer_thickness", {default = 10, min = 0.01, soft_max = 100}),
            },
            outputs = {P.mesh("out_mesh"), P.scalar("base_radius"), P.scalar("pitch_radius"), P.scalar("outer_radius"), P.scalar("root_radius"), P.scalar("rpm")},
            returns = "out_mesh"
        }
    }
)
