local P = require("params")
local F = require("font_library")
local NodeLibrary = require("node_library")
local V = require("vector_math")

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

local split_path = function(d)
    local cmds = {}
    while string.len(d) > 0 do
        local _, j, m, n = string.find(d, "^([MLCQZ])([%d%.%-,%s]*)")
        if m ~= nil then
            local params = parse_parameters(n)
            table.insert(cmds, {C=m, P=params})
            d = string.sub(d, j+1)
        else
            print("unsupported SVG path command:", d)
        end
    end
    return cmds
end

local translate_glyph = function(x, y, d_in)
    local d = ""
    local cmds = split_path(d_in)
    for _, cmd in pairs(cmds) do
        d = d .. cmd.C
        if cmd.P ~= nil then
            for i, v in pairs(cmd.P) do
                if i % 2 == 1 then
                    if i > 1 then
                        d = d .. " "
                    end
                    d = d .. (x + v)
                else
                    d = d .. " " .. (y + v)
                end
            end
        end
    end
    return d
end

local render = function(font, text)
    local d = ""
    local x = 0
    local y = 0
    while string.len(text) > 0 do
        local char = string.sub(text, 1, 1)
        local char_key = font.lookup[char]
        if char == "\n" then
            x = 0
            y = y - font.units_per_em + font.descent
        elseif char_key == nil then
            x = x + font.horiz_adv_x
        else
            local glyph = font.glyphs[char_key]
            if glyph ~= nil then
                d = d .. translate_glyph(x, y, glyph.d)
                if glyph.horiz_adv_x > 0 then
                    x = x + glyph.horiz_adv_x
                else
                    x = x + font.horiz_adv_x
                end
            else
                x = x + font.horiz_adv_x
            end
        end
        text = string.sub(text, 2)
    end
    return d
end

NodeLibrary:addNodes(
    {
        SVGFont = {
            label = "SVG Font",
            op = function(inputs)
                local font = F:getFont(inputs.font)
                local d = render(font, inputs.text)
                return {
                    d = d
                }
            end,
            inputs = {
                P.lua_str("text"),
                P.enum("font", F:listFonts(), 0)
            },
            outputs = {P.lua_str("d")},
            returns = "d"
        }
    }
)
