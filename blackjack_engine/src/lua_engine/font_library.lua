-- Copyright (C) 2023 setzer22 and contributors
--
-- This Source Code Form is subject to the terms of the Mozilla Public
-- License, v. 2.0. If a copy of the MPL was not distributed with this
-- file, You can obtain one at https://mozilla.org/MPL/2.0/.

local FontLibrary = {
    fonts = {}
}

function FontLibrary:addFonts(fonts)
    assert(type(fonts) == "table")

    for k, v in pairs(fonts) do
        print("FontLibrary:addFonts: registering font:", k)
        self.fonts[k] = v
    end
end

function FontLibrary:listFonts()
    local fonts = {}
    for k, _ in pairs(self.fonts) do
        table.insert(fonts, k)
    end
    table.sort(fonts, function(a,b) return a < b end)
    return fonts
end

function FontLibrary:getFont(font_name)
    local font = self.fonts[font_name]
    if font.glyphs == nil then
        print("FontLibrary:getFont: font_name=", font_name)
        print("font.glyphs is nil - loading data...")
        print("font.data_file=", font.data_file)
        local f = assert(loadfile(font.data_file))
        f() -- execute data file
        font = self.fonts[font_name]  -- reload the font data
        print("got font.glyphs:", #font.glyphs)
    end
    return font
end

function FontLibrary:scriptPath()
    local lines = debug.traceback()
    local location = ""
    for s in lines:gmatch("[^\r\n]+%s*") do
        local _, j, m = string.find(s, '^%[string "(.*.lua)"%]:')
        if m ~= nil then
            location = m
        else
            local _, j, m = string.find(s, "^(.*.lua):")
            if m ~= nil then
                location = m
            end
        end
    end
    return location .. ".data"
end

return FontLibrary
