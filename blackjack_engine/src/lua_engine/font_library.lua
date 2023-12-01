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
        require("../font_data/font_" .. font_name)
        font = self.fonts[font_name]  -- reload the font data
        local lookup = {}
        for k, v in pairs(font.glyphs) do
            lookup[v.char] = k
        end
        font.lookup = lookup
    end
    return font
end

return FontLibrary
