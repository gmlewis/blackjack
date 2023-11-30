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
    return fonts
end

function FontLibrary:getFont(font_name)
    return self.fonts[font_name]
end

function FontLibrary:scriptPath()
    local str = debug.getinfo(2, "S").source
    return str .. ".data"
end

return FontLibrary
