local P = require("params")
local NodeLibrary = require("node_library")

NodeLibrary:addNodes(
    {
        RPM = {
            label = "RPM",
            op = function(inputs)
                -- rpm * seconds * 360deg/rot / 60sec/min
                local direction = inputs.direction == "Clockwise" and -1 or 1
                return {
                   degrees = direction * inputs.rpm * os.clock() * 60
                }
            end,
            inputs = {
                P.scalar("rpm", {default = 0.5, soft_max = 100}),
                P.enum("direction", {"Clockwise", "Counter-Clockwise"}, 0),
            },
            outputs = {P.scalar("degrees")},
            returns = "degrees",
        }
    }
)
