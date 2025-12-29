#!/bin/bash -ex
# -*- compile-command: "./gen-lua-fonts.sh"; -*-
go run ${HOME}/go/src/github.com/gmlewis/go-fonts/cmd/font2lua/main.go ${HOME}/go/src/github.com/gmlewis/go-fonts-*/fonts/*/*.svg
go run gen-fonts.go
