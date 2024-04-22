// -*- compile-command: "go run gen-fonts.go"; -*-

// gen-fonts generates the Lua scripts in the ../fonts directory from this font_data.
//
// Usage:
//
//	go run gen-fonts.go
package main

import (
	"bytes"
	"fmt"
	"io/fs"
	"log"
	"os"
	"path/filepath"
	"strings"
)

const (
	fontsDir = "../fonts"
)

var (
	startCut = []byte("local glyphs = {")
	endCut   = []byte("F:addFonts(")
)

func main() {
	fileSystem := os.DirFS(".")

	fs.WalkDir(fileSystem, ".", func(path string, d fs.DirEntry, err error) error {
		if err != nil {
			log.Fatal(err)
		}
		if !strings.HasSuffix(path, ".lua") {
			return nil
		}

		log.Printf("Processing file: %v", path)
		return genFont(path)
	})
}

func genFont(path string) error {
	buf, err := os.ReadFile(path)
	if err != nil {
		return err
	}

	startIndex := bytes.Index(buf, startCut)
	endIndex := bytes.Index(buf, endCut)
	if startIndex < 0 || endIndex < 0 {
		return fmt.Errorf("bad startIndex=%v or endIndex=%v", startIndex, endIndex)
	}

	preamble := buf[0:startIndex]
	trailer := bytes.Replace(buf[endIndex:], []byte("glyphs = glyphs,"), []byte("glyphs = nil,"), 1)

	newBuf := append([]byte{}, preamble...)
	newBuf = append(newBuf, trailer...)
	outPath := filepath.Join(fontsDir, path)

	return os.WriteFile(outPath, newBuf, 0644)
}
