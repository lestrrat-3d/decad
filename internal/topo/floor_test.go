package topo_test

import (
	"go/build"
	"go/parser"
	"go/token"
	"path/filepath"
	"slices"
	"sort"
	"strconv"
	"strings"
	"testing"

	"github.com/stretchr/testify/require"
)

const modulePath = "github.com/lestrrat-3d/decad"

// floor is the fixed set of internal packages topo may reach. No package in it
// may ever import topo, and every other internal package may.
var floor = []string{
	"decaderr", "denotation", "measurement", "orderedwork", "pair", "pair/planar",
	"planarsnapshot", "proof", "proofbound", "provenance", "sectionrecord",
	"surfacegeom", "surfacenormal",
}

// pendingFiles are files whose imports this test ignores because a planned PR
// moves them out of their package. It is empty: H1b.0 moved
// surfacegeom/placement.go to surfaceplace.
var pendingFiles = map[string]struct{}{}

// TestFloor walks topo's internal imports and requires its closure to equal
// the floor.
func TestFloor(t *testing.T) {
	root, err := filepath.Abs(filepath.Join("..", ".."))
	require.NoError(t, err)
	seen := map[string]struct{}{}
	var walk func(rel string)
	walk = func(rel string) {
		dir := filepath.Join(root, "internal", filepath.FromSlash(rel))
		pkg, err := build.ImportDir(dir, 0)
		require.NoError(t, err, rel)
		fset := token.NewFileSet()
		for _, name := range pkg.GoFiles {
			if _, ok := pendingFiles[rel+"/"+name]; ok {
				continue
			}
			f, err := parser.ParseFile(fset, filepath.Join(dir, name), nil, parser.ImportsOnly)
			require.NoError(t, err, name)
			for _, spec := range f.Imports {
				path, err := strconv.Unquote(spec.Path.Value)
				require.NoError(t, err)
				dep, ok := strings.CutPrefix(path, modulePath+"/internal/")
				if !ok {
					require.False(t, strings.HasPrefix(path, modulePath), "%s imports %s", rel, path)
					continue
				}
				if _, ok := seen[dep]; ok {
					continue
				}
				seen[dep] = struct{}{}
				walk(dep)
			}
		}
	}
	walk("topo")
	got := make([]string, 0, len(seen))
	for dep := range seen {
		got = append(got, dep)
	}
	sort.Strings(got)
	want := slices.Clone(floor)
	sort.Strings(want)
	require.Equal(t, want, got)
}
