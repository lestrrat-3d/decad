package decad_test

// CI: isolated-tests

import (
	"bytes"
	"go/ast"
	"go/parser"
	"go/token"
	"io/fs"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"github.com/stretchr/testify/require"
)

// TestProducerCounterAccess parses every non-test .go file of the root package
// and of internal/ and fails when one reads or writes a document's identity
// counters (the NextProducer, NextLevel and NextCurve fields) through a
// selector, or names nextProducerID. Only root document.go (producerAt and
// advanceProducer) and internal/topo/document.go (ProducerAt, SeedsAt, MintLevel
// and MintCurve) may touch the counters, so a later change that routes them
// through the calling context edits those two files alone. A composite literal
// that seeds a transient document names the field as a key, not a selector,
// and passes.
func TestProducerCounterAccess(t *testing.T) {
	t.Parallel()
	counters := map[string]struct{}{"NextProducer": {}, "NextLevel": {}, "NextCurve": {}}
	allowed := map[string]struct{}{"document.go": {}, "internal/topo/document.go": {}}
	var bad []string
	check := func(path string) error {
		if _, ok := allowed[filepath.ToSlash(path)]; ok {
			return nil
		}
		src, err := os.ReadFile(path)
		if err != nil {
			return err
		}
		if !bytes.Contains(src, []byte("NextProducer")) && !bytes.Contains(src, []byte("NextLevel")) &&
			!bytes.Contains(src, []byte("NextCurve")) && !bytes.Contains(src, []byte("nextProducerID")) {
			return nil
		}
		fset := token.NewFileSet()
		f, err := parser.ParseFile(fset, path, src, parser.SkipObjectResolution)
		if err != nil {
			return err
		}
		ast.Inspect(f, func(n ast.Node) bool {
			switch x := n.(type) {
			case *ast.SelectorExpr:
				if _, ok := counters[x.Sel.Name]; ok {
					bad = append(bad, fset.Position(x.Pos()).String()+": "+x.Sel.Name+" accessed")
				}
			case *ast.Ident:
				if x.Name == "nextProducerID" {
					bad = append(bad, fset.Position(x.Pos()).String()+": nextProducerID named")
				}
			}
			return true
		})
		return nil
	}
	rootFiles, err := filepath.Glob("*.go")
	require.NoError(t, err)
	for _, path := range rootFiles {
		if strings.HasSuffix(path, "_test.go") {
			continue
		}
		require.NoError(t, check(path))
	}
	err = filepath.WalkDir("internal", func(path string, de fs.DirEntry, err error) error {
		if err != nil {
			return err
		}
		if de.IsDir() || !strings.HasSuffix(path, ".go") || strings.HasSuffix(path, "_test.go") {
			return nil
		}
		return check(path)
	})
	require.NoError(t, err)
	require.Empty(t, bad, "read the identity counters through producerAt, ProducerAt, MintLevel and MintCurve")
}
