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

// TestProducerCounterAccess parses every non-test .go file of the module (the
// root package, internal/, linkage/ and any other package that imports
// internal/topo) and fails when code outside an allowed function reads or
// writes a document's identity counters (the NextProducer, NextLevel and
// NextCurve fields) through a selector, or names nextProducerID. Only the
// functions producerAt and advanceProducer of root document.go and ProducerAt,
// SeedsAt, MintLevel and MintCurve of internal/topo/document.go may touch the
// counters, plus snapshot and restore of root document.go, which only save the
// producer counter and put it back inside an atomic rollback, so a later change
// that routes the counters through the calling context edits those functions
// alone. Any other function, including one added to an
// allowed file, fails. A composite literal that seeds a transient document
// names the field as a key, not a selector, and passes.
func TestProducerCounterAccess(t *testing.T) {
	t.Parallel()
	counters := map[string]struct{}{"NextProducer": {}, "NextLevel": {}, "NextCurve": {}}
	allowed := map[string]struct{}{
		"document.go:producerAt":               {},
		"document.go:advanceProducer":          {},
		"document.go:snapshot":                 {}, // saves the counter for an atomic rollback; reads no new identity
		"document.go:restore":                  {}, // puts the saved counter back; mints no identity
		"internal/topo/document.go:ProducerAt": {},
		"internal/topo/document.go:SeedsAt":    {},
		"internal/topo/document.go:MintLevel":  {},
		"internal/topo/document.go:MintCurve":  {},
	}
	var bad []string
	check := func(path string) error {
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
		slash := filepath.ToSlash(path)
		for _, decl := range f.Decls {
			fn := ""
			if fd, ok := decl.(*ast.FuncDecl); ok {
				fn = fd.Name.Name
			}
			_, ok := allowed[slash+":"+fn]
			if ok {
				continue
			}
			ast.Inspect(decl, func(n ast.Node) bool {
				switch x := n.(type) {
				case *ast.SelectorExpr:
					if _, ok := counters[x.Sel.Name]; ok {
						bad = append(bad, fset.Position(x.Pos()).String()+": "+x.Sel.Name+" accessed in "+fn)
					}
				case *ast.Ident:
					if x.Name == "nextProducerID" {
						bad = append(bad, fset.Position(x.Pos()).String()+": nextProducerID named in "+fn)
					}
				}
				return true
			})
		}
		return nil
	}
	err := filepath.WalkDir(".", func(path string, de fs.DirEntry, err error) error {
		if err != nil {
			return err
		}
		if de.IsDir() {
			name := de.Name()
			if path != "." && (strings.HasPrefix(name, ".") || name == "testdata" || name == "vendor") {
				return filepath.SkipDir
			}
			return nil
		}
		if !strings.HasSuffix(path, ".go") || strings.HasSuffix(path, "_test.go") {
			return nil
		}
		return check(path)
	})
	require.NoError(t, err)
	require.Empty(t, bad, "read the identity counters through producerAt, ProducerAt, MintLevel and MintCurve")
}
