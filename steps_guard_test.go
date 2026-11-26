package decad_test

// CI: isolated-tests

import (
	"go/ast"
	"go/parser"
	"go/token"
	"io/fs"
	"path/filepath"
	"strings"
	"testing"

	"github.com/stretchr/testify/require"
)

// TestStepCommitPathsGate parses every non-test .go file of the root package and fails when a function that
// publishes to a document (it assigns a LiveSet or calls appendOperationLocked) does not call stepGate, or, for
// the commit helpers, advanceProducer. A new commit path must check the step rules before it changes the document
// (agent-docs/steps-design.md).
func TestStepCommitPathsGate(t *testing.T) {
	t.Parallel()
	// exempt functions publish nothing new: they retire, roll back or rebuild what an earlier gated commit made.
	exempt := map[string]struct{}{"retire": {}, "restore": {}, "restoreStepLocked": {}}
	noAdvance := map[string]struct{}{"remove": {}} // Remove produces no body, so no producer identity advances
	var bad []string
	err := filepath.WalkDir(".", func(path string, de fs.DirEntry, err error) error {
		if err != nil {
			return err
		}
		if de.IsDir() {
			if path != "." {
				return filepath.SkipDir
			}
			return nil
		}
		if !strings.HasSuffix(path, ".go") || strings.HasSuffix(path, "_test.go") {
			return nil
		}
		fset := token.NewFileSet()
		f, err := parser.ParseFile(fset, path, nil, parser.SkipObjectResolution)
		if err != nil {
			return err
		}
		for _, decl := range f.Decls {
			fd, ok := decl.(*ast.FuncDecl)
			if !ok || fd.Body == nil {
				continue
			}
			publishes, gates, advances := false, false, false
			ast.Inspect(fd.Body, func(n ast.Node) bool {
				switch x := n.(type) {
				case *ast.AssignStmt:
					for _, lhs := range x.Lhs {
						if sel, ok := lhs.(*ast.SelectorExpr); ok && sel.Sel.Name == "LiveSet" {
							publishes = true
						}
					}
				case *ast.CallExpr:
					if sel, ok := x.Fun.(*ast.SelectorExpr); ok {
						switch sel.Sel.Name {
						case "appendOperationLocked":
							publishes = true
						case "stepGate":
							gates = true
						case "advanceProducer":
							advances = true
						}
					}
				}
				return true
			})
			if _, ok := exempt[fd.Name.Name]; ok || !publishes {
				continue
			}
			if !gates {
				bad = append(bad, fset.Position(fd.Pos()).String()+": "+fd.Name.Name+" publishes without stepGate")
			}
			if _, ok := noAdvance[fd.Name.Name]; !ok && !advances {
				bad = append(bad, fset.Position(fd.Pos()).String()+": "+fd.Name.Name+" publishes without advanceProducer")
			}
		}
		return nil
	})
	require.NoError(t, err)
	require.Empty(t, bad)
}

// TestLiveSetReadsGoThroughViews fails when code outside internal/topo calls Live() on a document node: a read of
// the whole live set depends on the other steps' progress, so it must read LiveIn(ctx) instead.
func TestLiveSetReadsGoThroughViews(t *testing.T) {
	t.Parallel()
	var bad []string
	err := filepath.WalkDir(".", func(path string, de fs.DirEntry, err error) error {
		if err != nil {
			return err
		}
		if de.IsDir() {
			name := de.Name()
			if path != "." && (strings.HasPrefix(name, ".") || name == "testdata" || name == "vendor" ||
				filepath.ToSlash(path) == "internal/topo") {
				return filepath.SkipDir
			}
			return nil
		}
		if !strings.HasSuffix(path, ".go") || strings.HasSuffix(path, "_test.go") {
			return nil
		}
		fset := token.NewFileSet()
		f, err := parser.ParseFile(fset, path, nil, parser.SkipObjectResolution)
		if err != nil {
			return err
		}
		ast.Inspect(f, func(n ast.Node) bool {
			call, ok := n.(*ast.CallExpr)
			if !ok || len(call.Args) != 0 {
				return true
			}
			if sel, ok := call.Fun.(*ast.SelectorExpr); ok && sel.Sel.Name == "Live" {
				bad = append(bad, fset.Position(call.Pos()).String()+": Live() reads the whole live set")
			}
			return true
		})
		return nil
	})
	require.NoError(t, err)
	require.Empty(t, bad, "read the live set through LiveIn(ctx)")
}
