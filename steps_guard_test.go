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

// TestStepCommitPathsGate parses every non-test .go file of the root package and fails when a function that
// publishes to a document (it assigns a LiveSet or calls appendOperationLocked) does not call stepGate, or, for
// the commit helpers, advanceProducer. A new commit path must check the step rules before it changes the document
// (agent-docs/steps-design.md).
func TestStepCommitPathsGate(t *testing.T) {
	t.Parallel()
	// exempt functions publish nothing new: they retire, roll back or rebuild what an earlier gated commit made.
	exempt := map[string]struct{}{"retire": {}, "restore": {}, "restoreStepLocked": {}}
	// commitFixedArgs is the number of parameters of each commit helper before its variadic consumed list.
	commitFixedArgs := map[string]int{
		"commitOperationIn": 5, "commitOperationSpanIn": 6, "commitOperationSpanPrefixIn": 7,
		"commitOperationManyIn": 5, "commitOperationManyFacesIn": 6,
	}
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
			publishes, gates, advances, retires, consumes := false, false, false, false, false
			var unconsumed []token.Pos
			ast.Inspect(fd.Body, func(n ast.Node) bool {
				switch x := n.(type) {
				case *ast.Ident:
					if x.Name == "InputConsumed" {
						consumes = true
					}
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
						case "retire":
							retires = true
						}
						if fixed, ok := commitFixedArgs[sel.Sel.Name]; ok && len(x.Args) <= fixed && x.Ellipsis == token.NoPos {
							unconsumed = append(unconsumed, x.Pos())
						}
					}
				}
				return true
			})
			// A function that retires a body takes the step rules into account first, whatever else it does.
			if _, ok := exempt[fd.Name.Name]; !ok && retires && !gates {
				bad = append(bad, fset.Position(fd.Pos()).String()+": "+fd.Name.Name+" retires a body without stepGate")
			}
			// A commit that lists a consumed input (Use: InputConsumed) must pass the consumed bodies to the
			// commit, or stepGate cannot refuse a step that retires a body it did not build.
			if consumes {
				for _, pos := range unconsumed {
					bad = append(bad, fset.Position(pos).String()+": "+fd.Name.Name+
						" consumes an input but commits with an empty consumed list")
				}
			}
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

// liveSetOwners names the functions outside internal/topo that read or write d.n.LiveSet directly. Each holds
// LiveMu and works on one commit, a rollback or a view builder, which is the reason it may. A new function that
// touches the field is a new aggregate read of the live set: read LiveIn(ctx) instead, or add the function here
// after review.
var liveSetOwners = map[string]struct{}{
	"remove": {}, "commitOperationSpanPrefixIn": {}, "commitOperationManyFacesIn": {}, "commitIn": {},
	"snapshot": {}, "restore": {}, "retire": {}, "commitPairedCut": {}, "removeConsumed": {},
	"viewSizesLocked": {}, "stepSnapshotLocked": {}, "restoreStepLocked": {}, "commitBlindGroupCut": {},
	"Bodies": {}, "VerifyContainers": {},
}

// TestLiveSetReadsGoThroughViews fails when code outside internal/topo reads the whole live set of a document
// node: a call of Live(), a method value of Live (d.n.Live), or a use of the LiveSet field outside the functions
// in liveSetOwners. The whole live set depends on the other steps' progress, so a reader must use LiveIn(ctx).
//
// Limits: the test works on syntax, not types. A method value reached through a local alias of the node
// (n := &d.n; f := n.Live) and a Live selector on a receiver not spelled n or Doc are not seen. The determinism
// test (TestRunStepsDispatchOrderDoesNotChangeResults) is the check for those.
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
		src, err := os.ReadFile(path)
		if err != nil {
			return err
		}
		// Every reading is spelled out in the source, so a file without the name has nothing to report.
		if !bytes.Contains(src, []byte("Live")) {
			return nil
		}
		fset := token.NewFileSet()
		f, err := parser.ParseFile(fset, path, src, parser.SkipObjectResolution)
		if err != nil {
			return err
		}
		for _, decl := range f.Decls {
			owner := ""
			if fd, ok := decl.(*ast.FuncDecl); ok {
				owner = fd.Name.Name
			}
			called := map[*ast.SelectorExpr]struct{}{}
			ast.Inspect(decl, func(n ast.Node) bool {
				switch x := n.(type) {
				case *ast.CallExpr:
					sel, ok := x.Fun.(*ast.SelectorExpr)
					if !ok || sel.Sel.Name != "Live" {
						return true
					}
					called[sel] = struct{}{}
					if len(x.Args) == 0 {
						bad = append(bad, fset.Position(x.Pos()).String()+": Live() reads the whole live set")
					}
				case *ast.SelectorExpr:
					switch x.Sel.Name {
					case "Live":
						if _, ok := called[x]; !ok && nodeReceiver(x.X) {
							bad = append(bad, fset.Position(x.Pos()).String()+": a method value of Live reads the whole live set")
						}
					case "LiveSet":
						if _, ok := liveSetOwners[owner]; !ok {
							bad = append(bad, fset.Position(x.Pos()).String()+": "+owner+" reads the LiveSet field")
						}
					}
				}
				return true
			})
		}
		return nil
	})
	require.NoError(t, err)
	require.Empty(t, bad, "read the live set through LiveIn(ctx)")
}

// nodeReceiver reports whether x is spelled like a document node: n, or something ending in .n or .Doc.
func nodeReceiver(x ast.Expr) bool {
	switch v := x.(type) {
	case *ast.Ident:
		return v.Name == "n"
	case *ast.SelectorExpr:
		return v.Sel.Name == "n" || v.Sel.Name == "Doc"
	}
	return false
}
