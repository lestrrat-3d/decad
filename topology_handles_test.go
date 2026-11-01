package decad_test

import (
	"go/ast"
	"go/parser"
	"go/token"
	"io/fs"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"testing"

	"github.com/stretchr/testify/require"
)

// bornHandles maps each root topology type whose node carries an Owner to the
// constructor every handle of that type is born through
// (topology_handles.go). A handle born any other way has a node with no
// Owner, and the first link read that maps it back panics.
var bornHandles = map[string]string{
	"Vertex":    "bornVertex",
	"Body":      "bornBody",
	"Face":      "bornFace",
	"Edge":      "bornEdge",
	"Loop":      "bornLoop",
	"FaceShell": "bornFaceShell",
	"Lump":      "bornLump",
	"Document":  "bornDocument",
	"Mesh":      "bornMesh",
}

// TestTopologyHandlesAreBorn parses every root .go file, tests included, and
// fails on a handle of a born type that does not come from its constructor:
// a T{...} literal or new(T) that is not the constructor's argument, an
// elided element of a []*T, [N]*T or map[K]*T literal, or a variable, field
// or parameter declared with the value type T.
func TestTopologyHandlesAreBorn(t *testing.T) {
	t.Parallel()
	files, err := filepath.Glob("*.go")
	require.NoError(t, err)
	require.NotEmpty(t, files)
	var bad []string
	for _, name := range files {
		src, err := os.ReadFile(name)
		require.NoError(t, err)
		fset := token.NewFileSet()
		f, err := parser.ParseFile(fset, name, src, 0)
		require.NoError(t, err)
		bad = append(bad, unbornHandles(fset, f)...)
	}
	require.Empty(t, bad, "every topology handle is born through its constructor (topology_handles.go)")
}

func unbornHandles(fset *token.FileSet, f *ast.File) []string {
	var bad []string
	report := func(n ast.Node, what string) {
		bad = append(bad, fset.Position(n.Pos()).String()+": "+what)
	}
	bornType := func(e ast.Expr) (string, bool) {
		id, ok := e.(*ast.Ident)
		if !ok {
			return "", false
		}
		_, ok = bornHandles[id.Name]
		return id.Name, ok
	}
	unsafeNames := map[string]struct{}{}
	for _, spec := range f.Imports {
		path, err := strconv.Unquote(spec.Path.Value)
		if err != nil || path != "unsafe" {
			continue
		}
		name := filepath.Base(path)
		if spec.Name != nil {
			name = spec.Name.Name
		}
		unsafeNames[name] = struct{}{}
	}
	sizeofProbes := map[ast.Node]struct{}{}
	ast.Inspect(f, func(n ast.Node) bool {
		call, ok := n.(*ast.CallExpr)
		if !ok || len(call.Args) != 1 {
			return true
		}
		selector, ok := call.Fun.(*ast.SelectorExpr)
		if !ok || selector.Sel.Name != "Sizeof" {
			return true
		}
		pkg, ok := selector.X.(*ast.Ident)
		if !ok {
			return true
		}
		if _, ok := unsafeNames[pkg.Name]; !ok {
			return true
		}
		arg := call.Args[0]
		for {
			paren, ok := arg.(*ast.ParenExpr)
			if !ok {
				break
			}
			arg = paren.X
		}
		if literal, ok := arg.(*ast.CompositeLit); ok {
			if _, ok := bornType(literal.Type); ok {
				sizeofProbes[literal] = struct{}{}
			}
		}
		return true
	})
	// The literals and new calls each constructor receives.
	owned := map[ast.Node]struct{}{}
	ast.Inspect(f, func(n ast.Node) bool {
		call, ok := n.(*ast.CallExpr)
		if !ok || len(call.Args) != 1 {
			return true
		}
		fn, ok := call.Fun.(*ast.Ident)
		if !ok {
			return true
		}
		arg := call.Args[0]
		if u, ok := arg.(*ast.UnaryExpr); ok && u.Op == token.AND {
			arg = u.X
		}
		if cl, ok := arg.(*ast.CompositeLit); ok {
			if typ, ok := bornType(cl.Type); ok && bornHandles[typ] == fn.Name {
				owned[cl] = struct{}{}
			}
		}
		if inner, ok := arg.(*ast.CallExpr); ok && isNewOf(inner) != "" && bornHandles[isNewOf(inner)] == fn.Name {
			owned[inner] = struct{}{}
		}
		return true
	})
	ast.Inspect(f, func(n ast.Node) bool {
		switch n := n.(type) {
		case *ast.CompositeLit:
			if typ, ok := bornType(n.Type); ok {
				if _, sizeOnly := sizeofProbes[n]; sizeOnly {
					return true
				}
				if _, ok := owned[n]; !ok {
					report(n, typ+" literal not born through "+bornHandles[typ])
				}
			}
			if elem := elidedPointerElem(n.Type); elem != "" {
				if _, ok := bornHandles[elem]; ok {
					for _, e := range n.Elts {
						if kv, ok := e.(*ast.KeyValueExpr); ok {
							e = kv.Value
						}
						if cl, ok := e.(*ast.CompositeLit); ok && cl.Type == nil {
							report(cl, "elided "+elem+" literal")
						}
					}
				}
			}
		case *ast.CallExpr:
			if typ := isNewOf(n); typ != "" {
				if _, ok := owned[n]; !ok {
					report(n, "new("+typ+") not born through "+bornHandles[typ])
				}
			}
		case *ast.ValueSpec:
			if typ, ok := bornType(n.Type); ok {
				report(n, "variable of value type "+typ)
			}
		case *ast.Field:
			if typ, ok := bornType(n.Type); ok {
				report(n, "field or parameter of value type "+typ)
			}
		}
		return true
	})
	return bad
}

func TestTopologyHandleSizeofProbeDoesNotCountAsAllocation(t *testing.T) {
	for _, tc := range []struct {
		name string
		src  string
		want int
	}{
		{"size probe", "package p\nimport \"unsafe\"\nfunc f() uintptr { return unsafe.Sizeof(Body{}) }", 0},
		{"real literal", "package p\nimport \"unsafe\"\nfunc f() uintptr { _ = unsafe.Sizeof(Body{}); _ = Body{}; return 0 }", 1},
	} {
		t.Run(tc.name, func(t *testing.T) {
			fset := token.NewFileSet()
			f, err := parser.ParseFile(fset, "probe.go", tc.src, 0)
			require.NoError(t, err)
			require.Len(t, unbornHandles(fset, f), tc.want)
		})
	}
}

// isNewOf returns T for new(T) of a born type, and "" otherwise.
func isNewOf(call *ast.CallExpr) string {
	fn, ok := call.Fun.(*ast.Ident)
	if !ok || fn.Name != "new" || len(call.Args) != 1 {
		return ""
	}
	id, ok := call.Args[0].(*ast.Ident)
	if !ok {
		return ""
	}
	if _, ok := bornHandles[id.Name]; !ok {
		return ""
	}
	return id.Name
}

// elidedPointerElem returns T when a composite literal of type typ may elide
// &T{...} elements: []*T, [N]*T, or map[K]*T.
func elidedPointerElem(typ ast.Expr) string {
	var elem ast.Expr
	switch typ := typ.(type) {
	case *ast.ArrayType:
		elem = typ.Elt
	case *ast.MapType:
		elem = typ.Value
	default:
		return ""
	}
	star, ok := elem.(*ast.StarExpr)
	if !ok {
		return ""
	}
	id, ok := star.X.(*ast.Ident)
	if !ok {
		return ""
	}
	return id.Name
}

// documentWriteFields are the topo.Document fields only root and the node's
// own methods write: the live set and the producer counter (root operation commit helpers,
// retire, New and VerifyMotion's transient document)
// and the level and curve counters (MintLevel and MintCurve in
// internal/topo/document.go). No other type has a field with these names, so a
// match by name is a write to a document node.
var documentWriteFields = map[string]struct{}{
	"LiveSet":      {},
	"NextProducer": {},
	"NextLevel":    {},
	"NextCurve":    {},
}

// TestTopoDocumentWriteGuard parses every non-test .go file under internal/
// except internal/topo/document.go and fails on a write to a document node's
// live set or counters, or on a write lock of its LiveMu: an assignment, op=
// assignment, ++ or -- whose left side ends in one of documentWriteFields; a
// composite literal that sets one of them; a call of X.LiveMu.Lock or
// X.LiveMu.Unlock. Reads, RLock and RUnlock, and calls of MintLevel,
// MintCurve, IsLive and Live pass. Root is the writer and is not scanned
// (agent-docs/evaluator-design.md §8).
func TestTopoDocumentWriteGuard(t *testing.T) {
	t.Parallel()
	var bad []string
	err := filepath.WalkDir("internal", func(path string, de fs.DirEntry, err error) error {
		if err != nil {
			return err
		}
		if de.IsDir() || !strings.HasSuffix(path, ".go") || strings.HasSuffix(path, "_test.go") ||
			filepath.ToSlash(path) == "internal/topo/document.go" {
			return nil
		}
		src, err := os.ReadFile(path)
		if err != nil {
			return err
		}
		fset := token.NewFileSet()
		f, err := parser.ParseFile(fset, path, src, 0)
		if err != nil {
			return err
		}
		bad = append(bad, documentNodeWrites(fset, f)...)
		return nil
	})
	require.NoError(t, err)
	require.Empty(t, bad, "only root writes a document node's live set and counters, and only root locks LiveMu for writing")
}

func documentNodeWrites(fset *token.FileSet, f *ast.File) []string {
	var bad []string
	report := func(n ast.Node, what string) {
		bad = append(bad, fset.Position(n.Pos()).String()+": "+what)
	}
	written := func(e ast.Expr) (string, bool) {
		for {
			switch x := e.(type) {
			case *ast.ParenExpr:
				e = x.X
			case *ast.IndexExpr:
				e = x.X
			case *ast.StarExpr:
				e = x.X
			case *ast.SelectorExpr:
				_, ok := documentWriteFields[x.Sel.Name]
				return x.Sel.Name, ok
			default:
				return "", false
			}
		}
	}
	ast.Inspect(f, func(n ast.Node) bool {
		switch x := n.(type) {
		case *ast.AssignStmt:
			for _, lhs := range x.Lhs {
				if name, ok := written(lhs); ok {
					report(lhs, name+" written")
				}
			}
		case *ast.IncDecStmt:
			if name, ok := written(x.X); ok {
				report(x.X, name+" written")
			}
		case *ast.KeyValueExpr:
			if id, ok := x.Key.(*ast.Ident); ok {
				if _, guarded := documentWriteFields[id.Name]; guarded {
					report(x, id.Name+" set in a literal")
				}
			}
		case *ast.CallExpr:
			sel, ok := x.Fun.(*ast.SelectorExpr)
			if !ok || (sel.Sel.Name != "Lock" && sel.Sel.Name != "Unlock") {
				return true
			}
			if mu, ok := sel.X.(*ast.SelectorExpr); ok && mu.Sel.Name == "LiveMu" {
				report(x, "LiveMu."+sel.Sel.Name+" called")
			}
		}
		return true
	})
	return bad
}
