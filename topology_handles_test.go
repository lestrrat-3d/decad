package decad_test

import (
	"go/ast"
	"go/parser"
	"go/token"
	"os"
	"path/filepath"
	"testing"

	"github.com/stretchr/testify/require"
)

// bornHandles maps each root topology type whose node carries an Owner to the
// constructor every handle of that type is born through
// (topology_handles.go). A handle born any other way has a node with no
// Owner, and the first link read that maps it back panics.
var bornHandles = map[string]string{
	"Vertex": "bornVertex",
	"Body":   "bornBody",
	"Face":   "bornFace",
	"Edge":   "bornEdge",
	"Loop":   "bornLoop",
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
