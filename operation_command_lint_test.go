package decad

import (
	"go/ast"
	"go/parser"
	"go/token"
	"os"
	"slices"
	"strings"
	"testing"

	"github.com/stretchr/testify/require"
)

// TestCommandConstructorsAreInUnusedResultFuncs fails when an exported function
// returning a *<X>Command is missing from the govet unusedresult funcs setting
// of .golangci.yml, because decad.Union(a, b) written as a statement would then
// pass lint (agent-docs/command-api-design.md §10).
func TestCommandConstructorsAreInUnusedResultFuncs(t *testing.T) {
	t.Parallel()
	fset := token.NewFileSet()
	pkgs, err := parser.ParseDir(fset, ".", func(fi os.FileInfo) bool {
		return !strings.HasSuffix(fi.Name(), "_test.go")
	}, 0)
	require.NoError(t, err)

	var constructors []string
	for _, f := range pkgs["decad"].Files {
		for _, decl := range f.Decls {
			fn, ok := decl.(*ast.FuncDecl)
			if !ok || fn.Recv != nil || !fn.Name.IsExported() {
				continue
			}
			results := fn.Type.Results
			if results == nil || len(results.List) != 1 {
				continue
			}
			star, ok := results.List[0].Type.(*ast.StarExpr)
			if !ok {
				continue
			}
			if id, ok := star.X.(*ast.Ident); ok && strings.HasSuffix(id.Name, "Command") {
				constructors = append(constructors, fn.Name.Name)
			}
		}
	}
	require.NotEmpty(t, constructors, "the scan must find the command constructors")

	raw, err := os.ReadFile(".golangci.yml")
	require.NoError(t, err)
	var listed []string
	for line := range strings.SplitSeq(string(raw), "\n") {
		if name, ok := strings.CutPrefix(strings.TrimSpace(line), "- github.com/lestrrat-3d/decad."); ok {
			listed = append(listed, name)
		}
	}

	var missing []string
	for _, name := range constructors {
		if !slices.Contains(listed, name) {
			missing = append(missing, name)
		}
	}
	require.Empty(t, missing, "constructors missing from govet unusedresult.funcs in .golangci.yml")

	var stale []string
	for _, name := range listed {
		if !slices.Contains(constructors, name) {
			stale = append(stale, name)
		}
	}
	require.Empty(t, stale, "unusedresult.funcs names functions that are no command constructor")
}
