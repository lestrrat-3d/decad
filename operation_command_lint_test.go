package decad

import (
	"go/ast"
	"go/parser"
	"go/token"
	"io/fs"
	"os"
	"path/filepath"
	"slices"
	"strconv"
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
	constructors := commandConstructorNames(t)
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

// commandConstructorNames returns every exported root function that returns
// one *<X>Command.
func commandConstructorNames(t *testing.T) []string {
	t.Helper()
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
	return constructors
}

// TestCommandConstructorCallsEndInDo fails when a command constructor call is
// an expression statement, alone or chained without Do, such as
// decad.Cut(a, b).MatchingFaces(). The unusedresult lint setting catches only
// the bare constructor call (agent-docs/command-api-design.md §10). The scan
// is syntactic and covers apitest/, examples/, the root tests and _gallery/.
func TestCommandConstructorCallsEndInDo(t *testing.T) {
	t.Parallel()
	constructors := commandConstructorNames(t)
	require.NotEmpty(t, constructors, "the scan must find the command constructors")

	var files []string
	rootTests, err := filepath.Glob("*_test.go")
	require.NoError(t, err)
	files = append(files, rootTests...)
	for _, dir := range []string{"apitest", "examples", "_gallery"} {
		err := filepath.WalkDir(dir, func(path string, d fs.DirEntry, err error) error {
			if err != nil {
				return err
			}
			if d.IsDir() && path != dir && (strings.HasPrefix(d.Name(), ".") || d.Name() == "testdata") {
				return filepath.SkipDir
			}
			if !d.IsDir() && strings.HasSuffix(path, ".go") {
				files = append(files, path)
			}
			return nil
		})
		require.NoError(t, err)
	}
	require.NotEmpty(t, files)

	var offenders []string
	for _, path := range files {
		fset := token.NewFileSet()
		file, err := parser.ParseFile(fset, path, nil, 0)
		require.NoError(t, err)
		qualifier, bare := "", file.Name.Name == "decad"
		for _, imp := range file.Imports {
			if p, _ := strconv.Unquote(imp.Path.Value); p != "github.com/lestrrat-3d/decad" {
				continue
			}
			qualifier = "decad"
			if imp.Name != nil {
				qualifier = imp.Name.Name
			}
		}
		ast.Inspect(file, func(n ast.Node) bool {
			stmt, ok := n.(*ast.ExprStmt)
			if !ok {
				return true
			}
			outer, ok := stmt.X.(*ast.CallExpr)
			if !ok {
				return true
			}
			// Walk down the receiver chain to the constructor call.
			call, last := outer, ""
			for {
				sel, ok := call.Fun.(*ast.SelectorExpr)
				if !ok {
					break
				}
				recv, ok := sel.X.(*ast.CallExpr)
				if !ok {
					break
				}
				if call == outer {
					last = sel.Sel.Name
				}
				call = recv
			}
			name := ""
			switch fun := call.Fun.(type) {
			case *ast.SelectorExpr:
				if id, ok := fun.X.(*ast.Ident); ok && qualifier != "" && id.Name == qualifier {
					name = fun.Sel.Name
				}
			case *ast.Ident:
				if bare {
					name = fun.Name
				}
			}
			if name != "" && slices.Contains(constructors, name) && last != "Do" {
				offenders = append(offenders, fset.Position(stmt.Pos()).String()+": "+name)
			}
			return true
		})
	}
	require.Empty(t, offenders, "a command constructor call used as a statement, or chained without Do")
}

// TestModifyConstructorsAreCommandConstructors fails when a modify-family
// constructor stops returning a *<X>Command, which would drop it from both
// lint checks above (agent-docs/command-api-design.md §12).
func TestModifyConstructorsAreCommandConstructors(t *testing.T) {
	t.Parallel()
	constructors := commandConstructorNames(t)
	for _, name := range []string{
		"Fillet", "Chamfer", "FilletChord", "ChamferAngle", "Shell", "ShellClosed",
		"Draft", "Offset", "Thicken", "MoveFace", "ConeFromFace",
	} {
		require.Contains(t, constructors, name)
	}
}
