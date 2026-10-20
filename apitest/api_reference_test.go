package apitest_test

import (
	"go/ast"
	"go/parser"
	"go/token"
	"os"
	"regexp"
	"strings"
	"testing"

	"github.com/stretchr/testify/require"
)

// exportedNames returns the exported top-level identifiers of the package in
// dir and, per receiver type, its exported method names. Test files are
// skipped: the reference documents the library, not its tests.
func exportedNames(t *testing.T, dir string) (map[string]bool, map[string]map[string]bool) {
	t.Helper()
	fset := token.NewFileSet()
	pkgs, err := parser.ParseDir(fset, dir, func(fi os.FileInfo) bool {
		return !strings.HasSuffix(fi.Name(), "_test.go")
	}, 0)
	require.NoError(t, err)
	names := map[string]bool{}
	methods := map[string]map[string]bool{}
	for _, pkg := range pkgs {
		for _, f := range pkg.Files {
			for _, decl := range f.Decls {
				switch d := decl.(type) {
				case *ast.FuncDecl:
					if !d.Name.IsExported() {
						continue
					}
					if d.Recv == nil {
						names[d.Name.Name] = true
						continue
					}
					recv := d.Recv.List[0].Type
					if star, ok := recv.(*ast.StarExpr); ok {
						recv = star.X
					}
					if idx, ok := recv.(*ast.IndexExpr); ok {
						recv = idx.X
					}
					if id, ok := recv.(*ast.Ident); ok {
						if methods[id.Name] == nil {
							methods[id.Name] = map[string]bool{}
						}
						methods[id.Name][d.Name.Name] = true
					}
				case *ast.GenDecl:
					for _, spec := range d.Specs {
						switch s := spec.(type) {
						case *ast.TypeSpec:
							names[s.Name.Name] = true
						case *ast.ValueSpec:
							for _, n := range s.Names {
								names[n.Name] = true
							}
						}
					}
				}
			}
		}
	}
	return names, methods
}

// TestAPIReferenceNamesExist fails when docs/api.md names a root or export
// identifier that no longer exists, so the user-facing reference cannot drift
// from the code. It checks `decad.X`, `export.X` and `Type.Method` tokens.
func TestAPIReferenceNamesExist(t *testing.T) {
	t.Parallel()
	raw, err := os.ReadFile("../docs/api.md")
	require.NoError(t, err)
	doc := string(raw)

	rootNames, rootMethods := exportedNames(t, "..")
	exportNames, _ := exportedNames(t, "../export")

	var missing []string
	for _, m := range regexp.MustCompile(`\bdecad\.([A-Z]\w*)`).FindAllStringSubmatch(doc, -1) {
		if !rootNames[m[1]] {
			missing = append(missing, "decad."+m[1])
		}
	}
	for _, m := range regexp.MustCompile(`\bexport\.([A-Z]\w*)`).FindAllStringSubmatch(doc, -1) {
		if !exportNames[m[1]] {
			missing = append(missing, "export."+m[1])
		}
	}
	for _, m := range regexp.MustCompile("`(?:\\*?)([A-Z]\\w*)\\.([A-Z]\\w*)`").FindAllStringSubmatch(doc, -1) {
		if _, isType := rootMethods[m[1]]; !isType {
			continue
		}
		if !rootMethods[m[1]][m[2]] {
			missing = append(missing, m[1]+"."+m[2])
		}
	}
	require.Empty(t, missing, "docs/api.md names identifiers that do not exist")
}
