package decad

import (
	"context"
	"fmt"
	"go/ast"
	"go/build"
	"go/importer"
	"go/parser"
	"go/token"
	"go/types"
	"io"
	"os"
	"os/exec"
	"path/filepath"
	"sort"
	"strings"
	"sync"
	"testing"

	"github.com/stretchr/testify/require"
)

// moduleSource is every non-test package of this module, type-checked from source with go/types. The tests that
// guard the body digest (TestDigestTypeCoverage, TestIdentityArithmeticAllowList) read it. It is built once per test
// binary.
type moduleSource struct {
	path string // module path
	fset *token.FileSet
	pkgs map[string]*typedPackage
}

type typedPackage struct {
	pkg   *types.Package
	files []*ast.File
	info  *types.Info
}

var (
	declSourceOnce, bodySourceOnce sync.Once
	declSourceVal, bodySourceVal   *moduleSource
	declSourceErr, bodySourceErr   error
)

// loadModuleDecls type-checks the declarations of the module and skips function bodies, which is several times
// faster. A test that reads only types uses it.
func loadModuleDecls(t *testing.T) *moduleSource {
	t.Helper()
	declSourceOnce.Do(func() { declSourceVal, declSourceErr = typeCheckModule(t.Context(), true) })
	require.NoError(t, declSourceErr)
	return declSourceVal
}

// loadModuleSource type-checks the whole module, function bodies included.
func loadModuleSource(t *testing.T) *moduleSource {
	t.Helper()
	bodySourceOnce.Do(func() { bodySourceVal, bodySourceErr = typeCheckModule(t.Context(), false) })
	require.NoError(t, bodySourceErr)
	return bodySourceVal
}

func typeCheckModule(ctx context.Context, declsOnly bool) (*moduleSource, error) {
	const modPath = "github.com/lestrrat-3d/decad"
	ms := &moduleSource{path: modPath, fset: token.NewFileSet(), pkgs: map[string]*typedPackage{}}
	dirs := map[string]string{}
	err := filepath.WalkDir(".", func(p string, d os.DirEntry, err error) error {
		if err != nil {
			return err
		}
		if !d.IsDir() {
			return nil
		}
		name := d.Name()
		if p != "." && (strings.HasPrefix(name, "_") || strings.HasPrefix(name, ".") || name == "testdata") {
			return filepath.SkipDir
		}
		if _, statErr := os.Stat(filepath.Join(p, "go.mod")); statErr == nil && p != "." {
			return filepath.SkipDir
		}
		dirs[importPathOf(modPath, p)] = p
		return nil
	})
	if err != nil {
		return nil, err
	}
	ext, err := externalImporter(ctx, ms.fset, modPath, dirs)
	if err != nil {
		return nil, err
	}
	imp := &moduleImporter{ms: ms, dirs: dirs, ext: ext, declsOnly: declsOnly}
	paths := make([]string, 0, len(dirs))
	for p := range dirs {
		paths = append(paths, p)
	}
	sort.Strings(paths)
	for _, p := range paths {
		if _, err := imp.Import(p); err != nil {
			return nil, err
		}
	}
	return ms, nil
}

// externalImporter reads the packages the module imports from outside itself (the standard library and other
// modules) from the compiler's export data, which go list builds or finds in the build cache.
func externalImporter(ctx context.Context, fset *token.FileSet, modPath string, dirs map[string]string) (types.Importer, error) {
	want := map[string]struct{}{}
	for _, dir := range dirs {
		bp, err := build.ImportDir(dir, 0)
		if err != nil {
			continue
		}
		for _, imp := range bp.Imports {
			if imp != "C" && imp != modPath && !strings.HasPrefix(imp, modPath+"/") {
				want[imp] = struct{}{}
			}
		}
	}
	args := []string{"list", "-export", "-deps", "-f", "{{.ImportPath}}\t{{.Export}}"}
	for p := range want {
		args = append(args, p)
	}
	sort.Strings(args[5:])
	out, err := exec.CommandContext(ctx, "go", args...).Output()
	if err != nil {
		return nil, fmt.Errorf("go list -export: %w", err)
	}
	exports := map[string]string{}
	for line := range strings.SplitSeq(string(out), "\n") {
		if p, file, ok := strings.Cut(line, "\t"); ok && file != "" {
			exports[p] = file
		}
	}
	lookup := func(p string) (io.ReadCloser, error) {
		file, ok := exports[p]
		if !ok {
			return nil, fmt.Errorf("no export data for %q", p)
		}
		return os.Open(file)
	}
	return importer.ForCompiler(fset, "gc", lookup), nil
}

func importPathOf(mod, dir string) string {
	if dir == "." {
		return mod
	}
	return mod + "/" + filepath.ToSlash(dir)
}

type moduleImporter struct {
	ms        *moduleSource
	dirs      map[string]string
	ext       types.Importer
	declsOnly bool
}

func (m *moduleImporter) Import(p string) (*types.Package, error) {
	if tp, ok := m.ms.pkgs[p]; ok {
		return tp.pkg, nil
	}
	dir, ok := m.dirs[p]
	if !ok {
		return m.ext.Import(p)
	}
	bp, err := build.ImportDir(dir, 0)
	if err != nil {
		if _, none := err.(*build.NoGoError); none {
			return types.NewPackage(p, filepath.Base(dir)), nil
		}
		return nil, err
	}
	var files []*ast.File
	for _, name := range bp.GoFiles {
		f, err := parser.ParseFile(m.ms.fset, filepath.Join(dir, name), nil, parser.SkipObjectResolution)
		if err != nil {
			return nil, err
		}
		files = append(files, f)
	}
	info := &types.Info{
		Types: map[ast.Expr]types.TypeAndValue{}, Uses: map[*ast.Ident]types.Object{},
		Defs: map[*ast.Ident]types.Object{}, Selections: map[*ast.SelectorExpr]*types.Selection{},
		Instances: map[*ast.Ident]types.Instance{},
	}
	conf := types.Config{Importer: m, IgnoreFuncBodies: m.declsOnly}
	pkg, err := conf.Check(p, m.ms.fset, files, info)
	if err != nil {
		return nil, err
	}
	m.ms.pkgs[p] = &typedPackage{pkg: pkg, files: files, info: info}
	return pkg, nil
}
