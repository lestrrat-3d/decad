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
// decad.Cut(a, b).MatchingFaces(), and likewise a statement that starts at a
// value builder (NewHoleThread, NewBossBore, NewBossRibs). The unusedresult lint setting catches only
// the bare constructor call (agent-docs/command-api-design.md §10). The scan
// is syntactic and covers apitest/, examples/, the root tests and _gallery/.
func TestCommandConstructorCallsEndInDo(t *testing.T) {
	t.Parallel()
	constructors := commandConstructorNames(t)
	require.NotEmpty(t, constructors, "the scan must find the command constructors")
	// The value builders are no commands, but a statement that starts at one drops
	// the value its chain builds, such as decad.NewBossBore(d, z).Head(h), and no
	// lint catches that (agent-docs/command-api-design.md §10).
	constructors = append(constructors, "NewHoleThread", "NewBossBore", "NewBossRibs")

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
			// A generic constructor may spell its type argument.
			callFun := call.Fun
			if inst, ok := callFun.(*ast.IndexExpr); ok {
				callFun = inst.X
			}
			switch fun := callFun.(type) {
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

// TestFeatureConstructorsAreCommandConstructors fails when a sketch-fed feature
// constructor stops returning a *<X>Command, which would drop it from both lint
// checks above (agent-docs/command-api-design.md §12).
func TestFeatureConstructorsAreCommandConstructors(t *testing.T) {
	t.Parallel()
	constructors := commandConstructorNames(t)
	for _, name := range []string{
		"Extrude", "Revolve", "Sweep", "Coil", "Loft", "LoftChain", "LoftFromPoint", "LoftSections",
		"LoftFaces", "LoftFacesToCone", "PatchProfile", "Emboss", "WebPath", "CutExtrude", "CutRevolve",
	} {
		require.Contains(t, constructors, name)
	}
}

// TestPlacementConstructorsAreCommandConstructors fails when a placement or
// pattern constructor stops returning a *<X>Command, which would drop it from
// both lint checks above (agent-docs/command-api-design.md §12).
func TestPlacementConstructorsAreCommandConstructors(t *testing.T) {
	t.Parallel()
	constructors := commandConstructorNames(t)
	for _, name := range []string{
		"Transform", "Copy", "Mirrored", "MirroredJoin", "MirroredCopy", "ScaledAt", "QuarterTurn", "Stretch",
		"PatternCopies", "Patterned", "PathPattern", "Arrange",
	} {
		require.Contains(t, constructors, name)
	}
}

// TestCompositeConstructorsAreCommandConstructors fails when a spec-composite
// constructor stops returning a *<X>Command, which would drop it from both lint
// checks above (agent-docs/command-api-design.md §12).
func TestCompositeConstructorsAreCommandConstructors(t *testing.T) {
	t.Parallel()
	constructors := commandConstructorNames(t)
	for _, name := range []string{"Hole", "HoleShaped", "Boss", "BossShaped", "Thread", "Infill", "SheetMetal"} {
		require.Contains(t, constructors, name)
	}
}

// TestSurfaceConstructorsAreCommandConstructors fails when a surface,
// mesh-to-solid or retirement constructor stops returning a *<X>Command, which
// would drop it from both lint checks above
// (agent-docs/command-api-design.md §12).
func TestSurfaceConstructorsAreCommandConstructors(t *testing.T) {
	t.Parallel()
	constructors := commandConstructorNames(t)
	for _, name := range []string{
		"PatchEdges", "Stitch", "Unstitch", "Trim", "Extend", "Split", "SolidFromMesh", "Remove",
	} {
		require.Contains(t, constructors, name)
	}
}

// mentionsBody reports whether the type expression holds a *Body anywhere:
// as itself, an element of a slice or array, a map key or value, a type
// argument (iter.Seq[*Body]) or a struct field.
func mentionsBody(e ast.Expr) bool {
	found := false
	ast.Inspect(e, func(n ast.Node) bool {
		if star, ok := n.(*ast.StarExpr); ok {
			if id, ok := star.X.(*ast.Ident); ok && id.Name == "Body" {
				found = true
			}
		}
		return !found
	})
	return found
}

// buildsOrRetiresABody is the predicate of the scan below. It returns the
// reason fn looks like an operation, or "" when it does not. An operation
// either returns a body (or anything that holds one), or takes a *Body and
// returns only error, which is the shape the function-form Remove had.
func buildsOrRetiresABody(fn *ast.FuncDecl) string {
	if fn.Type.Results != nil {
		for _, r := range fn.Type.Results.List {
			if mentionsBody(r.Type) {
				return "returns a body"
			}
		}
	}
	results := fn.Type.Results
	if results == nil || len(results.List) != 1 || len(results.List[0].Names) > 1 {
		return ""
	}
	if id, ok := results.List[0].Type.(*ast.Ident); !ok || id.Name != "error" {
		return ""
	}
	for _, p := range fn.Type.Params.List {
		if mentionsBody(p.Type) {
			return "takes a body and returns only error"
		}
	}
	return ""
}

// receiverName returns the receiver's type name, or "" for a function.
func receiverName(fn *ast.FuncDecl) string {
	if fn.Recv == nil || len(fn.Recv.List) == 0 {
		return ""
	}
	e := fn.Recv.List[0].Type
	if star, ok := e.(*ast.StarExpr); ok {
		e = star.X
	}
	if id, ok := e.(*ast.Ident); ok {
		return id.Name
	}
	return ""
}

// TestBuildsOrRetiresABodyPredicate pins the scan's predicate on parsed
// snippets, so the scan below cannot go blind when a type is renamed: each
// operation shape must be matched, each non-operation shape must not.
func TestBuildsOrRetiresABodyPredicate(t *testing.T) {
	t.Parallel()
	const src = `package p
func Build() *Body { return nil }
func BuildMany() ([]*Body, error) { return nil, nil }
func Retire(b *Body) error { return nil }
func RetireMany(bs ...*Body) error { return nil }
func (d *Document) Import() (*Body, error) { return nil, nil }
func Lookup() map[string]*Body { return nil }
func Walk() iter.Seq[*Body] { return nil }
func Wrapped() struct{ B *Body } { return struct{ B *Body }{} }
func Name() string { return "" }
func Measure(b *Body) (float64, error) { return 0, nil }
func Cmd(b *Body) *RemoveCommand { return nil }
`
	f, err := parser.ParseFile(token.NewFileSet(), "snippet.go", src, 0)
	require.NoError(t, err)
	want := map[string]bool{
		"Build": true, "BuildMany": true, "Retire": true, "RetireMany": true, "Import": true,
		"Lookup": true, "Walk": true, "Wrapped": true,
		"Name": false, "Measure": false, "Cmd": false,
	}
	seen := 0
	for _, decl := range f.Decls {
		fn := decl.(*ast.FuncDecl)
		seen++
		require.Equal(t, want[fn.Name.Name], buildsOrRetiresABody(fn) != "", "predicate on %s", fn.Name.Name)
	}
	require.Equal(t, len(want), seen)
}

// TestNoExportedFunctionBuildsRetiresOrRegistersABody fails when an exported
// root function or method builds, retires or registers a body without being a
// command, so an operation written as a function cannot return after the
// command-API migration (agent-docs/command-api-design.md §1). The scan flags
// an exported function or method that returns a *Body or a value holding one,
// and one that takes a *Body and returns only error. Methods on a
// *<X>Command are the commands themselves and are exempt.
//
// Three exceptions are neither operations nor queries and keep the function
// form (agent-docs/api-design.md §6.3): Container.AddBody registers an
// existing body in a container, and Container.ChildNamed and
// Container.NewUniqueChild return a container. They change container
// placement, which is a document mutation outside the operation set. The last
// two return no body, so the scan never matches them.
//
// The readers Document.Bodies, Container.Bodies and Operation.Outputs return
// existing bodies and build nothing. decadtest.NewPrism and decadtest.NewBlock
// build and register bodies but live in another package that this scan does
// not read; agent-docs/command-api-design.md §1 keeps decadtest as it is.
func TestNoExportedFunctionBuildsRetiresOrRegistersABody(t *testing.T) {
	t.Parallel()
	allowed := map[string]string{
		"Document.Bodies":   "reads the live bodies",
		"Container.Bodies":  "reads the container's bodies",
		"Operation.Outputs": "reads the bodies a recorded operation produced",
		"Container.AddBody": "container placement, neither an operation nor a query",
	}
	fset := token.NewFileSet()
	pkgs, err := parser.ParseDir(fset, ".", func(fi os.FileInfo) bool {
		return !strings.HasSuffix(fi.Name(), "_test.go")
	}, 0)
	require.NoError(t, err)
	var offenders []string
	commandDoMatches := 0
	for _, f := range pkgs["decad"].Files {
		for _, decl := range f.Decls {
			fn, ok := decl.(*ast.FuncDecl)
			if !ok || !fn.Name.IsExported() {
				continue
			}
			reason := buildsOrRetiresABody(fn)
			if reason == "" {
				continue
			}
			recv := receiverName(fn)
			if strings.HasSuffix(recv, "Command") {
				if fn.Name.Name == "Do" {
					commandDoMatches++
				}
				continue
			}
			name := fn.Name.Name
			if recv != "" {
				name = recv + "." + name
			}
			if _, ok := allowed[name]; ok {
				delete(allowed, name)
				continue
			}
			offenders = append(offenders, name+" "+reason)
		}
	}
	require.NotZero(t, commandDoMatches, "the scan must match the Do methods of the commands, or it reads nothing")
	require.Empty(t, allowed, "an allowlisted function no longer matches the scan: remove it")
	require.Empty(t, offenders, "an exported function builds, retires or registers a body: make it a command")
}
