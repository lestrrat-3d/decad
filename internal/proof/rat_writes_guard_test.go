package proof_test

import (
	"fmt"
	"go/ast"
	"go/parser"
	"go/token"
	"os"
	"path/filepath"
	"runtime"
	"slices"
	"strings"
	"testing"

	"github.com/stretchr/testify/require"
)

// AddRat, SubRat, MulRat and QuoRat assume operands in lowest terms (see
// AddRat). That assumption rests on three things together. The producer audit:
// every math/big constructor and arithmetic operation except Rat.GobDecode
// returns lowest terms, GobDecode installs the encoded numerator and
// denominator as they are (math/big/ratmarsh.go), and the repository has no
// use of it. The runtime detector: a check of every operand and result of the
// five helpers found 0 non-reduced values over `go test ./internal/...` (more
// than 100M checks), `./apitest` (more than 222M), `./internal/meshbool` (more
// than 21M) and the root package (241,365,639). And this guard.
//
// The guard is a tripwire for the known write shapes, not a proof. Reflection,
// unsafe and user-defined methods named like math/big methods are out of its
// scope, and the match below is on selector names with no type check. It keeps the references Rat.Num and Rat.Denom return out of the
// module's non-test files. It flags each use of an expression x.Num() or
// x.Denom() (a call with no arguments), or its method-expression form
// (*big.Rat).Num(x), outside internal/proof that is not
//
//   - the receiver of a read-only big.Int method other than Bits (Bits returns
//     the storage itself), for example x.Num().Sign(), or
//   - a direct argument in an operand slot of a math/big method called on
//     another receiver: z.Add(x.Num(), y), z.Mul(a, x.Denom()), z.Cmp(x.Num()),
//     new(big.Int).Quo(x.Num(), g). Slots that the method writes (GCD
//     arguments 0 and 1, QuoRem and DivMod arguments 2) do not count.
//
// So (*big.Rat).Num(x).SetInt64(3), n := x.Num(), return x.Num(), []*big.Int{x.Num()}, *x.Num(), x.Num().Bits(),
// x.Denom().Lsh(x.Denom(), 3), (*big.Int).SetInt64(x.Num(), 3) (a method
// expression takes the receiver as its first argument), and g(x.Num()) for any
// function, closure, conversion or method not listed in ratOperandMethods
// (so a helper that mutates its argument) are findings. Each production use
// that the scan refuses is rewritten to pass a copy, new(big.Int).Set(x.Num()).
// The scan also flags every call to GobDecode and every import of
// encoding/gob in any non-test file, including internal/proof.
//
// It skips directories whose names start with "." or "_", vendor and
// testdata, any nested module (a directory with its own go.mod) and test
// files. It does not check Num and Denom use in internal/proof, which writes
// through them: each write builds an odd numerator over a power of two, an
// integer, or a lowest-terms split result.

// ratWriteAllowList names files outside internal/proof allowed to hold Num or
// Denom references, as module-relative slash paths. It is empty.
var ratWriteAllowList = map[string]struct{}{}

// ratReaders are the big.Int methods that do not write their receiver and do
// not return its storage. Bits is absent on purpose.
var ratReaders = map[string]struct{}{
	"Sign": {}, "Cmp": {}, "CmpAbs": {}, "BitLen": {}, "Bytes": {}, "Int64": {},
	"IsInt64": {}, "Uint64": {}, "IsUint64": {}, "String": {}, "Text": {}, "Append": {},
	"TrailingZeroBits": {}, "Bit": {}, "Float64": {}, "FillBytes": {}, "ProbablyPrime": {},
	"Format": {}, "MarshalText": {}, "MarshalJSON": {},
}

// ratOperandMethods are the math/big methods on another receiver that take
// a Num or Denom call as an operand and only read it, with the argument slots
// where that holds (nil: every slot). GCD(x, y, a, b) sets x and y,
// QuoRem(x, y, r) and DivMod(x, y, m) set r and m. An argument of any method
// not listed here, of a function, closure or conversion, and the first
// argument of a method expression is a finding.
var ratOperandMethods = map[string][]int{
	"Add": nil, "Sub": nil, "Mul": nil, "Quo": nil, "Rem": nil, "Div": nil, "Mod": nil,
	"Cmp": nil, "CmpAbs": nil, "And": nil, "Or": nil, "Xor": nil, "AndNot": nil,
	"Set": nil, "Exp": nil, "ModInverse": nil, "Neg": nil, "Abs": nil, "Not": nil,
	"Sqrt": nil, "ModSqrt": nil, "Lsh": {0}, "Rsh": {0}, "SetFrac": nil, "SetInt": nil,
	"GCD": {2, 3}, "QuoRem": {0, 1}, "DivMod": {0, 1},
}

// argIndex is the position of arg among call's arguments, or -1.
func argIndex(call *ast.CallExpr, arg ast.Node) int {
	for i, a := range call.Args {
		if ast.Node(a) == arg {
			return i
		}
	}
	return -1
}

type ratWrite struct {
	File string
	Line int
	Call string
}

func (w ratWrite) String() string { return fmt.Sprintf("%s:%d %s", w.File, w.Line, w.Call) }

// isNumDenomCall reports whether e is a call x.Num() or x.Denom(), or the
// method-expression form of one: (*big.Rat).Num(x), big.Rat.Denom(x), a call
// with one argument on a parenthesised pointer type or on a type qualified by
// an imported package. imports holds the local names of the file's imports.
func isNumDenomCall(e ast.Expr, imports map[string]struct{}) bool {
	call, ok := e.(*ast.CallExpr)
	if !ok {
		return false
	}
	sel, ok := call.Fun.(*ast.SelectorExpr)
	if !ok || (sel.Sel.Name != "Num" && sel.Sel.Name != "Denom") {
		return false
	}
	switch len(call.Args) {
	case 0:
		return true
	case 1:
		switch x := sel.X.(type) {
		case *ast.ParenExpr:
			return true
		case *ast.SelectorExpr:
			id, ok := x.X.(*ast.Ident)
			if !ok {
				return false
			}
			_, isPkg := imports[id.Name]
			return isPkg
		}
	}
	return false
}

// classifyNumDenomUse returns why the use of the Num/Denom call at the bottom
// of stack is not allowed, or "" when it is. stack holds its ancestors,
// outermost first; imports holds the local names of the file's imports.
func classifyNumDenomUse(stack []ast.Node, call ast.Expr, imports map[string]struct{}) string {
	child := ast.Node(call)
	i := len(stack) - 1
	for i >= 0 {
		paren, ok := stack[i].(*ast.ParenExpr)
		if !ok {
			break
		}
		child = paren
		i--
	}
	if i < 0 {
		return "outside any statement"
	}
	switch p := stack[i].(type) {
	case *ast.SelectorExpr:
		if p.X != child {
			return "selector"
		}
		if p.Sel.Name == "Bits" {
			return "Bits returns the storage of a Num/Denom value"
		}
		if _, ok := ratReaders[p.Sel.Name]; !ok {
			return p.Sel.Name + " on a Num/Denom value"
		}
		if i == 0 {
			return "method value " + p.Sel.Name
		}
		if outer, ok := stack[i-1].(*ast.CallExpr); !ok || outer.Fun != p {
			return "method value " + p.Sel.Name
		}
		return ""
	case *ast.CallExpr:
		if p.Fun == child {
			return "called"
		}
		fn, ok := p.Fun.(*ast.SelectorExpr)
		if !ok {
			return "passed to a function, closure or conversion"
		}
		switch x := fn.X.(type) {
		case *ast.ParenExpr:
			return "passed to a method expression " + fn.Sel.Name
		case *ast.Ident:
			if _, isPkg := imports[x.Name]; isPkg {
				return "passed to the function " + x.Name + "." + fn.Sel.Name
			}
		case *ast.SelectorExpr:
			if id, ok := x.X.(*ast.Ident); ok {
				if _, isPkg := imports[id.Name]; isPkg {
					return "passed to a method expression " + fn.Sel.Name
				}
			}
		}
		slots, known := ratOperandMethods[fn.Sel.Name]
		if !known {
			return "passed to the unknown callee " + fn.Sel.Name
		}
		if slots != nil && !slices.Contains(slots, argIndex(p, child)) {
			return fn.Sel.Name + " writes this argument"
		}
		return ""
	case *ast.AssignStmt:
		return "assigned"
	case *ast.ValueSpec:
		return "declared"
	case *ast.ReturnStmt:
		return "returned"
	case *ast.SendStmt:
		return "sent"
	case *ast.CompositeLit, *ast.KeyValueExpr:
		return "stored in a composite literal"
	case *ast.StarExpr:
		return "dereferenced"
	}
	return fmt.Sprintf("used in a %T", stack[i])
}

// scanRatAccess parses src and returns each GobDecode call and each import of
// encoding/gob, and, when checkNumDenom is set, each Num/Denom use that
// classifyNumDenomUse refuses.
func scanRatAccess(filename string, src []byte, checkNumDenom bool) ([]ratWrite, error) {
	fset := token.NewFileSet()
	f, err := parser.ParseFile(fset, filename, src, 0)
	if err != nil {
		return nil, err
	}
	var out []ratWrite
	report := func(pos token.Pos, what string) {
		out = append(out, ratWrite{File: filename, Line: fset.Position(pos).Line, Call: what})
	}
	imports := map[string]struct{}{}
	for _, spec := range f.Imports {
		path := strings.Trim(spec.Path.Value, `"`)
		name := path[strings.LastIndex(path, "/")+1:]
		if spec.Name != nil {
			name = spec.Name.Name
		}
		imports[name] = struct{}{}
		if path == "encoding/gob" {
			report(spec.Pos(), "encoding/gob can decode a big.Rat without reducing it")
		}
	}
	var stack []ast.Node
	ast.Inspect(f, func(n ast.Node) bool {
		if n == nil {
			stack = stack[:len(stack)-1]
			return true
		}
		if call, ok := n.(*ast.CallExpr); ok {
			if sel, ok := call.Fun.(*ast.SelectorExpr); ok && sel.Sel.Name == "GobDecode" {
				report(call.Pos(), "GobDecode installs an unreduced Rat")
			}
			if checkNumDenom && isNumDenomCall(call, imports) {
				if why := classifyNumDenomUse(stack, call, imports); why != "" {
					report(call.Pos(), why)
				}
			}
		}
		stack = append(stack, n)
		return true
	})
	return out, nil
}

// exemptFromRatWriteScan reports whether the module-relative slash path is in
// internal/proof, which owns the writes.
func exemptFromRatWriteScan(rel string) bool { return strings.HasPrefix(rel, "internal/proof/") }

// scanModuleRatWrites walks the non-test .go files under root and returns the
// writes outside internal/proof and the allow-list, with the number of files
// it parsed. It skips directories that start with "." or "_", vendor and
// testdata, and any nested module (a directory holding its own go.mod).
func scanModuleRatWrites(root string) ([]ratWrite, int, error) {
	var out []ratWrite
	files := 0
	err := filepath.WalkDir(root, func(path string, d os.DirEntry, err error) error {
		if err != nil {
			return err
		}
		rel, err := filepath.Rel(root, path)
		if err != nil {
			return err
		}
		rel = filepath.ToSlash(rel)
		if d.IsDir() {
			if rel == "." {
				return nil
			}
			name := d.Name()
			if strings.HasPrefix(name, ".") || strings.HasPrefix(name, "_") || name == "vendor" || name == "testdata" {
				return filepath.SkipDir
			}
			if _, err := os.Stat(filepath.Join(path, "go.mod")); err == nil {
				return filepath.SkipDir
			}
			return nil
		}
		if !strings.HasSuffix(rel, ".go") || strings.HasSuffix(rel, "_test.go") {
			return nil
		}
		src, err := os.ReadFile(path)
		if err != nil {
			return err
		}
		files++
		_, allowed := ratWriteAllowList[rel]
		found, err := scanRatAccess(rel, src, !exemptFromRatWriteScan(rel) && !allowed)
		if err != nil {
			return err
		}
		out = append(out, found...)
		return nil
	})
	return out, files, err
}

func moduleRoot(t *testing.T) string {
	t.Helper()
	_, file, _, ok := runtime.Caller(0)
	require.True(t, ok)
	dir := filepath.Dir(file)
	for {
		if _, err := os.Stat(filepath.Join(dir, "go.mod")); err == nil {
			return dir
		}
		parent := filepath.Dir(dir)
		require.NotEqual(t, dir, parent, "no go.mod above %s", file)
		dir = parent
	}
}

// TestNoRatWritesThroughNumDenom fails when a non-test file outside
// internal/proof writes through Rat.Num or Rat.Denom, which could hand
// AddRat, SubRat, MulRat or QuoRat an operand not in lowest terms.
func TestNoRatWritesThroughNumDenom(t *testing.T) {
	t.Parallel()
	require.Empty(t, ratWriteAllowList)
	found, files, err := scanModuleRatWrites(moduleRoot(t))
	require.NoError(t, err)
	require.Greater(t, files, 300, "the scan reached too few files")
	var lines []string
	for _, w := range found {
		lines = append(lines, w.String())
	}
	require.Empty(t, lines, "writes through Num or Denom outside internal/proof")
}

func TestRatWriteScannerFlagsEachShape(t *testing.T) {
	t.Parallel()
	const head = "package p\n\nimport \"math/big\"\n\n"
	cases := []struct {
		name string
		body string
		want int
	}{
		// Writes and aliases.
		{"set on Num call", "func f(r *big.Rat) { r.Num().Set(big.NewInt(3)) }", 1},
		{"lsh on Denom call", "func f(r *big.Rat) { r.Denom().Lsh(r.Denom(), 3) }", 1},
		{"Bits shares storage (a)", "func f(r *big.Rat) { r.Num().Bits()[0] = 3 }", 1},
		{"bound then reassigned (b)", "func f(r, one, other *big.Int, q *big.Rat) { n := q.Num(); n.Add(n, one); n = other }", 1},
		{"second alias (c)", "func f(r *big.Rat, one *big.Int) { n := r.Num(); z := n; z.Add(z, one) }", 1},
		{"bound denom", "func f(r *big.Rat) { x := r.Denom(); x.Lsh(x, 3) }", 1},
		{"returned from a helper", "func f(r *big.Rat) *big.Int { return r.Num() }", 1},
		{"slice literal", "func f(r *big.Rat) []*big.Int { s := []*big.Int{r.Num()}; return s }", 1},
		{"composite literal field", "type pair struct{ n *big.Int }\n\nfunc f(r *big.Rat) pair { return pair{n: r.Num()} }", 1},
		{"write in a closure", "func f(r *big.Rat) { f := func() { r.Num().SetInt64(1) }; f() }", 1},
		{"pair binding", "func f(r *big.Rat) { n, d := r.Num(), r.Denom(); n.Sign(); d.Sign() }", 2},
		{"var declaration", "func f(r *big.Rat) { var d = r.Denom(); d.Neg(d) }", 1},
		{"channel send", "func f(r *big.Rat, c chan *big.Int) { c <- r.Num() }", 1},
		{"dereference", "func f(r *big.Rat) big.Int { return *r.Num() }", 1},
		{"assignment through pointer", "func f(r *big.Rat) { *r.Num() = *big.NewInt(1) }", 1},
		{"stored by append", "func f(r *big.Rat, s []*big.Int) []*big.Int { return append(s, r.Num()) }", 1},
		{"stored in an interface", "func f(r *big.Rat) any { return any(r.Num()) }", 1},
		{"GCD out-parameter", "func f(r *big.Rat, a, b *big.Int) { new(big.Int).GCD(r.Num(), nil, a, b) }", 1},
		{"QuoRem remainder", "func f(r *big.Rat, a, b *big.Int) { new(big.Int).QuoRem(a, b, r.Denom()) }", 1},
		{"package-level value", "var v = func(r *big.Rat) { r.Num().Quo(r.Num(), r.Denom()) }", 1},
		{"method expression Num mutated", "func f(r *big.Rat) { (*big.Rat).Num(r).SetInt64(3) }", 1},
		{"method expression Denom bound", "func f(r *big.Rat) { d := (*big.Rat).Denom(r); d.SetInt64(3) }", 1},
		{"method expression on the type", "func f(r *big.Rat) { big.Rat.Num(*r).SetInt64(3) }", 1},
		{"GobDecode call", "func f(r *big.Rat, b []byte) error { return r.GobDecode(b) }", 1},
		{"gob import with big.Rat", "import \"encoding/gob\"\n\nvar _ = gob.NewEncoder\n\nvar _ big.Rat", 1},
		{"gob import without big.Rat", "import \"encoding/gob\"\n\nvar _ = gob.NewEncoder", 1},
		{"method expression receiver", "func f(r *big.Rat) { (*big.Int).SetInt64(r.Num(), 3) }", 1},
		{"method expression Add", "func f(r *big.Rat, a, b *big.Int) { (*big.Int).Add(r.Denom(), a, b) }", 1},
		{"helper that mutates its argument", "func mut(n *big.Int) { n.SetInt64(3) }\n\nfunc f(r *big.Rat) { mut(r.Num()) }", 1},
		{"closure value", "func f(r *big.Rat, h func(*big.Int)) { h(r.Num()) }", 1},
		{"go statement on a function", "func g(*big.Int) {}\n\nfunc f(r *big.Rat) { go g(r.Num()) }", 1},
		{"unknown method", "type t struct{}\n\nfunc (t) Keep(*big.Int) {}\n\nfunc f(r *big.Rat, x t) { x.Keep(r.Num()) }", 1},
		{"conversion", "type I big.Int\n\nfunc f(r *big.Rat) *I { return (*I)(r.Num()) }", 1},
		// Allowed shapes.
		{"argument of Add", "func f(r *big.Rat, x, y *big.Int) { x.Add(r.Num(), y) }", 0},
		{"argument of CmpAbs", "func f(r *big.Rat, z *big.Int) int { return z.CmpAbs(r.Denom()) }", 0},
		{"argument of Quo", "func f(r *big.Rat, g *big.Int) *big.Int { return new(big.Int).Quo(r.Num(), g) }", 0},
		{"operand of Lsh", "func f(r *big.Rat) *big.Int { return new(big.Int).Lsh(r.Num(), 3) }", 0},
		{"method expression as an operand", "func f(r *big.Rat, x *big.Int) int { return x.Cmp((*big.Rat).Num(r)) }", 0},
		{"method expression read", "func f(r *big.Rat) int { return (*big.Rat).Num(r).Sign() }", 0},
		{"copy", "func f(r *big.Rat) *big.Int { return new(big.Int).Set(r.Num()) }", 0},
		{"argument of Cmp", "func f(r *big.Rat, x *big.Int) int { return x.Cmp(r.Num()) }", 0},
		{"argument of a fresh Add", "func f(r *big.Rat, y *big.Int) *big.Int { return new(big.Int).Add(r.Num(), y) }", 0},
		{"argument of Mul", "func f(r *big.Rat, x, a *big.Int) { x.Mul(a, r.Denom()) }", 0},
		{"GCD input argument", "func f(r *big.Rat, a *big.Int) { new(big.Int).GCD(nil, nil, a, r.Denom()) }", 0},
		{"QuoRem input arguments", "func f(r *big.Rat, m *big.Int) { new(big.Int).QuoRem(r.Num(), r.Denom(), m) }", 0},
		{"pure readers", "func f(r *big.Rat) int { return r.Num().Sign() + r.Denom().Cmp(r.Num()) + r.Num().BitLen() }", 0},
	}
	for _, tc := range cases {
		for _, file := range []string{"x.go", "internal/other/x.go"} {
			src := head + tc.body
			if strings.HasPrefix(tc.body, "import") {
				src = "package p\n\nimport \"math/big\"\n" + tc.body
			}
			got, err := scanRatAccess(file, []byte(src), !exemptFromRatWriteScan(file))
			require.NoError(t, err, tc.name)
			require.Len(t, got, tc.want, "%s in %s: %v", tc.name, file, got)
		}
	}
	// internal/proof is exempt from the module scan, and internal/proofbound is not.
	require.True(t, exemptFromRatWriteScan("internal/proof/x.go"))
	require.False(t, exemptFromRatWriteScan("internal/proofbound/x.go"))
}

func TestRatWriteScannerWalksModuleTree(t *testing.T) {
	t.Parallel()
	probe := "package p\n\nimport \"math/big\"\n\nfunc f(r *big.Rat) { n := r.Num(); n.SetInt64(3) }\n"
	gobProbe := "package p\n\nimport \"encoding/gob\"\n\nvar _ = gob.NewEncoder\n"
	root := t.TempDir()
	files := map[string]string{
		"go.mod":                        "module example.com/m\n",
		"root_probe.go":                 probe,
		"internal/other/probe.go":       probe,
		"internal/deep/pkg/probe.go":    probe,
		"internal/proof/probe.go":       probe,
		"internal/other/gob.go":         gobProbe,
		"internal/proof/gob.go":         gobProbe,
		".worktrees/w/gob.go":           gobProbe,
		"internal/proofbound/ok.go":     "package p\n",
		"internal/proofbound/b_test.go": probe,
		"internal/x/testdata/probe.go":  probe,
		".worktrees/w/probe.go":         probe,
		".tmp/probe.go":                 probe,
		"_gallery/probe.go":             probe,
		"_shardgen/probe.go":            probe,
		"vendor/v/probe.go":             probe,
		"nested/go.mod":                 "module example.com/n\n",
		"nested/probe.go":               probe,
		"nested/internal/probe.go":      probe,
	}
	for rel, content := range files {
		path := filepath.Join(root, filepath.FromSlash(rel))
		require.NoError(t, os.MkdirAll(filepath.Dir(path), 0o755))
		require.NoError(t, os.WriteFile(path, []byte(content), 0o644))
	}
	found, n, err := scanModuleRatWrites(root)
	require.NoError(t, err)
	var got []string
	for _, w := range found {
		got = append(got, w.File)
	}
	slices.Sort(got)
	// The root package and internal packages are scanned; dot and underscore
	// directories, vendor, testdata, nested modules, test files and
	// internal/proof are not.
	// A gob import is refused module-wide, internal/proof included, and in a
	// file that never mentions big.Rat.
	require.Equal(t, []string{
		"internal/deep/pkg/probe.go", "internal/other/gob.go", "internal/other/probe.go",
		"internal/proof/gob.go", "root_probe.go",
	}, got)
	require.Equal(t, 7, n, "parsed: root_probe, other/probe, other/gob, deep/pkg/probe, proof/probe, proof/gob, proofbound/ok")
}
