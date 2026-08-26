package decad

import (
	"go/ast"
	"go/parser"
	"go/token"
	"strings"
	"testing"

	"github.com/stretchr/testify/require"
)

// This file is the durable guard on spline_sagitta.go's own metering rule, and
// it is the part of the fix that keeps the class of work-accounting drift
// closed rather than merely repaired.
//
// Two review rounds already corrected this counter's numbers, and the same
// class reopened both times, because the cost model was a hand-written parallel
// description of the code: it restated both what each callee does and how many
// times each caller invokes it, so it could drift on either axis at every edit
// and nothing forced a re-derivation. Moving each charge into its own callee
// makes the multiplicity a call count, which no edit can restate wrongly — but
// only for as long as every exact-arithmetic operation in the file happens
// inside a metered primitive. A new line of big.Rat arithmetic dropped into a
// walk function would be unmetered again, and no per-site number test would
// notice.
//
// So this test reads the file's own syntax: outside the bodies of the named
// metered primitives, spline_sagitta.go may not allocate a math/big value or
// call an arithmetic method on one. Reading a value's SHAPE is allowed
// everywhere — Num, Denom and BitLen are what the cost closures themselves are
// built from, and they perform no arithmetic.

// meteredPrimitives names every function in spline_sagitta.go whose body is
// permitted to run exact arithmetic: each one takes the *freeformWork counter
// that pays for it and charges its own documented cost before doing any work.
// A method is named receiverType.methodName.
//
// ADDING A NAME HERE IS A DESIGN DECISION, never a way to quiet this test. The
// entry is only correct if the function actually charges its own cost first and
// returns having done nothing when the counter refuses.
var meteredPrimitives = map[string]struct{}{
	"chordSegmentSquaredDistance": {},
	"ratChordFrame":               {},
	"ratRunningMax":               {},
	"ratPointCopy":                {},
	"dyadicSpan.ratPointAt":       {},
	"spanChordVector":             {},
	"spanChordSquared":            {},
	"spanHodographGapSquared":     {},
	"ratQuarterOf":                {},
}

// bigArithmeticMethods are the math/big method names that DO work: they
// allocate, mutate, compare or convert a value whose size is unbounded. A call
// to any of them outside a metered primitive is unmetered exact arithmetic.
//
// Num, Denom, BitLen and Sign are deliberately absent. The first three read a
// value's shape in constant time and are exactly what the cost closures ask
// before charging; Sign is a single word test. None of them is the work the
// counter exists to bound.
var bigArithmeticMethods = map[string]struct{}{
	"Abs": {}, "Add": {}, "Cmp": {}, "CmpAbs": {}, "Div": {}, "Exp": {},
	"Float64": {}, "GCD": {}, "Int64": {}, "Inv": {}, "Lsh": {}, "MantExp": {},
	"Mul": {}, "Neg": {}, "Quo": {}, "Rat": {}, "Rsh": {}, "Set": {},
	"SetFloat64": {}, "SetFrac": {}, "SetInt": {}, "SetInt64": {},
	"SetMantExp": {}, "SetRat": {}, "SetString": {}, "Sqrt": {}, "Sub": {},
}

func TestSplineSagittaRunsNoExactArithmeticOutsideAMeteredPrimitive(t *testing.T) {
	const path = "spline_sagitta.go"
	fset := token.NewFileSet()
	file, err := parser.ParseFile(fset, path, nil, parser.SkipObjectResolution)
	require.NoError(t, err)

	seen := make(map[string]struct{}, len(meteredPrimitives))
	for _, decl := range file.Decls {
		fn, ok := decl.(*ast.FuncDecl)
		if !ok || fn.Body == nil {
			continue
		}
		name := funcDeclName(fn)
		if _, metered := meteredPrimitives[name]; metered {
			seen[name] = struct{}{}
			continue
		}
		for _, where := range unmeteredExactArithmetic(fn.Body) {
			t.Errorf(
				"%s: %s runs exact arithmetic (%s) outside a metered primitive; move it into one that charges its own cost first, or make this function one",
				fset.Position(where.pos), name, where.what,
			)
		}
	}

	for name := range meteredPrimitives {
		require.Contains(t, seen, name,
			"the guard names %s as a metered primitive, but %s declares no such function — a renamed or deleted primitive must be renamed here too, never silently dropped", name, path)
	}
}

// funcDeclName renders a declaration as the guard names it: bare for a
// function, receiverType.methodName for a method, with the receiver's pointer
// star stripped.
func funcDeclName(fn *ast.FuncDecl) string {
	if fn.Recv == nil || len(fn.Recv.List) == 0 {
		return fn.Name.Name
	}
	return strings.TrimPrefix(exprString(fn.Recv.List[0].Type), "*") + "." + fn.Name.Name
}

func exprString(e ast.Expr) string {
	switch e := e.(type) {
	case *ast.Ident:
		return e.Name
	case *ast.StarExpr:
		return "*" + exprString(e.X)
	case *ast.SelectorExpr:
		return exprString(e.X) + "." + e.Sel.Name
	case *ast.IndexExpr:
		return exprString(e.X)
	default:
		return ""
	}
}

type exactArithmeticSite struct {
	pos  token.Pos
	what string
}

// unmeteredExactArithmetic finds every place a body allocates a math/big value
// or calls an arithmetic method on one.
//
// A bare *big.Rat in a var declaration is not flagged: declaring the variable a
// fold's running maximum lives in allocates nothing and performs nothing. What
// is flagged is the allocation itself (new(big.Rat), big.NewRat, a composite
// literal) and every arithmetic call.
func unmeteredExactArithmetic(body *ast.BlockStmt) []exactArithmeticSite {
	var found []exactArithmeticSite
	ast.Inspect(body, func(n ast.Node) bool {
		switch n := n.(type) {
		case *ast.CallExpr:
			sel, ok := n.Fun.(*ast.SelectorExpr)
			if !ok {
				if ident, ok := n.Fun.(*ast.Ident); ok && ident.Name == "new" && len(n.Args) == 1 && mentionsBig(n.Args[0]) {
					found = append(found, exactArithmeticSite{n.Pos(), "new(" + exprString(n.Args[0]) + ")"})
				}
				return true
			}
			if ident, ok := sel.X.(*ast.Ident); ok && ident.Name == "big" {
				found = append(found, exactArithmeticSite{n.Pos(), "big." + sel.Sel.Name})
				return true
			}
			if _, ok := bigArithmeticMethods[sel.Sel.Name]; ok {
				found = append(found, exactArithmeticSite{n.Pos(), "." + sel.Sel.Name})
			}
		case *ast.CompositeLit:
			if mentionsBig(n.Type) {
				found = append(found, exactArithmeticSite{n.Pos(), exprString(n.Type) + "{...}"})
			}
		}
		return true
	})
	return found
}

func mentionsBig(e ast.Expr) bool {
	return strings.HasPrefix(strings.TrimPrefix(exprString(e), "*"), "big.")
}
