package decad

import (
	"go/ast"
	"go/constant"
	"go/token"
	"go/types"
	"path/filepath"
	"slices"
	"sort"
	"strings"
	"testing"

	"github.com/stretchr/testify/require"
)

// identityTypes are the three identity types the digest relabels by first visit.
var identityTypes = []string{
	"github.com/lestrrat-3d/decad/internal/provenance.Producer",
	"github.com/lestrrat-3d/decad/internal/denotation.LevelID",
	"github.com/lestrrat-3d/decad/internal/denotation.CurveID",
}

func isIdentityType(t types.Type) bool {
	n, ok := types.Unalias(t).(*types.Named)
	return ok && n.Obj().Pkg() != nil && slices.Contains(identityTypes, namedKey(n))
}

// identityUse is one operation on an identity value other than comparison for equality, found in function fn of file.
type identityUse struct {
	file, fn string
}

func (u identityUse) String() string { return u.file + " " + u.fn }

// containsIdentity reports whether t is an identity type or a slice, array, pointer or map of one. A struct that holds
// an id is not followed: a generic function cannot do arithmetic on a field it cannot name.
func containsIdentity(t types.Type, seen map[types.Type]struct{}) bool {
	if t == nil {
		return false
	}
	if isIdentityType(t) {
		return true
	}
	if _, ok := seen[t]; ok {
		return false
	}
	seen[t] = struct{}{}
	switch u := types.Unalias(t).Underlying().(type) {
	case *types.Slice:
		return containsIdentity(u.Elem(), seen)
	case *types.Array:
		return containsIdentity(u.Elem(), seen)
	case *types.Pointer:
		return containsIdentity(u.Elem(), seen)
	case *types.Map:
		return containsIdentity(u.Key(), seen) || containsIdentity(u.Elem(), seen)
	}
	return false
}

// comparesToPositiveConstant reports an == or != of an identity against a constant above 0, which names one
// particular id and so is not relabel-safe.
func comparesToPositiveConstant(info *types.Info, n *ast.BinaryExpr) bool {
	for _, pair := range [][2]ast.Expr{{n.X, n.Y}, {n.Y, n.X}} {
		if !isIdentityType(info.TypeOf(pair[0])) {
			continue
		}
		tv, ok := info.Types[pair[1]]
		if !ok || tv.Value == nil {
			continue
		}
		if v, ok := constant.Int64Val(tv.Value); ok && v > 0 {
			return true
		}
	}
	return false
}

// passesIdentityToInterface reports a call that hands an identity to an interface-typed parameter, which boxes it.
func passesIdentityToInterface(info *types.Info, n *ast.CallExpr) bool {
	sig, ok := info.TypeOf(n.Fun).(*types.Signature)
	if !ok {
		return false
	}
	params := sig.Params()
	for i, a := range n.Args {
		if !isIdentityType(info.TypeOf(a)) {
			continue
		}
		var pt types.Type
		switch {
		case sig.Variadic() && i >= params.Len()-1:
			pt = params.At(params.Len() - 1).Type()
			if s, ok := pt.(*types.Slice); ok && !n.Ellipsis.IsValid() {
				pt = s.Elem()
			}
		case i < params.Len():
			pt = params.At(i).Type()
		default:
			continue
		}
		if _, isIface := pt.Underlying().(*types.Interface); isIface {
			return true
		}
	}
	return false
}

// scanIdentityArithmetic lists every operation outside == and != on an identity type, every conversion of one to
// another type, every use of one as an index, every comparison of one with a positive constant, min and max over
// ids, a switch on an id, an id boxed into an interface parameter, and a generic instantiated with a type that
// holds an id.
func scanIdentityArithmetic(ms *moduleSource) []identityUse {
	var uses []identityUse
	for _, tp := range ms.pkgs {
		for _, f := range tp.files {
			file := strings.TrimPrefix(filepath.ToSlash(ms.fset.Position(f.Pos()).Filename), "./")
			for _, decl := range f.Decls {
				fn := declName(decl)
				ast.Inspect(decl, func(n ast.Node) bool {
					scanIdentityNode(tp.info, n, func() { uses = append(uses, identityUse{file, fn}) })
					return true
				})
			}
		}
	}
	sort.Slice(uses, func(i, j int) bool { return uses[i].String() < uses[j].String() })
	return slices.Compact(uses)
}

func declName(d ast.Decl) string {
	fd, ok := d.(*ast.FuncDecl)
	if !ok {
		return "<package>"
	}
	return fd.Name.Name
}

func scanIdentityNode(info *types.Info, n ast.Node, add func()) {
	typeOf := func(e ast.Expr) types.Type { return info.TypeOf(e) }
	switch n := n.(type) {
	case *ast.BinaryExpr:
		if n.Op == token.EQL || n.Op == token.NEQ {
			if comparesToPositiveConstant(info, n) {
				add()
			}
			return
		}
		if isIdentityType(typeOf(n.X)) || isIdentityType(typeOf(n.Y)) {
			add()
		}
	case *ast.UnaryExpr:
		if isIdentityType(typeOf(n.X)) && n.Op != token.AND {
			add()
		}
	case *ast.IncDecStmt:
		if isIdentityType(typeOf(n.X)) {
			add()
		}
	case *ast.AssignStmt:
		if n.Tok == token.ASSIGN || n.Tok == token.DEFINE {
			return
		}
		if isIdentityType(typeOf(n.Lhs[0])) {
			add()
		}
	case *ast.IndexExpr:
		if tv := info.Types[n.Index]; !tv.IsType() && isIdentityType(typeOf(n.Index)) {
			if _, isMap := typeOf(n.X).Underlying().(*types.Map); !isMap {
				add()
			}
		}
	case *ast.SwitchStmt:
		if n.Tag != nil && isIdentityType(typeOf(n.Tag)) {
			add()
		}
	case *ast.Ident:
		inst, ok := info.Instances[n]
		if !ok || n.Name == "TypeFor" { // reflect.TypeFor reads only the type
			return
		}
		for ta := range inst.TypeArgs.Types() {
			if containsIdentity(ta, map[types.Type]struct{}{}) {
				add()
			}
		}
	case *ast.CallExpr:
		if passesIdentityToInterface(info, n) {
			add()
		}
		if id, ok := ast.Unparen(n.Fun).(*ast.Ident); ok {
			if b, ok := info.Uses[id].(*types.Builtin); ok && (b.Name() == "min" || b.Name() == "max") {
				for _, a := range n.Args {
					if isIdentityType(typeOf(a)) {
						add()
					}
				}
			}
		}
		tv, ok := info.Types[n.Fun]
		if !ok || !tv.IsType() || len(n.Args) != 1 {
			return
		}
		from, to := typeOf(n.Args[0]), tv.Type
		if isIdentityType(to) && !isIdentityType(from) {
			add()
		}
		if isIdentityType(from) && !isIdentityType(to) {
			add()
		}
	}
}

// identityArithmeticAllowList names every function that does more with a producer, level or curve identity than
// compare it for equality, and says why that does not matter to the digest. The digest replaces each identity by
// the order of its first visit, so two bodies built after different histories get equal digests. That is sound
// only when the evaluator reads an identity by equality or sign, and when every identity it computes is the fresh
// id of a result. Condition C2 of agent-docs/body-digest-design.md.
//
// A new function that does arithmetic on an identity fails the test until a reviewer reads it and adds it here.
const (
	advancesCounter = "advances the document counter"
	idFieldCopy     = "int fields of digestIDFields"
)

var identityArithmeticAllowList = map[string]string{
	// Fresh result ids: a base taken from the document counter (nextProducerID, MintLevel, MintCurve), offset by a
	// loop index. No input identity enters the sum.
	"boolean.go performBoolean":                                       "base of a paired Cut's span, from the result's own fresh producer",
	"boolean_group_blind.go commitBlindGroupCut":                      advancesCounter,
	"boolean_regroup.go commitPairedCut":                              advancesCounter,
	"boolean_regroup.go pairedCutContinuation":                        "mints step ids from the counter; the < and > test only that the counter has room for the span",
	"brep_modify.go commitResult":                                     "span of the result",
	"document.go commitIn":                                            advancesCounter,
	"document.go commitOperationManyFacesIn":                          advancesCounter,
	"document.go commitOperationSpanPrefixIn":                         advancesCounter,
	"operation_history.go advanceProducerSpan":                        advancesCounter,
	"operation_history.go nextProducerID":                             "reads the document counter, which names only the uncommitted result",
	"internal/topo/document.go MintCurve":                             "advances the curve counter",
	"internal/topo/document.go MintLevel":                             "advances the level counter",
	"pattern.go Patterned":                                            "offsets from the counter by instance index",
	"pattern.go buildInstances":                                       "offsets from the counter by instance index",
	"tool_combine.go combineTools":                                    "offsets from the counter by step",
	"tool_combine.go combineToolSteps":                                "offsets from the counter by step",
	"unstitch.go Unstitch":                                            "offsets from the counter by face index",
	"internal/booleandispatch/regroup.go regroup":                     "offsets the env ref by the steps already built",
	"internal/booleandispatch/rim_split_rebuild.go RebuildUnionSplit": "offsets the env ref by the steps already built",
	"internal/brepmodify/cap_edge_cutter.go obliqueCapEdgeCut":        "env ref plus one",
	"internal/brepmodify/chain_cutter_chamfer.go cutCrossChains":      "env ref plus steps used",
	"internal/brepmodify/freeform_rim.go cutRims":                     "env ref plus steps used",
	"internal/brepmodify/partial_loop_chamfer.go cutRuns":             "env ref plus steps used",
	"internal/brepmodify/retained_loop_chamfer.go cutLoops":           "env ref plus steps used",
	"internal/brepmodify/rounded_fillet.go RoundedBrepCapEdgeFillet":  "env ref plus one",
	"internal/retainedblend/saddle_tool.go cutSaddles":                "env ref plus one",
	"internal/surfacetrim/ops.go Split":                               "env ref plus piece index",
	"internal/surfacetrim/ops.go splitPointPlane":                     "env ref plus one",
	"internal/surfacetrim/split_revolve.go SplitRevolve":              "env ref plus piece index",
	"internal/unionmany/prism_chain.go PrismChain":                    "env ref plus pair index",

	// A comparison with a constant above 0 names one id. Only tests reach it, never the Intersect path.
	"internal/loftcontact/quarter_held_pair.go quarterPairLive": "tests-only helper, off the Intersect path; compares a producer with a constant",

	// A generic instantiated with the producer type.
	"internal/extent/through_all.go ThroughAll": "ThroughStops[T any] stores the producer and returns it unread",

	// Sign checks: an id of 0 or less means "none", which the digest keeps raw.
	"selector.go validatePredicateRef":                                      "refuses a negative producer in a caller's ref",
	"internal/facetedrecord/exact_copy_moments.go NewExactCopyMomentRecord": "refuses a producer of 0 or less, then copies it into the int fields of digestIDFields",

	// Copies of a producer into an int field that digestIDFields relabels with the producer type, or into text.
	"internal/facetedrecord/exact_copy_moments.go Read":                               idFieldCopy,
	"internal/massdispatch/contained_cut.go CaptureContainedCutMoments":               idFieldCopy,
	"internal/massdispatch/contained_cut_quantities.go CaptureContainedCutQuantities": idFieldCopy,
	"internal/massdispatch/exact_copy.go CaptureExactCopyMoments":                     idFieldCopy,
	"internal/massdispatch/exact_copy.go sourceMomentSnapshot":                        idFieldCopy,
	"selection_error.go renderRef":                                                    "error text, which the digest's equal answers do not cover",
}

func TestIdentityArithmeticAllowList(t *testing.T) {
	ms := loadModuleSource(t)
	found := map[string]struct{}{}
	for _, u := range scanIdentityArithmetic(ms) {
		found[u.String()] = struct{}{}
	}
	var unlisted []string
	for k := range found {
		if _, ok := identityArithmeticAllowList[k]; !ok {
			unlisted = append(unlisted, k)
		}
	}
	sort.Strings(unlisted)
	require.Empty(t, unlisted,
		"these functions do arithmetic on, or convert, a producer, level or curve id: read each and list it in identityArithmeticAllowList")
	for k := range identityArithmeticAllowList {
		_, ok := found[k]
		require.True(t, ok, "%s is listed but no longer does arithmetic on an id", k)
	}

	// Every int field that carries a producer is listed in digestIDFields.
	for typ, fields := range digestIDFields {
		for name := range fields {
			f, ok := typ.FieldByName(name)
			require.True(t, ok, "%s.%s", typ, name)
			require.Equal(t, "int", f.Type.String())
		}
	}
}
