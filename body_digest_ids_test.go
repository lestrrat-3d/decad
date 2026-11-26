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

// containsIdentityDeep reports whether t is an identity type, or a struct, pointer, slice, array or map that holds
// one at any depth. A value of such a type, handed to fmt or reflect, can have its id read or formatted.
func containsIdentityDeep(t types.Type, seen map[types.Type]struct{}) bool {
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
		return containsIdentityDeep(u.Elem(), seen)
	case *types.Array:
		return containsIdentityDeep(u.Elem(), seen)
	case *types.Pointer:
		return containsIdentityDeep(u.Elem(), seen)
	case *types.Map:
		return containsIdentityDeep(u.Key(), seen) || containsIdentityDeep(u.Elem(), seen)
	case *types.Struct:
		for f := range u.Fields() {
			if containsIdentityDeep(f.Type(), seen) {
				return true
			}
		}
	}
	return false
}

// passesIdentityToFmtOrReflect reports a call to a function or method of package fmt or reflect with an argument whose
// static type holds an identity at any depth.
func passesIdentityToFmtOrReflect(info *types.Info, n *ast.CallExpr) bool {
	sel, ok := ast.Unparen(n.Fun).(*ast.SelectorExpr)
	if !ok {
		return false
	}
	fn, ok := info.Uses[sel.Sel].(*types.Func)
	if !ok || fn.Pkg() == nil || (fn.Pkg().Path() != "fmt" && fn.Pkg().Path() != "reflect") {
		return false
	}
	for _, a := range n.Args {
		if containsIdentityDeep(info.TypeOf(a), map[types.Type]struct{}{}) {
			return true
		}
	}
	return false
}

// isPositiveIDConstant reports whether e is a constant expression of an identity type with a value above 0. Such a
// constant names one particular id, whether it is compared, passed as an implicit argument, stored in a composite
// literal, declared as a typed constant or assigned to a variable.
func isPositiveIDConstant(info *types.Info, e ast.Expr) bool {
	tv, ok := info.Types[e]
	if !ok || tv.Value == nil || !isIdentityType(tv.Type) {
		return false
	}
	v, ok := constant.Int64Val(constant.ToInt(tv.Value))
	return ok && v > 0
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
// another type, every use of one as an index or slice bound, every constant of an id type above 0, min, max and
// make over ids, a range over one, a switch on an id, an id boxed into an interface parameter, and a generic
// instantiated with a type that holds an id.
//
// It also lists every call into fmt or reflect with an argument whose static type holds an id at any depth: a
// struct field, pointer, slice, array or map element or map key. Such a call can print an id or read its value.
//
// The scan does not cover everything. It misses an id that reaches fmt or reflect through a value already typed as
// an interface (any or a named interface), and an id boxed anywhere other than as a bare id argument. It misses a
// generic function instantiated with a struct that holds an id, because containsIdentity does not follow struct
// fields. It misses a map keyed by a struct that holds an id and later iterated in key order. Planted examples of
// all three (a generic struct field, an ordered map of struct keys, an id kept in an any variable) are not
// reported. Closing them needs boxing and generic scans, so do not read a green run as proof that no id is ordered
// or formatted.
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

// scanBuiltinIDArgs reports min and max over ids, and make with an id as a length or capacity.
func scanBuiltinIDArgs(info *types.Info, name string, args []ast.Expr, add func()) {
	switch name {
	case "min", "max":
	case "make":
		if len(args) > 0 {
			args = args[1:]
		}
	default:
		return
	}
	for _, a := range args {
		if isIdentityType(info.TypeOf(a)) {
			add()
		}
	}
}

func scanIdentityNode(info *types.Info, n ast.Node, add func()) {
	typeOf := func(e ast.Expr) types.Type { return info.TypeOf(e) }
	if e, ok := n.(ast.Expr); ok && isPositiveIDConstant(info, e) {
		add()
	}
	switch n := n.(type) {
	case *ast.SliceExpr:
		for _, e := range []ast.Expr{n.Low, n.High, n.Max} {
			if e != nil && isIdentityType(typeOf(e)) {
				add()
			}
		}
	case *ast.RangeStmt:
		if isIdentityType(typeOf(n.X)) {
			add()
		}
	case *ast.BinaryExpr:
		if n.Op == token.EQL || n.Op == token.NEQ {
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
		if c, ok := info.Defs[n].(*types.Const); ok && isIdentityType(c.Type()) {
			if v, ok := constant.Int64Val(constant.ToInt(c.Val())); ok && v > 0 {
				add()
			}
		}
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
		if passesIdentityToInterface(info, n) || passesIdentityToFmtOrReflect(info, n) {
			add()
		}
		if id, ok := ast.Unparen(n.Fun).(*ast.Ident); ok {
			if b, ok := info.Uses[id].(*types.Builtin); ok {
				scanBuiltinIDArgs(info, b.Name(), n.Args, add)
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
	errorTextOnly   = "formats the value into the text of a returned modify error; no answer, ordering or digest reads it"
)

var identityArithmeticAllowList = map[string]string{
	// Fresh result ids: a base taken from the document counter (nextProducer, MintLevel, MintCurve), offset by a
	// loop index. No input identity enters the sum.
	"boolean.go performBoolean":                                       "base of a paired Cut's span, from the result's own fresh producer",
	"boolean_regroup.go pairedCutContinuation":                        "mints step ids from the counter; the < and > test only that the counter has room for the span",
	"brep_modify.go commitResult":                                     "span of the result",
	"document.go advanceProducer":                                     advancesCounter,
	"document.go commitOperationManyFacesIn":                          advancesCounter,
	"document.go commitOperationSpanPrefixIn":                         advancesCounter,
	"operation_history.go advanceProducerSpan":                        advancesCounter,
	"operation_history.go nextProducer":                               "reads the document counter, which names only the uncommitted result",
	"internal/topo/document.go MintCurve":                             "advances the curve counter",
	"internal/topo/document.go MintLevel":                             "advances the level counter",
	"internal/topo/document.go WithTransientIDs":                      "seeds a private transient counter from the document counters",
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

	// A constant above 0 of an id type: a span or step count of 1, passed to advanceProducer or returned as a span.
	// Each function was read and passes a count only, never the id of an input.
	"boolean_group_blind.go commitBlindGroupCut": "advanceProducer(ctx, 1) passes a span of one",
	"boolean_many.go unionManyBodies":            "returns a span of 1 for the orthogonal route",
	"document.go commitIn":                       "advanceProducer(ctx, 1) passes a span of one",
	"document.go commitOperationIn":              "passes a span of 1 to commitOperationSpanIn",
	"internal/unionmany/routes.go Routes":        "returns a span of 1 for a single-route result",
	"union_refined.go UnionRefined":              "passes a span of 1 to commitUnionMany",

	// A fixture constant: the producer id every kit body carries. Only the tests of internal/trimgraph,
	// internal/trimrebuild and internal/trimshell import the package, so no Intersect path reaches it.
	"internal/trimtest/trimtest.go <package>":  "test kit: fixed producer id 9 for fixture bodies",
	"internal/trimtest/trimtest.go Env":        "test kit: builds an env with the fixed producer id",
	"internal/trimtest/trimtest.go PlaneGroup": "test kit: stamps fixture origins with the fixed producer id",

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

	// A call into fmt or reflect with an argument whose type holds an id. Each was read.
	"body_digest.go digestMemo":                                        "reflect.ValueOf(b) feeds the digest encoder, which relabels every id by first visit",
	"body_digest.go bodyDigestSideTables":                              "reflect.ValueOf of document records handed to the digest encoder, which relabels every id by first visit",
	"internal/topo/mesh.go facetBoundKey":                              "reflect Pointer() of the face-bound map: its address only, never an id, and the mesh is skipped by the digest",
	"internal/brepmodify/chamfer.go Chamfer":                           errorTextOnly,
	"internal/brepmodify/displaced_blend.go displacedLateralBlend":     errorTextOnly,
	"internal/brepmodify/fillet.go Fillet":                             errorTextOnly,
	"internal/brepmodify/revolve_blend.go BlendRevolveJunctions":       errorTextOnly,
	"internal/loftcylinder/chamfer_middle_reach.go ChamferMiddleFacet": errorTextOnly,
	"modify_options.go resolveAsymmetricReference":                     errorTextOnly,
	"selection_error.go renderRef":                                     "error text, which the digest's equal answers do not cover",
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
	var stale []string
	for k := range identityArithmeticAllowList {
		if _, ok := found[k]; !ok {
			stale = append(stale, k)
		}
	}
	sort.Strings(stale)
	require.Empty(t, stale, "these functions are listed but no longer do arithmetic on an id")

	// Every int field that carries a producer is listed in digestIDFields.
	for typ, fields := range digestIDFields {
		for name := range fields {
			f, ok := typ.FieldByName(name)
			require.True(t, ok, "%s.%s", typ, name)
			require.Equal(t, "int", f.Type.String())
		}
	}
}
