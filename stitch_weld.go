package decad

import (
	"math"

	"github.com/lestrrat-3d/r3"
)

// This file is docs/surface-design.md §6.2's Table J admission, and the
// shared vertex table it is decided over. buildStitchWeldPlan is the whole
// entry point: it registers every vertex reached from the operand faces
// Stitch is called with, decides which pairs of free edges join, and returns
// a *stitchWeldPlan that stitch.go's evalStitchContext replays — never
// re-derives — for both the initial build and every later Placed/Duplicate/
// PlacedCopy re-evaluation (docs/surface-design.md's "recorded weld"
// decision).
//
// No tolerance, residual or near-coincidence test appears anywhere in this
// file. §1.3 refuses a tolerant stitch permanently, and every admission below
// is decided by bit-identical held data or by a proven zero bound — never by
// how close two floats happen to sit.

// stitchVertexKey is the bit pattern of a vertex's three held coordinates.
// Using math.Float64bits rather than the coordinates themselves keeps a
// negative zero and a positive zero distinct keys, which is what the vertex
// table's own merge rule below requires: a vertex recorded as -0.0 and one
// recorded as 0.0 are not proven to denote the same point merely because
// they compare equal under ==.
type stitchVertexKey struct {
	x, y, z uint64
}

func stitchKeyOf(v r3.Vec) stitchVertexKey {
	return stitchVertexKey{math.Float64bits(v.X), math.Float64bits(v.Y), math.Float64bits(v.Z)}
}

// stitchVertexTable is the shared vertex table docs/surface-design.md §6
// names: every vertex reached from every operand face resolves to one
// table CLASS (an index into verts), and two vertices resolve to the SAME
// class only when their keys match AND both carry a zero bound.
//
// A bounded vertex always gets its own class even when another vertex holds
// the same coordinate: the merge condition is pairwise, so a coincidence
// between two vertices where either carries a nonzero bound proves nothing
// about the two true points they stand for, and welding them would be
// exactly the tolerant admission §1.3 refuses. This is J5 applied AT THE
// VERTEX, and it is what makes two triangles sharing a table index — the
// fact loft_audit.go's crossing audit is built on — a proof rather than a
// convention: the shared index exists only because both vertices are
// EXACT and IDENTICAL, never because they merely landed close.
//
// A key with more than one zero-bound vertex merges all of them into ONE
// class (pairwise zero-bound-and-equal-key is transitive once every member
// satisfies it against the first), while a key mixing zero- and
// nonzero-bound vertices merges only the zero-bound ones together and gives
// every nonzero-bound vertex its own class, exactly as the pairwise rule
// states.
//
// A SECOND, independent merge route exists beside the zero-bound one: two
// vertices with the SAME bit-identical key that also carry an equal
// non-zero CURVE token (sameCurve, denotation.go) merge into one class too,
// whatever bound either one carries — the shared-denotation certificate's
// own vertex-level admission (docs/surface-design.md §6.2's amendment). The
// certificate ADMITS; the bit-identical key is what NARROWS it — a curve
// token match with a DIFFERENT key never merges, exactly as
// CLAUDE.md's rule that bit-identity alone never admits is answered here by
// its converse: the certificate alone never admits either, without the key.
type stitchVertexTable struct {
	class           map[*Vertex]int
	zeroClassByKey  map[stitchVertexKey]int
	curveClassByKey map[stitchCurveKey]int
	verts           []r3.Vec
	// boundByClass is each class's own representative bound, read before any
	// placement widens it: 0 for a class that arose from a zero-bound merge
	// (every member is zero-bound by the merge rule itself), the FIRST
	// member's own bound for a class that arose from a curve-token merge
	// (both members bound the same true point, so either one's own bound is
	// a sound representative), or the one vertex's own bound for a singleton
	// class.
	boundByClass []float64
	// denotByClass is each class's own representative curve token, read from
	// the FIRST vertex registered for the class: the zero value for a class
	// no member carries one for, or a merge formed entirely over the
	// zero-bound route with no token in sight. Reusing the first member's own
	// token — rather than fabricating a fresh one — is what lets
	// rebuildStitchTopology restate an identity a LATER copy or stitch can
	// still recognize, and it stays sound whichever route merged the class: a
	// curve-token merge's members all carry the identical token by
	// construction, and a zero-bound merge's members all denote the same
	// TRUE point regardless of any token, so crediting that point with one
	// member's own proven identity asserts nothing false about the others.
	denotByClass []curveToken
}

// stitchCurveKey is a candidate vertex-class merge key for the CURVE route:
// the vertex's own bit-identical position key alongside the curveToken it
// carries — both must match another registered vertex's for the two to
// merge, so a token match at a DIFFERENT held coordinate never merges (the
// bit-identity guard, kept as a reject-only narrowing layered on the
// certificate).
type stitchCurveKey struct {
	key   stitchVertexKey
	curve curveID
	xform r3.Transform
}

func newStitchVertexTable() *stitchVertexTable {
	return &stitchVertexTable{class: map[*Vertex]int{}}
}

// classOf returns v's table index, registering v on first sight. Called
// again for the same *Vertex it returns the identical index: registration
// is idempotent, so callers may look a vertex up as many times as their own
// traversal happens to revisit it.
func (t *stitchVertexTable) classOf(v *Vertex) int {
	if idx, ok := t.class[v]; ok {
		return idx
	}
	key := stitchKeyOf(v.position)
	zero := v.bound.Base() == 0
	if zero {
		if idx, ok := t.zeroClassByKey[key]; ok {
			t.class[v] = idx
			return idx
		}
	}
	hasCurve := v.denot.id != 0
	var ckey stitchCurveKey
	if hasCurve {
		ckey = stitchCurveKey{key: key, curve: v.denot.id, xform: v.denot.xform}
		if idx, ok := t.curveClassByKey[ckey]; ok {
			t.class[v] = idx
			return idx
		}
	}
	idx := len(t.verts)
	t.verts = append(t.verts, v.position)
	t.boundByClass = append(t.boundByClass, v.bound.Base())
	t.denotByClass = append(t.denotByClass, v.denot)
	t.class[v] = idx
	if zero {
		if t.zeroClassByKey == nil {
			t.zeroClassByKey = map[stitchVertexKey]int{}
		}
		t.zeroClassByKey[key] = idx
	}
	if hasCurve {
		if t.curveClassByKey == nil {
			t.curveClassByKey = map[stitchCurveKey]int{}
		}
		t.curveClassByKey[ckey] = idx
	}
	return idx
}

// stitchWeldKey is the unordered pair of vertex-table classes a candidate
// free edge's two endpoints resolve to.
type stitchWeldKey struct{ a, b int }

func newStitchWeldKey(a, b int) stitchWeldKey {
	if a > b {
		a, b = b, a
	}
	return stitchWeldKey{a, b}
}

// stitchWeldPlan is Table J's admission decision over one Stitch call's whole
// operand set, computed once by buildStitchWeldPlan and replayed verbatim by
// every later placement (docs/surface-design.md's "recorded weld" decision):
// a rigid motion rounds every coordinate, so re-running Table J after a
// motion could admit a pair the unplaced geometry never proved coincident,
// or fail to admit one it did — either way a silently different topology
// than the one this plan already proved.
type stitchWeldPlan struct {
	table  *stitchVertexTable
	group  map[*Edge]int // welded edges only; absent means the edge stays free
	groups int
}

// buildStitchWeldPlan is Table J's whole admission (docs/surface-design.md
// §6.2), carried here verbatim from that section:
//
// Bit-identical held coordinates prove the HELD data equal; they prove
// nothing about the two TRUE curves when each is only known within its own
// bound. Two edges each bounded by 1e-6 mm may hold the same coordinate and
// lie 2e-6 mm apart, and welding them would claim a meeting that was never
// proven. A zero bound closes the gap: the held value IS the true value, so
// held equality is true coincidence. No tolerance, residual or
// near-coincidence test appears anywhere in this file.
//
// J1 (free) and J2 (same curve variant) narrow the field before J3/J4/J5 are
// even asked: correction 4 over docs/surface-design.md's original Table J
// resolves J5 as UNDECIDABLE for every variant but Line3, because Circle3
// and Arc3 carry Center, Axis and Radius with no bound field at all — there
// is no held value to ask "does this carry a zero bound" of. Line3 alone
// admits J5, and it does so VACUOUSLY: a straight edge's whole geometry
// lives in its two vertices, which J4 already compares, so J3 (the
// variant's own parameters) states nothing further to prove. So a free
// edge whose Curve is Circle3, Arc3, NURBSCurve or FacetedCurve DECLINES
// outright and stays free — Table C's first row, and never an error — while
// a free Line3 edge is admitted to weld exactly when another free Line3
// edge shares its unordered pair of vertex-table classes: J4 (bit-identical
// endpoints, matched unordered) and the vertex half of J5 (both endpoints
// zero-bound) are then both already proven by the SHARED CLASS itself,
// since stitchVertexTable never merges two vertices unless both hold.
//
// Grouping every admitted free edge by that unordered class pair and welding
// a group of exactly two is the rest of §6.2/§6.4: a group of one stays free
// (a residual, Table C's first row again), and a group of three or more
// joins NONE of its members — welding an arbitrary two would be a silent
// choice among equally-claimed edges and welding all three would be
// non-manifold, so every edge in an ambiguous group stays free and the
// result names the ambiguity through its own residual free edges.
//
// A free edge whose Curve is NOT Line3 still enters this grouping when it
// carries a non-zero CURVE token (denotation.go): the shared-denotation
// certificate's own edge-level admission, which lifts J5 for exactly that
// pair rather than deciding it for the variant in general (a Circle3/Arc3
// edge minted by no builder still declines, as it always has). Two edges
// sharing one weld key are only actually WELDED, though, when
// stitchEdgesJoin says so — both Line3 (J3 vacuous, J4/J5 already proven by
// the shared class), or an equal non-zero curve token AND a bit-identical
// curve variant (J2/J3's own reject-only guard, kept layered on the
// certificate exactly as J4's shared-class guard already is).
func buildStitchWeldPlan(faces []*Face) *stitchWeldPlan {
	table := newStitchVertexTable()
	for _, f := range faces {
		for _, l := range f.loops {
			for _, ce := range l.coedges {
				table.classOf(ce.Start())
				table.classOf(ce.End())
			}
		}
	}

	byKey := map[stitchWeldKey][]*Edge{}
	var order []stitchWeldKey
	seen := map[*Edge]struct{}{}
	for _, f := range faces {
		for _, l := range f.loops {
			for _, ce := range l.coedges {
				e := ce.edge
				if _, dup := seen[e]; dup {
					continue
				}
				seen[e] = struct{}{}
				if !e.IsFree() { // J1
					continue
				}
				_, isLine := e.curve.(Line3)
				if !isLine && e.denot.id == 0 { // J2, then J5 by correction 4, or the certificate
					continue
				}
				key := newStitchWeldKey(table.classOf(e.start), table.classOf(e.end))
				if _, ok := byKey[key]; !ok {
					order = append(order, key)
				}
				byKey[key] = append(byKey[key], e)
			}
		}
	}

	plan := &stitchWeldPlan{table: table, group: map[*Edge]int{}}
	for _, key := range order {
		edges := byKey[key]
		if len(edges) != 2 {
			// One: a residual free edge, not an error. Three or more: a weld
			// key claimed by three or more free edges joins none of them —
			// all stay free and the result's own free edges name the
			// ambiguity.
			continue
		}
		if !stitchEdgesJoin(edges[0], edges[1]) {
			continue
		}
		id := plan.groups
		plan.groups++
		plan.group[edges[0]] = id
		plan.group[edges[1]] = id
	}
	return plan
}

// stitchEdgesJoin decides Table J's J2/J3 for a pair of free edges that
// already share a weld key (so J4/J5's vertex half is already proven by the
// shared class, either by the zero-bound route or the certificate's own
// vertex-level route). A straight edge pair needs nothing further: a
// Line3's whole geometry lives in its two vertices, so J3 states nothing
// beyond what the shared class already proved (§6.2's amendment). Any other
// pair joins only through the shared-denotation certificate: an equal
// non-zero curve token (sameCurve, denotation.go) AND a bit-identical curve
// variant (sameCurveVariant) — the certificate admits, and the bit-identical
// guard is what keeps admission from ever resting on the token alone.
func stitchEdgesJoin(a, b *Edge) bool {
	_, aLine := a.curve.(Line3)
	_, bLine := b.curve.(Line3)
	if aLine && bLine {
		return true
	}
	return sameCurve(a.denot, b.denot) && sameCurveVariant(a.curve, b.curve)
}

// sameCurveVariant is Table J's J3 for a non-Line3 pair: bit-identical held
// float64 parameters, up to the one sign freedom the variant allows. Line3
// is vacuous here (stitchEdgesJoin never calls this for two Line3 edges) —
// its own J3 is answered entirely by J4's shared vertex class. Circle3 and
// Arc3 compare Center and Radius bit-identical and Axis up to sign (a
// reflected or oppositely-walked copy of the same circle carries the
// negated axis). Any other pairing, including a mismatched variant type,
// declines: this evaluator mints no curve token for a NURBSCurve or
// FacetedCurve edge, so the certificate's own edge-level route never
// reaches them regardless.
func sameCurveVariant(a, b Curve) bool {
	switch av := a.(type) {
	case Circle3:
		bv, ok := b.(Circle3)
		if !ok {
			return false
		}
		return av.Center == bv.Center && av.Radius.Base() == bv.Radius.Base() &&
			(av.Axis == bv.Axis || av.Axis == bv.Axis.Scale(-1))
	case Arc3:
		bv, ok := b.(Arc3)
		if !ok {
			return false
		}
		return av.Center == bv.Center && av.Radius.Base() == bv.Radius.Base() &&
			(av.Axis == bv.Axis || av.Axis == bv.Axis.Scale(-1))
	default:
		return false
	}
}
