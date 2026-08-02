package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// This file is PR1 of docs/prism-boolean-design.md: the analytic reduction of
// Union over two co-directional coplanar straight prisms, routed entirely
// through sketch (§4) rather than through the mesh boolean (evaluator-design
// §9). The entry gate (§3) is reject-only and never surfaces an error on a
// miss — the caller falls back to the unchanged mesh path exactly as it did
// before this file existed. Only once §4.2's resolution finds a unique
// candidate does a further problem become a genuine, typed refusal (§3.4,
// §9) rather than a reroute to the mesh path.
//
// PR1 implements Union's hole-free select-all/merge/chain path only (§4.2);
// Cut/Intersect's clean-nesting structural match (PR2) and the general
// per-cell edge-orientation classification (PR3) are not implemented, so
// those ops — and any Union topology this increment's chain-closure cannot
// resolve — fall through to tryPrismUnion returning ok=false, err=nil.

// tryPrismUnion attempts the PR1 analytic reduction for op. ok=false (err
// always nil in that case) means "not admitted" per §3.1/§3.4: the caller
// MUST fall back to the unchanged mesh path with no error surfaced. A
// non-nil err means resolution found a unique candidate (§3.4's point of no
// return) and then failed — the caller MUST propagate it rather than reroute
// to the mesh path.
func tryPrismUnion(ctx context.Context, op OpKind, a, b *Body) (prismPayload, bool, error) {
	// §4.2 implements resolution for Union only; every other op falls
	// through even where G1-G5 would pass (§4.4).
	if op != OpUnion {
		return prismPayload{}, false, nil
	}
	pa, pb, ok := admitPrismPair(a, b) // G1-G4
	if !ok {
		return prismPayload{}, false, nil
	}
	if len(pa.profile.Holes) != 0 || len(pb.profile.Holes) != 0 { // G6
		return prismPayload{}, false, nil
	}
	if !prismUnionZIntervalMatches(pa, pb) { // G5, §3.2's Union row
		return prismPayload{}, false, nil
	}

	budget := newWorkBudget(ctx) // §10: one counter for the whole attempt
	merged, resolved, err := resolvePrismUnion(budget, pa, pb)
	if err != nil {
		return prismPayload{}, false, err
	}
	if !resolved {
		return prismPayload{}, false, nil // §4.4: this topology is unresolved
	}

	// Point of no return (§3.4): every further problem is a genuine refusal.
	if err := auditPrismUnionSection(budget, pa, merged); err != nil {
		return prismPayload{}, false, err
	}
	result := prismPayload{
		profile: merged,
		frame:   pa.frame,
		z0:      pa.z0,
		z1:      pa.z1,
		xform:   pa.xform,
	}
	return result, true, nil
}

// admitPrismPair checks G1 (both operands a prismPayload), G2 (neither
// placement a reflection), G3 (the composed world planes are the same plane,
// exactly) and G4 (every segment of both records is line/circle/arc) — §3.1.
// G3's comparisons are ordinary Go == on the stored r3.Vec/float64 values
// (the design's "bit-identical" / "the literal zero"): Go's == already
// treats -0.0 and 0.0 as equal, so no separate zero-sign handling is needed.
// A miss on any row returns ok=false, never an error (§3.1: "passing them is
// not admission" for what follows, but MISSING one is never a refusal).
func admitPrismPair(a, b *Body) (pa, pb prismPayload, ok bool) {
	pa, aok := a.payload.(prismPayload) // G1
	pb, bok := b.payload.(prismPayload)
	if !aok || !bok {
		return prismPayload{}, prismPayload{}, false
	}
	if pa.reflected() || pb.reflected() { // G2
		return prismPayload{}, prismPayload{}, false
	}
	if !prismProfileIsAnalytic(pa.profile) || !prismProfileIsAnalytic(pb.profile) { // G4
		return prismPayload{}, prismPayload{}, false
	}
	worldNormalA := pa.xform.ApplyDir(pa.frame.N())
	worldNormalB := pb.xform.ApplyDir(pb.frame.N())
	if worldNormalA != worldNormalB { // G3: co-directional, bit-identical
		return prismPayload{}, prismPayload{}, false
	}
	worldOriginA := pa.xform.Apply(pa.frame.Origin())
	worldOriginB := pb.xform.Apply(pb.frame.Origin())
	if worldOriginB.Sub(worldOriginA).Dot(worldNormalA) != 0.0 { // G3: coplanar
		return prismPayload{}, prismPayload{}, false
	}
	return pa, pb, true
}

// prismProfileIsAnalytic reports G4: every segment of every loop is a
// LineSeg, CircleSeg or ArcSeg. A single free-form segment blinds sketch's
// whole-scene TExact gate (§3.1's own reasoning), so the class excludes the
// kind entirely rather than admitting "the free-form parts don't touch."
func prismProfileIsAnalytic(p ProfileRecord) bool {
	for _, loop := range append([]LoopRecord{p.Outer}, p.Holes...) {
		for _, seg := range loop.Segments {
			switch seg.(type) {
			case LineSeg, CircleSeg, ArcSeg:
			default:
				return false
			}
		}
	}
	return true
}

// prismUnionZIntervalMatches is G5 for Union (§3.2): operand B's [z0, z1] is
// re-expressed onto operand A's normal axis by the exact origin shift G3
// already certified lies on a shared axis, and Union requires the two
// intervals to match exactly.
func prismUnionZIntervalMatches(pa, pb prismPayload) bool {
	worldNormalA := pa.xform.ApplyDir(pa.frame.N())
	worldOriginA := pa.xform.Apply(pa.frame.Origin())
	worldOriginB := pb.xform.Apply(pb.frame.Origin())
	shift := worldOriginB.Sub(worldOriginA).Dot(worldNormalA)
	return pa.z0 == pb.z0+shift && pa.z1 == pb.z1+shift
}

// resolvePrismUnion is §4.2's hole-free select-all/merge/chain path.
// resolved=false (err always nil in that case) means this pair's topology is
// unresolved (§4.4): the caller falls back to the mesh path with no error. A
// non-nil error — including ctx cancellation surfacing through budget — is
// always genuine and must propagate.
func resolvePrismUnion(budget *workBudget, pa, pb prismPayload) (ProfileRecord, bool, error) {
	s, err := buildPrismUnionScene(budget, pa, pb)
	if err != nil {
		return ProfileRecord{}, false, err
	}
	if err := budget.err(); err != nil {
		return ProfileRecord{}, false, err
	}
	profiles := s.Profiles()
	if err := budget.step(); err != nil {
		return ProfileRecord{}, false, err
	}
	if len(profiles) == 0 {
		return ProfileRecord{}, false, nil // §4.4: the scene holds no bounded cell at all
	}

	// Union, hole-free operands (G6): select every returned cell — by
	// construction there is no bounded cell that is material of neither
	// operand (§4.2). Count every boundary edge (Outer and every Hole loop
	// of every selected cell) by (Entity, TStart, TEnd): a shared wall
	// between two adjacent cells is walked in opposite senses by each but
	// reports the SAME natural-direction TStart < TEnd range (Reversed, not
	// the order, states the walk direction — sketch-seam-design's own
	// contract), so this key alone matches it on both sides.
	type edgeKey struct {
		entity sketch.Entity
		t0, t1 float64
	}
	counts := map[edgeKey]int{}
	for _, p := range profiles {
		if err := budget.step(); err != nil {
			return ProfileRecord{}, false, err
		}
		if !p.Valid {
			// RB1: a candidate region the merge depends on is invalid. Every
			// cell in this two-operand scene is selected for Union, so any
			// invalid cell is a genuine refusal, not an unresolved topology.
			return ProfileRecord{}, false, fmt.Errorf(`%w: the union scene's arrangement reports an invalid region`, ErrUnsupported)
		}
		for _, loop := range append([][]sketch.BoundaryEdge{p.Outer}, p.Holes...) {
			for _, e := range loop {
				if err := budget.step(); err != nil {
					return ProfileRecord{}, false, err
				}
				counts[edgeKey{entity: e.Entity, t0: e.TStart, t1: e.TEnd}]++
			}
		}
	}

	// Second pass, in the arrangement's own deterministic profile/edge
	// order: keep every edge counted exactly once (a wall against the
	// unbounded exterior, or a coincident-carrier wall reported under only
	// one operand's entity); drop every edge counted exactly twice (an
	// interior wall between two selected cells). A count outside {1, 2} is
	// topology this increment does not cover.
	var survivors []sketch.BoundaryEdge
	for _, p := range profiles {
		for _, loop := range append([][]sketch.BoundaryEdge{p.Outer}, p.Holes...) {
			for _, e := range loop {
				if err := budget.step(); err != nil {
					return ProfileRecord{}, false, err
				}
				n := counts[edgeKey{entity: e.Entity, t0: e.TStart, t1: e.TEnd}]
				switch n {
				case 1:
					survivors = append(survivors, e)
				case 2:
					// dropped: an interior wall
				default:
					return ProfileRecord{}, false, nil // §4.4: not a shape this increment covers
				}
			}
		}
	}
	if len(survivors) == 0 {
		return ProfileRecord{}, false, nil
	}

	chain, resolved, err := chainPrismUnionSurvivors(budget, survivors)
	if err != nil {
		return ProfileRecord{}, false, err
	}
	if !resolved {
		return ProfileRecord{}, false, nil // §4.4: the survivors do not close into one simple loop
	}

	// Point of no return crossed within this function's own contract: every
	// further problem (a rejected TExact fragment, §9's RB8) is genuine.
	segs := make([]CurveSegment, len(chain))
	for i, e := range chain {
		if err := budget.step(); err != nil {
			return ProfileRecord{}, false, err
		}
		seg, err := recordEdge(e)
		if err != nil {
			return ProfileRecord{}, false, err
		}
		segs[i] = seg
	}
	return ProfileRecord{Outer: LoopRecord{Segments: segs}}, true, nil
}

// chainPrismUnionSurvivors chains the surviving boundary edges into one
// closed directed walk (§4.2), the same directed-edge-loop-closure shape
// boolean_body.go's face-patch construction performs for the mesh boolean's
// own loops. resolved=false means the survivors do not close into exactly
// one simple loop using every one of them — "not resolved" (§4.4): a
// disjoint pair of footprints (two separate lumps) or a union enclosing an
// internal void both leave a dangling end or a smaller cycle here.
//
// Connectivity is read off sketch's OWN walked boundary sample — Polyline[0]
// is the walk's start, Polyline[len-1] its end (sketch-seam-design's
// contract) — matched by exact (u, v) equality: these are vertices the SAME
// arrangement pass already computed and shares between the edges that meet
// there, so no tolerance is introduced. This is bookkeeping on sketch's own
// answer, not a re-derived geometric fact (CLAUDE.md's carve-out for this
// design).
func chainPrismUnionSurvivors(budget *workBudget, survivors []sketch.BoundaryEdge) (chain []sketch.BoundaryEdge, resolved bool, err error) {
	byStart := make(map[Point2]int, len(survivors))
	for i, e := range survivors {
		if err := budget.step(); err != nil {
			return nil, false, err
		}
		if len(e.Polyline) < 2 {
			return nil, false, nil // defensive: no walked endpoints to key on
		}
		start := Point2{U: e.Polyline[0][0], V: e.Polyline[0][1]}
		if _, dup := byStart[start]; dup {
			return nil, false, nil // ambiguous: more than one survivor leaves this vertex
		}
		byStart[start] = i
	}

	used := make([]bool, len(survivors))
	chain = make([]sketch.BoundaryEdge, 0, len(survivors))
	cur := 0
	for range survivors {
		if err := budget.step(); err != nil {
			return nil, false, err
		}
		if used[cur] {
			return nil, false, nil // a smaller cycle closed before every edge was used
		}
		used[cur] = true
		e := survivors[cur]
		chain = append(chain, e)
		end := Point2{U: e.Polyline[len(e.Polyline)-1][0], V: e.Polyline[len(e.Polyline)-1][1]}
		next, ok := byStart[end]
		if !ok {
			return nil, false, nil // a dead end: no survivor continues from here
		}
		cur = next
	}
	if cur != 0 {
		return nil, false, nil // the walk did not return to its own start
	}
	return chain, true, nil
}

// auditPrismUnionSection runs the modify §5 audit (fillet_audit.go,
// §6 of this design) on the merged section, reused verbatim with an empty
// blend map — shell_offset.go's own precedent for auditing a
// decad-constructed section with no cutback data. orig supplies the S8 sign
// reference: operand A's own outer loop, whose CCW orientation and
// non-degenerate area the merged Outer loop must match — the merge is
// correct only when it keeps that same convention. S9 (nesting) is a no-op
// here: G6 keeps every operand, and so the merged result, hole-free.
func auditPrismUnionSection(budget *workBudget, pa prismPayload, merged ProfileRecord) error {
	loops, err := prismCornerLoopsBudget(budget, prismPayload{profile: merged})
	if err != nil {
		return err
	}
	blendAt := make([]map[int]*cornerBlend, len(loops))
	for i := range blendAt {
		blendAt[i] = map[int]*cornerBlend{}
	}
	orig := ProfileRecord{Outer: pa.profile.Outer}
	return auditRewriteBudget(budget, orig, merged, loops, blendAt)
}

// buildPrismUnionScene is §4.1's scene construction: one private sketch.Sketch
// holding both operands' recorded entities. Operand A's segments are created
// verbatim (A's frame is the reference); operand B's are re-expressed into
// A's frame first (reexpressPrismPoint) — the one new rounding this design
// introduces. Entities are deduplicated WITHIN each operand (the same
// dedup key discipline moments_validate.go's momentRecordScene already uses
// for one record) but NEVER across operands: a coincident carrier is handed
// to sketch as two separate, numerically matching entities, and sketch's own
// coincident-carrier resolution decides whether they merge.
//
// Every entity is built from the segment's own WALKED geometry (walkOf),
// never from a Partial segment's recorded Center/Start/End as if it named a
// whole curve. A Partial segment's Start/End/Center are its SOURCE entity's
// own defining data, verbatim (record.go), which is exactly what a whole
// curve's fields hold too — nothing in a CurveSegment's own shape marks a
// fragment as partial. Recreating that source curve WHOLE is correct for an
// operand recorded straight from a live sketch profile (a genuine curve that
// may cross the OTHER operand beyond this operand's own walked range, which
// is exactly why momentRecordScene does the same for a single record's own
// self-consistency check). It stops being correct once an operand is itself
// a prior prism-boolean result: a Partial segment surviving THAT merge
// traces to one of the ORIGINAL pre-merge operands' own walls, and the
// portion beyond its walked range is now genuinely INTERIOR material, not a
// boundary of anything — recreating it whole resurrects a wall this
// operand's own solid does not have, which can silently misclassify a
// region in whatever new arrangement it enters. Every segment's OWN walked
// portion, and nothing more, is what this operand's boundary IS, whole or
// partial alike — so that is what gets built.
func buildPrismUnionScene(budget *workBudget, pa, pb prismPayload) (*sketch.Sketch, error) {
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	if err != nil {
		return nil, fmt.Errorf(`decad: failed to build the private union scene: %w`, err)
	}
	invA, err := pa.xform.Inverse()
	if err != nil {
		return nil, fmt.Errorf(`decad: the receiver's placement has no inverse: %w`, err)
	}

	points := map[Point2]*sketch.Point{}
	point := func(p Point2) *sketch.Point {
		if existing, ok := points[p]; ok {
			return existing
		}
		created := s.CreatePoint(p.U, p.V)
		points[p] = created
		return created
	}

	addOperand := func(profile ProfileRecord, reexpress bool) error {
		type entityKey struct {
			kind    uint8 // 1 = line, 2 = whole circle, 3 = arc (incl. a partial circle)
			a, b, c Point2
			radius  float64
		}
		// Fresh per operand: §4.1 forbids deduplicating an entity across the
		// two operands, even where the same physical curve appears in both.
		entities := map[entityKey]struct{}{}
		reexpressPt := func(p Point2) Point2 {
			if !reexpress {
				return p
			}
			return reexpressPrismPoint(pa.frame, invA, pb, p)
		}
		for _, seg := range profile.Outer.Segments {
			if err := budget.step(); err != nil {
				return err
			}
			w, err := walkOf(seg, nil) // G4 admits only Line/Circle/Arc: no free-form work counter needed
			if err != nil {
				return err
			}
			switch {
			case w.isLine():
				start := reexpressPt(Point2{U: w.startU, V: w.startV})
				end := reexpressPt(Point2{U: w.endU, V: w.endV})
				key := entityKey{kind: 1, a: start, b: end}
				if _, ok := entities[key]; !ok {
					s.CreateLine(point(start), point(end))
					entities[key] = struct{}{}
				}
			case w.isCircular() && w.closed:
				center := reexpressPt(Point2{U: w.cU, V: w.cV})
				key := entityKey{kind: 2, a: center, radius: w.radius}
				if _, ok := entities[key]; !ok {
					s.CreateCircle(point(center), w.radius)
					entities[key] = struct{}{}
				}
			case w.isCircular():
				// sketch.CreateArc sweeps CCW from its second point to its
				// third; the walk's own OWN direction may run either way, so
				// the two candidate endpoints are passed in ascending-angle
				// order — the physical set of points between th0 and th1 is
				// the same set either way, since a walked arc never spans a
				// full turn.
				loU, loV, hiU, hiV := w.startU, w.startV, w.endU, w.endV
				if w.th1 < w.th0 {
					loU, loV, hiU, hiV = hiU, hiV, loU, loV
				}
				center := reexpressPt(Point2{U: w.cU, V: w.cV})
				lo := reexpressPt(Point2{U: loU, V: loV})
				hi := reexpressPt(Point2{U: hiU, V: hiV})
				key := entityKey{kind: 3, a: center, b: lo, c: hi}
				if _, ok := entities[key]; !ok {
					s.CreateArc(point(center), point(lo), point(hi))
					entities[key] = struct{}{}
				}
			default:
				// G4 already excludes every other kind before this runs.
				return fmt.Errorf(`%w: a %T segment is not part of the admitted class`, ErrUnsupported, seg)
			}
		}
		return nil
	}

	if err := addOperand(pa.profile, false); err != nil {
		return nil, err
	}
	if err := addOperand(pb.profile, true); err != nil {
		return nil, err
	}
	return s, nil
}

// reexpressPrismPoint is §4.1's coordinate re-expression: a plane-local point
// of operand B is lifted to world space through B's own composed map, then
// projected into operand A's local frame — frameA.ToLocal(invA.Apply(...)),
// dropping the resulting local z, which G3 already certified is the shared
// plane's own zero axis. invA is xformA.Inverse(), computed once by the
// caller (r3.Transform's inverse is exact — the transpose — since a Frame
// and a rigid Transform are both orthonormal, so this is a dot product,
// never a solve). This is the one and only new rounding this design
// introduces: an ordinary rigid-transform coordinate computation, rounded
// once per coordinate, on operand B's segments only.
func reexpressPrismPoint(frameA r3.Frame, invA r3.Transform, pb prismPayload, p Point2) Point2 {
	worldPt := pb.xform.Apply(pb.frame.ToWorldUV(p.U, p.V))
	local := frameA.ToLocal(invA.Apply(worldPt))
	return Point2{U: local.X, V: local.Y}
}
