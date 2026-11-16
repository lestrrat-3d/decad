package stackedunion

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"

	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stackedbrep"
	"github.com/lestrrat-3d/sketch"
)

// prepareHoled arranges and charges every scene a holed union uses before any
// loop is written, so the shared spans sketch resolved are known to the whole
// build. It requires a span between a hole of one operand and an outer of the
// other (the gate), reads each span's carriers into one canonical carrier per
// class (the alias), records each circular span's ends for the junction rule,
// and charges the alias. The scene cache makes the later passes reuse these
// scenes.
func (b *ubBuild) prepareHoled(ctx context.Context) error {
	var scenes []*ubScene
	seen := map[*ubScene]struct{}{}
	use := func(refsA, refsB []ubRef) error {
		sc, err := b.scene(ctx, refsA, refsB)
		if err != nil {
			return err
		}
		if err := b.charge(sc); err != nil {
			return err
		}
		if _, ok := seen[sc]; !ok {
			seen[sc] = struct{}{}
			scenes = append(scenes, sc)
		}
		return nil
	}
	for k := range b.reach {
		refsA, refsB := b.refs(false, b.reach[k][0]), b.refs(true, b.reach[k][1])
		if len(refsA) == 0 || len(refsB) == 0 {
			continue
		}
		if err := use(refsA, refsB); err != nil {
			return err
		}
	}
	for k := 1; k < len(b.reach); k++ {
		_, refsA, refsB, ok, err := b.interfaceRefs(k)
		if err != nil {
			return err
		}
		if !ok {
			continue
		}
		if err := use(refsA, refsB); err != nil {
			return err
		}
	}
	gated := false
	var circular []circularSpan
	for _, sc := range scenes {
		if err := b.sameOperandSpansExact(sc); err != nil {
			return err
		}
		for _, sp := range sc.delta.Shared().Spans {
			cs, err := b.readSpan(sc, sp)
			if err != nil {
				return err
			}
			gated = gated || cs.gates
			if cs.circular {
				circular = append(circular, cs)
			}
		}
	}
	if !gated {
		return errUBMiss
	}
	alias, err := b.aliasOf(circular)
	if err != nil {
		return err
	}
	// The alias is set before the span ends: AddSpanEnd records an end under
	// the canonical carrier the alias names.
	b.geom.SetAlias(alias.canonical)
	b.aliasCharge = alias.charge
	for _, cs := range circular {
		ends, err := arcSpanEnds(cs)
		if err != nil {
			return err
		}
		for _, p := range ends {
			b.geom.AddSpanEnd(cs.named, p)
		}
	}
	return nil
}

// arcSpanEnds is the recorded corner at each end of a span where sketch puts
// that end at the named arc's own bound: its parameter on the arc is exactly 0
// or 1, and the arc's reported edge ending there is whole (not Partial), so
// the bound is the entity's own and no parameter the arrangement computed.
// The corner is read from the operand's record segment, never from sketch's
// reported vertex, which can sit a few ulps from it: sketch.CreateArc runs
// counter-clockwise from the walk's lower-angle end (prismcells.BuildScene),
// so parameter 0 is that end and 1 the other. A span whose named entity is a
// circle, or whose end is a cut on the arc, records nothing, and a junction
// there misses. The junction then published at the corner (stackedbrep's
// span-end junction) rests on A3's one-carrier reading of the span.
func arcSpanEnds(cs circularSpan) ([]stackedbrep.Point2, error) {
	if cs.sp.Whole {
		return nil, nil
	}
	if _, ok := cs.sp.Named.(*sketch.Arc); !ok {
		return nil, nil
	}
	if _, ok := cs.namedSeg.(sectionrecord.ArcSeg); !ok {
		return nil, nil
	}
	w, err := boundarywalk.WalkOf(cs.namedSeg, nil)
	if err != nil {
		return nil, errUBMiss
	}
	lo := stackedbrep.Point2{U: w.StartU + 0, V: w.StartV + 0}
	hi := stackedbrep.Point2{U: w.EndU + 0, V: w.EndV + 0}
	if w.Th1 < w.Th0 {
		lo, hi = hi, lo
	}
	var out []stackedbrep.Point2
	for _, t := range cs.sp.NamedT {
		if (t != 0 && t != 1) || !wholeEdgeAt(cs.edges, cs.sp.Named, t) {
			continue
		}
		if t == 0 {
			out = append(out, lo)
		} else {
			out = append(out, hi)
		}
	}
	return out, nil
}

// wholeEdgeAt reports whether some reported edge of ent ends at its own bound
// t (0 or 1) and spans the whole entity.
func wholeEdgeAt(edges []sketch.BoundaryEdge, ent sketch.Entity, t float64) bool {
	for _, e := range edges {
		if e.Entity != ent || e.Partial {
			continue
		}
		if (t == 0 && min(e.TStart, e.TEnd) == 0) || (t == 1 && max(e.TStart, e.TEnd) == 1) {
			return true
		}
	}
	return false
}

// circularSpan is one cross-operand span read from its two record segments.
type circularSpan struct {
	sp                  prismcells.CoincidentSpan
	edges               []sketch.BoundaryEdge // the scene's reported edges of the span's named entity
	gates, circular     bool
	named, losing       stackedbrep.Carrier
	namedSeg, losingSeg sectionrecord.CurveSegment
}

// originSegment is the record segment a scene entity's origin names.
func (b *ubBuild) originSegment(sc *ubScene, o prismcells.Origin) (sectionrecord.CurveSegment, error) {
	refs := sc.refsA
	if o.IsB {
		refs = sc.refsB
	}
	if o.Region < 0 || o.Region >= len(refs) {
		return nil, errUBMiss
	}
	rec := b.record(refs[o.Region])
	loop := rec.Outer
	if o.Hole >= 0 {
		if o.Hole >= len(rec.Holes) {
			return nil, errUBMiss
		}
		loop = rec.Holes[o.Hole]
	}
	if o.Segment < 0 || o.Segment >= len(loop.Segments) {
		return nil, errUBMiss
	}
	return loop.Segments[o.Segment], nil
}

// sameOperandSpansExact requires every span two records of one operand share
// in the scene to have a zero gap: a width there has no band to ride.
func (b *ubBuild) sameOperandSpansExact(sc *ubScene) error {
	reading, ok, err := prismcells.CoincidentEdgesRegions(b.st.budget, sc.tags, sc.profiles)
	if err != nil {
		return err
	}
	if !ok {
		return errUBMiss
	}
	for _, sp := range reading.Spans {
		named, ok1 := sc.tags[sp.Named]
		losing, ok2 := sc.tags[sp.Losing]
		if !ok1 || !ok2 {
			return errUBMiss
		}
		if named.IsB != losing.IsB {
			continue
		}
		if (prismcells.CoincidentReading{Spans: []prismcells.CoincidentSpan{sp}}).Gap() != 0 {
			return errUBMiss
		}
	}
	return nil
}

// readSpan reads one cross-operand span: whether it joins a hole entity to an
// outer entity, and for a circular span the carrier of each entity. A line
// span with a nonzero gap is a miss.
func (b *ubBuild) readSpan(sc *ubScene, sp prismcells.CoincidentSpan) (circularSpan, error) {
	cs := circularSpan{sp: sp}
	named, ok1 := sc.tags[sp.Named]
	losing, ok2 := sc.tags[sp.Losing]
	if !ok1 || !ok2 || named.IsB == losing.IsB {
		return cs, errUBMiss
	}
	cs.gates = (named.Hole >= 0 && losing.Hole == -1) || (named.Hole == -1 && losing.Hole >= 0)
	_, namedLine := sp.Named.(*sketch.Line)
	_, losingLine := sp.Losing.(*sketch.Line)
	if namedLine || losingLine {
		if (prismcells.CoincidentReading{Spans: []prismcells.CoincidentSpan{sp}}).Gap() != 0 {
			return cs, errUBMiss
		}
		return cs, nil
	}
	var err error
	if cs.namedSeg, err = b.originSegment(sc, named); err != nil {
		return cs, err
	}
	if cs.losingSeg, err = b.originSegment(sc, losing); err != nil {
		return cs, err
	}
	if cs.named, err = circularCarrier(cs.namedSeg); err != nil {
		return cs, err
	}
	if cs.losing, err = circularCarrier(cs.losingSeg); err != nil {
		return cs, err
	}
	for _, cell := range sc.profiles {
		for _, loop := range append([][]sketch.BoundaryEdge{cell.Outer}, cell.Holes...) {
			for _, e := range loop {
				if e.Entity == sp.Named {
					cs.edges = append(cs.edges, e)
				}
			}
		}
	}
	cs.circular = true
	return cs, nil
}

// circularCarrier is a circle or arc segment's carrier; any other segment is
// a miss.
func circularCarrier(seg sectionrecord.CurveSegment) (stackedbrep.Carrier, error) {
	switch seg.(type) {
	case sectionrecord.CircleSeg, sectionrecord.ArcSeg:
	default:
		return stackedbrep.Carrier{}, errUBMiss
	}
	c, err := stackedbrep.CarrierOf(seg)
	if err != nil {
		return c, errUBMiss
	}
	if c.Kind != stackedbrep.CircleCarrier {
		return c, errUBMiss
	}
	return c, nil
}

// holedAlias is the carrier alias a holed union reads and its charge.
type holedAlias struct {
	canonical map[stackedbrep.Carrier]stackedbrep.Carrier
	charge    float64
}

// aliasOf unions the two carriers of every circular cross-operand span and
// names each class's canonical carrier, the least member by the order of
// (Kind, Axis, Level, A.U, A.V, B.U, B.V, R).
//
// The charge is read from plan §12 B1. A piece about the canonical centre c0,
// whose radius is read from a Start on member Cj, sits from its own member
// Ck at most |rj - rk| + |cj - ck| + 2|ck - c0| + allow, and every pair of
// members is joined by at most (size - 1) spans, each at most crossing apart,
// so D = (size - 1)·crossing bounds |Δc| + |Δr| between any two members and
// the sum is at most 3·D plus the allowance the engine already carries. A
// canonical R that came from an arc is a rounded hypot, so the largest
// prismcells.CircleRadiusBound of every member segment from that R rides on
// top, outward.
func (b *ubBuild) aliasOf(spans []circularSpan) (holedAlias, error) {
	parent := map[stackedbrep.Carrier]stackedbrep.Carrier{}
	var find func(c stackedbrep.Carrier) stackedbrep.Carrier
	find = func(c stackedbrep.Carrier) stackedbrep.Carrier {
		p, ok := parent[c]
		if !ok {
			parent[c] = c
			return c
		}
		if p == c {
			return c
		}
		root := find(p)
		parent[c] = root
		return root
	}
	for _, cs := range spans {
		ra, rb := find(cs.named), find(cs.losing)
		if ra != rb {
			parent[ra] = rb
		}
	}
	classes := map[stackedbrep.Carrier][]stackedbrep.Carrier{}
	for c := range parent {
		root := find(c)
		classes[root] = append(classes[root], c)
	}
	out := holedAlias{canonical: map[stackedbrep.Carrier]stackedbrep.Carrier{}}
	largest := 0
	canonicalOf := map[stackedbrep.Carrier]stackedbrep.Carrier{}
	for _, members := range classes {
		if len(members) < 2 {
			continue
		}
		largest = max(largest, len(members))
		least := members[0]
		for _, m := range members[1:] {
			if carrierLess(m, least) {
				least = m
			}
		}
		for _, m := range members {
			canonicalOf[m] = least
			if m != least {
				out.canonical[m] = least
			}
		}
	}
	if largest < 2 {
		// No two carriers join, but a span between two carriers equal bit for
		// bit can still carry a width (an arc whose end sits off the radius
		// its start defines), so the charge never falls below the crossing.
		out.charge = b.crossing
		return out, nil
	}
	radiusBound := 0.0
	segs, err := b.recordSegments()
	if err != nil {
		return out, err
	}
	for _, seg := range segs {
		c, err := stackedbrep.CarrierOf(seg)
		if err != nil {
			continue
		}
		canon, ok := canonicalOf[c]
		if !ok {
			continue
		}
		radiusBound = max(radiusBound, prismcells.CircleRadiusBound(seg, canon.R))
	}
	d := proofbound.ProductUpper(float64(largest-1), b.crossing)
	out.charge = proofbound.AbsSumUpper(d, d, d, radiusBound, b.crossing)
	if proofbound.IsNonFinite(out.charge) {
		return out, errUBMiss
	}
	return out, nil
}

// recordSegments lists every circular segment of every record of both
// operands: the segments a class member's carrier can be read from.
func (b *ubBuild) recordSegments() ([]sectionrecord.CurveSegment, error) {
	var out []sectionrecord.CurveSegment
	for _, op := range []Operand{b.st.va, b.st.vb} {
		for _, slab := range op.Slabs {
			for _, region := range slab.Regions {
				for _, loop := range append([]sectionrecord.LoopRecord{region.Outer}, region.Holes...) {
					for _, seg := range loop.Segments {
						if err := b.st.budget.Step(); err != nil {
							return nil, err
						}
						switch seg.(type) {
						case sectionrecord.CircleSeg, sectionrecord.ArcSeg:
							out = append(out, seg)
						}
					}
				}
			}
		}
	}
	return out, nil
}

// carrierLess orders carriers by (Kind, Axis, Level, A.U, A.V, B.U, B.V, R).
func carrierLess(a, b stackedbrep.Carrier) bool {
	keys := func(c stackedbrep.Carrier) [8]float64 {
		return [8]float64{float64(c.Kind), float64(c.Axis), c.Level, c.A.U, c.A.V, c.B.U, c.B.V, c.R}
	}
	ka, kb := keys(a), keys(b)
	for i := range ka {
		if ka[i] != kb[i] {
			return ka[i] < kb[i]
		}
	}
	return false
}
