package prismcells

import (
	"math"
	"math/big"
	"sort"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/sketch"
)

// This file reads the coincident carriers sketch resolves in a scene: two
// operands' lines on one carrier line (sketch's
// docs/coincident-carrier-resolution-design.md, "Coincident line carriers"),
// or two operands' arcs and circles on one carrier circle (the same document's
// circular case). Where two such entities share a span, sketch emits the span
// once, under the entity it names, and withdraws it from the other — the
// losing entity — as a window its fragments do not cover. Every edge of the
// named entity inside that window is then a boundary of BOTH operands, and
// the classification and the crossing charge read it as such: Classify takes
// both operands' membership directly from it and never propagates either
// across it, and CrossingCharge does not read the two entities as crossing.
//
// Nothing here decides coincidence from geometry. The windows are the gaps in
// the losing entity's own reported fragments; a window's ends are the
// vertices sketch split both entities at (the losing entity's own fragment
// Polyline ends, or its own endpoint where the window reaches it); and the
// partner is the one entity of the other operand, on the same kind of
// carrier, that has fragment ends at both of those vertices. sketch splits
// the named entity at both window ends (its certifySuppression step), so the
// true partner is always such an entity. Two of them make the reading
// unresolved; none means the window is no shared span at all — a part of an
// open chain no bounded cell uses — and it is skipped. A circle withdrawn
// whole has no window end to read: its partner is the other operand's circle
// with the same recorded centre and radius bit for bit, and anything else is
// unresolved.

// Coincidence is what one coincident edge carries for its partner: the
// losing entity it shares its span with, and whether the losing entity's
// natural direction opposes the named entity's. Arcs and circles both run
// counter-clockwise, so only two lines can oppose.
type Coincidence struct {
	Partner  sketch.Entity
	Opposite bool
}

type edgeSpan struct {
	entity sketch.Entity
	t0, t1 float64
}

// lineFrag is one reported fragment of an entity: its natural-direction
// range and the Polyline points at each end of it.
type lineFrag struct {
	t0, t1 float64
	p0, p1 [2]float64
}

// CoincidentSpan is one span sketch resolved between two operands'
// coincident entities: the named entity the span is emitted under, the
// losing entity it is withdrawn from, and, for an open span, the two
// vertices bounding it (Whole marks a circle withdrawn whole).
type CoincidentSpan struct {
	Named, Losing sketch.Entity
	A, B          [2]float64
	Whole         bool
}

// CoincidentReading is a scene's coincident spans and, keyed by entity and
// range, every edge of a named entity inside one of them.
type CoincidentReading struct {
	Edges map[edgeSpan]Coincidence
	Spans []CoincidentSpan
}

// Partners reports whether a and b are the two entities of one span.
func (r CoincidentReading) Partners(a, b sketch.Entity) bool {
	for _, sp := range r.Spans {
		if (sp.Named == a && sp.Losing == b) || (sp.Named == b && sp.Losing == a) {
			return true
		}
	}
	return false
}

// Lookup returns the coincidence an edge of entity over [t0, t1] carries.
func (r CoincidentReading) Lookup(entity sketch.Entity, t0, t1 float64) (Coincidence, bool) {
	c, ok := r.Edges[edgeSpan{entity: entity, t0: t0, t1: t1}]
	return c, ok
}

// window is one parameter range of a losing entity no fragment covers.
type window struct {
	lo, hi         float64
	pLo, pHi       [2]float64
	haveLo, haveHi bool
}

// CoincidentEdges reads every span sketch resolved between two operands'
// coincident entities. resolved=false (err always nil then) means a window
// this reading cannot attribute to one partner; callers treat the scene as
// unresolved. An empty reading is the ordinary scene with no shared span.
func CoincidentEdges(budget *proofbound.WorkBudget, tags map[sketch.Entity]Origin, profiles []*sketch.Profile) (CoincidentReading, bool, error) {
	return coincidentEdges(budget, tags, profiles, otherOperand)
}

// CoincidentEdgesRegions is CoincidentEdges with a span's partner taken from
// any other record of the scene — another region of the same operand as
// well as the other operand (docs/general-boolean-design.md §3 "A1 as a
// brep") — so two records of one operand sharing a wall read as A3 reads
// two operands sharing one.
func CoincidentEdgesRegions(budget *proofbound.WorkBudget, tags map[sketch.Entity]Origin, profiles []*sketch.Profile) (CoincidentReading, bool, error) {
	return coincidentEdges(budget, tags, profiles, otherRecord)
}

// otherOperand admits a partner of the other operand.
func otherOperand(losing, candidate Origin) bool { return losing.IsB != candidate.IsB }

// otherRecord admits a partner of any other record: the other operand, or
// another region of the same one.
func otherRecord(losing, candidate Origin) bool {
	return losing.IsB != candidate.IsB || losing.Region != candidate.Region
}

// coincidentEdges is CoincidentEdges with other deciding which entities may
// partner a losing entity's window.
func coincidentEdges(budget *proofbound.WorkBudget, tags map[sketch.Entity]Origin, profiles []*sketch.Profile, other func(losing, candidate Origin) bool) (CoincidentReading, bool, error) {
	frags := map[sketch.Entity][]lineFrag{}
	seen := map[edgeSpan]struct{}{}
	for _, p := range profiles {
		for _, loop := range append([][]sketch.BoundaryEdge{p.Outer}, p.Holes...) {
			for _, e := range loop {
				if err := budget.Step(); err != nil {
					return CoincidentReading{}, false, err
				}
				k := edgeSpan{entity: e.Entity, t0: e.TStart, t1: e.TEnd}
				if _, dup := seen[k]; dup {
					continue
				}
				seen[k] = struct{}{}
				if len(e.Polyline) < 2 {
					return CoincidentReading{}, false, nil
				}
				first, last := e.Polyline[0], e.Polyline[len(e.Polyline)-1]
				if e.Reversed {
					first, last = last, first
				}
				frags[e.Entity] = append(frags[e.Entity], lineFrag{t0: e.TStart, t1: e.TEnd, p0: first, p1: last})
			}
		}
	}
	for _, fs := range frags {
		sort.Slice(fs, func(i, j int) bool { return fs[i].t0 < fs[j].t0 })
	}

	out := CoincidentReading{Edges: map[edgeSpan]Coincidence{}}
	// claim fails when one span is claimed by two losing entities, which
	// this reading does not cover; that holds whatever order the entities
	// are visited in.
	claim := func(edges []edgeSpan, c Coincidence) bool {
		for _, e := range edges {
			if _, taken := out.Edges[e]; taken {
				return false
			}
			out.Edges[e] = c
		}
		return true
	}
	for ent, origin := range tags {
		if err := budget.Step(); err != nil {
			return CoincidentReading{}, false, err
		}
		fs := frags[ent]
		if circle, ok := ent.(*sketch.Circle); ok && len(fs) == 0 {
			// Withdrawn whole: either no bounded cell uses this circle at
			// all, or the other operand holds the same circle.
			partner, ok := sameCircle(tags, origin, other, frags, circle)
			if !ok {
				return CoincidentReading{}, false, nil
			}
			if partner == nil {
				continue
			}
			var edges []edgeSpan
			for _, f := range frags[partner] {
				edges = append(edges, edgeSpan{entity: partner, t0: f.t0, t1: f.t1})
			}
			if !claim(edges, Coincidence{Partner: ent}) {
				return CoincidentReading{}, false, nil
			}
			out.Spans = append(out.Spans, CoincidentSpan{Named: partner, Losing: ent, Whole: true})
			continue
		}
		windows, ok := entityWindows(ent, fs)
		if !ok {
			return CoincidentReading{}, false, nil
		}
		for _, w := range windows {
			partner, edges, found, ok := spanPartner(tags, origin, other, ent, frags, w.pLo, w.pHi)
			if !ok {
				return CoincidentReading{}, false, nil
			}
			if !found {
				continue // no shared span: an open part no bounded cell uses
			}
			opposite := false
			if line, isLine := ent.(*sketch.Line); isLine {
				partnerLine, isLine := partner.(*sketch.Line)
				if !isLine {
					return CoincidentReading{}, false, nil
				}
				if opposite, ok = linesOpposed(line, partnerLine); !ok {
					return CoincidentReading{}, false, nil
				}
			}
			if !claim(edges, Coincidence{Partner: ent, Opposite: opposite}) {
				return CoincidentReading{}, false, nil
			}
			out.Spans = append(out.Spans, CoincidentSpan{Named: partner, Losing: ent, A: w.pLo, B: w.pHi})
		}
	}
	return out, true, nil
}

// entityWindows is every parameter range of ent its sorted fragments fs do
// not cover, each with the vertex at both of its ends. Fragments of one
// entity meet at exactly equal parameters. A circle's window across its seam
// is read as one window. ok=false means a window end this reading cannot
// name a vertex for.
func entityWindows(ent sketch.Entity, fs []lineFrag) ([]window, bool) {
	var windows []window
	at := 0.0
	var atPoint [2]float64
	haveAt := false
	for _, f := range fs {
		if f.t0 > at {
			windows = append(windows, window{lo: at, hi: f.t0, pLo: atPoint, haveLo: haveAt, pHi: f.p0, haveHi: true})
		}
		if f.t1 > at {
			at, atPoint, haveAt = f.t1, f.p1, true
		}
	}
	if at < 1 {
		windows = append(windows, window{lo: at, hi: 1, pLo: atPoint, haveLo: haveAt})
	}
	if len(windows) == 0 {
		return nil, true
	}
	switch e := ent.(type) {
	case *sketch.Line:
		g := e.Geometry()
		return entityEnds(windows, [2]float64{g.Start.X, g.Start.Y}, [2]float64{g.End.X, g.End.Y})
	case *sketch.Arc:
		g := e.Geometry()
		return entityEnds(windows, [2]float64{g.Start.X, g.Start.Y}, [2]float64{g.End.X, g.End.Y})
	case *sketch.Circle:
		// The seam is a vertex only where a fragment ends there; a window
		// reaching it from either side continues across it.
		first, last := &windows[0], &windows[len(windows)-1]
		if !first.haveLo && !last.haveHi {
			if len(windows) == 1 {
				return nil, false // no fragment at all: read as a whole circle
			}
			first.pLo, first.haveLo = last.pLo, last.haveLo
			first.lo = last.lo - 1
			windows = windows[:len(windows)-1]
		}
		// A window ending at the seam from one side only takes its vertex
		// from the fragment meeting the seam from the other.
		for i := range windows {
			w := &windows[i]
			for _, f := range fs {
				if !w.haveLo && w.lo == 0 && f.t1 == 1 {
					w.pLo, w.haveLo = f.p1, true
				}
				if !w.haveHi && w.hi == 1 && f.t0 == 0 {
					w.pHi, w.haveHi = f.p0, true
				}
			}
			if !w.haveLo || !w.haveHi {
				return nil, false
			}
		}
		return windows, true
	}
	return nil, false // a free-form entity with a window: not read here
}

// entityEnds names an open entity's own endpoints at the windows reaching
// them.
func entityEnds(windows []window, start, end [2]float64) ([]window, bool) {
	for i := range windows {
		w := &windows[i]
		if !w.haveLo {
			if w.lo != 0 {
				return nil, false
			}
			w.pLo, w.haveLo = start, true
		}
		if !w.haveHi {
			if w.hi != 1 {
				return nil, false
			}
			w.pHi, w.haveHi = end, true
		}
	}
	return windows, true
}

// spanPartner finds the one entity other admits for losing's origin, on the
// same kind of carrier as losing, with fragment ends exactly at both a and b,
// and returns its fragments between them, which must cover that range
// without a gap. On a line the span runs between the two parameters in
// either order; on an arc or a circle it runs counter-clockwise from a to b,
// as the losing entity's window does, across a circle's seam if it must.
// found=false with ok=true means no such entity; ok=false means two of them,
// or one whose fragments leave a gap.
func spanPartner(tags map[sketch.Entity]Origin, losingOrigin Origin, other func(losing, candidate Origin) bool, losing sketch.Entity, frags map[sketch.Entity][]lineFrag, a, b [2]float64) (sketch.Entity, []edgeSpan, bool, bool) {
	_, losingLine := losing.(*sketch.Line)
	var partner sketch.Entity
	var edges []edgeSpan
	for ent, origin := range tags {
		if !other(losingOrigin, origin) {
			continue
		}
		fs := frags[ent]
		tA, okA := fragParamAt(fs, a, false)
		tB, okB := fragParamAt(fs, b, true)
		if !okA || !okB {
			continue
		}
		var ranges [][2]float64
		switch ent.(type) {
		case *sketch.Line:
			if !losingLine {
				continue
			}
			ranges = [][2]float64{{min(tA, tB), max(tA, tB)}}
		case *sketch.Arc:
			if losingLine || tA >= tB {
				continue
			}
			ranges = [][2]float64{{tA, tB}}
		case *sketch.Circle:
			if losingLine {
				continue
			}
			if tA < tB {
				ranges = [][2]float64{{tA, tB}}
			} else {
				ranges = [][2]float64{{tA, 1}, {0, tB}}
			}
		default:
			continue
		}
		if partner != nil {
			return nil, nil, false, false
		}
		partner = ent
		edges = edges[:0]
		for _, r := range ranges {
			covered, ok := coveringFrags(ent, fs, r[0], r[1])
			if !ok {
				return nil, nil, false, false
			}
			edges = append(edges, covered...)
		}
	}
	return partner, edges, partner != nil, true
}

// coveringFrags returns ent's sorted fragments inside [lo, hi], which must
// cover it without a gap.
func coveringFrags(ent sketch.Entity, fs []lineFrag, lo, hi float64) ([]edgeSpan, bool) {
	if lo >= hi {
		return nil, false
	}
	var out []edgeSpan
	at := lo
	for _, f := range fs {
		if f.t0 < lo || f.t1 > hi {
			continue
		}
		if f.t0 != at {
			return nil, false
		}
		out = append(out, edgeSpan{entity: ent, t0: f.t0, t1: f.t1})
		at = f.t1
	}
	return out, at == hi
}

// fragParamAt is the parameter of the fragment end that sits exactly at p.
// A circle's seam point is both 0 and 1: asEnd picks 1 there, for the end of
// a counter-clockwise span, and 0 otherwise.
func fragParamAt(fs []lineFrag, p [2]float64, asEnd bool) (float64, bool) {
	found := false
	var t float64
	for _, f := range fs {
		for _, c := range []struct {
			t float64
			p [2]float64
		}{{f.t0, f.p0}, {f.t1, f.p1}} {
			if c.p != p {
				continue
			}
			if !found || (asEnd && c.t > t) || (!asEnd && c.t < t) {
				t, found = c.t, true
			}
		}
	}
	return t, found
}

// sameCircle finds the circle of the other operand whose recorded centre and
// radius equal circle's bit for bit. partner=nil with ok=true means none;
// ok=false means one exists but its fragments do not cover it whole.
func sameCircle(tags map[sketch.Entity]Origin, losingOrigin Origin, other func(losing, candidate Origin) bool, frags map[sketch.Entity][]lineFrag, circle *sketch.Circle) (sketch.Entity, bool) {
	g := circle.Geometry()
	for ent, origin := range tags {
		if !other(losingOrigin, origin) {
			continue
		}
		other, ok := ent.(*sketch.Circle)
		if !ok {
			continue
		}
		og := other.Geometry()
		if og.Center.X != g.Center.X || og.Center.Y != g.Center.Y || og.Radius != g.Radius {
			continue
		}
		if _, ok := coveringFrags(ent, frags[ent], 0, 1); !ok {
			return nil, false
		}
		return ent, true
	}
	return nil, true
}

// linesOpposed reports whether two lines' natural directions oppose, by the
// exact sign of their direction vectors' dot product over the held floats.
func linesOpposed(a, b *sketch.Line) (bool, bool) {
	ga, gb := a.Geometry(), b.Geometry()
	r, ok := floatRats(ga.Start.X, ga.Start.Y, ga.End.X, ga.End.Y, gb.Start.X, gb.Start.Y, gb.End.X, gb.End.Y)
	if !ok {
		return false, false
	}
	dax, day := new(big.Rat).Sub(r[2], r[0]), new(big.Rat).Sub(r[3], r[1])
	dbx, dby := new(big.Rat).Sub(r[6], r[4]), new(big.Rat).Sub(r[7], r[5])
	dot := new(big.Rat).Add(new(big.Rat).Mul(dax, dbx), new(big.Rat).Mul(day, dby))
	if dot.Sign() == 0 {
		return false, false
	}
	return dot.Sign() < 0, true
}

// OnBoundary reports whether every span edge appears in exactly one of the
// selected cells, so the merged result keeps it as boundary.
func (r CoincidentReading) OnBoundary(budget *proofbound.WorkBudget, selected []*sketch.Profile) (bool, error) {
	if len(r.Edges) == 0 {
		return true, nil
	}
	count := make(map[edgeSpan]int, len(r.Edges))
	for _, p := range selected {
		for _, loop := range append([][]sketch.BoundaryEdge{p.Outer}, p.Holes...) {
			for _, e := range loop {
				if err := budget.Step(); err != nil {
					return false, err
				}
				k := edgeSpan{entity: e.Entity, t0: e.TStart, t1: e.TEnd}
				if _, ok := r.Edges[k]; ok {
					count[k]++
				}
			}
		}
	}
	for k := range r.Edges {
		if count[k] != 1 {
			return false, nil
		}
	}
	return true, nil
}

// Gap is a proven upper bound on how far apart the two recorded entities of
// any span sit over it: sketch resolves entities that are the same carrier
// at round-off, so a result recording the span on the named entity differs
// from one recording it on the losing entity by at most this. Two lines'
// separation over a span is largest at its ends, and at each end it is at
// most the end vertex's distance from one line plus its distance from the
// other. A point of one circle sits from the other circle at most their
// centres' distance plus their radii's difference; an arc's radius is read
// at both of its recorded endpoints. Every square root rounds outward. Two
// walls drawn on one carrier have a zero gap.
func (r CoincidentReading) Gap() float64 {
	gap := 0.0
	for _, sp := range r.Spans {
		if _, isLine := sp.Named.(*sketch.Line); isLine {
			for _, p := range [][2]float64{sp.A, sp.B} {
				gap = max(gap, proofbound.AbsSumUpper(pointLineDistanceUpper(p, sp.Named), pointLineDistanceUpper(p, sp.Losing)))
			}
			continue
		}
		gap = max(gap, circleGapUpper(sp.Named, sp.Losing))
	}
	return gap
}

// pointLineDistanceUpper bounds the distance from p to the line ent carries:
// |(p − S) × (E − S)| / |E − S|, exactly to the square root, which rounds up.
func pointLineDistanceUpper(p [2]float64, ent sketch.Entity) float64 {
	line, ok := ent.(*sketch.Line)
	if !ok {
		return math.Inf(1)
	}
	g := line.Geometry()
	r, ok := floatRats(p[0], p[1], g.Start.X, g.Start.Y, g.End.X, g.End.Y)
	if !ok {
		return math.Inf(1)
	}
	px, py := new(big.Rat).Sub(r[0], r[2]), new(big.Rat).Sub(r[1], r[3])
	dx, dy := new(big.Rat).Sub(r[4], r[2]), new(big.Rat).Sub(r[5], r[3])
	cross := new(big.Rat).Sub(new(big.Rat).Mul(px, dy), new(big.Rat).Mul(py, dx))
	if cross.Sign() == 0 {
		return 0
	}
	len2 := new(big.Rat).Add(new(big.Rat).Mul(dx, dx), new(big.Rat).Mul(dy, dy))
	if len2.Sign() == 0 {
		return math.Inf(1)
	}
	return proofbound.RatSqrtUp(new(big.Rat).Quo(new(big.Rat).Mul(cross, cross), len2))
}

// circleCarrier is a circular entity's centre and its squared radius as read
// at each recorded value that defines it (a circle's one radius, an arc's
// start and end points).
func circleCarrier(ent sketch.Entity) (cx, cy *big.Rat, r2 []*big.Rat, ok bool) {
	switch e := ent.(type) {
	case *sketch.Circle:
		g := e.Geometry()
		c, ok := floatRats(g.Center.X, g.Center.Y, g.Radius)
		if !ok {
			return nil, nil, nil, false
		}
		return c[0], c[1], []*big.Rat{new(big.Rat).Mul(c[2], c[2])}, true
	case *sketch.Arc:
		g := e.Geometry()
		c, ok := floatRats(g.Center.X, g.Center.Y, g.Start.X, g.Start.Y, g.End.X, g.End.Y)
		if !ok {
			return nil, nil, nil, false
		}
		sq := func(x, y *big.Rat) *big.Rat {
			dx, dy := new(big.Rat).Sub(x, c[0]), new(big.Rat).Sub(y, c[1])
			return new(big.Rat).Add(new(big.Rat).Mul(dx, dx), new(big.Rat).Mul(dy, dy))
		}
		return c[0], c[1], []*big.Rat{sq(c[2], c[3]), sq(c[4], c[5])}, true
	}
	return nil, nil, nil, false
}

// circleGapUpper bounds the largest distance from a point of one circular
// carrier to the other: |c1 − c2| + max |r1 − r2|, each radius difference
// bounded by the outward square roots of the squared radii.
func circleGapUpper(a, b sketch.Entity) float64 {
	ax, ay, ar, okA := circleCarrier(a)
	bx, by, br, okB := circleCarrier(b)
	if !okA || !okB {
		return math.Inf(1)
	}
	dx, dy := new(big.Rat).Sub(ax, bx), new(big.Rat).Sub(ay, by)
	centres := proofbound.RatSqrtUp(new(big.Rat).Add(new(big.Rat).Mul(dx, dx), new(big.Rat).Mul(dy, dy)))
	radii := 0.0
	for _, p := range ar {
		for _, q := range br {
			hi, lo := p, q
			if hi.Cmp(lo) < 0 {
				hi, lo = lo, hi
			}
			if hi.Cmp(lo) == 0 {
				continue
			}
			radii = max(radii, math.Nextafter(proofbound.RatSqrtUp(hi)-proofbound.RatSqrtDown(lo), math.Inf(1)))
		}
	}
	return proofbound.AbsSumUpper(centres, radii)
}

// floatRats is each value as an exact rational; ok=false on a non-finite one.
func floatRats(vals ...float64) ([]*big.Rat, bool) {
	out := make([]*big.Rat, len(vals))
	for i, v := range vals {
		if out[i] = proofarith.FloatRat(v); out[i] == nil {
			return nil, false
		}
	}
	return out, true
}
