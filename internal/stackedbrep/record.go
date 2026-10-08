package stackedbrep

import (
	"fmt"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/units"
)

func requireStackedAnalyticWalk(w survey2d.SegmentWalk) error {
	if w.Kind != survey2d.WalkFreeform {
		return nil
	}
	return fmt.Errorf("%w: a stacked union face does not support a free-form boundary segment", boundarywalk.ErrUnsupported)
}

func cmpFloat(a, b float64) int {
	switch {
	case a < b:
		return -1
	case a > b:
		return 1
	}
	return 0
}

func arcSegment(center, start, end Point2, ccw bool) CurveSegment {
	if ccw {
		return ArcSeg{Center: center, Start: start, End: end, TStart: 0, TEnd: 1}
	}
	return ArcSeg{Center: center, Start: end, End: start, TStart: 1, TEnd: 0}
}

// loopOf restates one recorded loop as units between canonical vertices.
func (b *Engine) LoopOf(region brepgeom.Profile, budget *proofbound.WorkBudget) (Loop, error) {
	if len(region.Holes) != 0 {
		return Loop{}, brepgeom.ErrStackedWallMiss
	}
	segs := region.Outer.Segments
	n := len(segs)
	if n == 0 {
		return Loop{}, brepgeom.ErrStackedWallMiss
	}
	carriers := make([]Carrier, n)
	starts := make([]ubEnd, n)
	ends := make([]ubEnd, n)
	ccw := make([]bool, n)
	dir := make([]Point2, n)
	for i, seg := range segs {
		if err := budget.Step(); err != nil {
			return Loop{}, err
		}
		w, err := boundarywalk.WalkOf(seg, nil)
		if err != nil {
			return Loop{}, err
		}
		if err := requireStackedAnalyticWalk(w); err != nil {
			return Loop{}, err
		}
		carriers[i], err = ubCarrierOf(seg, w)
		if err != nil {
			return Loop{}, err
		}
		starts[i], ends[i], err = ubEnds(seg, w)
		if err != nil {
			return Loop{}, err
		}
		ccw[i] = w.Th1 > w.Th0
		dir[i] = Point2{U: w.EndU - w.StartU, V: w.EndV - w.StartV}
	}
	// Consecutive segments on one carrier are one unit: a circle cut at its
	// own seam, or a line run restated at other cells' vertices.
	same := func(i, j int) bool {
		if carriers[i] != carriers[j] {
			return false
		}
		if carriers[i].Kind == ubCircle {
			return ccw[i] == ccw[j]
		}
		return dir[i].U*dir[j].U+dir[i].V*dir[j].V > 0
	}
	start := -1
	for i := range n {
		if !same((i+n-1)%n, i) {
			start = i
			break
		}
	}
	if start < 0 {
		// One carrier round the whole Loop: a whole circle, possibly cut
		// at its seam. A straight loop closes nothing.
		if carriers[0].Kind != ubCircle {
			return Loop{}, brepgeom.ErrStackedWallMiss
		}
		return Loop{Closed: true, Units: []Unit{{Carrier: carriers[0], CCW: ccw[0]}}}, nil
	}
	type group struct {
		first, last int
	}
	var groups []group
	for k := range n {
		i := (start + k) % n
		if k > 0 && same((i+n-1)%n, i) {
			groups[len(groups)-1].last = i
			continue
		}
		groups = append(groups, group{first: i, last: i})
	}
	if len(groups) < 2 {
		return Loop{}, brepgeom.ErrStackedWallMiss
	}
	units := make([]Unit, len(groups))
	for gi, g := range groups {
		units[gi] = Unit{Carrier: carriers[g.first], CCW: ccw[g.first]}
	}
	for gi := range groups {
		gj := (gi + 1) % len(groups)
		p, delta, err := b.junction(units[gi].Carrier, ends[groups[gi].last], units[gj].Carrier, starts[groups[gj].first])
		if err != nil {
			return Loop{}, err
		}
		units[gi].To, units[gj].From = p, p
		b.record(p, delta, units[gi].Carrier, units[gj].Carrier)
	}
	return Loop{Units: units}, nil
}

// junction is the one canonical vertex where carriers ci and cj meet, near
// the two walked ends the loop states there. Two axis-aligned lines meet at
// their exact levels. A line crossing a circle takes the keyed table's one
// float for that crossing — the first scene to reach the key records the
// line's walked point, whose fixed coordinate is exact, with its allowance —
// keyed by the side of the circle's centre the crossing lies on, read along
// the line. A crossing too close to the centre's foot to decide its side is a
// tangency when an axis-aligned line lies exactly the radius from the centre
// (tangentFoot), and takes the exact tangent point, keyed with no side: a
// tangent line meets its circle once (docs/shell-opening-design.md §4.3).
// Any other crossing that close, an oblique line's included, stands only at
// one recorded point (recordedJunction), and so do two circles; where the two
// walked ends differ, the junction misses.
func (b *Engine) junction(ci Carrier, ei ubEnd, cj Carrier, sj ubEnd) (Point2, float64, error) {
	if ci == cj {
		return Point2{}, 0, brepgeom.ErrStackedWallMiss
	}
	if ci.Kind == ubPlane && cj.Kind == ubPlane {
		if ci.Axis == cj.Axis {
			return Point2{}, 0, brepgeom.ErrStackedWallMiss
		}
		if ci.Axis == 0 {
			return Point2{U: ci.Level, V: cj.Level}, 0, nil
		}
		return Point2{U: cj.Level, V: ci.Level}, 0, nil
	}
	if ci.Kind == ubCircle && cj.Kind == ubCircle {
		if ei.p != sj.p {
			return Point2{}, 0, brepgeom.ErrStackedWallMiss
		}
		return b.recordedJunction(ci, cj, ei, sj, circleSide(ci, cj, ei.p))
	}
	// Prefer the straight carrier's own walked point: an axis-aligned
	// line's fixed coordinate is exact.
	hint, line, curve := ei, ci, cj
	if ci.Kind == ubCircle || (ci.Kind == ubLine && cj.Kind == ubPlane) {
		hint, line, curve = sj, cj, ci
	}
	p := hint.p
	if line.Kind == ubPlane {
		if line.Axis == 0 {
			p.U = line.Level
		} else {
			p.V = line.Level
		}
	}
	// The point's own proven distance from the crossing joins its allowance
	// (docs/general-boolean-design.md §3 A1, §5).
	off := crossingOffset(ei, sj, p)
	side := 0
	if curve.Kind != ubCircle {
		if proofbound.IsNonFinite(off) {
			return Point2{}, 0, brepgeom.ErrStackedWallMiss
		}
		hint.allow = max(hint.allow, off)
	}
	if curve.Kind == ubCircle {
		var diff, scale float64
		switch line.Kind {
		case ubPlane:
			if line.Axis == 0 {
				diff = p.V - curve.A.V
			} else {
				diff = p.U - curve.A.U
			}
			scale = 1
		default:
			// The crossing's position along the line from the centre's
			// foot, as the axis-aligned case reads it.
			du, dv := line.B.U-line.A.U, line.B.V-line.A.V
			diff = du*(p.U-curve.A.U) + dv*(p.V-curve.A.V)
			scale = math.Abs(du) + math.Abs(dv)
		}
		switch {
		case math.Abs(diff) > 2*max(hint.allow, off)*scale:
			hint.allow = max(hint.allow, off)
			side = 1
			if diff < 0 {
				side = -1
			}
		default:
			foot, ok := tangentFoot(line, curve)
			if !ok {
				// No exact tangency to read: the junction stands only where
				// both segments end at one recorded point.
				if ei.p != sj.p {
					return Point2{}, 0, brepgeom.ErrStackedWallMiss
				}
				return b.recordedJunction(ci, cj, ei, sj, 0)
			}
			// The line touches the circle at one point, the centre's foot on
			// it; the walked end lies within the side test's band of it, so
			// the foot is charged that band on top of the walked allowance.
			p = foot
			hint.allow = proofbound.AbsSumUpper(hint.allow, math.Abs(diff))
		}
	}
	pair := ubPair(ci, cj)
	key := ubKey{c1: pair[0], c2: pair[1], side: side}
	entry, ok := b.table[key]
	if !ok {
		entry = ubEntry{p: p, delta: hint.allow}
		b.table[key] = entry
	}
	b.allow = math.Max(b.allow, entry.delta)
	return entry.p, entry.delta, nil
}

// tangentFoot is the one point where an axis-aligned line touches a circle:
// it reports false unless the line's level lies exactly the circle's radius
// from the centre's coordinate across it, read in exact rational arithmetic
// over the held floats. The point is the centre's coordinate along the line
// with the line's level across it, so it is exact. An oblique line is never
// decided tangent.
func tangentFoot(line, circle Carrier) (Point2, bool) {
	if line.Kind != ubPlane || circle.Kind != ubCircle {
		return Point2{}, false
	}
	across := circle.A.U
	if line.Axis == 1 {
		across = circle.A.V
	}
	gap := new(big.Rat).Sub(new(big.Rat).SetFloat64(line.Level), new(big.Rat).SetFloat64(across))
	if new(big.Rat).Abs(gap).Cmp(new(big.Rat).SetFloat64(circle.R)) != 0 {
		return Point2{}, false
	}
	if line.Axis == 0 {
		return Point2{U: line.Level, V: circle.A.V}, true
	}
	return Point2{U: circle.A.U, V: line.Level}, true
}

// recordedJunction is the junction of two carriers at the one recorded point
// both walked ends hold, a known join of the record: two circles meeting
// there (docs/shell-opening-design.md §4.3), or a line and a circle whose
// crossing the side test cannot place and that are not exactly tangent, such
// as an offset arc join meeting an oblique offset line at its foot. The point
// is taken as recorded, with the larger walked allowance, so no float sign
// decides it. It is keyed by side, and a later loop must reach the key at the
// same point: a different point under one key misses, so two crossings are
// never merged into one vertex.
func (b *Engine) recordedJunction(ci, cj Carrier, ei, sj ubEnd, side int) (Point2, float64, error) {
	pair := ubPair(ci, cj)
	key := ubKey{c1: pair[0], c2: pair[1], side: side}
	delta := math.Max(ei.allow, sj.allow)
	entry, ok := b.table[key]
	switch {
	case !ok:
		entry = ubEntry{p: ei.p, delta: delta}
	case entry.p != ei.p:
		return Point2{}, 0, brepgeom.ErrStackedWallMiss
	default:
		entry.delta = math.Max(entry.delta, delta)
	}
	b.table[key] = entry
	b.allow = math.Max(b.allow, entry.delta)
	return entry.p, entry.delta, nil
}

// circleSide is the side of the line through two circles' centres that p
// lies on, read in exact rational arithmetic over the held floats with the
// pair in its table order: the two crossings of two circles lie on opposite
// sides, and a point on the line reads zero.
func circleSide(ci, cj Carrier, p Point2) int {
	pair := ubPair(ci, cj)
	rat := func(f float64) *big.Rat { return new(big.Rat).SetFloat64(f) }
	ax, ay := rat(pair[0].A.U), rat(pair[0].A.V)
	dx := new(big.Rat).Sub(rat(pair[1].A.U), ax)
	dy := new(big.Rat).Sub(rat(pair[1].A.V), ay)
	px := new(big.Rat).Sub(rat(p.U), ax)
	py := new(big.Rat).Sub(rat(p.V), ay)
	return new(big.Rat).Sub(new(big.Rat).Mul(dx, py), new(big.Rat).Mul(dy, px)).Sign()
}

// record registers a canonical vertex with the carriers it was found on.
func (b *Engine) record(p Point2, delta float64, carriers ...Carrier) {
	v, ok := b.verts[p]
	if !ok {
		v = &ubVertex{}
		b.verts[p] = v
	}
	v.delta = math.Max(v.delta, delta)
	for _, c := range carriers {
		if !slices.Contains(v.carriers, c) {
			v.carriers = append(v.carriers, c)
		}
	}
}

// Event marks p as a vertex of the body at level, so CutsOnLine splits a
// plane unit there and a wall's vertical edge through p is not merged across
// that level. RecordJunctions marks every point whose junction changes between
// slabs; a caller marks a point it knows is a vertex although its junction
// does not change, such as a shell's reflex opening corner, which lies inside
// the cavity floor's walk along the removed face's carrier
// (docs/shell-opening-design.md §4.3).
func (b *Engine) Event(p Point2, level int) {
	if b.Events[p] == nil {
		b.Events[p] = map[int]struct{}{}
	}
	b.Events[p][level] = struct{}{}
}

func (b *Engine) HasEvent(p Point2, level int) bool {
	_, ok := b.Events[p][level]
	return ok
}

// addFace keeps one horizontal face and marks its vertices at its level.
func (b *Engine) AddFace(loop Loop, level int, outward bool) error {
	b.Faces = append(b.Faces, Face{Loop: loop, Level: level, Outward: outward})
	if loop.Closed {
		return nil
	}
	for _, u := range loop.Units {
		b.Event(u.From, level)
	}
	return nil
}

// recordJunctions reads every slab loop's junctions, then marks the levels
// where a junction starts, ends or changes carriers as vertex events.
func (b *Engine) RecordJunctions(n int) {
	for k, loops := range b.SlabLoops {
		for _, loop := range loops {
			if loop.Closed {
				continue
			}
			for i, u := range loop.Units {
				if b.junctions[u.From] == nil {
					b.junctions[u.From] = map[int][2]Carrier{}
				}
				prev := loop.Units[(i+len(loop.Units)-1)%len(loop.Units)]
				if _, taken := b.junctions[u.From][k]; taken {
					// Two loops of one slab meeting at a point share no
					// vertical edge this build can state.
					b.junctions[u.From][k] = [2]Carrier{}
					continue
				}
				b.junctions[u.From][k] = ubPair(prev.Carrier, u.Carrier)
			}
		}
	}
	for p, bySlab := range b.junctions {
		for level := 0; level <= n; level++ {
			below, hasBelow := bySlab[level-1]
			above, hasAbove := bySlab[level]
			if hasBelow != hasAbove || (hasBelow && below != above) {
				b.Event(p, level)
			}
		}
	}
}

// cutsOnLine lists the vertices at level lying strictly inside a plane
// unit, in walk order. Position along the line is the free coordinate,
// compared exactly.
func (b *Engine) CutsOnLine(u Unit, level int) []Point2 {
	free := func(p Point2) float64 {
		if u.Carrier.Axis == 0 {
			return p.V
		}
		return p.U
	}
	lo, hi := free(u.From), free(u.To)
	forward := lo < hi
	if !forward {
		lo, hi = hi, lo
	}
	var cuts []Point2
	for p, v := range b.verts {
		if !slices.Contains(v.carriers, u.Carrier) || !b.HasEvent(p, level) {
			continue
		}
		if f := free(p); f > lo && f < hi {
			cuts = append(cuts, p)
		}
	}
	slices.SortFunc(cuts, func(a, c Point2) int { return cmpFloat(free(a), free(c)) })
	if !forward {
		slices.Reverse(cuts)
	}
	return cuts
}

// circlePoints lists a circle unit's vertices in walk order: for an open
// unit its two ends with every vertex strictly inside its sweep, for a whole
// circle every vertex on it from the first back to itself. Two vertices
// within 1e-9 rad are not ordered by their held points and miss, as class
// B's split does (general-boolean §5).
func (b *Engine) circlePoints(u Unit) ([]Point2, error) {
	const tol = 1e-9
	c := u.Carrier.A
	angle := func(p Point2) float64 { return math.Atan2(p.V-c.V, p.U-c.U) }
	sweep := func(from, to float64) float64 {
		d := to - from
		if !u.CCW {
			d = -d
		}
		for d < 0 {
			d += 2 * math.Pi
		}
		for d >= 2*math.Pi {
			d -= 2 * math.Pi
		}
		return d
	}
	type cut struct {
		p Point2
		s float64
	}
	var cuts []cut
	a0, total := 0.0, 2*math.Pi
	closed := u.From == u.To
	if !closed {
		a0 = angle(u.From)
		total = sweep(a0, angle(u.To))
	}
	for p, v := range b.verts {
		if !slices.Contains(v.carriers, u.Carrier) {
			continue
		}
		if !closed && (p == u.From || p == u.To) {
			continue
		}
		s := sweep(a0, angle(p))
		if !closed {
			if s >= total+tol {
				continue
			}
			if s <= tol || s >= total-tol {
				return nil, brepgeom.ErrStackedWallMiss
			}
		}
		cuts = append(cuts, cut{p: p, s: s})
	}
	slices.SortFunc(cuts, func(x, y cut) int { return cmpFloat(x.s, y.s) })
	for i := 1; i < len(cuts); i++ {
		if cuts[i].s-cuts[i-1].s <= tol {
			return nil, brepgeom.ErrStackedWallMiss
		}
	}
	if closed && len(cuts) > 1 && 2*math.Pi-cuts[len(cuts)-1].s+cuts[0].s <= tol {
		return nil, brepgeom.ErrStackedWallMiss
	}
	var points []Point2
	if !closed {
		points = append(points, u.From)
	}
	for _, k := range cuts {
		points = append(points, k.p)
	}
	if closed {
		if len(points) > 0 {
			points = append(points, points[0])
		}
		return points, nil
	}
	return append(points, u.To), nil
}

// ubArc is the recorded piece of a circle carrier between two points, in a
// sense; a whole circle when from and to coincide with no vertex between.
func ubArc(c Carrier, from, to Point2, ccw bool) CurveSegment {
	return arcSegment(c.A, from, to, ccw)
}

func ubWholeCircle(c Carrier, ccw bool) CurveSegment {
	seg := CircleSeg{Center: c.A, Radius: units.Millimeters(c.R), CCW: ccw, TStart: 0, TEnd: 1}
	if !ccw {
		seg.TStart, seg.TEnd = 1, 0
	}
	return seg
}

// unitSegments writes a unit's record pieces: a plane line split at the
// vertices at level, an oblique line whole, a circle split at every vertex
// on it.
func (b *Engine) UnitSegments(u Unit, closed bool, level int) ([]CurveSegment, error) {
	switch u.Carrier.Kind {
	case ubPlane:
		points := append([]Point2{u.From}, b.CutsOnLine(u, level)...)
		points = append(points, u.To)
		var out []CurveSegment
		for i := 0; i+1 < len(points); i++ {
			out = append(out, LineSeg{Start: points[i], End: points[i+1], TStart: 0, TEnd: 1})
		}
		return out, nil
	case ubLine:
		for p, v := range b.verts {
			if slices.Contains(v.carriers, u.Carrier) && p != u.From && p != u.To {
				return nil, brepgeom.ErrStackedWallMiss
			}
		}
		return []CurveSegment{LineSeg{Start: u.From, End: u.To, TStart: 0, TEnd: 1}}, nil
	}
	points, err := b.circlePoints(u)
	if err != nil {
		return nil, err
	}
	if closed && len(points) == 0 {
		return []CurveSegment{ubWholeCircle(u.Carrier, u.CCW)}, nil
	}
	var out []CurveSegment
	for i := 0; i+1 < len(points); i++ {
		out = append(out, ubArc(u.Carrier, points[i], points[i+1], u.CCW))
	}
	return out, nil
}

// faceRecord writes one horizontal face's region.
func (b *Engine) FaceRecord(f Face) (brepgeom.Profile, error) {
	var loop LoopRecord
	for _, u := range f.Loop.Units {
		segs, err := b.UnitSegments(u, f.Loop.Closed, f.Level)
		if err != nil {
			return brepgeom.Profile{}, err
		}
		loop.Segments = append(loop.Segments, segs...)
	}
	return brepgeom.Profile{Outer: loop}, nil
}
