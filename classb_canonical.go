package decad

import (
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// This file is the crossing reach's vertex work (docs/general-boolean-design.md
// §5, §10): the keyed crossing table and the canonical vertex of every
// junction a scene returns, the rewrite of each face's segments between
// those vertices, the split of every edge at every vertex on it, and the
// cylinder pieces read off the planar faces across each cylinder's axis.

// cbVertex is one canonical vertex: its point in X's local axes, the
// carriers it lies on, and its proven displacement from the crossing it
// denotes.
type cbVertex struct {
	point    [3]float64
	carriers []cbCarrier
	delta    float64
}

// cbKey2 keys a cylinder's crossing with a plane parallel to its axis: the
// cylinder, the plane, and the side of the cylinder's centre the crossing
// lies on along the remaining axis.
type cbKey2 struct {
	cyl        cbCarrier
	planeAxis  int
	planeLevel float64
	side       int
}

type cbEntry struct {
	value [3]float64
	delta float64
}

// seedRecordedCorners records, delta zero, every corner either section
// already states where a circular segment meets a line: the crossing is the
// recorded point, not a computed one.
func (b *cbBuild) seedRecordedCorners() error {
	for op, p := range []prismPayload{b.x, b.cp.y} {
		frame := b.fX
		if op == cbY {
			frame = b.fG
		}
		segs := p.profile.Outer.Segments
		for i := range segs {
			j := (i + 1) % len(segs)
			ci, cj := cbCarrier{op, i}, cbCarrier{op, j}
			gi, gj := b.geom(ci), b.geom(cj)
			if gi.cyl == gj.cyl {
				continue
			}
			w, err := walkOf(segs[i], nil)
			if err != nil {
				return err
			}
			cyl, plane := ci, gj
			if gj.cyl {
				cyl, plane = cj, gi
			}
			x := frame.toX([3]float64{w.EndU, w.EndV, 0})
			key, ok := b.key2(cyl, plane, x)
			if !ok {
				return errCBMiss
			}
			b.table[key] = cbEntry{value: x}
		}
	}
	return nil
}

// key2 keys a cylinder's crossing with a plane parallel to its axis at the
// point x. The side is decided by comparing x's free coordinate with the
// centre's; a crossing that lands on the centre's coordinate is too close
// to tangency to key and misses.
func (b *cbBuild) key2(cyl cbCarrier, plane cbGeom, x [3]float64) (cbKey2, bool) {
	g := b.geom(cyl)
	free := 3 - g.axis - plane.axis
	if free < 0 || free > 2 || plane.axis == g.axis {
		return cbKey2{}, false
	}
	diff := x[free] - g.center[free]
	if diff == 0 {
		return cbKey2{}, false
	}
	side := 1
	if diff < 0 {
		side = -1
	}
	return cbKey2{cyl: cyl, planeAxis: plane.axis, planeLevel: plane.level, side: side}, true
}

// canonicalPoint is the one point the reach records for the vertex where
// carriers c1 and c2 meet face carrier f, near the scene's own point x with
// its displacement delta (§10). Three planes meet at their exact levels. A
// cylinder and two planes meet at the keyed crossing of the cylinder with
// the plane along its axis: the table's recorded float, the first scene to
// reach a key recording it, at the axial plane's exact level.
func (b *cbBuild) canonicalPoint(f, c1, c2 cbCarrier, x [3]float64, delta float64) (cbVertex, error) {
	carriers := []cbCarrier{f, c1, c2}
	var planes []cbGeom
	var cyl *cbCarrier
	for i, c := range carriers {
		g := b.geom(c)
		if g.cyl {
			if cyl != nil && *cyl != c {
				return cbVertex{}, errCBMiss
			}
			cyl = &carriers[i]
			continue
		}
		planes = append(planes, g)
	}
	var out [3]float64
	switch {
	case cyl == nil:
		seen := [3]bool{}
		for _, p := range planes {
			if seen[p.axis] {
				return cbVertex{}, errCBMiss
			}
			seen[p.axis] = true
			out[p.axis] = p.level
		}
		return cbVertex{point: out, carriers: carriers}, nil
	case len(planes) == 2:
		g := b.geom(*cyl)
		axial, along := planes[0], planes[1]
		if along.axis == g.axis {
			axial, along = along, axial
		}
		if axial.axis != g.axis || along.axis == g.axis {
			return cbVertex{}, errCBMiss
		}
		free := 3 - g.axis - along.axis
		if math.Abs(x[free]-g.center[free]) <= 2*delta {
			return cbVertex{}, errCBMiss // too close to tangency to key (§10)
		}
		key, ok := b.key2(*cyl, along, x)
		if !ok {
			return cbVertex{}, errCBMiss
		}
		entry, ok := b.table[key]
		if !ok {
			entry = cbEntry{value: x, delta: delta}
			entry.value[along.axis] = along.level
			b.table[key] = entry
		}
		out = entry.value
		out[g.axis] = axial.level
		return cbVertex{point: out, carriers: carriers, delta: entry.delta}, nil
	default:
		return cbVertex{}, errCBMiss
	}
}

// record registers a canonical vertex with every carrier it was found on.
func (b *cbBuild) record(v cbVertex) {
	existing, ok := b.verts[v.point]
	if !ok {
		c := v
		c.carriers = slices.Clone(v.carriers)
		b.verts[v.point] = &c
		return
	}
	existing.delta = max(existing.delta, v.delta)
	for _, c := range v.carriers {
		if !slices.Contains(existing.carriers, c) {
			existing.carriers = append(existing.carriers, c)
		}
	}
}

// canonicalize merges a region's consecutive fragments of one carrier (a
// circle cut at its own seam), then replaces every junction with its
// canonical vertex and rewrites each segment between its two canonical ends:
// a line as a whole line, a circular fragment as an arc pinned there. The
// face carries the largest displacement of any keyed vertex it uses.
func (b *cbBuild) canonicalize(f *cbFace) error {
	loops := append([]LoopRecord{f.region.Outer}, f.region.Holes...)
	for li := range loops {
		segs, carriers := b.mergeSeams(loops[li].Segments, f.carriers[li])
		n := len(segs)
		if n == 1 {
			loops[li] = LoopRecord{Segments: segs}
			f.carriers[li] = carriers
			continue
		}
		walks := make([]survey2d.SegmentWalk, n)
		for i, seg := range segs {
			w, err := walkOf(seg, nil)
			if err != nil {
				return err
			}
			walks[i] = w
		}
		ends := make([][3]float64, n)
		for i := range segs {
			j := (i + 1) % n
			// The scene's own point: a line's lerp keeps its fixed coordinate
			// exact, so a line's end is preferred to an arc's.
			pick, wk := segs[i], walks[i]
			u, v := wk.EndU, wk.EndV
			bound := wk.EndBound
			if _, line := segs[i].(LineSeg); !line {
				if _, nextLine := segs[j].(LineSeg); nextLine {
					pick, wk = segs[j], walks[j]
					u, v, bound = wk.StartU, wk.StartV, wk.StartBound
				}
			}
			x := f.frame.toX([3]float64{u, v, f.level})
			delta := proofbound.AbsSumUpper(proofbound.CutDisplacementAllow(classBSpeed(pick)), proofbound.WalkEndBoundAllow(bound))
			vx, err := b.canonicalPoint(f.carrier, carriers[i], carriers[j], x, delta)
			if err != nil {
				return err
			}
			b.record(vx)
			ends[i] = vx.point
			f.delta = max(f.delta, vx.delta)
		}
		out := make([]CurveSegment, n)
		for i := range segs {
			from, to := ends[(i+n-1)%n], ends[i]
			out[i] = cbBetween(walks[i], f.frame, from, to)
		}
		loops[li] = LoopRecord{Segments: out}
		f.carriers[li] = carriers
	}
	f.region = ProfileRecord{Outer: loops[0], Holes: loops[1:]}
	return nil
}

// mergeSeams joins consecutive fragments that share one circular carrier:
// sketch cuts a whole circle at its own seam, which is no vertex of the
// body.
func (b *cbBuild) mergeSeams(segs []CurveSegment, carriers []cbCarrier) ([]CurveSegment, []cbCarrier) {
	n := len(segs)
	if n < 2 {
		return segs, carriers
	}
	outS, outC := []CurveSegment{}, []cbCarrier{}
	for i := range n {
		if i > 0 && carriers[i] == outC[len(outC)-1] && b.geom(carriers[i]).cyl {
			outS[len(outS)-1] = cbJoin(outS[len(outS)-1], segs[i])
			continue
		}
		outS, outC = append(outS, segs[i]), append(outC, carriers[i])
	}
	if len(outS) > 1 && outC[0] == outC[len(outC)-1] && b.geom(outC[0]).cyl {
		outS[0] = cbJoin(outS[len(outS)-1], outS[0])
		outS, outC = outS[:len(outS)-1], outC[:len(outC)-1]
	}
	return outS, outC
}

// cbJoin joins two consecutive fragments of one circle into one fragment
// over both parameter ranges, reading a whole circle's range as one turn.
func cbJoin(a, b CurveSegment) CurveSegment {
	ca, okA := a.(CircleSeg)
	cb, okB := b.(CircleSeg)
	if !okA || !okB {
		return a
	}
	out := ca
	switch {
	case ca.TEnd == 1 && cb.TStart == 0:
		out.TEnd = 1 + cb.TEnd
	case ca.TEnd == 0 && cb.TStart == 1:
		out.TEnd = cb.TEnd - 1
	default:
		out.TEnd = cb.TEnd
	}
	return out
}

// cbBetween rewrites one region segment to run from one canonical vertex to
// the next: a line as a whole line, a circular fragment as an arc pinned at
// both, about the carrier's own recorded centre and in the fragment's own
// sense.
func cbBetween(w survey2d.SegmentWalk, frame cbFrame, from, to [3]float64) CurveSegment {
	pf, pt := frame.toLocal(from), frame.toLocal(to)
	start, end := Point2{U: pf[0], V: pf[1]}, Point2{U: pt[0], V: pt[1]}
	if w.IsLine() {
		return LineSeg{Start: start, End: end, TStart: 0, TEnd: 1}
	}
	return arcSegment(Point2{U: w.CU, V: w.CV}, start, end, w.Th1 > w.Th0)
}

// split cuts every edge of a face at every canonical vertex lying on it: a
// line at each vertex on both the face's carrier and the line's, an arc at
// each vertex on its cylinder, read at the face's level, strictly inside
// its span. Edges two faces share are then split alike, so they pair.
func (b *cbBuild) split(f *cbFace) error {
	loops := append([]LoopRecord{f.region.Outer}, f.region.Holes...)
	for li, loop := range loops {
		var segs []CurveSegment
		var carriers []cbCarrier
		for si, seg := range loop.Segments {
			c := f.carriers[li][si]
			pieces, err := b.splitSegment(f, seg, c)
			if err != nil {
				return err
			}
			for _, p := range pieces {
				segs = append(segs, p)
				carriers = append(carriers, c)
			}
		}
		loops[li] = LoopRecord{Segments: segs}
		f.carriers[li] = carriers
	}
	f.region = ProfileRecord{Outer: loops[0], Holes: loops[1:]}
	return nil
}

func (b *cbBuild) splitSegment(f *cbFace, seg CurveSegment, c cbCarrier) ([]CurveSegment, error) {
	w, err := walkOf(seg, nil)
	if err != nil {
		return nil, err
	}
	g := b.geom(c)
	type cut struct {
		at  [3]float64
		key float64
	}
	var cuts []cut
	if w.IsLine() {
		start := f.frame.toX([3]float64{w.StartU, w.StartV, f.level})
		end := f.frame.toX([3]float64{w.EndU, w.EndV, f.level})
		free := -1
		for k := range 3 {
			if start[k] != end[k] {
				free = k
			}
		}
		if free < 0 {
			return nil, errCBMiss
		}
		lo, hi := math.Min(start[free], end[free]), math.Max(start[free], end[free])
		for p, v := range b.verts {
			if !slices.Contains(v.carriers, f.carrier) || !slices.Contains(v.carriers, c) {
				continue
			}
			if p[free] > lo && p[free] < hi {
				cuts = append(cuts, cut{at: p, key: math.Abs(p[free] - start[free])})
			}
		}
		slices.SortFunc(cuts, func(a, b cut) int { return cmpFloat(a.key, b.key) })
		points := [][3]float64{start}
		for _, k := range cuts {
			points = append(points, k.at)
		}
		points = append(points, end)
		var out []CurveSegment
		for i := 0; i+1 < len(points); i++ {
			if points[i] == points[i+1] {
				return nil, errCBMiss
			}
			pf, pt := f.frame.toLocal(points[i]), f.frame.toLocal(points[i+1])
			out = append(out, LineSeg{Start: Point2{U: pf[0], V: pf[1]}, End: Point2{U: pt[0], V: pt[1]}, TStart: 0, TEnd: 1})
		}
		return out, nil
	}
	if !g.cyl || f.frame.axis[2] != g.axis {
		return []CurveSegment{seg}, nil
	}
	// An arc on a cylinder, in a face across the cylinder's axis: cut at the
	// angle of every vertex on the cylinder.
	ccw := w.Th1 > w.Th0
	angle := func(p [3]float64) float64 {
		l := f.frame.toLocal(p)
		return math.Atan2(l[1]-w.CV, l[0]-w.CU)
	}
	sweep := func(from, to float64) float64 {
		d := to - from
		if !ccw {
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
	startPt := f.frame.toX([3]float64{w.StartU, w.StartV, f.level})
	endPt := f.frame.toX([3]float64{w.EndU, w.EndV, f.level})
	a0 := angle(startPt)
	total := sweep(a0, angle(endPt))
	if w.Closed {
		total = 2 * math.Pi
	}
	// Two vertices this close in angle are not ordered by their held
	// points, so the pair misses rather than guess their order.
	const tol = 1e-9
	for p, v := range b.verts {
		if !slices.Contains(v.carriers, c) {
			continue
		}
		q := p
		q[g.axis] = f.frame.toX([3]float64{0, 0, f.level})[g.axis]
		if q == startPt || q == endPt {
			continue
		}
		s := sweep(a0, angle(q))
		if !w.Closed {
			if s >= total+tol {
				continue // outside the arc's span
			}
			if s <= tol || s >= total-tol {
				return nil, errCBMiss // too close to an end to order
			}
		}
		cuts = append(cuts, cut{at: q, key: s})
	}
	slices.SortFunc(cuts, func(a, b cut) int { return cmpFloat(a.key, b.key) })
	// Vertices at other levels project to one point: keep it once.
	cuts = slices.CompactFunc(cuts, func(a, b cut) bool { return a.at == b.at })
	for i := 1; i < len(cuts); i++ {
		if cuts[i].key-cuts[i-1].key <= tol {
			return nil, errCBMiss
		}
	}
	if len(cuts) == 0 {
		return []CurveSegment{seg}, nil
	}
	points := [][3]float64{startPt}
	for _, k := range cuts {
		points = append(points, k.at)
	}
	if w.Closed {
		// A whole circle cut at its vertices starts and ends at the first.
		points = points[1:]
		points = append(points, points[0])
	} else {
		points = append(points, endPt)
	}
	var out []CurveSegment
	for i := 0; i+1 < len(points); i++ {
		pf, pt := f.frame.toLocal(points[i]), f.frame.toLocal(points[i+1])
		out = append(out, arcSegment(Point2{U: w.CU, V: w.CV}, Point2{U: pf[0], V: pf[1]}, Point2{U: pt[0], V: pt[1]}, ccw))
	}
	return out, nil
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

// cylinderPieces reads every cylinder's surviving pieces off the planar
// faces across its axis (§5): each arc the faces carry on the cylinder is a
// fragment, the levels of the faces carrying one fragment sorted along the
// axis pair up into the pieces' ends, and each piece is the fragment swept
// between a pair. X's pieces keep X's sense; Y's are walked reversed, since
// Y's material leaves the result.
func (b *cbBuild) cylinderPieces() ([]brepFace, error) {
	type fragKey struct {
		c        cbCarrier
		from, to [2]float64
	}
	type frag struct {
		seg    CurveSegment
		frame  cbFrame
		levels []float64
		delta  float64
	}
	frags := map[fragKey]*frag{}
	var order []fragKey
	for _, f := range b.faces {
		loops := append([]LoopRecord{f.region.Outer}, f.region.Holes...)
		for li, loop := range loops {
			for si, seg := range loop.Segments {
				c := f.carriers[li][si]
				g := b.geom(c)
				if !g.cyl || f.frame.axis[2] != g.axis {
					continue
				}
				w, err := walkOf(seg, nil)
				if err != nil {
					return nil, err
				}
				// Key the fragment by its two ends in the cylinder's own plane,
				// in counter-clockwise order, so both faces bounding one piece
				// name it alike whichever way they walk it.
				from, to := [2]float64{w.StartU, w.StartV}, [2]float64{w.EndU, w.EndV}
				if w.Th1 < w.Th0 {
					from, to = to, from
				}
				k := fragKey{c: c, from: from, to: to}
				fr, ok := frags[k]
				if !ok {
					fr = &frag{seg: seg, frame: f.frame}
					frags[k] = fr
					order = append(order, k)
				}
				fr.levels = append(fr.levels, f.frame.toX([3]float64{0, 0, f.level})[g.axis])
				fr.delta = max(fr.delta, f.delta)
			}
		}
	}
	var out []brepFace
	for _, k := range order {
		fr := frags[k]
		slices.Sort(fr.levels)
		if len(fr.levels)%2 != 0 {
			return nil, errCBMiss
		}
		own := b.fX
		if k.c.op == cbY {
			own = b.fG
		}
		// The fragment in its own operand's frame, in that operand's walk
		// sense: X's material lies left of X's walk; Y's pieces bound the
		// result, so they run reversed.
		wseg, err := b.reframe(fr.seg, fr.frame, own)
		if err != nil {
			return nil, err
		}
		recorded := b.x.profile.Outer.Segments[k.c.idx]
		if k.c.op == cbY {
			recorded = b.cp.y.profile.Outer.Segments[k.c.idx]
		}
		rw, err := walkOf(recorded, nil)
		if err != nil {
			return nil, err
		}
		ww, err := walkOf(wseg, nil)
		if err != nil {
			return nil, err
		}
		same := (ww.Th1 > ww.Th0) == (rw.Th1 > rw.Th0)
		if (k.c.op == cbX) != same {
			rev, err := reverseLoopRecord(LoopRecord{Segments: []CurveSegment{wseg}})
			if err != nil {
				return nil, err
			}
			wseg = rev.Segments[0]
		}
		for i := 0; i+1 < len(fr.levels); i += 2 {
			z0 := own.toLocal(b.axisPoint(own, fr.levels[i]))[2]
			z1 := own.toLocal(b.axisPoint(own, fr.levels[i+1]))[2]
			if z0 > z1 {
				z0, z1 = z1, z0
			}
			if !(z0 < z1) {
				return nil, errCBMiss
			}
			out = append(out, brepFace{frame: own.frame, wall: wseg, z0: z0, z1: z1, delta: fr.delta})
		}
	}
	return out, nil
}

// axisPoint is the X-local point at level along frame's own axis.
func (b *cbBuild) axisPoint(frame cbFrame, level float64) [3]float64 {
	var x [3]float64
	x[frame.axis[2]] = level
	return x
}

// reframe restates a segment of a face in from in frame to: both frames'
// normals land on one X axis, so the in-plane coordinates permute exactly.
func (b *cbBuild) reframe(seg CurveSegment, from, to cbFrame) (CurveSegment, error) {
	mapPt := func(p Point2) Point2 {
		l := to.toLocal(from.toX([3]float64{p.U, p.V, 0}))
		return Point2{U: l[0], V: l[1]}
	}
	reflected := false
	{
		// A map that swaps handedness in the plane reverses every sense.
		e1 := to.toLocal(from.toX([3]float64{1, 0, 0}))
		e2 := to.toLocal(from.toX([3]float64{0, 1, 0}))
		reflected = e1[0]*e2[1]-e1[1]*e2[0] < 0
	}
	switch s := seg.(type) {
	case LineSeg:
		s.Start, s.End = mapPt(s.Start), mapPt(s.End)
		return s, nil
	case ArcSeg:
		s.Center, s.Start, s.End = mapPt(s.Center), mapPt(s.Start), mapPt(s.End)
		if reflected {
			s.Start, s.End = s.End, s.Start
		}
		return s, nil
	case CircleSeg:
		s.Center = mapPt(s.Center)
		if reflected {
			s.CCW = !s.CCW
			s.TStart, s.TEnd = s.TEnd, s.TStart
		}
		return s, nil
	}
	return nil, errCBMiss
}
