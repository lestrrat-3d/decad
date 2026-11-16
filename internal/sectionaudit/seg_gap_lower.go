package sectionaudit

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// This file is SegGapLower, the certified gap lower bound of
// agent-docs/carrier-record-design.md §6.2. segMinDist is a float closed form
// whose crossing tests ignore a crossing within segCrossEps of a segment end,
// so it reads such a crossing as a gap of about 1e-7·ℓ; SegGapLower reads
// every walk as exact data and rounds each square root toward the lower
// bound.

type gapPoint struct {
	p   [2]*big.Rat // an exact point within err of the point it stands for
	err *big.Rat
}

type gapPiece struct {
	line   bool
	closed bool
	// line: the two exact ends.
	p0, p1 [2]*big.Rat
	// circular: exact centre, exact radius (the walk's float radius) and, for
	// an arc, its two ends in ascending-angle order, each as an exact point
	// near the point at the radius on the float end's bearing.
	c      [2]*big.Rat
	r      *big.Rat
	e      [2]gapPoint
	a, b   [2]*big.Rat // directions from the centre to the ascending-angle ends
	small  bool        // sweep decidedly below π
	large  bool        // sweep decidedly above π
	endsOK bool
}

func ratPt(u, v float64) ([2]*big.Rat, bool) {
	ru, rv := proofarith.FloatRat(u), proofarith.FloatRat(v)
	if ru == nil || rv == nil {
		return [2]*big.Rat{}, false
	}
	return [2]*big.Rat{ru, rv}, true
}

func subPt(a, b [2]*big.Rat) [2]*big.Rat {
	return [2]*big.Rat{new(big.Rat).Sub(a[0], b[0]), new(big.Rat).Sub(a[1], b[1])}
}

func dotR(a, b [2]*big.Rat) *big.Rat {
	x := new(big.Rat).Mul(a[0], b[0])
	return x.Add(x, new(big.Rat).Mul(a[1], b[1]))
}

func crossR(a, b [2]*big.Rat) *big.Rat {
	x := new(big.Rat).Mul(a[0], b[1])
	return x.Sub(x, new(big.Rat).Mul(a[1], b[0]))
}

func sqrtLo(q *big.Rat) *big.Rat { return proofarith.FloatRat(proofbound.RatSqrtDown(q)) }
func sqrtHi(q *big.Rat) *big.Rat { return proofarith.FloatRat(proofbound.RatSqrtUp(q)) }

func newGapPiece(w survey2d.SegmentWalk) (gapPiece, bool) {
	var g gapPiece
	if w.IsLine() {
		var ok1, ok2 bool
		g.line = true
		g.p0, ok1 = ratPt(w.StartU, w.StartV)
		g.p1, ok2 = ratPt(w.EndU, w.EndV)
		return g, ok1 && ok2
	}
	if !w.IsCircular() {
		return g, false
	}
	var ok bool
	g.c, ok = ratPt(w.CU, w.CV)
	g.r = proofarith.FloatRat(w.Radius)
	if !ok || g.r == nil || g.r.Sign() <= 0 {
		return g, false
	}
	if w.Closed {
		g.closed = true
		return g, true
	}
	lo, hi := [2]float64{w.StartU, w.StartV}, [2]float64{w.EndU, w.EndV}
	if w.Th1 < w.Th0 {
		lo, hi = hi, lo
	}
	span := math.Abs(w.Th1 - w.Th0)
	for i, f := range [2][2]float64{lo, hi} {
		pt, ok := ratPt(f[0], f[1])
		if !ok {
			return g, false
		}
		d := subPt(pt, g.c)
		d2 := dotR(d, d)
		if d2.Sign() == 0 {
			return g, false
		}
		kLo := proofarith.FloatRat(proofbound.RatFloatDown(new(big.Rat).Quo(g.r, sqrtHi(d2))))
		kHi := proofarith.FloatRat(proofbound.RatFloatUp(new(big.Rat).Quo(g.r, sqrtLo(d2))))
		if kLo == nil || kHi == nil || kLo.Sign() <= 0 {
			return g, false
		}
		p := [2]*big.Rat{
			new(big.Rat).Add(g.c[0], new(big.Rat).Mul(d[0], kLo)),
			new(big.Rat).Add(g.c[1], new(big.Rat).Mul(d[1], kLo)),
		}
		width := new(big.Rat).Sub(kHi, kLo)
		err := proofarith.FloatRat(proofbound.RatFloatUp(width.Mul(width, sqrtHi(d2))))
		if err == nil {
			return g, false
		}
		g.e[i] = gapPoint{p: p, err: err}
		if i == 0 {
			g.a = d
		} else {
			g.b = d
		}
	}
	const slack = 1e-9
	g.small = span < math.Pi-slack
	g.large = span > math.Pi+slack
	g.endsOK = true
	return g, true
}

// provenOut reports that every point of the box spanned by the given
// corner vectors (relative to the centre) lies outside the arc's bearing
// range. It never claims it for a closed circle or an undecided sweep.
func (g gapPiece) provenOut(corners [][2]*big.Rat) bool {
	if g.closed || g.line || !g.endsOK {
		return false
	}
	all := func(f func(c [2]*big.Rat) bool) bool {
		for _, c := range corners {
			if !f(c) {
				return false
			}
		}
		return true
	}
	switch {
	case g.small:
		// the convex wedge from a counter-clockwise to b
		return all(func(d [2]*big.Rat) bool { return crossR(g.a, d).Sign() < 0 }) ||
			all(func(d [2]*big.Rat) bool { return crossR(d, g.b).Sign() < 0 })
	case g.large:
		// the arc is the complement of the open wedge from b to a
		return all(func(d [2]*big.Rat) bool { return crossR(g.b, d).Sign() > 0 && crossR(d, g.a).Sign() > 0 })
	}
	return false
}

func (g gapPiece) dirOut(d [2]*big.Rat) bool { return g.provenOut([][2]*big.Rat{d}) }

// distLowerRat is a lower bound, as a rational, on the distance between two
// exact points.
func distLowerRat(p, q [2]*big.Rat) *big.Rat {
	d := subPt(p, q)
	return sqrtLo(dotR(d, d))
}

func subClamp(x, y *big.Rat) *big.Rat {
	d := new(big.Rat).Sub(x, y)
	if d.Sign() < 0 {
		return new(big.Rat)
	}
	return d
}

// pointToSegment is a lower bound on the distance from exact point q to the
// segment p0p1.
func pointToSegment(q, p0, p1 [2]*big.Rat) *big.Rat {
	d := subPt(p1, p0)
	dd := dotR(d, d)
	if dd.Sign() == 0 {
		return distLowerRat(q, p0)
	}
	t := new(big.Rat).Quo(dotR(subPt(q, p0), d), dd)
	switch {
	case t.Sign() < 0:
		t = new(big.Rat)
	case t.Cmp(big.NewRat(1, 1)) > 0:
		t = big.NewRat(1, 1)
	}
	foot := [2]*big.Rat{
		new(big.Rat).Add(p0[0], new(big.Rat).Mul(d[0], t)),
		new(big.Rat).Add(p0[1], new(big.Rat).Mul(d[1], t)),
	}
	return distLowerRat(q, foot)
}

// radialLower is a lower bound on | |p − c| − r |.
func radialLower(p, c [2]*big.Rat, r *big.Rat) *big.Rat {
	d := subPt(p, c)
	d2 := dotR(d, d)
	if d2.Cmp(new(big.Rat).Mul(r, r)) >= 0 {
		return subClamp(sqrtLo(d2), r)
	}
	return subClamp(r, sqrtHi(d2))
}

// pointToPiece is a lower bound on the distance from an exact point to the
// piece.
func pointToPiece(q [2]*big.Rat, g gapPiece) *big.Rat {
	var best *big.Rat
	take := func(x *big.Rat) {
		if best == nil || x.Cmp(best) < 0 {
			best = x
		}
	}
	switch {
	case g.line:
		take(pointToSegment(q, g.p0, g.p1))
	case g.closed:
		take(radialLower(q, g.c, g.r))
	default:
		for _, e := range g.e {
			take(subClamp(distLowerRat(q, e.p), e.err))
		}
		if !g.dirOut(subPt(q, g.c)) {
			take(radialLower(q, g.c, g.r))
		}
	}
	return best
}

func segmentsMeet(a0, a1, b0, b1 [2]*big.Rat) bool {
	orient := func(p, q, r [2]*big.Rat) int { return crossR(subPt(q, p), subPt(r, p)).Sign() }
	onSeg := func(p, q, r [2]*big.Rat) bool { // r collinear with pq: within its box
		return minR(p[0], q[0]).Cmp(r[0]) <= 0 && r[0].Cmp(maxR(p[0], q[0])) <= 0 &&
			minR(p[1], q[1]).Cmp(r[1]) <= 0 && r[1].Cmp(maxR(p[1], q[1])) <= 0
	}
	o1, o2 := orient(a0, a1, b0), orient(a0, a1, b1)
	o3, o4 := orient(b0, b1, a0), orient(b0, b1, a1)
	if o1*o2 < 0 && o3*o4 < 0 {
		return true
	}
	return (o1 == 0 && onSeg(a0, a1, b0)) || (o2 == 0 && onSeg(a0, a1, b1)) ||
		(o3 == 0 && onSeg(b0, b1, a0)) || (o4 == 0 && onSeg(b0, b1, a1))
}

func minR(a, b *big.Rat) *big.Rat {
	if a.Cmp(b) <= 0 {
		return a
	}
	return b
}

func maxR(a, b *big.Rat) *big.Rat {
	if a.Cmp(b) >= 0 {
		return a
	}
	return b
}

// SegGapLower is a proven lower bound on the distance between two closed
// walks, each read as exact data: its float ends, centre and radius as exact
// rationals, an arc being the points at that radius whose bearing lies in the
// walk's range (agent-docs/carrier-record-design.md §6.2). It answers 0
// unless the pieces are proven apart.
func SegGapLower(a, b survey2d.SegmentWalk) float64 {
	ga, okA := newGapPiece(a)
	gb, okB := newGapPiece(b)
	if !okA || !okB {
		return 0
	}
	if gb.line && !ga.line {
		ga, gb = gb, ga
	}
	var out *big.Rat
	switch {
	case ga.line && gb.line:
		out = gapLineLine(ga, gb)
	case ga.line:
		out = gapLineCircular(ga, gb)
	default:
		out = gapCircularCircular(ga, gb)
	}
	if out == nil || out.Sign() <= 0 {
		return 0
	}
	return proofbound.RatFloatDown(out)
}

func zeroR() *big.Rat { return new(big.Rat) }

func minOf(xs ...*big.Rat) *big.Rat {
	var best *big.Rat
	for _, x := range xs {
		if best == nil || x.Cmp(best) < 0 {
			best = x
		}
	}
	return best
}

func gapLineLine(a, b gapPiece) *big.Rat {
	if segmentsMeet(a.p0, a.p1, b.p0, b.p1) {
		return zeroR()
	}
	return minOf(pointToSegment(a.p0, b.p0, b.p1), pointToSegment(a.p1, b.p0, b.p1),
		pointToSegment(b.p0, a.p0, a.p1), pointToSegment(b.p1, a.p0, a.p1))
}

func ratBox(lo0, hi0, lo1, hi1 *big.Rat) [][2]*big.Rat {
	return [][2]*big.Rat{{lo0, lo1}, {lo0, hi1}, {hi0, lo1}, {hi0, hi1}}
}

func relTo(corners [][2]*big.Rat, c [2]*big.Rat) [][2]*big.Rat {
	out := make([][2]*big.Rat, len(corners))
	for i, p := range corners {
		out[i] = subPt(p, c)
	}
	return out
}

// gapLineCircular is the gap between a line segment and a circle or arc.
func gapLineCircular(l, g gapPiece) *big.Rat {
	d := subPt(l.p1, l.p0)
	dd := dotR(d, d)
	if dd.Sign() == 0 {
		return pointToPiece(l.p0, g)
	}
	cp := subPt(g.c, l.p0)
	proj := dotR(cp, d)
	sf := new(big.Rat).Quo(proj, dd)
	perp2 := new(big.Rat).Sub(dotR(cp, cp), new(big.Rat).Quo(new(big.Rat).Mul(proj, proj), dd))
	r2 := new(big.Rat).Mul(g.r, g.r)
	if perp2.Cmp(r2) <= 0 {
		h2 := new(big.Rat).Quo(new(big.Rat).Sub(r2, perp2), dd)
		sLo, sHi := sqrtLo(h2), sqrtHi(h2)
		allOut := true
		for _, sign := range []int{-1, 1} {
			var lo, hi *big.Rat
			if sign < 0 {
				lo, hi = new(big.Rat).Sub(sf, sHi), new(big.Rat).Sub(sf, sLo)
			} else {
				lo, hi = new(big.Rat).Add(sf, sLo), new(big.Rat).Add(sf, sHi)
			}
			if hi.Sign() < 0 || lo.Cmp(big.NewRat(1, 1)) > 0 {
				continue // outside the line piece
			}
			at := func(s *big.Rat, k int) *big.Rat {
				return new(big.Rat).Add(l.p0[k], new(big.Rat).Mul(d[k], s))
			}
			var b [2][2]*big.Rat
			for k := range 2 {
				x0, x1 := at(lo, k), at(hi, k)
				b[k] = [2]*big.Rat{minR(x0, x1), maxR(x0, x1)}
			}
			if g.provenOut(relTo(ratBox(b[0][0], b[0][1], b[1][0], b[1][1]), g.c)) {
				continue
			}
			allOut = false
			break
		}
		if !allOut {
			return zeroR()
		}
	}
	cands := []*big.Rat{pointToPiece(l.p0, g), pointToPiece(l.p1, g)}
	if !g.closed {
		for _, e := range g.e {
			cands = append(cands, subClamp(pointToSegment(e.p, l.p0, l.p1), e.err))
		}
	}
	if sf.Sign() >= 0 && sf.Cmp(big.NewRat(1, 1)) <= 0 {
		foot := [2]*big.Rat{
			new(big.Rat).Add(l.p0[0], new(big.Rat).Mul(d[0], sf)),
			new(big.Rat).Add(l.p0[1], new(big.Rat).Mul(d[1], sf)),
		}
		fc := subPt(foot, g.c)
		fc2 := dotR(fc, fc)
		if fc2.Sign() == 0 {
			cands = append(cands, g.r)
		} else {
			neg := [2]*big.Rat{new(big.Rat).Neg(fc[0]), new(big.Rat).Neg(fc[1])}
			if !g.dirOut(fc) {
				cands = append(cands, radialLower(foot, g.c, g.r))
			}
			if !g.dirOut(neg) {
				cands = append(cands, new(big.Rat).Add(sqrtLo(fc2), g.r))
			}
		}
	}
	return minOf(cands...)
}

// gapCircularCircular is the gap between two circles or arcs.
func gapCircularCircular(a, b gapPiece) *big.Rat {
	d := subPt(b.c, a.c)
	d2 := dotR(d, d)
	cands := []*big.Rat{}
	if !a.closed {
		for _, e := range a.e {
			cands = append(cands, subClamp(pointToPiece(e.p, b), e.err))
		}
	}
	if !b.closed {
		for _, e := range b.e {
			cands = append(cands, subClamp(pointToPiece(e.p, a), e.err))
		}
	}
	if d2.Sign() == 0 {
		diff := new(big.Rat).Sub(a.r, b.r)
		if diff.Sign() == 0 {
			if a.closed || b.closed {
				return zeroR() // a whole circle lies on its carrier
			}
			// Two arcs of one circle are apart exactly as far as their
			// nearest ends: an overlap puts an end inside the other arc, and
			// that end's distance reads zero.
			return minOf(cands...)
		}
		// The radial gap counts only where the two ranges overlap: arcs whose
		// every end lies provably outside the other are apart along the circle.
		disjoint := !a.closed && !b.closed && b.dirOut(a.a) && b.dirOut(a.b) && a.dirOut(b.a) && a.dirOut(b.b)
		if !disjoint {
			cands = append(cands, new(big.Rat).Abs(diff))
		}
		return minOf(cands...)
	}
	sum2 := new(big.Rat).Add(a.r, b.r)
	sum2.Mul(sum2, sum2)
	dif2 := new(big.Rat).Sub(a.r, b.r)
	dif2.Mul(dif2, dif2)
	if d2.Cmp(sum2) <= 0 && d2.Cmp(dif2) >= 0 {
		// the carriers meet: the two meeting points
		ra2, rb2 := new(big.Rat).Mul(a.r, a.r), new(big.Rat).Mul(b.r, b.r)
		af := new(big.Rat).Add(d2, new(big.Rat).Sub(ra2, rb2))
		af.Quo(af, new(big.Rat).Mul(big.NewRat(2, 1), d2))
		m := [2]*big.Rat{
			new(big.Rat).Add(a.c[0], new(big.Rat).Mul(d[0], af)),
			new(big.Rat).Add(a.c[1], new(big.Rat).Mul(d[1], af)),
		}
		h2 := new(big.Rat).Sub(ra2, new(big.Rat).Mul(new(big.Rat).Mul(af, af), d2))
		h2.Quo(h2, d2)
		if h2.Sign() < 0 {
			h2 = zeroR()
		}
		sLo, sHi := sqrtLo(h2), sqrtHi(h2)
		perp := [2]*big.Rat{new(big.Rat).Neg(d[1]), d[0]}
		for _, sign := range []int{-1, 1} {
			var box [2][2]*big.Rat
			for k := range 2 {
				x0 := new(big.Rat).Add(m[k], new(big.Rat).Mul(perp[k], new(big.Rat).Mul(big.NewRat(int64(sign), 1), sLo)))
				x1 := new(big.Rat).Add(m[k], new(big.Rat).Mul(perp[k], new(big.Rat).Mul(big.NewRat(int64(sign), 1), sHi)))
				box[k] = [2]*big.Rat{minR(x0, x1), maxR(x0, x1)}
			}
			corners := ratBox(box[0][0], box[0][1], box[1][0], box[1][1])
			if a.provenOut(relTo(corners, a.c)) || b.provenOut(relTo(corners, b.c)) {
				continue
			}
			return zeroR()
		}
	}
	dLo, dHi := sqrtLo(d2), sqrtHi(d2)
	for _, s1 := range []int{-1, 1} {
		for _, s2 := range []int{-1, 1} {
			da := [2]*big.Rat{new(big.Rat).Mul(big.NewRat(int64(s1), 1), d[0]), new(big.Rat).Mul(big.NewRat(int64(s1), 1), d[1])}
			db := [2]*big.Rat{new(big.Rat).Mul(big.NewRat(int64(s2), 1), d[0]), new(big.Rat).Mul(big.NewRat(int64(s2), 1), d[1])}
			if a.dirOut(da) || b.dirOut(db) {
				continue
			}
			k := new(big.Rat).Sub(new(big.Rat).Mul(big.NewRat(int64(s2), 1), b.r), new(big.Rat).Mul(big.NewRat(int64(s1), 1), a.r))
			lo, hi := new(big.Rat).Add(dLo, k), new(big.Rat).Add(dHi, k)
			switch {
			case lo.Sign() > 0:
				cands = append(cands, lo)
			case hi.Sign() < 0:
				cands = append(cands, new(big.Rat).Neg(hi))
			default:
				cands = append(cands, zeroR())
			}
		}
	}
	return minOf(cands...)
}

// WalkGapBox is a bounding box of the piece SegGapLower reads for a walk
// (§6.2), rounded outward: a line's two exact ends, a closed circle's full
// box, and for an arc the enclosures of its two ends (the points at its
// radius on the float ends' bearings, each widened by its error) together with
// every cardinal point of the circle that is not proven outside its bearing
// range. It is built from the same endpoint enclosures as SegGapLower, so a
// gap between two such boxes is a lower bound on SegGapLower's own distance.
// ok=false when the walk is not readable.
func WalkGapBox(w survey2d.SegmentWalk) (minU, minV, maxU, maxV float64, ok bool) {
	g, ok := newGapPiece(w)
	if !ok {
		return 0, 0, 0, 0, false
	}
	var lo, hi [2]*big.Rat
	fold := func(p [2]*big.Rat, err *big.Rat) {
		for k := range 2 {
			a, b := p[k], p[k]
			if err != nil {
				a, b = new(big.Rat).Sub(p[k], err), new(big.Rat).Add(p[k], err)
			}
			if lo[k] == nil || a.Cmp(lo[k]) < 0 {
				lo[k] = a
			}
			if hi[k] == nil || b.Cmp(hi[k]) > 0 {
				hi[k] = b
			}
		}
	}
	if g.line {
		fold(g.p0, nil)
		fold(g.p1, nil)
	} else {
		if !g.closed {
			fold(g.e[0].p, g.e[0].err)
			fold(g.e[1].p, g.e[1].err)
		}
		zero := zeroR()
		for _, dir := range [][2]*big.Rat{{g.r, zero}, {new(big.Rat).Neg(g.r), zero}, {zero, g.r}, {zero, new(big.Rat).Neg(g.r)}} {
			if g.dirOut(dir) {
				continue
			}
			fold([2]*big.Rat{new(big.Rat).Add(g.c[0], dir[0]), new(big.Rat).Add(g.c[1], dir[1])}, nil)
		}
	}
	return proofbound.RatFloatDown(lo[0]), proofbound.RatFloatDown(lo[1]),
		proofbound.RatFloatUp(hi[0]), proofbound.RatFloatUp(hi[1]), true
}
