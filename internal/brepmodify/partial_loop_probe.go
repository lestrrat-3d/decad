package brepmodify

import (
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// fineSagitta is how far a chord of the boundary polygon the end probes read may lie inside an arc of the loop, in
// millimetres. It is far below partialSagitta, so a feature the probe cannot see is one the cutter's own bound hides.
const fineSagitta = 1e-6

// fineBoundary samples the loop as a closed polygon whose chords lie within fineSagitta of its arcs.
func (l *chamferLoop) fineBoundary() []pt2 {
	var out []pt2
	for _, s := range l.segs {
		out = append(out, s.p0)
		if !s.arc {
			continue
		}
		a0 := math.Atan2(s.p0.y-s.c.y, s.p0.x-s.c.x)
		delta := s.sweep()
		if !s.ccw {
			delta = -delta
		}
		cells := max(1, int(math.Ceil(s.sweep()/(2*math.Acos(1-min(fineSagitta/s.r, 1))))))
		for k := 1; k < cells; k++ {
			phi := a0 + delta*float64(k)/float64(cells)
			out = append(out, pt2{s.c.x + s.r*math.Cos(phi), s.c.y + s.r*math.Sin(phi)})
		}
	}
	return out
}

// probeLine is the line n.p = c, and probeCircle the circle about c.
type probeLine struct {
	n pt2
	c float64
}

type probeCircle struct {
	c pt2
	r float64
}

// probeRegion is the open region of the cap's plane that the cutter reaches past a convex end of its run, less the
// run-out zone: the stretch of the cap beside the next wall, on its material side, where the bevel runs out onto that
// wall and the cap's material belongs to the bevel. It is bounded by lines and circles, listed with member, which
// decides membership of a point strictly, and sweep, which lists one point in every cell the boundaries cut it into.
type probeRegion struct {
	lines   []probeLine
	circles []probeCircle
	member  func(p pt2) bool
	sweep   func() []pt2
	lo, hi  pt2
}

// runOutZone returns the two lines that bound the run-out zone of the end e, and the zone's membership test: the
// points on the material side of the next wall's line, within the next edge's length. The wall's own line lies in it.
func runOutZone(e runEnd) ([2]probeLine, func(p pt2) bool) {
	e2in := e.e2away.leftNormal()
	if e2in.dot(e.away.scale(-1)) < 0 {
		e2in = e2in.scale(-1)
	}
	lines := [2]probeLine{{e2in, e2in.dot(e.v) - 1e-9}, {e.e2away, e.e2away.dot(e.v) + e.e2Len}}
	return lines, func(p pt2) bool { return e2in.dot(p) > lines[0].c && e.e2away.dot(p) < lines[1].c }
}

// lineProbeRegion is the strip a line run's cutter reaches past its convex end e along tan, for ext, and reach in
// along in, less the run-out zone.
func lineProbeRegion(e runEnd, tan, in pt2, ext, reach float64) probeRegion {
	zone, inZone := runOutZone(e)
	reg := probeRegion{lines: zone[:], lo: pt2{math.Inf(1), math.Inf(1)}, hi: pt2{math.Inf(-1), math.Inf(-1)}}
	reg.lines = append(reg.lines, probeLine{tan, tan.dot(e.v)}, probeLine{tan, tan.dot(e.v) + ext},
		probeLine{in, in.dot(e.v)}, probeLine{in, in.dot(e.v) + reach})
	for _, a := range []float64{0, ext} {
		for _, b := range []float64{0, reach} {
			p := e.v.add(tan.scale(a)).add(in.scale(b))
			reg.lo = pt2{min(reg.lo.x, p.x), min(reg.lo.y, p.y)}
			reg.hi = pt2{max(reg.hi.x, p.x), max(reg.hi.y, p.y)}
		}
	}
	reg.member = func(p pt2) bool {
		a, b := p.sub(e.v).dot(tan), p.sub(e.v).dot(in)
		return a > 0 && a < ext && b > 0 && b < reach && !inZone(p)
	}
	reg.sweep = func() []pt2 {
		// Along a, the zone's lines cross the strip's sides b = 0 and b = reach at the events; along b, they cross the
		// ray at the crossings of the zone's lines.
		events := []float64{0, ext}
		for _, z := range zone {
			for _, b := range []float64{0, reach} {
				// n.(v + a tan + b in) = c
				if den := z.n.dot(tan); den != 0 {
					events = append(events, (z.c-z.n.dot(e.v)-b*z.n.dot(in))/den)
				}
			}
		}
		return cellPoints(events, 0, ext, func(a float64) (pt2, pt2) { return e.v.add(tan.scale(a)), in }, 0, reach, zone,
			reg.member)
	}
	return reg
}

// arcProbeRegion is the annular sector an arc run's cutter reaches past its convex end e along the circle of seg,
// turning alpha, to reach on both sides of the circle, less the run-out zone.
func arcProbeRegion(e runEnd, seg loopSeg, alpha, reach float64) probeRegion {
	zone, inZone := runOutZone(e)
	lo, hi := max(seg.r-reach, 0), seg.r+reach
	a0 := math.Atan2(e.v.y-seg.c.y, e.v.x-seg.c.x)
	sign := math.Copysign(1, alpha)
	steps := math.Abs(alpha)
	reg := probeRegion{lines: zone[:], circles: []probeCircle{{seg.c, lo}, {seg.c, hi}},
		lo: pt2{seg.c.x - hi, seg.c.y - hi}, hi: pt2{seg.c.x + hi, seg.c.y + hi}}
	for _, th := range []float64{0, alpha} {
		dir := pt2{math.Cos(a0 + th), math.Sin(a0 + th)}
		n := dir.leftNormal()
		reg.lines = append(reg.lines, probeLine{n, n.dot(seg.c)})
	}
	reg.member = func(p pt2) bool {
		rho := p.dist(seg.c)
		if !(rho > lo && rho < hi) {
			return false
		}
		rel := sign * math.Remainder(math.Atan2(p.y-seg.c.y, p.x-seg.c.x)-a0, 2*math.Pi)
		return rel > 0 && rel < steps && !inZone(p)
	}
	reg.sweep = func() []pt2 {
		events := []float64{0, steps}
		for _, z := range zone {
			for _, rho := range []float64{lo, hi} {
				// The line n.p = c meets the circle where n.(centre + rho u) = c, u = (cos phi, sin phi).
				d := z.c - z.n.dot(seg.c)
				if rho > 0 && math.Abs(d) <= rho {
					psi := math.Atan2(z.n.y, z.n.x)
					for _, phi := range []float64{psi - math.Acos(d/rho), psi + math.Acos(d/rho)} {
						events = append(events, sign*math.Remainder(phi-a0, 2*math.Pi))
					}
				}
			}
		}
		for i, x := range events {
			if x < 0 {
				events[i] = x + 2*math.Pi
			}
		}
		return cellPoints(events, 0, steps, func(th float64) (pt2, pt2) {
			dir := pt2{math.Cos(a0 + sign*th), math.Sin(a0 + sign*th)}
			return seg.c, dir
		}, lo, hi, zone, reg.member)
	}
	return reg
}

// cellPoints lists a point inside every cell of the region: between each pair of consecutive events (parameters of a
// family of rays, within lo..hi), the rays' crossings of the zone's lines and the band's limits cut a ray into pieces,
// and the middle of every piece that the region holds is listed. ray gives the origin and direction of the ray at an
// event parameter, along which the point is origin + b dir.
func cellPoints(events []float64, from, to float64, ray func(float64) (pt2, pt2), b0, b1 float64, zone [2]probeLine,
	member func(pt2) bool) []pt2 {
	var ts []float64
	for _, x := range events {
		if x > from && x < to {
			ts = append(ts, x)
		}
	}
	ts = append(ts, from, to)
	slices.Sort(ts)
	var out []pt2
	for i := 0; i+1 < len(ts); i++ {
		if ts[i+1]-ts[i] <= 0 {
			continue
		}
		o, dir := ray((ts[i] + ts[i+1]) / 2)
		bs := []float64{b0, b1}
		for _, z := range zone {
			if den := z.n.dot(dir); den != 0 {
				if b := (z.c - z.n.dot(o)) / den; b > b0 && b < b1 {
					bs = append(bs, b)
				}
			}
		}
		slices.Sort(bs)
		for j := 0; j+1 < len(bs); j++ {
			if p := o.add(dir.scale((bs[j] + bs[j+1]) / 2)); member(p) {
				out = append(out, p)
			}
		}
	}
	return out
}

// refuseMaterial refuses the run when the cap's material lies in the region. It is exact for the boundary polygon poly,
// which lies within fineSagitta of the loop. The loop's boundary meets the region where an edge of poly has a stretch in
// it, which lies between two consecutive crossings of the edge with the region's boundary curves, so the middle of
// each such stretch is tested. When no edge meets the region, each cell of it is wholly material or wholly void, and one
// point of each says which. Only the refusal is decided here: a region that the loop does not enter is void when the
// run's end is convex, and a witness inside material refuses.
func (reg probeRegion) refuseMaterial(l *chamferLoop, poly []pt2) error {
	refuse := fmt.Errorf(`%w: material of the cap lies beside a chamfered run's end, where the bevel would cut it`,
		decaderr.ErrUnsupported)
	for i, a := range poly {
		if reg.stretchIn(a, poly[(i+1)%len(poly)]) {
			return refuse
		}
	}
	for _, p := range reg.sweep() {
		if contains(poly, p) == l.outer {
			return refuse
		}
	}
	return nil
}

// stretchIn reports whether the segment ab has a stretch in the region: it lies between two consecutive crossings of
// the segment with the region's boundary curves, so the middle of each such stretch is tested.
func (reg probeRegion) stretchIn(a, b pt2) bool {
	if max(a.x, b.x) < reg.lo.x || min(a.x, b.x) > reg.hi.x || max(a.y, b.y) < reg.lo.y || min(a.y, b.y) > reg.hi.y {
		return false
	}
	ts := []float64{0, 1}
	d := b.sub(a)
	for _, ln := range reg.lines {
		if den := ln.n.dot(d); den != 0 {
			ts = append(ts, (ln.c-ln.n.dot(a))/den)
		}
	}
	for _, c := range reg.circles {
		f := a.sub(c.c)
		qa, qb, qc := d.dot(d), 2*f.dot(d), f.dot(f)-c.r*c.r
		if disc := qb*qb - 4*qa*qc; qa > 0 && disc >= 0 {
			root := math.Sqrt(disc)
			ts = append(ts, (-qb-root)/(2*qa), (-qb+root)/(2*qa))
		}
	}
	ts = slices.DeleteFunc(ts, func(t float64) bool { return t < 0 || t > 1 })
	slices.Sort(ts)
	for k := 0; k+1 < len(ts); k++ {
		if ts[k+1] > ts[k] && reg.member(a.add(d.scale((ts[k]+ts[k+1])/2))) {
			return true
		}
	}
	return false
}

// shortNextEdge decides a convex end e of a line run whose next edge is shorter than the cutter's reach along it,
// 1.5 reach/sin(turn). The end is built when the next edge is a line that holds the bevel's own run-out with room to
// spare, at least 1.5 d/sin(turn) long, and no edge of the loop but the next edge has a stretch in the bevel's
// footprint past the end: the strip from the corner along tan, as far as the cutter reaches past the end, and 1.5 d
// into the material along in. The material the cutter meets in that strip then lies between the run's wall carried on
// and the next wall, where the bevel runs out onto the next wall, and the bevel ends on that wall alone. The probe of
// the region past the end (refuseMaterial) still tests the rest of the cutter's reach. Any other such end is refused.
func (r chamferRun) shortNextEdge(e runEnd, seg loopSeg, tan, in pt2, d, reach float64, poly []pt2) error {
	refuse := fmt.Errorf(`%w: a chamfered run ends at a corner whose next edge is shorter than the bevel's run-out`,
		decaderr.ErrUnsupported)
	if seg.arc || e.nextArc || e.e2Len < 1.5*d/math.Sin(e.turn) {
		return refuse
	}
	ext, width := e.extFor(d, reach), 1.5*d
	reg := probeRegion{lo: pt2{math.Inf(1), math.Inf(1)}, hi: pt2{math.Inf(-1), math.Inf(-1)}}
	reg.lines = []probeLine{{tan, tan.dot(e.v)}, {tan, tan.dot(e.v) + ext}, {in, in.dot(e.v)}, {in, in.dot(e.v) + width}}
	for _, a := range []float64{0, ext} {
		for _, b := range []float64{0, width} {
			p := e.v.add(tan.scale(a)).add(in.scale(b))
			reg.lo = pt2{min(reg.lo.x, p.x), min(reg.lo.y, p.y)}
			reg.hi = pt2{max(reg.hi.x, p.x), max(reg.hi.y, p.y)}
		}
	}
	reg.member = func(p pt2) bool {
		a, b := p.sub(e.v).dot(tan), p.sub(e.v).dot(in)
		return a > 0 && a < ext && b > 0 && b < width
	}
	v2 := e.v.add(e.e2away.scale(e.e2Len))
	at := func(p, q pt2) bool { return p.dist(q) <= 1e-9*max(1, q.len()) }
	for i, a := range poly {
		b := poly[(i+1)%len(poly)]
		if (at(a, e.v) && at(b, v2)) || (at(a, v2) && at(b, e.v)) {
			continue
		}
		if reg.stretchIn(a, b) {
			return fmt.Errorf(`%w: another edge of the loop lies in the bevel's run-out past a chamfered run's end`,
				decaderr.ErrUnsupported)
		}
	}
	return nil
}
