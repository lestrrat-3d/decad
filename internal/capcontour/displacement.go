package capcontour

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// Join records the held point of a miter, G1 join, or connector arc.
type Join struct {
	Arc, G1 bool
	VU, VV  float64
	M       sectionrecord.Point2
	PA, PB  sectionrecord.Point2
}

// Displacement bounds every held point of a cap contour against the point
// denoted by its recorded walks and joins over the stated setback's span.
// The caller first checks that each circular wall's offset radius survives
// the construction's float gates.
func Displacement(walks []survey2d.SideWalk, joins []Join, d, dDelta float64) (float64, bool) {
	span, ok := OffsetSpan(d, dDelta)
	if !ok {
		return 0, false
	}
	delta := 0.0
	for _, w := range walks {
		if !w.IsCircular() {
			continue
		}
		inside := 1.0
		if w.Th1 < w.Th0 {
			inside = -1
		}
		held := w.Radius - inside*d
		exact, ok := ExactOffsetRadiusOver(w, span)
		if !ok {
			return 0, false
		}
		delta = math.Max(delta, proofbound.IntervalFloatError(exact, held))
	}
	n := len(walks)
	for i, j := range joins {
		if n == 0 {
			break
		}
		prev, cur := walks[(i+n-1)%n], walks[i]
		if j.Arc || j.G1 {
			a, okA := joinFoot(prev, true, span)
			b, okB := joinFoot(cur, false, span)
			if !okA || !okB {
				return 0, false
			}
			if j.Arc {
				delta = math.Max(delta, math.Max(a.Reach(j.PA.U, j.PA.V), b.Reach(j.PB.U, j.PB.V)))
				continue
			}
			// A G1 join uses the hull of the two shared-normal feet.
			delta = math.Max(delta, Union(a, b).Reach(j.M.U, j.M.V))
			continue
		}
		ca, okA := CarrierOver(prev, span)
		cb, okB := CarrierOver(cur, span)
		if !okA || !okB {
			return 0, false
		}
		cands, ok := Intersect(ca, cb)
		if !ok {
			return 0, false
		}
		enc, ok := Nearest(cands, j.VU, j.VV)
		if !ok {
			return 0, false
		}
		delta = math.Max(delta, enc.Reach(j.M.U, j.M.V))
	}
	if proofbound.IsNonFinite(delta) {
		return 0, false
	}
	return delta, true
}

// joinFoot encloses one wall's offset foot at a corner over span: the end of
// the wall when atEnd, else its start. The foot stands on the endpoint the
// wall's own end bound encloses and steps along the normal walkTangentEnclosure
// derives from the recorded data: for a line, the difference of its two
// enclosed endpoints; for a circle, the radius from its recorded centre to that
// enclosed endpoint. It never reads the held tangent or the join's held corner.
// A line's held tangent is the endpoint difference rounded to float64, and a
// circular wall's is a math.Sincos at a computed angle that its walk states no
// bound for (TanInBound and TanOutBound are +Inf there).
func joinFoot(w survey2d.SideWalk, atEnd bool, span proofbound.RatInterval) (Point, bool) {
	u, v, bound := w.StartU, w.StartV, w.StartBound
	if atEnd {
		u, v, bound = w.EndU, w.EndV, w.EndBound
	}
	corner, ok := WalkPointEnclosure(u, v, bound)
	if !ok {
		return Point{}, false
	}
	return OffsetFootEnclosure(corner, w, atEnd, span)
}
