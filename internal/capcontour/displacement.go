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
			a, okA := OffsetFootOver(j.VU, j.VV, prev.TanOutU, prev.TanOutV, span)
			b, okB := OffsetFootOver(j.VU, j.VV, cur.TanInU, cur.TanInV, span)
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
