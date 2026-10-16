package smoothloft

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// HomotheticSection retains only the authenticated base walk and quadratic
// scale of an original three-section whole-line loft. Its data is private so
// callers cannot change the source after the held-mesh proof is established.
type HomotheticSection struct {
	origin r3.Vec
	endZ   float64
	points []sectionrecord.Point2
	a, b   *big.Rat
}

func newHomotheticSection(origin r3.Vec, endZ float64, points []sectionrecord.Point2,
	a, b *big.Rat) *HomotheticSection {
	return &HomotheticSection{
		origin: origin, endZ: endZ,
		points: append([]sectionrecord.Point2(nil), points...),
		a:      new(big.Rat).Set(a), b: new(big.Rat).Set(b),
	}
}

// Origin is the common source-frame origin of the three authenticated inputs.
func (s *HomotheticSection) Origin() r3.Vec { return s.origin }

// At returns a full-line profile only when every scaled source coordinate is
// exactly representable at a level strictly between the two caps.
func (s *HomotheticSection) At(z float64) (momentinput.Profile, bool) {
	if s == nil || len(s.points) < 3 || math.IsNaN(z) || math.IsInf(z, 0) {
		return momentinput.Profile{}, false
	}
	level, start, end := rf(z), rf(s.origin.Z), rf(s.endZ)
	if level.Cmp(start) <= 0 || level.Cmp(end) >= 0 {
		return momentinput.Profile{}, false
	}
	t := div(sub(level, start), sub(end, start))
	q := quadraticAt(s.a, s.b, t)
	if q.Sign() <= 0 {
		return momentinput.Profile{}, false
	}
	points := make([]sectionrecord.Point2, len(s.points))
	for i, p := range s.points {
		x, exactX := mul(q, rf(p.U)).Float64()
		y, exactY := mul(q, rf(p.V)).Float64()
		if !exactX || !exactY || math.IsInf(x, 0) || math.IsInf(y, 0) {
			return momentinput.Profile{}, false
		}
		points[i] = sectionrecord.Point2{U: x, V: y}
	}
	profile := momentinput.Profile{Outer: sectionrecord.LoopRecord{
		Segments: make([]sectionrecord.CurveSegment, len(points)),
	}}
	for i, start := range points {
		profile.Outer.Segments[i] = sectionrecord.LineSeg{
			Start: start, End: points[(i+1)%len(points)], TStart: 0, TEnd: 1,
		}
	}
	return profile, true
}
