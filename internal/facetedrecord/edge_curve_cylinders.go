package facetedrecord

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
)

// parallelCylindersRim returns the generator in which two cylinders with
// parallel axes meet (agent-docs/faceted-source-retention-design.md §10.1 E10).
// The axes are parallel (|a1 × a2| ≤ edgeParallel) at distance s with
// |R1 − R2| < s < R1 + R2 within slack, so the cross-section's two circles meet
// in two points and each is a generator. Either passes if the falsifier E6
// accepts it; a pair that fails it declines and keeps its chord chain.
func parallelCylindersRim(sa, sb surfacegeom.Surface, chain []r3.Vec, bounds []float64) (surfacegeom.Curve, bool) {
	c1, ok1 := sa.(surfacegeom.Cylinder)
	c2, ok2 := sb.(surfacegeom.Cylinder)
	if !ok1 || !ok2 || c1.Axis.Cross(c2.Axis).Len() > edgeParallel {
		return nil, false
	}
	r1, ok1 := millimeters(c1.Radius)
	r2, ok2 := millimeters(c2.Radius)
	if !ok1 || !ok2 {
		return nil, false
	}
	scale := 1.0
	for _, v := range chain {
		scale = max(scale, math.Abs(v.X), math.Abs(v.Y), math.Abs(v.Z))
	}
	slack := 64 * edgeEps * scale
	axis := c1.Axis
	rel := c2.Origin.Sub(c1.Origin)
	rel = rel.Sub(axis.Scale(rel.Dot(axis)))
	s := rel.Len()
	if !(s > math.Abs(r1-r2)+slack) || !(s < r1+r2-slack) {
		return nil, false
	}
	toward := rel.Scale(1 / s)
	side := axis.Cross(toward)
	x := (s*s + r1*r1 - r2*r2) / (2 * s)
	half := r1*r1 - x*x
	if half < 0 {
		return nil, false
	}
	h := math.Sqrt(half)
	foot := c1.Origin.Add(toward.Scale(x))
	return acceptLine([]rimLine{{foot.Add(side.Scale(h)), axis}, {foot.Sub(side.Scale(h)), axis}},
		chain, bounds, slack)
}
