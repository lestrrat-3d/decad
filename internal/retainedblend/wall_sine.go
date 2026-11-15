package retainedblend

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/units"
)

// wallSine is a proven lower bound on the sine of the angle between the true wall
// face fi and end plane g, as a function of how far from g the wall point it
// bounds may lie (reach), and whether the wall is a plane, or nil when no bound
// follows from the wall's tag (agent-docs/retained-edge-blend-design.md §4.3,
// sign reading). The tag must state no positional departure (TagDeparture and
// the axial delta zero): Retention Claim R then puts every true point of the wall
// on it. The tag's normal bound lowers the sine by at most itself, since
// sin(θ − δ) ≥ sin θ − δ.
//
//   - A plane wall meets g at the sine of the angle between their normals, the
//     length of the cross product less its rounding.
//   - A cylinder wall whose axis runs parallel to g meets g, at the point with
//     unit radial direction r, at the sine √(1 − (r·n)²), n the plane's normal.
//     A wall point within reach of g has |r·n| ≤ (h + reach)/R, where h is the
//     axis's distance from g: n·(p − o) = n·(c − o) + t·(n·a) + R·r·n, with t the
//     axial coordinate of p from the tag's origin c, so the axial term vanishes
//     when n·a is zero. The axis is normalized, and read as parallel when
//     |n·a| ≤ 64 ulp; the axial term |n·a|·(scale + |c|), with scale the body's
//     largest coordinate, and the rounding of n·(c − o), 64 ulp·(scale + |c| +
//     |o|), are charged into h.
//   - Any other wall has none: the sine falls to zero somewhere on a cone or
//     sphere, and a free cylinder axis leaves r·n unbounded.
func (w *work) wallSine(fi int, g plane) (sine func(reach float64) float64, planar bool) {
	f := w.faceAt(fi)
	if f == nil || !f.RetainedTag || f.TagDeparture != 0 || !(f.NormalBound >= 0) ||
		(f.HasAxialDelta && f.AxialDelta != 0) {
		return nil, false
	}
	// The true wall's normal may tilt from its tag's by NormalBound radians, which
	// lowers the sine of its angle to g by at most that much.
	tilt := proofbound.AbsSumUpper(f.NormalBound, 64*ulp)
	rounding := 64 * ulp
	switch sf := f.Surface.(type) {
	case surfacegeom.Plane:
		sin := sf.Frame.N().Cross(g.n).Len() - tilt
		if !(sin > 0) {
			return nil, false
		}
		return func(float64) float64 { return sin }, true
	case surfacegeom.Cylinder:
		radius, err := sf.Radius.In(units.Millimeter)
		axis, ok := sf.Axis.Normalize()
		if err != nil || !(radius > 0) || !ok {
			return nil, false
		}
		along := math.Abs(g.n.Dot(axis))
		if along > rounding {
			return nil, false
		}
		far := proofbound.AbsSumUpper(w.r.scale, sf.Origin.Len())
		h := proofbound.AbsSumUpper(g.n.Dot(sf.Origin.Sub(g.o)),
			proofbound.ProductUpper(rounding, proofbound.AbsSumUpper(far, g.o.Len())),
			proofbound.ProductUpper(along, far))
		return func(reach float64) float64 {
			q := proofbound.DivUpper(proofbound.AbsSumUpper(h, reach), radius)
			if !(q < 1) {
				return 0
			}
			return math.Sqrt(1-q*q) - tilt
		}, false
	}
	return nil, false
}
