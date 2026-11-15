package retainedblend

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/units"
)

// wallSine is a proven lower bound on the sine of the angle between the true wall
// face fi and end plane g, as a function of how far from the wall's held patch
// the bound must hold (reach), or nil when no bound follows from the wall's tag
// (agent-docs/retained-edge-blend-design.md §4.3, sign reading). The tag must
// state no positional departure (TagDeparture and the axial delta zero): Retention
// Claim R then puts every true point of the wall on it. The tag's normal bound
// lowers the sine by at most itself, since sin(θ − δ) ≥ sin θ − δ.
//
//   - A plane wall meets g at the sine of the angle between their normals, the
//     length of the cross product less its rounding.
//   - A cylinder wall whose axis runs parallel to g meets g, at the point with
//     unit radial direction r, at the sine √(1 − (r·n)²), n the plane's normal.
//     A point of the wall within reach of g has |r·n| ≤ (h + reach)/R, where h is
//     the axis's distance from g: n·(p − o) = n·(c − o) + R·r·n, and the axial
//     term vanishes when n·a is zero. The axis is read as parallel when
//     |n·a| ≤ 64 ulp, and the term |n·a|·scale, with scale the largest coordinate,
//     is charged into h.
//   - Any other wall has none: the sine falls to zero somewhere on a cone or
//     sphere, and a free cylinder axis leaves r·n unbounded.
func (w *work) wallSine(fi int, g plane) func(reach float64) float64 {
	f := w.faceAt(fi)
	if f == nil || !f.RetainedTag || f.TagDeparture != 0 || !(f.NormalBound >= 0) ||
		(f.HasAxialDelta && f.AxialDelta != 0) {
		return nil
	}
	// The true wall's normal may tilt from its tag's by NormalBound radians, which
	// lowers the sine of its angle to g by at most that much.
	tilt := proofbound.AbsSumUpper(f.NormalBound, 64*ulp)
	rounding := 64 * ulp
	switch sf := f.Surface.(type) {
	case surfacegeom.Plane:
		sin := sf.Frame.N().Cross(g.n).Len() - tilt
		if !(sin > 0) {
			return nil
		}
		return func(float64) float64 { return sin }
	case surfacegeom.Cylinder:
		radius, err := sf.Radius.In(units.Millimeter)
		if err != nil || !(radius > 0) {
			return nil
		}
		axis := sf.Axis
		if math.Abs(g.n.Dot(axis)) > rounding {
			return nil
		}
		h := proofbound.AbsSumUpper(g.n.Dot(sf.Origin.Sub(g.o)), rounding*w.r.scale)
		return func(reach float64) float64 {
			q := proofbound.DivUpper(proofbound.AbsSumUpper(h, reach), radius)
			if !(q < 1) {
				return 0
			}
			return math.Sqrt(1-q*q) - tilt
		}
	}
	return nil
}
