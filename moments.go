package decad

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/units"
)

// This file is the mass-property engine of docs/evaluator-design.md §4:
// decad integrating its OWN records. sketch decides topology and
// admissibility; once a region is recorded, its areas and moments are decad's
// job, computed by closed-form boundary integrals (Green's theorem) per
// segment kind — exact for the increment-1 kinds, and ErrUnsupported for the
// free-form kinds until their evaluator increments land.

// Area returns the recorded region's net area — the outer loop minus its
// holes — as a quantity of Kind Area (mm²). It is exact: each boundary
// segment contributes its closed-form Green's-theorem integral in walk order,
// so a hole's clockwise walk subtracts without a special case.
//
// A region whose boundary contains a free-form segment kind (ellipse,
// elliptical arc, conic, spline, closed spline, fit spline, NURBS) is
// [ErrUnsupported] until that kind's evaluator increment lands
// (docs/evaluator-design.md §11) — never approximated.
func (r ProfileRecord) Area() (units.Value, error) {
	ig, err := r.integrals()
	if err != nil {
		return units.Value{}, err
	}
	return units.SquareMillimeters(ig.area), nil
}

// Centroid returns the recorded region's centroid in plane-local (u, v)
// millimetres — the docs/api-design.md §5.2 carve-out — from the region's
// exact first moments. A region whose net area is zero has no centroid and
// is [ErrDegenerate].
func (r ProfileRecord) Centroid() (Point2, error) {
	ig, err := r.integrals()
	if err != nil {
		return Point2{}, err
	}
	if ig.area == 0 {
		return Point2{}, fmt.Errorf(`%w: a region with zero net area has no centroid`, ErrDegenerate)
	}
	return Point2{U: ig.mu / ig.area, V: ig.mv / ig.area}, nil
}

// regionIntegrals accumulates the boundary integrals of one region: the net
// signed area and the first moments ∫u dA, ∫v dA. (The second and mixed
// moments a revolve centroid needs join in increment 2 —
// docs/evaluator-design.md §4/§6.)
type regionIntegrals struct {
	area float64
	mu   float64 // ∫u dA
	mv   float64 // ∫v dA
}

// integrals walks the outer loop and every hole in recorded walk order and
// sums each segment's closed-form contribution. Walk order carries the sign:
// the outer loop is counter-clockwise (positive), holes are clockwise
// (negative), so the sum IS the net region integral.
func (r ProfileRecord) integrals() (regionIntegrals, error) {
	var ig regionIntegrals
	for _, loop := range append([]LoopRecord{r.Outer}, r.Holes...) {
		for _, seg := range loop.Segments {
			if err := ig.add(seg); err != nil {
				return regionIntegrals{}, err
			}
		}
	}
	return ig, nil
}

// add accumulates one segment's boundary-integral contribution, in the
// segment's recorded walk direction (TStart→TEnd, which TStart > TEnd runs
// against the curve's natural sense — the sign falls out of the integral
// limits, no special case).
func (ig *regionIntegrals) add(seg CurveSegment) error {
	switch seg := seg.(type) {
	case LineSeg:
		ig.addLine(seg)
		return nil
	case CircleSeg:
		r, err := seg.Radius.In(units.Millimeter)
		if err != nil {
			return fmt.Errorf(`decad: a circle segment's radius is not a length: %w`, err)
		}
		// The arrangement's normalized t is the angle 2π·t from +u
		// (geom.BoundaryEdge); the recorded range order is the walk.
		ig.addCircular(seg.Center, r, 2*math.Pi*seg.TStart, 2*math.Pi*seg.TEnd)
		return nil
	case ArcSeg:
		// geom.Arc's derived readings, computed on the record's own pinned
		// points: radius from Center→Start, angles about the center, the
		// sweep CCW-wrapped into (0, 2π].
		radius := math.Hypot(seg.Start.U-seg.Center.U, seg.Start.V-seg.Center.V)
		a0 := math.Atan2(seg.Start.V-seg.Center.V, seg.Start.U-seg.Center.U)
		a1 := math.Atan2(seg.End.V-seg.Center.V, seg.End.U-seg.Center.U)
		sweep := math.Mod(a1-a0, 2*math.Pi)
		if sweep <= 0 {
			sweep += 2 * math.Pi
		}
		// normalized t maps to angle = a0 + t·sweep; the range order is the walk.
		ig.addCircular(seg.Center, radius, a0+seg.TStart*sweep, a0+seg.TEnd*sweep)
		return nil
	default:
		return fmt.Errorf(`%w: mass properties for a %T land with its evaluator increment`, ErrUnsupported, seg)
	}
}

// addLine accumulates the straight chord from the walk's start point to its
// end point. The recorded range picks the walked piece of the entity's own
// Start→End parameterization.
func (ig *regionIntegrals) addLine(seg LineSeg) {
	u0, v0 := lerp2(seg.Start, seg.End, seg.TStart)
	u1, v1 := lerp2(seg.Start, seg.End, seg.TEnd)
	// A     = ½ ∮ (u dv − v du)
	// ∫u dA = ½ ∮ u² dv
	// ∫v dA = −½ ∮ v² du
	ig.area += 0.5 * (u0*v1 - u1*v0)
	ig.mu += (v1 - v0) * (u0*u0 + u0*u1 + u1*u1) / 6
	ig.mv += -(u1 - u0) * (v0*v0 + v0*v1 + v1*v1) / 6
}

// addCircular accumulates a circular path about center c with radius r, from
// angle th0 to th1 in the walk direction (th1 < th0 walks clockwise). The
// antiderivatives are exact, so a whole period, a fragment, and a reversed
// walk are all the same formula.
func (ig *regionIntegrals) addCircular(c Point2, r, th0, th1 float64) {
	sin0, cos0 := math.Sincos(th0)
	sin1, cos1 := math.Sincos(th1)
	dth := th1 - th0

	// A = ½ ∫ (u v′ − v u′) dθ = ½ [r²·θ + c_u·r·sin θ + c_v·r·cos θ]
	ig.area += 0.5 * (r*r*dth + c.U*r*(sin1-sin0) - c.V*r*(cos1-cos0))

	// ∫u dA = ½ ∮ u² dv, with u = c_u + r cos θ, dv = r cos θ dθ:
	//   ½ r [c_u²·sin θ + 2 c_u r (θ/2 + sin 2θ/4) + r²(sin θ − sin³θ/3)]
	intCos := sin1 - sin0
	intCos2 := dth/2 + (math.Sin(2*th1)-math.Sin(2*th0))/4
	intCos3 := (sin1 - sin1*sin1*sin1/3) - (sin0 - sin0*sin0*sin0/3)
	ig.mu += 0.5 * r * (c.U*c.U*intCos + 2*c.U*r*intCos2 + r*r*intCos3)

	// ∫v dA = −½ ∮ v² du, with v = c_v + r sin θ, du = −r sin θ dθ:
	//   ½ r [c_v²·(−cos θ)′... i.e. ½ r ∫ v² sin θ dθ]
	intSin := cos0 - cos1
	intSin2 := dth/2 - (math.Sin(2*th1)-math.Sin(2*th0))/4
	intSin3 := (cos0 - cos0*cos0*cos0/3) - (cos1 - cos1*cos1*cos1/3)
	ig.mv += 0.5 * r * (c.V*c.V*intSin + 2*c.V*r*intSin2 + r*r*intSin3)
}

// lerp2 returns the point at parameter t on the segment start→end.
func lerp2(start, end Point2, t float64) (float64, float64) {
	return start.U + t*(end.U-start.U), start.V + t*(end.V-start.V)
}
