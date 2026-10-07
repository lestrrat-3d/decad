package offset2d

import (
	"errors"
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

var (
	ErrNoDirection    = errors.New("a corner walk has no direction")
	ErrNoIntersection = errors.New("offset carriers do not meet")
)

// Point is a held point in the section plane.
type Point struct{ U, V float64 }

// Join is one corner's offset miter, G1 point, or connector arc.
type Join struct {
	Arc, G1      bool
	VertU, VertV float64
	M, PA, PB    Point
}

// JoinsBudget resolves each coalesced walk's starting corner (modify §7).
// The caller supplies the construction's own tolerance and refusal class.
// The walk keeps material on its left, so s*t times the left unit normal
// places both offset feet. An arc connects a corner of sign −s. A G1 join
// uses the leaving foot because intersecting tangent carriers would solve a
// double root whose held discriminant can stray from zero. A cusp stays on
// the miter row, where missing intersections refuse it.
func JoinsBudget(budget *proofbound.WorkBudget, walks []survey2d.SideWalk, s, t, tol float64) ([]Join, error) {
	n := len(walks)
	joins := make([]Join, n)
	for i := range n {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return nil, err
		}
		prev := walks[(i+n-1)%n]
		cur := walks[i]
		vU, vV := cur.StartU, cur.StartV
		aox, aoy, la := Normalize(prev.TanOutU, prev.TanOutV)
		bix, biy, lb := Normalize(cur.TanInU, cur.TanInV)
		if la == 0 || lb == 0 {
			return nil, ErrNoDirection
		}
		cross := aox*biy - aoy*bix
		pA := Point{U: vU + s*t*(-aoy), V: vV + s*t*aox}
		pB := Point{U: vU + s*t*(-biy), V: vV + s*t*bix}
		if math.Abs(cross) > tol && (cross > 0) == (s < 0) {
			joins[i] = Join{Arc: true, VertU: vU, VertV: vV, PA: pA, PB: pB}
			continue
		}
		if math.Abs(cross) <= tol && aox*bix+aoy*biy > 0 {
			joins[i] = Join{G1: true, VertU: vU, VertV: vV, M: pB}
			continue
		}
		offA := offsetCarrier(prev, s, t, tol)
		offB := offsetCarrier(cur, s, t, tol)
		mx, my, ok := Intersect(offA, offB, vU, vV)
		if !ok {
			return nil, ErrNoIntersection
		}
		joins[i] = Join{VertU: vU, VertV: vV, M: Point{U: mx, V: my}}
	}
	return joins, nil
}

// OffsetRadius returns R − s*inside*t. A counterclockwise walk has material
// inside its circle, and a clockwise walk has material outside. A radius
// collapsed within the construction's tolerance reports ok false.
func OffsetRadius(w survey2d.SideWalk, s, t, tol float64) (float64, bool) {
	inside := 1.0
	if w.Th1 < w.Th0 {
		inside = -1.0
	}
	rr := w.Radius - s*inside*t
	if rr <= tol*math.Max(1, w.Radius) {
		return 0, false
	}
	return rr, true
}

func offsetCarrier(w survey2d.SideWalk, s, t, tol float64) Curve {
	if !w.IsCircular() {
		tx, ty, _ := Normalize(w.TanInU, w.TanInV)
		return Curve{IsLine: true, PX: w.StartU + s*t*(-ty), PY: w.StartV + s*t*tx, DX: tx, DY: ty}
	}
	rr, _ := OffsetRadius(w, s, t, tol)
	return Curve{CX: w.CU, CY: w.CV, Radius: rr}
}

// WalkConsumed reports whether a trimmed line stops advancing or a trimmed
// arc exceeds its source span (modify §7, S11a). A collapsed polygonal hole
// can keep its signed-area sign, so the section audit cannot detect this
// per-walk loss. For an arc, compare the overshoot as a length on the offset
// circle against the walk's coordinate scale. Held feet far from the origin
// can differ from their ideal radial angles by coordinate rounding.
func WalkConsumed(w survey2d.SideWalk, start, end Point, tol float64) bool {
	du, dv := end.U-start.U, end.V-start.V
	if !w.IsCircular() {
		tx, ty, l := Normalize(w.TanInU, w.TanInV)
		if l == 0 {
			return false // Other gates reject a walk with no direction.
		}
		adv := du*tx + dv*ty // Signed advance along the source tangent.
		return adv <= tol*math.Max(1, math.Hypot(du, dv))
	}
	// Feet that swap sides force the arc to take the long way around.
	a0 := math.Atan2(start.V-w.CV, start.U-w.CU)
	a1 := math.Atan2(end.V-w.CV, end.U-w.CU)
	span := a1 - a0
	if w.Th1 > w.Th0 {
		for span < 0 {
			span += 2 * math.Pi
		}
	} else {
		for span > 0 {
			span -= 2 * math.Pi
		}
		span = -span
	}
	overshoot := span - math.Abs(w.Th1-w.Th0)
	rr := math.Hypot(start.U-w.CU, start.V-w.CV)
	return rr*overshoot > tol*math.Max(1, math.Abs(w.CU)+math.Abs(w.CV)+2*w.Radius)
}
