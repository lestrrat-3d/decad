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
		j, err := CornerJoin(walks[(i+n-1)%n], walks[i], s, t, tol)
		if err != nil {
			return nil, err
		}
		joins[i] = j
	}
	return joins, nil
}

// CornerJoin resolves the one corner where prev arrives at cur's start, by
// the rule JoinsBudget applies to every corner of a loop. A caller offsetting
// an open chain reads its interior corners through it.
func CornerJoin(prev, cur survey2d.SideWalk, s, t, tol float64) (Join, error) {
	vU, vV := cur.StartU, cur.StartV
	aox, aoy, la := Normalize(prev.TanOutU, prev.TanOutV)
	bix, biy, lb := Normalize(cur.TanInU, cur.TanInV)
	if la == 0 || lb == 0 {
		return Join{}, ErrNoDirection
	}
	cross := aox*biy - aoy*bix
	pA := Point{U: vU + s*t*(-aoy), V: vV + s*t*aox}
	pB := Point{U: vU + s*t*(-biy), V: vV + s*t*bix}
	if math.Abs(cross) > tol && (cross > 0) == (s < 0) {
		return Join{Arc: true, VertU: vU, VertV: vV, PA: pA, PB: pB}, nil
	}
	if math.Abs(cross) <= tol && aox*bix+aoy*biy > 0 {
		return Join{G1: true, VertU: vU, VertV: vV, M: pB}, nil
	}
	offA := offsetCarrier(prev, s, t, tol)
	offB := offsetCarrier(cur, s, t, tol)
	mx, my, ok := Intersect(offA, offB, vU, vV)
	if !ok {
		return Join{}, ErrNoIntersection
	}
	return Join{VertU: vU, VertV: vV, M: Point{U: mx, V: my}}, nil
}

// MirrorCornerJoin resolves the corner a walk makes with its own mirror image
// across a line, at the walk's endpoint on that line: its end when atEnd,
// where the mirror image leaves, and its start otherwise, where the mirror
// image arrives. axis is that line, a point and a unit direction. This is the
// corner a revolve meridian's symmetric union shows where an on-axis walk was
// cancelled (modify-reach §9.3), and the join is that corner's join cut back
// to the walk's own side of the line, so the mirror image is never built.
//
// The mirror tangent is the walk's own reflected one, used only to classify
// the corner by JoinsBudget's rule. The join's points come from the walk and
// the line alone. A miter is the walk's offset carrier met with the line,
// where the mirror carrier meets it too. A G1 join is the walk's own offset
// foot, taken as the line point t from the corner. An arc runs from the walk's offset foot to the line point t from the
// corner along the line, which is that arc's midpoint: at the end the arc runs
// PA (the foot) to PB (the line point), and at the start PA (the line point) to
// PB (the foot), in the same rotational sense JoinsBudget's connector takes.
func MirrorCornerJoin(w survey2d.SideWalk, atEnd bool, axis Curve, s, t, tol float64) (Join, error) {
	dx, dy, ld := Normalize(axis.DX, axis.DY)
	if !axis.IsLine || ld == 0 {
		return Join{}, ErrNoDirection
	}
	var vU, vV, ax, ay, bx, by float64
	var la float64
	if atEnd {
		vU, vV = w.EndU, w.EndV
		ax, ay, la = Normalize(w.TanOutU, w.TanOutV)
		k := ax*dx + ay*dy
		bx, by = ax-2*k*dx, ay-2*k*dy
	} else {
		vU, vV = w.StartU, w.StartV
		bx, by, la = Normalize(w.TanInU, w.TanInV)
		k := bx*dx + by*dy
		ax, ay = bx-2*k*dx, by-2*k*dy
	}
	if la == 0 {
		return Join{}, ErrNoDirection
	}
	// The walk's own unit left normal at the corner, and its offset foot.
	nx, ny := -ay, ax
	if !atEnd {
		nx, ny = -by, bx
	}
	foot := Point{U: vU + s*t*nx, V: vV + s*t*ny}
	// on is the line point t from the corner on the normal's side, stepped
	// along the line's own direction so it stays on the line.
	sign := 1.0
	if nx*dx+ny*dy < 0 {
		sign = -1.0
	}
	on := Point{U: vU + s*t*sign*dx, V: vV + s*t*sign*dy}
	cross := ax*by - ay*bx
	if math.Abs(cross) > tol && (cross > 0) == (s < 0) {
		// The two feet are mirror images, so the arc between them crosses the
		// line on the bisector of the two normals, which is the line itself.
		if atEnd {
			return Join{Arc: true, VertU: vU, VertV: vV, PA: foot, PB: on}, nil
		}
		return Join{Arc: true, VertU: vU, VertV: vV, PA: on, PB: foot}, nil
	}
	if math.Abs(cross) <= tol && ax*bx+ay*by > 0 {
		// The walk meets the line at a right angle, so its offset foot is the
		// line point on. A foot stepped along a float normal (an arc's
		// tangent read through its angle) lands a rounding off the line, on
		// either side of it; on does not.
		return Join{G1: true, VertU: vU, VertV: vV, M: on}, nil
	}
	mx, my, ok := Intersect(offsetCarrier(w, s, t, tol), axis, vU, vV)
	if !ok {
		return Join{}, ErrNoIntersection
	}
	return Join{VertU: vU, VertV: vV, M: Point{U: mx, V: my}}, nil
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
