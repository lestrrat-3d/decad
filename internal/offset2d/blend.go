package offset2d

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/units"
)

// Blend is one corner's feet, cutbacks and connector for a section rewrite.
type Blend struct {
	FA, FB             sectionrecord.Point2
	CutbackA, CutbackB float64
	Connector          sectionrecord.CurveSegment
}

// carrier is one side of a corner as an offsettable curve: a line through the
// corner with unit travel tangent, or a circle with a signed material side
// (insideSign +1 when the material is inside the circle — a CCW walk).
type carrier struct {
	isLine     bool
	px, py     float64 // a point on the line (the corner)
	tx, ty     float64 // unit travel tangent
	cx, cy     float64 // circle center
	radius     float64 // circle radius
	insideSign float64 // +1 material inside, −1 outside
}

// offCurve is a carrier's material-relative offset: a line (point + unit dir)
// or a concentric circle.
type offCurve struct {
	isLine     bool
	px, py     float64
	dx, dy     float64
	cx, cy, rr float64
}

// carrierOf builds the carrier of a walk at a corner: (tx, ty) is the walk's
// unit travel tangent there.
func carrierOf(w survey2d.SideWalk, tx, ty float64) carrier {
	if !w.IsCircular() {
		return carrier{isLine: true, px: w.StartU, py: w.StartV, tx: tx, ty: ty}
	}
	inside := 1.0
	if w.Th1 < w.Th0 { // a clockwise walk has its material outside the circle
		inside = -1.0
	}
	return carrier{cx: w.CU, cy: w.CV, radius: w.Radius, insideSign: inside}
}

// offsetOf offsets a carrier by r, signed by offsetSign (+1 a convex corner
// offsets INTO the material, −1 a concave corner offsets away). A circular
// carrier whose offset radius is non-positive has no blend of that radius: S5.
func offsetOf(c carrier, offsetSign, r, tol float64) (offCurve, error) {
	if c.isLine {
		// The left unit normal points into the material for a walk with the
		// material on its left; the offset shifts the line that way for a
		// convex corner and the other way for a concave one.
		nlx, nly := -c.ty, c.tx
		s := offsetSign * r
		return offCurve{isLine: true, px: c.px + s*nlx, py: c.py + s*nly, dx: c.tx, dy: c.ty}, nil
	}
	rr := c.radius - offsetSign*c.insideSign*r
	// The gate accepts a fillet exactly when the offset radius rr clears
	// tol (1e-9 mm). In the ordinary case, where the offset shrinks the
	// wall (offsetSign*insideSign = +1), that means the fillet radius must sit
	// strictly below the wall radius less the tolerance (c.radius − tol).
	// When that boundary is at or below zero the wall itself is within the
	// evaluator's tolerance, so NO positive fillet fits — the remedy is a larger
	// wall, not a smaller radius.
	if rr <= tol {
		bound := c.radius - tol
		if bound <= 0 {
			return offCurve{}, fmt.Errorf(`%w: the circular wall radius %s is at or below the evaluator's tolerance, so no fillet of radius %s fits; enlarge the wall`, decaderr.ErrDegenerate, units.Millimeters(c.radius), units.Millimeters(r))
		}
		return offCurve{}, fmt.Errorf(`%w: no fillet of radius %s fits a circular wall of radius %s; the fillet radius must be a positive value strictly below %s`, decaderr.ErrDegenerate, units.Millimeters(r), units.Millimeters(c.radius), units.Millimeters(bound))
	}
	return offCurve{cx: c.cx, cy: c.cy, rr: rr}, nil
}

// Fillet builds a corner's blend from its two carriers (§6): S4 rejects
// a smooth or cusped corner, the two material-side offsets are intersected for
// the center nearest the corner (S5 when they never meet), and the tangent
// feet, cutbacks and arc sense follow.
func Fillet(walks []survey2d.SideWalk, ci int, r, tol float64) (*Blend, error) {
	n := len(walks)
	arrive := walks[(ci+n-1)%n] // walk A, arriving at the corner
	leave := walks[ci]          // walk B, leaving the corner
	px, py := leave.StartU, leave.StartV

	// Unit travel tangents at the corner: A's outgoing, B's incoming.
	ax, ay, la := Normalize(arrive.TanOutU, arrive.TanOutV)
	bx, by, lb := Normalize(leave.TanInU, leave.TanInV)
	if la == 0 || lb == 0 {
		return nil, fmt.Errorf(`%w: a corner walk has no direction`, decaderr.ErrDegenerate)
	}
	// S4: a smooth (tangent) or cusped (anti-tangent) corner is no corner.
	cross := ax*by - ay*bx
	if math.Abs(cross) <= tol {
		return nil, fmt.Errorf(`%w: the two walls meet smoothly — there is no corner to round`, decaderr.ErrDegenerate)
	}
	offsetSign := 1.0 // convex: the walk turns left (material wedge < π)
	if cross < 0 {
		offsetSign = -1.0 // concave: fill material in
	}

	carA := carrierOf(arrive, ax, ay)
	carB := carrierOf(leave, bx, by)
	offA, err := offsetOf(carA, offsetSign, r, tol)
	if err != nil {
		return nil, err
	}
	offB, err := offsetOf(carB, offsetSign, r, tol)
	if err != nil {
		return nil, err
	}
	ox, oy, err := intersectOffsets(offA, offB, px, py)
	if err != nil {
		return nil, err
	}

	fax, fay := footOn(carA, ox, oy)
	fbx, fby := footOn(carB, ox, oy)
	cutA := cutbackOn(carA, px, py, fax, fay)
	cutB := cutbackOn(carB, px, py, fbx, fby)

	// The arc's walk sense: at fA the boundary continues in A's travel
	// direction, so the CCW tangent there (rotate(fA−O, +90°)) agrees with tA
	// exactly when the arc is a CCW walk.
	rx, ry := -(fay - oy), fax-ox
	ccw := rx*ax+ry*ay > 0

	fA := sectionrecord.Point2{U: fax, V: fay}
	fB := sectionrecord.Point2{U: fbx, V: fby}
	return &Blend{
		FA:        fA,
		FB:        fB,
		CutbackA:  cutA,
		CutbackB:  cutB,
		Connector: ArcSegment(sectionrecord.Point2{U: ox, V: oy}, fA, fB, ccw),
	}, nil
}

// intersectOffsets intersects two offset curves and returns the root nearest
// the corner (px, py). No intersection is S5 — no blend of that radius exists.
func intersectOffsets(a, b offCurve, px, py float64) (float64, float64, error) {
	toCurve := func(c offCurve) Curve {
		return Curve{
			IsLine: c.isLine, PX: c.px, PY: c.py, DX: c.dx, DY: c.dy,
			CX: c.cx, CY: c.cy, Radius: c.rr,
		}
	}
	x, y, ok := Intersect(toCurve(a), toCurve(b), px, py)
	if !ok {
		return 0, 0, fmt.Errorf(`%w: no fillet of that radius fits this corner; try a smaller radius`, decaderr.ErrDegenerate)
	}
	return x, y, nil
}

// footOn returns the tangent foot of the center on a carrier: the perpendicular
// projection onto a line, the nearest circle point along the center ray for a
// circle.
func footOn(c carrier, ox, oy float64) (float64, float64) {
	if c.isLine {
		s := (ox-c.px)*c.tx + (oy-c.py)*c.ty
		return c.px + s*c.tx, c.py + s*c.ty
	}
	ux, uy, l := Normalize(ox-c.cx, oy-c.cy)
	if l == 0 {
		return c.cx + c.radius, c.cy
	}
	return c.cx + c.radius*ux, c.cy + c.radius*uy
}

// cutbackOn is the arc length a carrier loses from the corner to its foot: a
// chord on a line, an arc on a circle.
func cutbackOn(c carrier, px, py, fx, fy float64) float64 {
	if c.isLine {
		return math.Hypot(fx-px, fy-py)
	}
	ap := math.Atan2(py-c.cy, px-c.cx)
	af := math.Atan2(fy-c.cy, fx-c.cx)
	d := math.Mod(af-ap, 2*math.Pi)
	if d > math.Pi {
		d -= 2 * math.Pi
	}
	if d < -math.Pi {
		d += 2 * math.Pi
	}
	return c.radius * math.Abs(d)
}

// RewriteLoop rebuilds one loop's segments with its corners blended: each walk
// is trimmed to the feet its two ends' blends pin, and each blend's connector —
// a fillet's tangent arc or a chamfer's chord — is inserted between the walls it
// joins.
func RewriteLoop(budget *proofbound.WorkBudget, walks []survey2d.SideWalk, blends map[int]*Blend) ([]sectionrecord.CurveSegment, map[int]struct{}, error) {
	n := len(walks)
	var segs []sectionrecord.CurveSegment
	connectors := map[int]struct{}{}
	for i := range n {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return nil, nil, err
		}
		w := walks[i]
		startU, startV := w.StartU, w.StartV
		if cb := blends[i]; cb != nil { // corner i trims this walk's start
			startU, startV = cb.FB.U, cb.FB.V
		}
		endU, endV := w.EndU, w.EndV
		if cb := blends[(i+1)%n]; cb != nil { // corner i+1 trims this walk's end
			endU, endV = cb.FA.U, cb.FA.V
		}
		segs = append(segs, OriginalSegment(w, startU, startV, endU, endV))
		// A blend without a connector only moves this walk's end: the brep
		// route's trim of a face that holds a blended edge as a segment
		// (docs/brep-modify-design.md §5.3 step 4).
		if cb := blends[(i+1)%n]; cb != nil && cb.Connector != nil {
			segs = append(segs, cb.Connector)
			connectors[len(segs)-1] = struct{}{}
		}
	}
	return segs, connectors, nil
}

// Chamfer builds a corner's bevel from its two walks (§7): S4 rejects a
// smooth or cusped corner, then each walk is set back its own arc length from
// the corner — dA back along the arriving walk to foot fA, dB forward along
// the leaving walk to foot fB, equal for an equal-distance chamfer and the
// two distances of an asymmetric one (docs/modify-reach-design.md §6) — and
// the two feet are joined by a chord. The bevel is a plane, so the connector
// is a LineSeg (fA→fB), which continues the loop's own walk sense; there is
// no offset carrier and no S5, because a chord exists between any two
// distinct feet. IsConvex is not an input: a convex corner's
// chord cuts material away, a concave corner's fills it in, and both are the
// same construction (§7). An over-large setback is left to the §5 S6 audit,
// never clipped here.
func Chamfer(walks []survey2d.SideWalk, ci int, dA, dB, tol float64) (*Blend, error) {
	n := len(walks)
	arrive := walks[(ci+n-1)%n] // walk A, arriving at the corner
	leave := walks[ci]          // walk B, leaving the corner

	ax, ay, la := Normalize(arrive.TanOutU, arrive.TanOutV)
	bx, by, lb := Normalize(leave.TanInU, leave.TanInV)
	if la == 0 || lb == 0 {
		return nil, fmt.Errorf(`%w: a corner walk has no direction`, decaderr.ErrDegenerate)
	}
	// S4: a smooth (tangent) or cusped (anti-tangent) corner is no corner.
	if math.Abs(ax*by-ay*bx) <= tol {
		return nil, fmt.Errorf(`%w: the two walls meet smoothly — there is no corner to bevel`, decaderr.ErrDegenerate)
	}

	fA := setbackFoot(arrive, dA, true) // dA back from the arriving walk's end
	fB := setbackFoot(leave, dB, false) // dB forward from the leaving walk's start

	return &Blend{
		FA:        fA,
		FB:        fB,
		CutbackA:  dA,
		CutbackB:  dB,
		Connector: sectionrecord.LineSeg{Start: fA, End: fB, TStart: 0, TEnd: 1},
	}, nil
}

// setbackFoot is the point an arc length d from a corner along a walk, the
// setback §7 measures along the boundary curve. When back is true the corner is
// the walk's END and the step runs backwards along it (the arriving walk); else
// the corner is the walk's START and the step runs forwards (the leaving walk). A
// line steps by d along its unit travel tangent; an arc steps by the angle d/R
// in the walk's own turn sense — CCW (th1 > th0) increases the bearing, CW
// decreases it — so the foot lands on the arc itself, exactly.
func setbackFoot(w survey2d.SideWalk, d float64, back bool) sectionrecord.Point2 {
	if !w.IsCircular() {
		if back {
			ux, uy, _ := Normalize(w.TanOutU, w.TanOutV)
			return sectionrecord.Point2{U: w.EndU - d*ux, V: w.EndV - d*uy}
		}
		ux, uy, _ := Normalize(w.TanInU, w.TanInV)
		return sectionrecord.Point2{U: w.StartU + d*ux, V: w.StartV + d*uy}
	}
	dtheta := d / w.Radius
	sign := 1.0 // a CCW walk (th1 > th0) increases the bearing along travel
	if w.Th1 < w.Th0 {
		sign = -1.0
	}
	th := w.Th0 + sign*dtheta // forward from the start
	if back {
		th = w.Th1 - sign*dtheta // backward from the end
	}
	return sectionrecord.Point2{U: w.CU + w.Radius*math.Cos(th), V: w.CV + w.Radius*math.Sin(th)}
}

// OriginalSegment records a source walk trimmed to two section points.
func OriginalSegment(w survey2d.SideWalk, sU, sV, eU, eV float64) sectionrecord.CurveSegment {
	if !w.IsCircular() {
		return sectionrecord.LineSeg{
			Start: sectionrecord.Point2{U: sU, V: sV}, End: sectionrecord.Point2{U: eU, V: eV},
			TStart: 0, TEnd: 1,
		}
	}
	return ArcSegment(sectionrecord.Point2{U: w.CU, V: w.CV},
		sectionrecord.Point2{U: sU, V: sV}, sectionrecord.Point2{U: eU, V: eV}, w.Th1 > w.Th0)
}
