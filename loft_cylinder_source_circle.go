package decad

import (
	"fmt"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// loftCylinderSourceCircle certifies where one authenticated Loft tooth meets
// the exact blank circle in the open axial interval (z0, safeTop). Source
// caps, a possible top root collar, and the rest of the tooth's interior are
// outside this record's claim.
type loftCylinderSourceCircle struct {
	z0, z1, safeTop *big.Rat
	radius          *big.Rat
	gapAxis         loftCylinderXY
	connector       [2]loftCylinderSourceConnector
}

type loftCylinderSourceConnector struct {
	// At each cap, the source LineSeg wall is the two triangles through A,D,F.
	a, d, f [2]loftCylinderXY
}

type loftCylinderSourceXYBox struct {
	lo, hi loftCylinderXY
}

// loftCylinderSourceCircleOf rechecks the source claims needed by a later
// radial occupancy query. It refuses when the true root wall may reach the
// cylinder in the admitted open interval, a fitted wall may enter it, either
// true connector lacks one transverse crossing, or no exterior angular gap
// can be certified. The caller must retain the tooth from an audited,
// unplaced loft/cylinder Union or its centered-bore Cut.
func loftCylinderSourceCircleOf(lp loftPayload, radius float64) (loftCylinderSourceCircle, error) {
	refuse := func(reason string) (loftCylinderSourceCircle, error) {
		return loftCylinderSourceCircle{}, fmt.Errorf("%w: %s", ErrUnsupported, reason)
	}
	if radius <= 0 || proofbound.IsNonFinite(radius) || !lp.authenticatedReconstruction ||
		lp.sourceIdentity == nil || lp.surfaceResult || !loftCylinderAxialPlacement(lp.xform) ||
		lp.proof.FacetDeparture < 0 || proofbound.IsNonFinite(lp.proof.FacetDeparture) ||
		len(lp.profile0.Outer.Segments) != 6 || len(lp.profile1.Outer.Segments) != 6 ||
		!loftCylinderSectionKinds(lp) || len(lp.verts)%2 != 0 || len(lp.verts) < 16 ||
		len(lp.segment) != 1 {
		return refuse("source circle lacks an authenticated axial Loft")
	}
	n := len(lp.verts) / 2
	if len(lp.segment[0]) != n || len(lp.tris) != 4*n-4 ||
		lp.frame0.U() != (r3.Vec{X: 1}) || lp.frame0.V() != (r3.Vec{Y: 1}) ||
		lp.frame1.U() != (r3.Vec{X: 1}) || lp.frame1.V() != (r3.Vec{Y: 1}) {
		return refuse("source circle has no paired planar sections")
	}
	z0, z1 := lp.frame0.Origin().Z, lp.frame1.Origin().Z
	if z1 <= z0 || !proofbound.FiniteVec(r3.Vec{Z: z0}) || !proofbound.FiniteVec(r3.Vec{Z: z1}) ||
		lp.frame0.Origin() != (r3.Vec{Z: z0}) || lp.frame1.Origin() != (r3.Vec{Z: z1}) ||
		lp.verts[0].Z != z0 || lp.verts[n].Z != z1 {
		return refuse("source circle has no exact axial interval")
	}
	root := -1
	for j, segment := range lp.segment[0] {
		if segment == 5 && root < 0 {
			root = j
		}
		if root >= 0 && segment != 5 {
			return refuse("source circle has a nonterminal root arc")
		}
	}
	if root < 3 || root >= n-1 {
		return refuse("source circle has no paired root connectors")
	}
	gap, ok := loftCylinderRootRadiusGap(lp, radius)
	if !ok {
		return refuse("source root has no radial gap bound")
	}
	if _, ok := loftCylinderRadialGate(lp, root, radius, lp.proof.FacetDeparture, gap); !ok {
		return refuse("source fitted walls may meet the circle")
	}
	height := proofbound.RatFloatUp(new(big.Rat).Sub(proofarith.FloatRat(z1), proofarith.FloatRat(z0)))
	collar, ok := loftCylinderSourceStrictRootCollar(lp, radius, height)
	if !ok {
		return refuse("source root lacks a strictly interior open interval")
	}
	record := loftCylinderSourceCircle{
		z0: proofarith.FloatRat(z0), z1: proofarith.FloatRat(z1), radius: proofarith.FloatRat(radius),
	}
	record.safeTop = new(big.Rat).Sub(record.z1, proofarith.FloatRat(collar))
	if record.safeTop.Cmp(record.z0) <= 0 {
		return refuse("source root collar covers the axial interval")
	}
	for i, cell := range [2]int{0, root - 1} {
		connector, valid := loftCylinderSourceConnectorOf(lp, cell, root)
		if !valid || !connector.radialMonotone() || !connector.farOutside(record.radius) {
			return refuse("true source connector lacks one transverse circle crossing")
		}
		record.connector[i] = connector
	}
	axis, ok := loftCylinderSourceGapAxis(lp, root)
	if !ok {
		return refuse("source tooth has no exterior gap witness")
	}
	record.gapAxis = axis
	return record, nil
}

// The exact section point applies the dyadic axial placement basis to the
// recorded, untrimmed LineSeg endpoint. The section frames are checked above.
func loftCylinderSourcePoint(lp loftPayload, p Point2) loftCylinderXY {
	basis := lp.xform.Basis()
	c, s := proofarith.FloatRat(basis.EX.X), proofarith.FloatRat(basis.EX.Y)
	u, v := proofarith.FloatRat(p.U), proofarith.FloatRat(p.V)
	return loftCylinderXY{
		x: new(big.Rat).Sub(new(big.Rat).Mul(c, u), new(big.Rat).Mul(s, v)),
		y: new(big.Rat).Add(new(big.Rat).Mul(s, u), new(big.Rat).Mul(c, v)),
	}
}

// The existing root collar is zero when both cap radii are at most R. In
// that arm, convexity makes every interior ruling strict if one cap radius
// is strict. If both equal R, a certified nonzero angle change makes every
// interior ruling strict. Neither argument asserts strictness on a cap.
func loftCylinderSourceStrictRootCollar(lp loftPayload, radius, height float64) (float64, bool) {
	collar, ok := loftCylinderRootCollar(lp, radius, height)
	if !ok || collar > 0 {
		return collar, ok
	}
	basis := lp.xform.Basis()
	c, s := proofarith.FloatRat(basis.EX.X), proofarith.FloatRat(basis.EX.Y)
	scale2 := new(big.Rat).Add(new(big.Rat).Mul(c, c), new(big.Rat).Mul(s, s))
	r2 := new(big.Rat).Mul(proofarith.FloatRat(radius), proofarith.FloatRat(radius))
	var q [2]*big.Rat
	var angles [2][2]proofbound.RatInterval
	for cap, profile := range []profileRecord{lp.profile0, lp.profile1} {
		arc, valid := profile.Outer.Segments[5].(arcSeg)
		if !valid {
			return 0, false
		}
		x, y := proofarith.FloatRat(arc.Start.U), proofarith.FloatRat(arc.Start.V)
		q[cap] = new(big.Rat).Mul(scale2,
			new(big.Rat).Add(new(big.Rat).Mul(x, x), new(big.Rat).Mul(y, y)))
		if q[cap].Cmp(r2) > 0 {
			return 0, false
		}
		for end, p := range []Point2{arc.Start, arc.End} {
			angles[cap][end] = proofbound.Atan2Interval(
				proofarith.FloatRat(p.V), proofarith.FloatRat(p.U), false)
		}
		if angles[cap][1].Hi.Cmp(angles[cap][0].Lo) <= 0 {
			angles[cap][1] = proofbound.IntervalAdd(angles[cap][1], proofbound.TwoPiInterval())
		}
	}
	if q[0].Cmp(r2) < 0 || q[1].Cmp(r2) < 0 {
		return 0, true
	}
	twistSign := 0
	for end := range 2 {
		delta := proofbound.IntervalSub(angles[1][end], angles[0][end])
		sign := 0
		if delta.Lo.Sign() > 0 && delta.Hi.Cmp(proofarith.FloatRat(0.4)) < 0 {
			sign = 1
		} else if delta.Hi.Sign() < 0 && delta.Lo.Cmp(proofarith.FloatRat(-0.4)) > 0 {
			sign = -1
		}
		if sign == 0 || twistSign != 0 && sign != twistSign {
			return 0, false
		}
		twistSign = sign
	}
	return 0, true
}

func loftCylinderSourceConnectorOf(lp loftPayload, cell, root int) (loftCylinderSourceConnector, bool) {
	n := len(lp.verts) / 2
	near, far := 0, 1
	segment := 0
	if cell != 0 {
		near, far = root, root-1
		segment = 4
	}
	if lp.segment[0][cell] != segment {
		return loftCylinderSourceConnector{}, false
	}
	var shared []int
	for _, v := range lp.tris[2*cell] {
		for _, w := range lp.tris[2*cell+1] {
			if v == w {
				shared = append(shared, v)
			}
		}
	}
	if len(shared) != 2 {
		return loftCylinderSourceConnector{}, false
	}
	diagonal := [2]int{-1, -1}
	for _, index := range shared {
		switch index {
		case near, far:
			diagonal[0] = index
		case n + near, n + far:
			diagonal[1] = index - n
		default:
			return loftCylinderSourceConnector{}, false
		}
	}
	if diagonal[0] < 0 || diagonal[1] < 0 {
		return loftCylinderSourceConnector{}, false
	}
	var result loftCylinderSourceConnector
	for cap, profile := range []profileRecord{lp.profile0, lp.profile1} {
		line, ok := profile.Outer.Segments[segment].(lineSeg)
		if !ok || !proofbound.FiniteVec(r3.Vec{X: line.Start.U, Y: line.Start.V}) ||
			!proofbound.FiniteVec(r3.Vec{X: line.End.U, Y: line.End.V}) {
			return loftCylinderSourceConnector{}, false
		}
		points := [2]Point2{line.Start, line.End}
		if cell != 0 {
			points[0], points[1] = points[1], points[0]
		}
		result.a[cap] = loftCylinderSourcePoint(lp, points[0])
		result.f[cap] = loftCylinderSourcePoint(lp, points[1])
		switch diagonal[cap] {
		case near:
			result.d[cap] = result.a[cap]
		case far:
			result.d[cap] = result.f[cap]
		default:
			return loftCylinderSourceConnector{}, false
		}
	}
	return result, true
}

func loftCylinderSourceDot(a, b loftCylinderXY) *big.Rat {
	return new(big.Rat).Add(new(big.Rat).Mul(a.x, b.x), new(big.Rat).Mul(a.y, b.y))
}

func loftCylinderSourceSub(a, b loftCylinderXY) loftCylinderXY {
	return loftCylinderXY{x: new(big.Rat).Sub(a.x, b.x), y: new(big.Rat).Sub(a.y, b.y)}
}

func loftCylinderSourceLerp(a, b loftCylinderXY, t *big.Rat) loftCylinderXY {
	d := loftCylinderSourceSub(b, a)
	return loftCylinderXY{
		x: new(big.Rat).Add(a.x, new(big.Rat).Mul(t, d.x)),
		y: new(big.Rat).Add(a.y, new(big.Rat).Mul(t, d.y)),
	}
}

// Nonnegative quadratic Bernstein coefficients prove that each actual
// two-triangle source slice moves radially outward. An interior crossing is
// transverse because a nonzero segment has strictly increasing squared
// radius after its starting point.
func (c loftCylinderSourceConnector) radialMonotone() bool {
	for _, pair := range [][2][2]loftCylinderXY{{c.a, c.d}, {c.d, c.f}} {
		u0, u1 := pair[0][0], pair[0][1]
		d0 := loftCylinderSourceSub(pair[1][0], u0)
		d1 := loftCylinderSourceSub(pair[1][1], u1)
		b0 := loftCylinderSourceDot(u0, d0)
		b1 := new(big.Rat).Add(loftCylinderSourceDot(u0, d1), loftCylinderSourceDot(u1, d0))
		b2 := loftCylinderSourceDot(u1, d1)
		if b0.Sign() < 0 || b1.Sign() < 0 || b2.Sign() < 0 {
			return false
		}
	}
	return true
}

func loftCylinderSourceRadiusMin2(a, b loftCylinderXY) *big.Rat {
	d := loftCylinderSourceSub(b, a)
	den := loftCylinderSourceDot(d, d)
	if den.Sign() == 0 {
		return loftCylinderSourceDot(a, a)
	}
	t := new(big.Rat).Quo(new(big.Rat).Neg(loftCylinderSourceDot(a, d)), den)
	if t.Sign() < 0 {
		t.SetInt64(0)
	} else if t.Cmp(big.NewRat(1, 1)) > 0 {
		t.SetInt64(1)
	}
	p := loftCylinderSourceLerp(a, b, t)
	return loftCylinderSourceDot(p, p)
}

func (c loftCylinderSourceConnector) farOutside(radius *big.Rat) bool {
	r2 := new(big.Rat).Mul(radius, radius)
	return loftCylinderSourceRadiusMin2(c.f[0], c.f[1]).Cmp(r2) > 0
}

// A positive half-plane containing every matched source-boundary point
// leaves its opposite circle ray outside the bounded tooth solid. This
// witnesses the exterior gap used to choose the occupied arc between the
// two connector crossings.
func loftCylinderSourceGapAxis(lp loftPayload, root int) (loftCylinderXY, bool) {
	n := len(lp.verts) / 2
	axis := loftCylinderXY{x: new(big.Rat), y: new(big.Rat)}
	for _, index := range []int{0, root, n, n + root} {
		p := lp.verts[index]
		axis.x.Add(axis.x, proofarith.FloatRat(p.X))
		axis.y.Add(axis.y, proofarith.FloatRat(p.Y))
	}
	norm2 := loftCylinderSourceDot(axis, axis)
	if norm2.Sign() == 0 {
		return loftCylinderXY{}, false
	}
	delta2 := new(big.Rat).Mul(proofarith.FloatRat(lp.proof.FacetDeparture),
		proofarith.FloatRat(lp.proof.FacetDeparture))
	margin2 := new(big.Rat).Mul(delta2, norm2)
	for _, p := range lp.verts {
		dot := new(big.Rat).Add(new(big.Rat).Mul(axis.x, proofarith.FloatRat(p.X)),
			new(big.Rat).Mul(axis.y, proofarith.FloatRat(p.Y)))
		if dot.Sign() <= 0 || new(big.Rat).Mul(dot, dot).Cmp(margin2) <= 0 {
			return loftCylinderXY{}, false
		}
	}
	return axis, true
}

func (c loftCylinderSourceConnector) point(s, t *big.Rat) loftCylinderXY {
	var lo, hi [2]loftCylinderXY
	u := new(big.Rat).Set(s)
	if s.Cmp(big.NewRat(1, 1)) <= 0 {
		lo, hi = c.a, c.d
	} else {
		lo, hi = c.d, c.f
		u.Sub(u, big.NewRat(1, 1))
	}
	p0 := loftCylinderSourceLerp(lo[0], hi[0], u)
	p1 := loftCylinderSourceLerp(lo[1], hi[1], u)
	return loftCylinderSourceLerp(p0, p1, t)
}

func loftCylinderSourceBoxOf(points ...loftCylinderXY) loftCylinderSourceXYBox {
	box := loftCylinderSourceXYBox{
		lo: loftCylinderXY{x: new(big.Rat).Set(points[0].x), y: new(big.Rat).Set(points[0].y)},
		hi: loftCylinderXY{x: new(big.Rat).Set(points[0].x), y: new(big.Rat).Set(points[0].y)},
	}
	for _, p := range points[1:] {
		if p.x.Cmp(box.lo.x) < 0 {
			box.lo.x.Set(p.x)
		}
		if p.y.Cmp(box.lo.y) < 0 {
			box.lo.y.Set(p.y)
		}
		if p.x.Cmp(box.hi.x) > 0 {
			box.hi.x.Set(p.x)
		}
		if p.y.Cmp(box.hi.y) > 0 {
			box.hi.y.Set(p.y)
		}
	}
	return box
}

func loftCylinderSourceUnionBox(a, b loftCylinderSourceXYBox) loftCylinderSourceXYBox {
	return loftCylinderSourceBoxOf(a.lo, a.hi, b.lo, b.hi)
}

func (c loftCylinderSourceConnector) crossingBox(tLo, tHi, radius2 *big.Rat) loftCylinderSourceXYBox {
	low, high := big.NewRat(0, 1), big.NewRat(2, 1)
	for range 52 {
		mid := new(big.Rat).Quo(new(big.Rat).Add(low, high), big.NewRat(2, 1))
		p0, p1 := c.point(mid, tLo), c.point(mid, tHi)
		q0, q1 := loftCylinderSourceDot(p0, p0), loftCylinderSourceDot(p1, p1)
		if q0.Cmp(radius2) < 0 && q1.Cmp(radius2) < 0 {
			low = mid
			continue
		}
		if loftCylinderSourceRadiusMin2(p0, p1).Cmp(radius2) > 0 {
			high = mid
			continue
		}
		if tLo.Cmp(tHi) == 0 && q0.Cmp(radius2) == 0 {
			low, high = mid, mid
		}
		break
	}
	box := loftCylinderSourceBoxOf(
		c.point(low, tLo), c.point(low, tHi), c.point(high, tLo), c.point(high, tHi))
	if low.Cmp(big.NewRat(1, 1)) < 0 && high.Cmp(big.NewRat(1, 1)) > 0 {
		middle := big.NewRat(1, 1)
		box = loftCylinderSourceUnionBox(box, loftCylinderSourceBoxOf(c.point(middle, tLo), c.point(middle, tHi)))
	}
	return box
}

func loftCylinderSourceProductRange(aLo, aHi, bLo, bHi *big.Rat) (*big.Rat, *big.Rat) {
	products := [4]*big.Rat{
		new(big.Rat).Mul(aLo, bLo), new(big.Rat).Mul(aLo, bHi),
		new(big.Rat).Mul(aHi, bLo), new(big.Rat).Mul(aHi, bHi),
	}
	lo, hi := products[0], products[0]
	for _, p := range products[1:] {
		if p.Cmp(lo) < 0 {
			lo = p
		}
		if p.Cmp(hi) > 0 {
			hi = p
		}
	}
	return lo, hi
}

func loftCylinderSourceCrossLower(a, b loftCylinderSourceXYBox) *big.Rat {
	xyLo, _ := loftCylinderSourceProductRange(a.lo.x, a.hi.x, b.lo.y, b.hi.y)
	_, yxHi := loftCylinderSourceProductRange(a.lo.y, a.hi.y, b.lo.x, b.hi.x)
	return new(big.Rat).Sub(xyLo, yxHi)
}

func loftCylinderSourceDotLower(axis loftCylinderXY, box loftCylinderSourceXYBox) *big.Rat {
	xLo, _ := loftCylinderSourceProductRange(axis.x, axis.x, box.lo.x, box.hi.x)
	yLo, _ := loftCylinderSourceProductRange(axis.y, axis.y, box.lo.y, box.hi.y)
	return new(big.Rat).Add(xLo, yLo)
}

// CrossingBox encloses both true source connector/circle crossings for every
// height in the rational closed interval. A returned pair has positive CCW
// order and lies in the source tooth's positive gap-witness half-plane. A
// wide interval can refuse; the caller may subdivide it with a fixed budget.
// Cap planes are included only as limiting crossing positions. The occupancy
// claim applies to open heights z0 < z < safeTop.
func (r loftCylinderSourceCircle) CrossingBox(zLo, zHi *big.Rat) ([2]loftCylinderSourceXYBox, error) {
	var boxes [2]loftCylinderSourceXYBox
	if zLo == nil || zHi == nil || zLo.Cmp(r.z0) < 0 || zHi.Cmp(r.safeTop) > 0 ||
		zLo.Cmp(zHi) > 0 {
		return boxes, fmt.Errorf("%w: source circle query leaves its strict root interval", ErrUnsupported)
	}
	height := new(big.Rat).Sub(r.z1, r.z0)
	tLo := new(big.Rat).Quo(new(big.Rat).Sub(zLo, r.z0), height)
	tHi := new(big.Rat).Quo(new(big.Rat).Sub(zHi, r.z0), height)
	r2 := new(big.Rat).Mul(r.radius, r.radius)
	for i, connector := range r.connector {
		boxes[i] = connector.crossingBox(tLo, tHi, r2)
		if loftCylinderSourceDotLower(r.gapAxis, boxes[i]).Sign() <= 0 {
			return [2]loftCylinderSourceXYBox{},
				fmt.Errorf("%w: source crossing leaves its certified tooth half-plane", ErrUnsupported)
		}
	}
	if loftCylinderSourceCrossLower(boxes[0], boxes[1]).Sign() <= 0 {
		return [2]loftCylinderSourceXYBox{},
			fmt.Errorf("%w: source crossing interval loses tooth angular order", ErrUnsupported)
	}
	return boxes, nil
}

// ContainsTriangleSector proves that every projected point of an exact
// triangle lies strictly between both source connector/circle crossings at
// every height in [zLo,zHi]. The entire triangle must lie in that axial
// interval. The result concerns angle only: it does not prove radial tooth
// occupancy or clearance from another source wall. Endpoints on z0 or
// safeTop are limiting coordinates; cap planes and the top collar are outside
// the occupancy claim.
func (r loftCylinderSourceCircle) ContainsTriangleSector(zLo, zHi *big.Rat,
	tri [3]proofarith.Xpt) bool {
	boxes, err := r.CrossingBox(zLo, zHi)
	if err != nil {
		return false
	}
	for _, p := range tri {
		if p.X == nil || p.Y == nil || p.Z == nil || p.W == nil || p.W.Sign() <= 0 {
			return false
		}
		z := new(big.Rat).SetFrac(p.Z, p.W)
		if z.Cmp(zLo) < 0 || z.Cmp(zHi) > 0 {
			return false
		}
		xy := loftCylinderXY{
			x: new(big.Rat).SetFrac(p.X, p.W),
			y: new(big.Rat).SetFrac(p.Y, p.W),
		}
		point := loftCylinderSourceBoxOf(xy)
		if loftCylinderSourceCrossLower(boxes[0], point).Sign() <= 0 ||
			loftCylinderSourceCrossLower(point, boxes[1]).Sign() <= 0 {
			return false
		}
	}
	return true
}
