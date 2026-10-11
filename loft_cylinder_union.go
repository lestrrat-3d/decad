package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// A cylinder joined to a matching axial Loft keeps the original operands.
// Subsequent placements of that same Loft rebuild the common cylinder ring.
type loftCylinderJoinRecord struct {
	blank         *Body
	teeth         []*Body
	capOutlines   [2][]int
	capGroups     [2]int
	minSideRadius float64
	radius        float64
	z0, z1        float64
}

type loftCylinderTooth struct {
	body      *Body
	loft      loftPayload
	rootStart int
	bottom    [2]float64
	top       [2]float64
	bottomIV  [2]proofbound.RatInterval
	topIV     [2]proofbound.RatInterval
	mesh      *Mesh
	vertex    [][]int
	rootGap   float64
	inset     float64
	collar    float64
	wedge     float64
}

func tryLoftCylinderUnion(ctx context.Context, op meshbool.OperationKind,
	d *Document, ref producerID, a, b *Body) (*Body, bool, error) {
	if op != meshbool.OpUnion {
		return nil, false, nil
	}
	var blank *Body
	var teeth []*Body
	if _, ok := a.payload.(prismPayload); ok {
		blank, teeth = a, []*Body{b}
	} else if _, ok := b.payload.(prismPayload); ok {
		blank, teeth = b, []*Body{a}
	} else if fp, ok := a.payload.(facetedPayload); ok && fp.loftCylinderJoin != nil && fp.xform == r3.Identity() {
		blank = fp.loftCylinderJoin.blank
		teeth = append(append([]*Body(nil), fp.loftCylinderJoin.teeth...), b)
	} else if fp, ok := b.payload.(facetedPayload); ok && fp.loftCylinderJoin != nil && fp.xform == r3.Identity() {
		blank = fp.loftCylinderJoin.blank
		teeth = append(append([]*Body(nil), fp.loftCylinderJoin.teeth...), a)
	} else {
		return nil, false, nil
	}
	pp, ok := blank.payload.(prismPayload)
	if !ok || pp.surfaceResult || pp.xform != r3.Identity() ||
		pp.z0Delta != 0 || pp.z1Delta != 0 || pp.sectionDelta != 0 ||
		len(pp.profile.Holes) != 0 || len(pp.profile.Outer.Segments) != 1 ||
		pp.frame.Origin() != (r3.Vec{}) || pp.frame.U() != (r3.Vec{X: 1}) ||
		pp.frame.V() != (r3.Vec{Y: 1}) || pp.z1 <= pp.z0 {
		return nil, false, nil
	}
	circle, ok := pp.profile.Outer.Segments[0].(circleSeg)
	if !ok || circle.Center != (Point2{}) || !circle.CCW {
		return nil, false, nil
	}
	radius := circle.Radius.Base()
	if radius <= 0 || proofbound.IsNonFinite(radius) {
		return nil, false, nil
	}
	parts := make([]loftCylinderTooth, len(teeth))
	var source *loftSourceIdentity
	for i, tooth := range teeth {
		lp, ok := tooth.payload.(loftPayload)
		if !ok || !lp.authenticatedReconstruction || lp.surfaceResult ||
			lp.sourceIdentity == nil || len(lp.profile0.Holes) != 0 ||
			len(lp.profile1.Holes) != 0 || len(lp.profile0.Outer.Segments) != 6 ||
			len(lp.profile1.Outer.Segments) != 6 || len(lp.segment) != 1 ||
			len(lp.verts)%2 != 0 || len(lp.verts) < 16 {
			return nil, false, nil
		}
		if source == nil {
			source = lp.sourceIdentity
		} else if source != lp.sourceIdentity {
			return nil, false, nil
		}
		if !loftCylinderSectionKinds(lp) || lp.frame0.U() != (r3.Vec{X: 1}) ||
			lp.frame0.V() != (r3.Vec{Y: 1}) || lp.frame1.U() != (r3.Vec{X: 1}) ||
			lp.frame1.V() != (r3.Vec{Y: 1}) ||
			lp.frame0.Origin() != (r3.Vec{Z: pp.z0}) ||
			lp.frame1.Origin() != (r3.Vec{Z: pp.z1}) {
			return nil, false, nil
		}
		if !loftCylinderAxialPlacement(lp.xform) {
			return nil, false, nil
		}
		n := len(lp.verts) / 2
		if len(lp.segment[0]) != n || lp.walls != 2*n ||
			len(lp.tris) != 4*n-4 {
			return nil, false, nil
		}
		root := -1
		for j, segment := range lp.segment[0] {
			if segment == 5 && root < 0 {
				root = j
			}
			if (root < 0 && segment == 5) || (root >= 0 && segment != 5) {
				return nil, false, nil
			}
		}
		if root < 2 || root >= n-1 {
			return nil, false, nil
		}
		m, err := tooth.Tessellate(ctx, units.Millimeters(0.1), WithVerification(VerifyAll))
		if err != nil || !m.BoundaryVerified() || !m.VolumeVerified() {
			return nil, false, nil
		}
		parts[i] = loftCylinderTooth{body: tooth, loft: lp, rootStart: root, mesh: m}
		for cap := range 2 {
			offset := cap * n
			p0, p1 := lp.verts[offset], lp.verts[offset+root]
			start, end := math.Atan2(p0.Y, p0.X), math.Atan2(p1.Y, p1.X)
			startIV := proofbound.Atan2Interval(proofarith.FloatRat(p0.Y), proofarith.FloatRat(p0.X), false)
			endIV := proofbound.Atan2Interval(proofarith.FloatRat(p1.Y), proofarith.FloatRat(p1.X), false)
			for end <= start {
				end += 2 * math.Pi
				endIV = proofbound.IntervalAdd(endIV, proofbound.TwoPiInterval())
			}
			if end-start <= 0 || end-start >= math.Pi/2 {
				return nil, false, nil
			}
			if cap == 0 {
				parts[i].bottom = [2]float64{start, end}
				parts[i].bottomIV = [2]proofbound.RatInterval{startIV, endIV}
			} else {
				for start-parts[i].bottom[0] > math.Pi {
					start -= 2 * math.Pi
					end -= 2 * math.Pi
					startIV = proofbound.IntervalSub(startIV, proofbound.TwoPiInterval())
					endIV = proofbound.IntervalSub(endIV, proofbound.TwoPiInterval())
				}
				for start-parts[i].bottom[0] < -math.Pi {
					start += 2 * math.Pi
					end += 2 * math.Pi
					startIV = proofbound.IntervalAdd(startIV, proofbound.TwoPiInterval())
					endIV = proofbound.IntervalAdd(endIV, proofbound.TwoPiInterval())
				}
				parts[i].top = [2]float64{start, end}
				parts[i].topIV = [2]proofbound.RatInterval{startIV, endIV}
			}
		}
		gap, ok := loftCylinderRootRadiusGap(lp, radius)
		if !ok {
			return nil, false, nil
		}
		height := proofbound.RatFloatUp(new(big.Rat).Sub(
			proofarith.FloatRat(pp.z1), proofarith.FloatRat(pp.z0)))
		collar, ok := loftCylinderRootCollar(lp, radius, height)
		if !ok {
			return nil, false, nil
		}
		connectorCollar, ok := loftCylinderConnectorGate(lp, root, radius, height, m.Bound().Base())
		if !ok {
			return nil, false, nil
		}
		collar = max(collar, connectorCollar)
		inset, ok := loftCylinderRadialGate(lp, root, radius, m.Bound().Base(), gap)
		if !ok {
			return nil, false, nil
		}
		wedge, ok := loftCylinderWedgeGate(lp, root, pp.z0, pp.z1)
		if !ok {
			return nil, false, nil
		}
		parts[i].rootGap, parts[i].inset, parts[i].collar, parts[i].wedge = gap, inset, collar, wedge
	}
	if len(parts) == 0 || len(parts) > 32 {
		return nil, false, nil
	}
	sort.Slice(parts, func(i, j int) bool { return parts[i].bottom[0] < parts[j].bottom[0] })
	if !loftCylinderSeparate(parts) {
		return nil, false, nil
	}
	body, err := buildLoftCylinderUnion(ctx, d, ref, blank, pp, radius, parts)
	return body, true, err
}

func loftCylinderSectionKinds(lp loftPayload) bool {
	for _, profile := range []profileRecord{lp.profile0, lp.profile1} {
		segments := profile.Outer.Segments
		if _, ok := segments[0].(lineSeg); !ok {
			return false
		}
		if _, ok := segments[1].(fitSplineSeg); !ok {
			return false
		}
		if _, ok := segments[2].(arcSeg); !ok {
			return false
		}
		if _, ok := segments[3].(fitSplineSeg); !ok {
			return false
		}
		if _, ok := segments[4].(lineSeg); !ok {
			return false
		}
		arc, ok := segments[5].(arcSeg)
		if !ok || arc.Center != (Point2{}) || arc.TStart != 1 || arc.TEnd != 0 {
			return false
		}
		start := segments[0].(lineSeg)
		end := segments[4].(lineSeg)
		if start.TStart != 0 || start.TEnd != 1 || end.TStart != 0 || end.TEnd != 1 ||
			start.Start != arc.Start || end.End != arc.End {
			return false
		}
	}
	return true
}

func loftCylinderAxialPlacement(xform r3.Transform) bool {
	b := xform.Basis()
	return xform.Translation() == (r3.Vec{}) && b.EZ == (r3.Vec{Z: 1}) &&
		b.EX.Z == 0 && b.EY.Z == 0 && b.EX.X == b.EY.Y &&
		b.EX.Y == -b.EY.X
}

func loftCylinderRootRadiusGap(lp loftPayload, radius float64) (float64, bool) {
	gap := 0.0
	basis := lp.xform.Basis()
	c, s := proofarith.FloatRat(basis.EX.X), proofarith.FloatRat(basis.EX.Y)
	scale2 := new(big.Rat).Add(new(big.Rat).Mul(c, c), new(big.Rat).Mul(s, s))
	for _, profile := range []profileRecord{lp.profile0, lp.profile1} {
		arc, ok := profile.Outer.Segments[5].(arcSeg)
		if !ok || proofbound.IsNonFinite(arc.Start.U) || proofbound.IsNonFinite(arc.Start.V) {
			return 0, false
		}
		x, y := proofarith.FloatRat(arc.Start.U), proofarith.FloatRat(arc.Start.V)
		q := new(big.Rat).Mul(scale2,
			new(big.Rat).Add(new(big.Rat).Mul(x, x), new(big.Rat).Mul(y, y)))
		lo, hi := proofbound.RatSqrtDown(q), proofbound.RatSqrtUp(q)
		if lo <= 0 || proofbound.IsNonFinite(hi) {
			return 0, false
		}
		r := proofarith.FloatRat(radius)
		d0 := new(big.Rat).Abs(new(big.Rat).Sub(proofarith.FloatRat(lo), r))
		d1 := new(big.Rat).Abs(new(big.Rat).Sub(proofarith.FloatRat(hi), r))
		if d0.Cmp(d1) < 0 {
			d0 = d1
		}
		gap = max(gap, proofbound.RatFloatUp(d0))
	}
	return gap, gap < 0.01
}

// A matched pair of centered circular arcs has a constant radius at each
// cap and a linearly interpolated angular difference along its parameter.
// At every arc parameter, squared radius along a Loft ruling is convex in
// height. Both endpoint inequalities therefore certify the entire interval
// below the returned top collar. A positive top radius roundoff is allowed
// only when this proof places the root wall inside the blank below it.
func loftCylinderRootCollar(lp loftPayload, radius, height float64) (float64, bool) {
	var q [2]*big.Rat
	var angles [2][2]proofbound.RatInterval
	basis := lp.xform.Basis()
	c, s := proofarith.FloatRat(basis.EX.X), proofarith.FloatRat(basis.EX.Y)
	scale2 := new(big.Rat).Add(new(big.Rat).Mul(c, c), new(big.Rat).Mul(s, s))
	for cap, profile := range []profileRecord{lp.profile0, lp.profile1} {
		arc := profile.Outer.Segments[5].(arcSeg)
		x, y := proofarith.FloatRat(arc.Start.U), proofarith.FloatRat(arc.Start.V)
		q[cap] = new(big.Rat).Mul(scale2,
			new(big.Rat).Add(new(big.Rat).Mul(x, x), new(big.Rat).Mul(y, y)))
		for end, p := range []Point2{arc.Start, arc.End} {
			angles[cap][end] = proofbound.Atan2Interval(
				proofarith.FloatRat(p.V), proofarith.FloatRat(p.U), false)
		}
		if angles[cap][1].Hi.Cmp(angles[cap][0].Lo) <= 0 {
			angles[cap][1] = proofbound.IntervalAdd(angles[cap][1], proofbound.TwoPiInterval())
		}
	}
	r2 := new(big.Rat).Mul(proofarith.FloatRat(radius), proofarith.FloatRat(radius))
	if q[0].Cmp(r2) > 0 {
		return 0, false
	}
	if q[1].Cmp(r2) <= 0 {
		return 0, true
	}
	minTwist := new(big.Rat)
	for end := range 2 {
		delta := proofbound.IntervalSub(angles[1][end], angles[0][end])
		var lower *big.Rat
		switch {
		case delta.Lo.Sign() > 0 && delta.Hi.Cmp(proofarith.FloatRat(0.4)) < 0:
			lower = delta.Lo
		case delta.Hi.Sign() < 0 && delta.Lo.Cmp(proofarith.FloatRat(-0.4)) > 0:
			lower = new(big.Rat).Neg(delta.Hi)
		default:
			return 0, false
		}
		if end == 0 || lower.Cmp(minTwist) < 0 {
			minTwist = lower
		}
	}
	_, cosine, ok := proofbound.RadSinCosInterval(minTwist)
	if !ok {
		return 0, false
	}
	oneMinusCos := new(big.Rat).Sub(big.NewRat(1, 1), cosine.Hi)
	if oneMinusCos.Sign() <= 0 {
		return 0, false
	}
	rProductLo := proofarith.FloatRat(proofbound.RatSqrtDown(
		new(big.Rat).Mul(q[0], q[1])))
	for shift := 52; shift >= 8; shift-- {
		epsilon := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), uint(shift)))
		oneMinus := new(big.Rat).Sub(big.NewRat(1, 1), epsilon)
		weighted := new(big.Rat).Add(new(big.Rat).Mul(epsilon, q[0]),
			new(big.Rat).Mul(oneMinus, q[1]))
		inward := new(big.Rat).Mul(big.NewRat(2, 1), epsilon)
		inward.Mul(inward, oneMinus)
		inward.Mul(inward, rProductLo)
		inward.Mul(inward, oneMinusCos)
		if weighted.Sub(weighted, inward).Cmp(r2) < 0 {
			collar := proofbound.ProductUpper(proofbound.RatFloatUp(epsilon), height)
			return collar, !proofbound.IsNonFinite(collar)
		}
	}
	return 0, false
}

// A triangle's minimum projected radius is computed over its whole convex
// patch using exact dyadic inputs, not from its sampled corner radii.
func loftCylinderTriangleMinRadius(verts []r3.Vec, tri [3]int) float64 {
	type point struct{ x, y *big.Rat }
	var p [3]point
	for i, index := range tri {
		p[i] = point{proofarith.FloatRat(verts[index].X), proofarith.FloatRat(verts[index].Y)}
	}
	best := new(big.Rat)
	for i := range 3 {
		a, b := p[i], p[(i+1)%3]
		dx, dy := new(big.Rat).Sub(b.x, a.x), new(big.Rat).Sub(b.y, a.y)
		den := new(big.Rat).Add(new(big.Rat).Mul(dx, dx), new(big.Rat).Mul(dy, dy))
		t := new(big.Rat)
		if den.Sign() > 0 {
			num := new(big.Rat).Neg(new(big.Rat).Add(
				new(big.Rat).Mul(a.x, dx), new(big.Rat).Mul(a.y, dy)))
			t.Quo(num, den)
			if t.Sign() < 0 {
				t.SetInt64(0)
			} else if t.Cmp(big.NewRat(1, 1)) > 0 {
				t.SetInt64(1)
			}
		}
		qx := new(big.Rat).Add(a.x, new(big.Rat).Mul(t, dx))
		qy := new(big.Rat).Add(a.y, new(big.Rat).Mul(t, dy))
		q := new(big.Rat).Add(new(big.Rat).Mul(qx, qx), new(big.Rat).Mul(qy, qy))
		if i == 0 || q.Cmp(best) < 0 {
			best = q
		}
	}
	// If the origin lies inside the projected triangle, its distance is zero.
	var signs [3]int
	for i := range 3 {
		a, b := p[i], p[(i+1)%3]
		cross := new(big.Rat).Sub(new(big.Rat).Mul(a.x, b.y), new(big.Rat).Mul(a.y, b.x))
		signs[i] = cross.Sign()
	}
	if (signs[0] > 0 && signs[1] > 0 && signs[2] > 0) ||
		(signs[0] < 0 && signs[1] < 0 && signs[2] < 0) {
		return 0
	}
	return proofbound.RatSqrtDown(best)
}

// Whole fitted-flank facets stay outside the circle after the source mesh's
// certified displacement. Root and connector cells may enter only one narrow
// annulus, whose maximum inset is returned for the join proof.
func loftCylinderRadialGate(lp loftPayload, root int, radius, bound, rootGap float64) (float64, bool) {
	n := len(lp.verts) / 2
	if root < 2 || bound < 0 || proofbound.IsNonFinite(bound) {
		return 0, false
	}
	inset := 0.0
	for cap := range 2 {
		offset := cap * n
		for j := range n {
			p := lp.verts[offset+j]
			if !proofbound.FiniteVec(p) {
				return 0, false
			}
			if j == 0 || j >= root {
				x, y := proofarith.FloatRat(p.X), proofarith.FloatRat(p.Y)
				q := new(big.Rat).Add(new(big.Rat).Mul(x, x), new(big.Rat).Mul(y, y))
				radLo, radHi := proofbound.RatSqrtDown(q), proofbound.RatSqrtUp(q)
				allow := proofbound.AbsSumUpper(bound, rootGap)
				lower := new(big.Rat).Sub(proofarith.FloatRat(radius), proofarith.FloatRat(allow))
				upper := new(big.Rat).Add(proofarith.FloatRat(radius), proofarith.FloatRat(allow))
				if proofarith.FloatRat(radLo).Cmp(lower) < 0 || proofarith.FloatRat(radHi).Cmp(upper) > 0 {
					return 0, false
				}
			}
		}
	}
	for j := range n {
		if lp.segment[0][j] == 5 && j < root {
			return 0, false
		}
		for side := range 2 {
			lo := loftCylinderTriangleMinRadius(lp.verts, lp.tris[2*j+side])
			if j > 0 && j < root-1 {
				threshold := new(big.Rat).Add(proofarith.FloatRat(radius), proofarith.FloatRat(bound))
				if proofarith.FloatRat(lo).Cmp(threshold) <= 0 {
					return 0, false
				}
			} else {
				radialGap := new(big.Rat).Sub(proofarith.FloatRat(radius), proofarith.FloatRat(lo))
				if radialGap.Sign() < 0 {
					radialGap.SetInt64(0)
				}
				inset = max(inset, proofbound.AbsSumUpper(
					proofbound.RatFloatUp(radialGap), bound, rootGap))
			}
		}
	}
	return inset, !proofbound.IsNonFinite(inset) && inset < 0.05
}

// The root endpoint rulings delimit a moving angular sector. For one held
// triangle, its signed distance from either ruling is a quadratic in its
// barycentric coordinates. Every Bernstein coefficient bounds that quadratic
// over the triangle, so a negative coefficient gives a certified maximum
// lateral excursion into the cylinder's complementary gap.
func loftCylinderWedgeGate(lp loftPayload, root int, z0, z1 float64) (float64, bool) {
	n := len(lp.verts) / 2
	start := [2]r3.Vec{lp.verts[0], lp.verts[n]}
	end := [2]r3.Vec{lp.verts[root], lp.verts[n+root]}
	minStart := loftCylinderTriangleMinRadius(start[:], [3]int{0, 1, 1})
	minEnd := loftCylinderTriangleMinRadius(end[:], [3]int{0, 1, 1})
	if minStart <= 0 || minEnd <= 0 {
		return 0, false
	}
	cross := func(a, b r3.Vec) *big.Rat {
		return new(big.Rat).Sub(
			new(big.Rat).Mul(proofarith.FloatRat(a.X), proofarith.FloatRat(b.Y)),
			new(big.Rat).Mul(proofarith.FloatRat(a.Y), proofarith.FloatRat(b.X)))
	}
	departure := 0.0
	for _, tri := range lp.tris {
		var p, a, b [3]r3.Vec
		for i, index := range tri {
			p[i] = lp.verts[index]
			cap := -1
			switch p[i].Z {
			case z0:
				cap = 0
			case z1:
				cap = 1
			default:
				return 0, false
			}
			a[i], b[i] = start[cap], end[cap]
		}
		for i := range 3 {
			for j := i; j < 3; j++ {
				left, right := cross(a[i], p[j]), cross(p[i], b[j])
				if i != j {
					left.Add(left, cross(a[j], p[i]))
					left.Quo(left, big.NewRat(2, 1))
					right.Add(right, cross(p[j], b[i]))
					right.Quo(right, big.NewRat(2, 1))
				}
				if left.Sign() < 0 {
					departure = max(departure, proofbound.RatFloatUp(
						new(big.Rat).Quo(new(big.Rat).Neg(left), proofarith.FloatRat(minStart))))
				}
				if right.Sign() < 0 {
					departure = max(departure, proofbound.RatFloatUp(
						new(big.Rat).Quo(new(big.Rat).Neg(right), proofarith.FloatRat(minEnd))))
				}
			}
		}
	}
	return departure, !proofbound.IsNonFinite(departure) && departure < 0.02
}

func loftCylinderSeparate(parts []loftCylinderTooth) bool {
	// Each pair has one through-axis plane with the complete certified Loft
	// surfaces on opposite sides. Testing every held vertex is sufficient for
	// its triangular facets; the mesh bound covers the source surface.
	for i := range parts {
		for j := i + 1; j < len(parts); j++ {
			center := func(part loftCylinderTooth) (float64, float64) {
				angle := (part.bottom[0] + part.bottom[1] +
					part.top[0] + part.top[1]) / 4
				return math.Cos(angle), math.Sin(angle)
			}
			ax, ay := center(parts[i])
			bx, by := center(parts[j])
			nx, ny := bx-ax, by-ay
			if !proofbound.FiniteVec(r3.Vec{X: nx, Y: ny}) {
				return false
			}
			rx, ry := proofarith.FloatRat(nx), proofarith.FloatRat(ny)
			length2 := new(big.Rat).Add(new(big.Rat).Mul(rx, rx),
				new(big.Rat).Mul(ry, ry))
			length := proofbound.RatSqrtUp(length2)
			if length <= 0 || proofbound.IsNonFinite(length) {
				return false
			}
			for k, part := range []loftCylinderTooth{parts[i], parts[j]} {
				margin := proofbound.ProductUpper(length, part.mesh.Bound().Base())
				marginRat := proofarith.FloatRat(margin)
				for _, p := range part.loft.verts {
					dot := new(big.Rat).Add(new(big.Rat).Mul(rx, proofarith.FloatRat(p.X)),
						new(big.Rat).Mul(ry, proofarith.FloatRat(p.Y)))
					if k == 0 && dot.Cmp(new(big.Rat).Neg(marginRat)) >= 0 ||
						k == 1 && dot.Cmp(marginRat) <= 0 {
						return false
					}
				}
			}
		}
	}
	for i := range parts {
		j := (i + 1) % len(parts)
		for cap := range 2 {
			a, b := parts[i].bottom, parts[j].bottom
			if cap == 1 {
				a, b = parts[i].top, parts[j].top
			}
			for b[0] <= a[0] {
				b[0] += 2 * math.Pi
				b[1] += 2 * math.Pi
			}
			if a[1]+0.02 >= b[0] || b[0]-a[1] >= 2*math.Pi {
				return false
			}
		}
	}
	return true
}

func buildLoftCylinderUnion(ctx context.Context, d *Document, ref producerID,
	blank *Body, pp prismPayload, radius float64, parts []loftCylinderTooth) (*Body, error) {
	mesh := &bevelJoinedMesh{}
	blankFaces := blank.Faces()
	if len(blankFaces) != 3 {
		return nil, fmt.Errorf("%w: a circular blank has changed face topology", ErrUnsupported)
	}
	blankGroups := [3]int{-1, -1, -1}
	for _, f := range blankFaces {
		role := f.Origins()[0].Role
		group := -1
		switch role {
		case "side(0,0)":
			group = 0
		case roleCapStart:
			group = 1
		case roleCapEnd:
			group = 2
		}
		if group < 0 || blankGroups[group] >= 0 {
			return nil, fmt.Errorf("%w: a circular blank has changed face roles", ErrUnsupported)
		}
		blankGroups[group] = len(mesh.groups)
		mesh.groups = append(mesh.groups, facetGroup{origins: f.Origins(), planar: group != 0})
	}
	for _, group := range blankGroups {
		if group < 0 {
			return nil, fmt.Errorf("%w: a circular blank lost a source face", ErrUnsupported)
		}
	}
	toothBound, toothVolume, toothArea := 0.0, 0.0, 0.0
	for i := range parts {
		part := &parts[i]
		lp, n := part.loft, len(part.loft.verts)/2
		part.vertex = make([][]int, 2)
		for cap := range 2 {
			part.vertex[cap] = make([]int, part.rootStart+1)
			for j := range part.vertex[cap] {
				part.vertex[cap][j] = len(mesh.verts)
				mesh.verts = append(mesh.verts, lp.verts[cap*n+j])
			}
		}
		faces := part.body.Faces()
		if len(faces) != lp.walls+2 {
			return nil, fmt.Errorf("%w: a loft has changed face topology", ErrUnsupported)
		}
		for j := range part.rootStart {
			for side := range 2 {
				k := 2*j + side
				f := faces[2+k]
				if len(f.Origins()) == 0 || f.Origins()[0].Role != fmt.Sprintf("side(0,%d,%d)", j, side) {
					return nil, fmt.Errorf("%w: a loft wall lost its source role", ErrUnsupported)
				}
				tri := lp.tris[k]
				var mapped [3]int
				for corner, old := range tri {
					cap, station := old/n, old%n
					if cap > 1 || station > part.rootStart {
						return nil, fmt.Errorf("%w: a loft wall crosses its root arc", ErrUnsupported)
					}
					mapped[corner] = part.vertex[cap][station]
				}
				group := len(mesh.groups)
				mesh.groups = append(mesh.groups, facetGroup{origins: f.Origins(), planar: true})
				mesh.add(mapped, group)
			}
		}
		toothBound = max(toothBound, part.mesh.Bound().Base())
		toothVolume = proofbound.AbsSumUpper(toothVolume, part.mesh.volSymDiff)
		area, err := part.body.Area()
		if err != nil {
			return nil, err
		}
		toothArea = proofbound.AbsSumUpper(toothArea, area.Value.Base(), area.Bound.Base())
	}
	var outlines [2][]int
	maximumStep, maximumTwist, ringRound := 0.0, 0.0, 0.0
	for i := range parts {
		part := &parts[i]
		next := &parts[(i+1)%len(parts)]
		for cap := range 2 {
			outlines[cap] = append(outlines[cap], part.vertex[cap]...)
		}
		var gap [2][2]float64
		var gapIV [2][2]proofbound.RatInterval
		for cap := range 2 {
			a, b := part.bottom, next.bottom
			ai, bi := part.bottomIV, next.bottomIV
			if cap == 1 {
				a, b = part.top, next.top
				ai, bi = part.topIV, next.topIV
			}
			for b[0] <= a[0] {
				b[0] += 2 * math.Pi
				bi[0] = proofbound.IntervalAdd(bi[0], proofbound.TwoPiInterval())
			}
			gap[cap] = [2]float64{a[1], b[0]}
			gapIV[cap] = [2]proofbound.RatInterval{ai[1], bi[0]}
		}
		width := max(gap[0][1]-gap[0][0], gap[1][1]-gap[1][0])
		steps := int(math.Ceil(width / 0.02))
		if steps < 1 || steps > 512 {
			return nil, fmt.Errorf("%w: circular gap exceeds its chord budget", ErrUnsupported)
		}
		maximumStep = max(maximumStep, width/float64(steps))
		var rings [2][]int
		for cap := range 2 {
			rings[cap] = make([]int, steps+1)
			rings[cap][0] = part.vertex[cap][part.rootStart]
			rings[cap][steps] = next.vertex[cap][0]
			for step := 1; step < steps; step++ {
				t := float64(step) / float64(steps)
				angle := gap[cap][0] + t*(gap[cap][1]-gap[cap][0])
				fraction := big.NewRat(int64(step), int64(steps))
				trueAngle := proofbound.IntervalAdd(
					proofbound.IntervalScale(gapIV[cap][0], new(big.Rat).Sub(big.NewRat(1, 1), fraction)),
					proofbound.IntervalScale(gapIV[cap][1], fraction))
				angleError := proofbound.IntervalFloatError(trueAngle, angle)
				z := pp.z0
				if cap == 1 {
					z = pp.z1
				}
				p := r3.Vec{X: radius * math.Cos(angle), Y: radius * math.Sin(angle), Z: z}
				if !proofbound.FiniteVec(p) {
					return nil, fmt.Errorf("%w: circular ring is non-finite", ErrUnsupported)
				}
				sine, cosine, ok := proofbound.RadSinCosInterval(proofarith.FloatRat(angle))
				if !ok {
					return nil, fmt.Errorf("%w: circular ring has no trigonometric enclosure", ErrUnsupported)
				}
				r := proofarith.FloatRat(radius)
				pointRound := proofbound.Radius3D(max(
					proofbound.IntervalFloatError(proofbound.IntervalScale(cosine, r), p.X),
					proofbound.IntervalFloatError(proofbound.IntervalScale(sine, r), p.Y)))
				ringRound = max(ringRound, proofbound.AbsSumUpper(pointRound,
					proofbound.ProductUpper(radius, angleError)))
				rings[cap][step] = len(mesh.verts)
				mesh.verts = append(mesh.verts, p)
				outlines[cap] = append(outlines[cap], rings[cap][step])
			}
		}
		maximumTwist = max(maximumTwist,
			math.Abs(gap[1][0]-gap[0][0]), math.Abs(gap[1][1]-gap[0][1]))
		for step := range steps {
			b0, b1 := rings[0][step], rings[0][step+1]
			t0, t1 := rings[1][step], rings[1][step+1]
			if !loftCylinderRingOrder(mesh.verts[b0], mesh.verts[b1],
				mesh.verts[t0], mesh.verts[t1]) {
				return nil, fmt.Errorf("%w: circular ring loses angular order through height", ErrUnsupported)
			}
			mesh.add([3]int{b0, b1, t1}, blankGroups[0])
			mesh.add([3]int{b0, t1, t0}, blankGroups[0])
		}
	}
	for cap, outline := range outlines {
		points := make([]triangulation.Point2, len(outline))
		loop := make([]int, len(outline))
		for i, index := range outline {
			p := mesh.verts[index]
			points[i] = triangulation.Point2{U: p.X, V: p.Y}
			loop[i] = i
		}
		triangles, err := triangulation.Triangulate(ctx, points, [][]int{loop})
		if err != nil {
			return nil, triangulation.WrapLoftError(err)
		}
		for _, tri := range triangles {
			mapped := [3]int{outline[tri[0]], outline[tri[1]], outline[tri[2]]}
			if cap == 0 {
				mapped[1], mapped[2] = mapped[2], mapped[1]
			}
			mesh.add(mapped, blankGroups[cap+1])
		}
	}
	if maximumTwist > 0.4 || maximumStep > 0.025 {
		return nil, fmt.Errorf("%w: circular join exceeds its angular proof limit", ErrUnsupported)
	}
	// Every omitted root wall and every replacement cylinder facet lies in an
	// annulus. The held root facets' exact projected minima and the new
	// cylinder facets' chord sagitta both contribute to its inner edge.
	width := proofbound.AbsSumUpper(maximumTwist, maximumStep)
	seamInset := proofbound.ProductUpper(radius,
		proofbound.ProductUpper(width, width)/8)
	innerInset, outerExcess, topCollar := seamInset, ringRound, 0.0
	for i, tri := range mesh.tris {
		if mesh.src[i] != blankGroups[0] {
			continue
		}
		minimum := loftCylinderTriangleMinRadius(mesh.verts, tri)
		if minimum <= 0 {
			return nil, fmt.Errorf("%w: circular ring reaches the axis", ErrUnsupported)
		}
		innerInset = max(innerInset, proofbound.RatFloatUp(new(big.Rat).Sub(
			proofarith.FloatRat(radius), proofarith.FloatRat(minimum))))
		for _, index := range tri {
			p := mesh.verts[index]
			x, y := proofarith.FloatRat(p.X), proofarith.FloatRat(p.Y)
			q := new(big.Rat).Add(new(big.Rat).Mul(x, x), new(big.Rat).Mul(y, y))
			maximum := proofbound.RatSqrtUp(q)
			outerExcess = max(outerExcess, proofbound.RatFloatUp(new(big.Rat).Sub(
				proofarith.FloatRat(maximum), proofarith.FloatRat(radius))))
		}
	}
	for _, part := range parts {
		innerInset = max(innerInset, part.inset)
		innerInset = max(innerInset,
			proofbound.AbsSumUpper(part.wedge, part.mesh.Bound().Base()))
		outerExcess = max(outerExcess,
			proofbound.AbsSumUpper(part.rootGap, part.mesh.Bound().Base()))
		topCollar = max(topCollar, part.collar)
	}
	blankMesh, err := blank.Tessellate(ctx, units.Millimeters(0.1), WithVerification(VerifyAll))
	if err != nil {
		return nil, err
	}
	bound := proofbound.AbsSumUpper(toothBound,
		proofbound.ProductUpper(2, innerInset), outerExcess, topCollar)
	if proofbound.IsNonFinite(bound) || bound > 0.1 {
		return nil, fmt.Errorf("%w: circular join exceeds the 0.1 mm boundary proof: bound %g blank %g tooth %g inset %g round %g twist %g step %g", ErrUnsupported, bound, blankMesh.Bound().Base(), toothBound, seamInset, ringRound, maximumTwist, maximumStep)
	}
	if err := bevelOrientMesh(mesh); err != nil {
		return nil, err
	}
	if err := loftmesh.LoftCrossingAudit(proofbound.NewWorkBudget(ctx), mesh.verts, mesh.tris); err != nil {
		return nil, err
	}
	for cap, outline := range outlines {
		z := pp.z0
		if cap == 1 {
			z = pp.z1
		}
		if !loftCylinderAuditCap(ctx, mesh, outline, blankGroups[cap+1], z, cap == 0) {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			return nil, fmt.Errorf("%w: circular join cap does not tile its recorded outline", ErrUnsupported)
		}
	}
	minSideRadius := math.Inf(1)
	for i, tri := range mesh.tris {
		if mesh.src[i] == blankGroups[1] || mesh.src[i] == blankGroups[2] {
			continue
		}
		minSideRadius = min(minSideRadius, loftCylinderTriangleMinRadius(mesh.verts, tri))
	}
	if minSideRadius <= 0 || proofbound.IsNonFinite(minSideRadius) {
		return nil, fmt.Errorf("%w: circular join has no side clearance certificate", ErrUnsupported)
	}
	area, err := proofbound.PerturbedAreaUpperContext(ctx, mesh.verts, mesh.tris, bound)
	if err != nil {
		return nil, err
	}
	blankArea, err := blank.Area()
	if err != nil {
		return nil, err
	}
	// Outside this full radial band, the blank is either wholly occupied or
	// wholly absent, and the retained Loft facets have the same source roles.
	// Charge the entire band instead of assuming a facet correspondence at
	// the root seam. The source operands' own occupied-volume errors remain.
	if innerInset >= radius {
		return nil, fmt.Errorf("%w: circular join reaches the axis", ErrUnsupported)
	}
	bandWidth := proofbound.AbsSumUpper(innerInset, outerExcess)
	bandArea := proofbound.AbsSumUpper(
		proofbound.ProductUpper(proofbound.ProductUpper(2, radius), bandWidth),
		proofbound.ProductUpper(outerExcess, outerExcess))
	annulusArea := proofbound.ProductUpper(proofbound.RatFloatUp(proofbound.PiUpper), bandArea)
	height := proofbound.RatFloatUp(new(big.Rat).Sub(
		proofarith.FloatRat(pp.z1), proofarith.FloatRat(pp.z0)))
	annulusVolume := proofbound.ProductUpper(annulusArea, height)
	volumeGap := proofbound.AbsSumUpper(blankMesh.volSymDiff, toothVolume, annulusVolume)
	areaSlack := proofbound.AbsSumUpper(area, blankArea.Value.Base(), blankArea.Bound.Base(), toothArea)
	if proofbound.IsNonFinite(volumeGap) || proofbound.IsNonFinite(areaSlack) {
		return nil, fmt.Errorf("%w: circular join has no finite volume proof", ErrUnsupported)
	}
	vertexBound := make([]float64, len(mesh.verts))
	for i := range vertexBound {
		vertexBound[i] = bound
	}
	toothBodies := make([]*Body, len(parts))
	for i := range parts {
		toothBodies[i] = parts[i].body
	}
	fp := facetedPayload{verts: mesh.verts, tris: mesh.tris, src: mesh.src,
		groups: mesh.groups, vertexBound: vertexBound, contactAudited: true,
		meshBound: bound, volSymDiff: volumeGap, areaSlack: areaSlack,
		xform: r3.Identity(), loftCylinderJoin: &loftCylinderJoinRecord{
			blank: blank, teeth: toothBodies, capOutlines: outlines,
			capGroups:     [2]int{blankGroups[1], blankGroups[2]},
			minSideRadius: minSideRadius, radius: radius, z0: pp.z0, z1: pp.z1,
		}}
	return buildFacetedBody(ctx, d, ref, fp)
}

// The angle between neighboring ring rulings stays positive at every z:
// their cross product is a quadratic in height whose three Bernstein
// coefficients are positive. Continuity makes the ring cover each
// intermediate angular ray between its ordered endpoints.
func loftCylinderRingOrder(b0, b1, t0, t1 r3.Vec) bool {
	cross := func(a, b r3.Vec) *big.Rat {
		return new(big.Rat).Sub(
			new(big.Rat).Mul(proofarith.FloatRat(a.X), proofarith.FloatRat(b.Y)),
			new(big.Rat).Mul(proofarith.FloatRat(a.Y), proofarith.FloatRat(b.X)))
	}
	if cross(b0, b1).Sign() <= 0 || cross(t0, t1).Sign() <= 0 {
		return false
	}
	middle := new(big.Rat).Add(cross(b0, t1), cross(t0, b1))
	return middle.Sign() > 0
}
