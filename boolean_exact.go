package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/r3"
)

// This file is the exact-arithmetic kernel behind the mesh boolean
// (docs/evaluator-design.md §9): sign tests decided by an adaptive float
// filter that falls back to math/big.Rat exactly at the boundary cases, plus
// the rational point, segment and parity predicates the subdivision and
// classification passes run on. A sign decided exactly is a topology decision
// that cannot flip (core §2.1), which is what makes the stitched output
// watertight by construction on the tessellated geometry.

// xpt is an exact 3D point: rational coordinates, exact until the final
// rounding back to float64. The zero value is unusable; construct through
// xptOf or the arithmetic helpers.
type xpt struct{ x, y, z *big.Rat }

// xptOf lifts a finite float vertex into exact coordinates. A float64 is an
// exact rational, so no information is lost.
func xptOf(v r3.Vec) xpt {
	return xpt{mustRatOf(v.X), mustRatOf(v.Y), mustRatOf(v.Z)}
}

// The float-to-rational lift this file runs on is mustRatOf (clearance_poly.go),
// the package's one exact-rational helper: a float64 IS a rational, so the
// conversion is exact and lossless, and the two exact kernels — this one and
// the clearance brackets — share the checked converter rather than keep two
// conversions that could drift apart. Every mustRatOf call here is on a float
// already proven finite: prepBoolMesh rejects a non-finite vertex outright, so
// the operands this file sees hold none.
//
// vec rounds the exact point to the nearest float64 coordinates.
func (p xpt) vec() r3.Vec {
	x, _ := p.x.Float64()
	y, _ := p.y.Float64()
	z, _ := p.z.Float64()
	return r3.Vec{X: x, Y: y, Z: z}
}

// key is the exact identity of the point: two points weld exactly when their
// rational coordinates are identical — stitching by shared exact vertices,
// never by distance (docs/evaluator-design.md §9).
func (p xpt) key() string {
	return p.x.RatString() + "|" + p.y.RatString() + "|" + p.z.RatString()
}

func xsub(a, b xpt) xpt {
	return xpt{new(big.Rat).Sub(a.x, b.x), new(big.Rat).Sub(a.y, b.y), new(big.Rat).Sub(a.z, b.z)}
}

func xcross(a, b xpt) xpt {
	return xpt{
		new(big.Rat).Sub(new(big.Rat).Mul(a.y, b.z), new(big.Rat).Mul(a.z, b.y)),
		new(big.Rat).Sub(new(big.Rat).Mul(a.z, b.x), new(big.Rat).Mul(a.x, b.z)),
		new(big.Rat).Sub(new(big.Rat).Mul(a.x, b.y), new(big.Rat).Mul(a.y, b.x)),
	}
}

func xdot(a, b xpt) *big.Rat {
	s := new(big.Rat).Mul(a.x, b.x)
	s.Add(s, new(big.Rat).Mul(a.y, b.y))
	return s.Add(s, new(big.Rat).Mul(a.z, b.z))
}

// xlerp is a + t·(b − a), exact.
func xlerp(a, b xpt, t *big.Rat) xpt {
	d := xsub(b, a)
	return xpt{
		new(big.Rat).Add(a.x, new(big.Rat).Mul(t, d.x)),
		new(big.Rat).Add(a.y, new(big.Rat).Mul(t, d.y)),
		new(big.Rat).Add(a.z, new(big.Rat).Mul(t, d.z)),
	}
}

// orientVal is the exact value of det[b−a, c−a, d−a]: positive when d lies on
// the side the counter-clockwise normal of (a, b, c) points to.
func orientVal(a, b, c, d xpt) *big.Rat {
	return xdot(xcross(xsub(b, a), xsub(c, a)), xsub(d, a))
}

// orientSign is the adaptive-precision sign of det[b−a, c−a, d−a] for float
// inputs: a float evaluation whose forward error provably cannot cross zero
// decides the generic case; anything inside the error bound falls back to the
// exact rational value — the §9 discipline, so a sign is never wrong.
func orientSign(a, b, c, d r3.Vec) int {
	bax, bay, baz := b.X-a.X, b.Y-a.Y, b.Z-a.Z
	cax, cay, caz := c.X-a.X, c.Y-a.Y, c.Z-a.Z
	dax, day, daz := d.X-a.X, d.Y-a.Y, d.Z-a.Z
	det := bax*(cay*daz-caz*day) + bay*(caz*dax-cax*daz) + baz*(cax*day-cay*dax)
	perm := math.Abs(bax)*(math.Abs(cay)*math.Abs(daz)+math.Abs(caz)*math.Abs(day)) +
		math.Abs(bay)*(math.Abs(caz)*math.Abs(dax)+math.Abs(cax)*math.Abs(daz)) +
		math.Abs(baz)*(math.Abs(cax)*math.Abs(day)+math.Abs(cay)*math.Abs(dax))
	// The true forward error is bounded by a few ulps of the permanent; 1e-12
	// leaves three decades of margin, so a sign the filter accepts is proven.
	if err := 1e-12 * perm; det > err || det < -err {
		if det > 0 {
			return 1
		}
		return -1
	}
	return orientVal(xptOf(a), xptOf(b), xptOf(c), xptOf(d)).Sign()
}

// orientSignMixed is the exact plane-side sign of a rational probe against a
// float triangle: positive on the triangle's counter-clockwise-normal side.
func orientSignMixed(a, b, c r3.Vec, d xpt) int {
	return orientVal(xptOf(a), xptOf(b), xptOf(c), d).Sign()
}

// xp2 is an exact 2D point (a plane projection of an xpt).
type xp2 struct{ u, v *big.Rat }

// key2 is the exact 2D identity.
func (p xp2) key2() string { return p.u.RatString() + "|" + p.v.RatString() }

// cross2x is the exact value of (b − a) × (c − a): positive when a, b, c turn
// counter-clockwise.
func cross2x(a, b, c xp2) *big.Rat {
	bu := new(big.Rat).Sub(b.u, a.u)
	bv := new(big.Rat).Sub(b.v, a.v)
	cu := new(big.Rat).Sub(c.u, a.u)
	cv := new(big.Rat).Sub(c.v, a.v)
	return new(big.Rat).Sub(new(big.Rat).Mul(bu, cv), new(big.Rat).Mul(bv, cu))
}

// pointInTriX reports whether p lies inside or on the closed triangle a, b,
// c, whichever way it is wound — the exact analog of pointInTri.
func pointInTriX(p, a, b, c xp2) bool {
	d1 := cross2x(a, b, p).Sign()
	d2 := cross2x(b, c, p).Sign()
	d3 := cross2x(c, a, p).Sign()
	hasNeg := d1 < 0 || d2 < 0 || d3 < 0
	hasPos := d1 > 0 || d2 > 0 || d3 > 0
	return !hasNeg || !hasPos
}

// onSegment2 reports whether p lies on the closed segment (a, b) — collinear
// and within the endpoints. interior additionally excludes the endpoints.
func onSegment2(a, b, p xp2) (bool, bool) {
	if cross2x(a, b, p).Sign() != 0 {
		return false, false
	}
	// Collinear: order along the dominant axis of the segment.
	du := new(big.Rat).Sub(b.u, a.u)
	dv := new(big.Rat).Sub(b.v, a.v)
	var lo, hi, x *big.Rat
	if du.Sign() != 0 {
		lo, hi, x = a.u, b.u, p.u
	} else if dv.Sign() != 0 {
		lo, hi, x = a.v, b.v, p.v
	} else {
		// A zero-length segment holds only its own point.
		eq := p.u.Cmp(a.u) == 0 && p.v.Cmp(a.v) == 0
		return eq, false
	}
	if lo.Cmp(hi) > 0 {
		lo, hi = hi, lo
	}
	if x.Cmp(lo) < 0 || x.Cmp(hi) > 0 {
		return false, false
	}
	interior := x.Cmp(lo) > 0 && x.Cmp(hi) < 0
	return true, interior
}

// pointInPoly2 reports whether p lies strictly inside the simple polygon —
// exact parity along a +u ray with the half-open rule, so a crossing at a
// shared vertex counts exactly once. A p on the boundary reports onBoundary.
func pointInPoly2(budget *workBudget, poly []xp2, p xp2) (bool, bool, error) {
	n := len(poly)
	for i := range n {
		if err := budget.step(); err != nil {
			return false, false, err
		}
		on, _ := onSegment2(poly[i], poly[(i+1)%n], p)
		if on {
			return false, true, nil
		}
	}
	inside := false
	for i := range n {
		if err := budget.step(); err != nil {
			return false, false, err
		}
		a, b := poly[i], poly[(i+1)%n]
		if (a.v.Cmp(p.v) <= 0) == (b.v.Cmp(p.v) <= 0) {
			continue
		}
		// u of the crossing at height p.v: a.u + (p.v−a.v)·(b.u−a.u)/(b.v−a.v).
		t := new(big.Rat).Quo(new(big.Rat).Sub(p.v, a.v), new(big.Rat).Sub(b.v, a.v))
		u := new(big.Rat).Add(a.u, new(big.Rat).Mul(t, new(big.Rat).Sub(b.u, a.u)))
		if u.Cmp(p.u) > 0 {
			inside = !inside
		}
	}
	return inside, false, nil
}

// polyArea2 is twice the exact signed area of the polygon.
func polyArea2(budget *workBudget, poly []xp2) (*big.Rat, error) {
	total := new(big.Rat)
	n := len(poly)
	for i := range n {
		if err := budget.step(); err != nil {
			return nil, err
		}
		a, b := poly[i], poly[(i+1)%n]
		total.Add(total, new(big.Rat).Sub(new(big.Rat).Mul(a.u, b.v), new(big.Rat).Mul(b.u, a.v)))
	}
	return total, nil
}

// earClipX triangulates a weakly-simple counter-clockwise polygon (given as
// indices into pts) by exact ear clipping. Unlike the float cap triangulator
// it NEVER drops a collinear vertex — every input vertex appears in the
// output, which is what keeps a conforming subdivision conforming — so only
// strictly convex, unblocked ears are clipped. A stall means the polygon is
// not weakly simple, which is an internal error, never a wrong mesh.
func earClipX(budget *workBudget, pts []xp2, poly []int) ([][3]int, error) {
	idx := make([]int, len(poly))
	for i, vi := range poly {
		if err := budget.step(); err != nil {
			return nil, err
		}
		idx[i] = vi
	}
	tris := make([][3]int, 0, len(idx))
	for len(idx) > 3 {
		if err := budget.step(); err != nil {
			return nil, err
		}
		n := len(idx)
		clipped := false
		for i := range n {
			if err := budget.step(); err != nil {
				return nil, err
			}
			ia, ib, ic := idx[(i-1+n)%n], idx[i], idx[(i+1)%n]
			if cross2x(pts[ia], pts[ib], pts[ic]).Sign() <= 0 {
				continue
			}
			blocked, err := earBlockedX(budget, pts, idx, i)
			if err != nil {
				return nil, err
			}
			if blocked {
				continue
			}
			tris = append(tris, [3]int{ia, ib, ic})
			for j := i; j+1 < len(idx); j++ {
				if err := budget.step(); err != nil {
					return nil, err
				}
				idx[j] = idx[j+1]
			}
			idx = idx[:len(idx)-1]
			clipped = true
			break
		}
		if !clipped {
			return nil, fmt.Errorf(`%w: exact ear clipping stalled on a boolean subdivision polygon`, ErrBooleanFailed)
		}
	}
	if cross2x(pts[idx[0]], pts[idx[1]], pts[idx[2]]).Sign() > 0 {
		tris = append(tris, [3]int{idx[0], idx[1], idx[2]})
	} else if cross2x(pts[idx[0]], pts[idx[1]], pts[idx[2]]).Sign() < 0 {
		return nil, fmt.Errorf(`%w: a boolean subdivision polygon closed clockwise`, ErrBooleanFailed)
	}
	return tris, nil
}

// earBlockedX reports whether another polygon vertex lies inside the closed
// candidate ear — the exact analog of earBlocked, except that every OTHER
// vertex can block (collinear duplicates included), which is the conservative
// direction.
func earBlockedX(budget *workBudget, pts []xp2, idx []int, i int) (bool, error) {
	n := len(idx)
	ip, in := (i-1+n)%n, (i+1)%n
	a, b, c := pts[idx[ip]], pts[idx[i]], pts[idx[in]]
	for j := range n {
		if err := budget.step(); err != nil {
			return false, err
		}
		if j == ip || j == i || j == in {
			continue
		}
		p := pts[idx[j]]
		if p.u.Cmp(a.u) == 0 && p.v.Cmp(a.v) == 0 {
			continue
		}
		if p.u.Cmp(c.u) == 0 && p.v.Cmp(c.v) == 0 {
			continue
		}
		if pointInTriX(p, a, b, c) {
			return true, nil
		}
	}
	return false, nil
}

// axisRays is the deterministic retry list for the parity test: the six
// axis-aligned directions. axis names the swept coordinate, dir its sense,
// and (u, v) the two projection coordinates.
var axisRays = [6]struct {
	axis, u, v int
	dir        int
}{
	{0, 1, 2, 1}, {0, 1, 2, -1},
	{1, 2, 0, 1}, {1, 2, 0, -1},
	{2, 0, 1, 1}, {2, 0, 1, -1},
}

// coordOf reads one coordinate of a float vertex by axis index.
func coordOf(v r3.Vec, axis int) float64 {
	switch axis {
	case 0:
		return v.X
	case 1:
		return v.Y
	default:
		return v.Z
	}
}

// ratCoordOf reads one exact coordinate by axis index.
func ratCoordOf(p xpt, axis int) *big.Rat {
	switch axis {
	case 0:
		return p.x
	case 1:
		return p.y
	default:
		return p.z
	}
}

// meshParity reports, exactly, whether p lies inside the closed float-vertex
// mesh restricted to the given facet subset: the crossing parity of an
// axis-aligned ray. A ray the point's projection meets at a facet's projected
// boundary is ambiguous and the next axis is tried; a p exactly ON a facet is
// onBoundary. All six axes ambiguous is a genuine failure — never a guess.
func meshParityContext(ctx context.Context, p xpt, verts []r3.Vec, tris [][3]int, subset []int) (bool, bool, error) {
	for _, ray := range axisRays {
		crossings := 0
		ambiguous := false
		onBoundary := false
		for i, ti := range subset {
			if i%256 == 0 {
				if err := ctx.Err(); err != nil {
					return false, false, err
				}
			}
			tri := tris[ti]
			a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
			pa := xp2{ratCoordOf(p, ray.u), ratCoordOf(p, ray.v)}
			qa := xp2{mustRatOf(coordOf(a, ray.u)), mustRatOf(coordOf(a, ray.v))}
			qb := xp2{mustRatOf(coordOf(b, ray.u)), mustRatOf(coordOf(b, ray.v))}
			qc := xp2{mustRatOf(coordOf(c, ray.u)), mustRatOf(coordOf(c, ray.v))}
			s1 := cross2x(qa, qb, pa).Sign()
			s2 := cross2x(qb, qc, pa).Sign()
			s3 := cross2x(qc, qa, pa).Sign()
			neg := s1 < 0 || s2 < 0 || s3 < 0
			pos := s1 > 0 || s2 > 0 || s3 > 0
			if neg && pos {
				continue // strictly outside the projection
			}
			if s1 == 0 || s2 == 0 || s3 == 0 {
				// On the projected boundary: the ray may graze an edge or a
				// vertex, and the count would be unreliable — try another axis.
				ambiguous = true
				break
			}
			// Strictly inside the projection: the projected area is nonzero,
			// so the plane normal's swept component cannot vanish.
			xa, xb, xc := xptOf(a), xptOf(b), xptOf(c)
			n := xcross(xsub(xb, xa), xsub(xc, xa))
			nAxis := ratCoordOf(n, ray.axis)
			if nAxis.Sign() == 0 {
				ambiguous = true
				break
			}
			tNum := xdot(xsub(xa, p), n)
			t := new(big.Rat).Quo(tNum, nAxis)
			switch s := t.Sign() * ray.dir; {
			case s > 0:
				crossings++
			case t.Sign() == 0:
				onBoundary = true
			}
			if onBoundary {
				break
			}
		}
		if onBoundary {
			return false, true, nil
		}
		if ambiguous {
			continue
		}
		return crossings%2 == 1, false, nil
	}
	return false, false, fmt.Errorf(`%w: every parity ray was ambiguous`, ErrBooleanFailed)
}
