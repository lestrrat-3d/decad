// Package loftcylinder holds the loft/cylinder Union's exact ring-to-wall
// distances, split-ring cycles, projected triangle radii and radial and wedge
// gates. See docs/loft-cylinder-boolean-design.md.
package loftcylinder

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// These exact distances only decide whether a held Loft wall can meet a
// proposed ring after its authenticated, two-sided wall displacement.
// RingVec is an exact rational 3-vector.
type RingVec [3]*big.Rat

// RingPoint is the exact rational vector of a homogeneous point.
func RingPoint(p proofarith.Xpt) RingVec {
	return RingVec{
		new(big.Rat).SetFrac(p.X, p.W), new(big.Rat).SetFrac(p.Y, p.W),
		new(big.Rat).SetFrac(p.Z, p.W),
	}
}

// RingVertex is the exact rational vector of a float vertex.
func RingVertex(p r3.Vec) RingVec {
	return RingVec{proofarith.FloatRat(p.X), proofarith.FloatRat(p.Y), proofarith.FloatRat(p.Z)}
}

// RingSub is a - b.
func RingSub(a, b RingVec) RingVec {
	return RingVec{
		proofarith.SubRat(new(big.Rat), a[0], b[0]), proofarith.SubRat(new(big.Rat), a[1], b[1]),
		proofarith.SubRat(new(big.Rat), a[2], b[2]),
	}
}

// RingDot is the dot product of a and b.
func RingDot(a, b RingVec) *big.Rat {
	q := new(big.Rat)
	for i := range a {
		proofarith.AddRat(q, q, proofarith.MulRat(new(big.Rat), a[i], b[i]))
	}
	return q
}

// RingCross is the cross product of a and b.
func RingCross(a, b RingVec) RingVec {
	return RingVec{
		proofarith.SubRat(new(big.Rat), proofarith.MulRat(new(big.Rat), a[1], b[2]), proofarith.MulRat(new(big.Rat), a[2], b[1])),
		proofarith.SubRat(new(big.Rat), proofarith.MulRat(new(big.Rat), a[2], b[0]), proofarith.MulRat(new(big.Rat), a[0], b[2])),
		proofarith.SubRat(new(big.Rat), proofarith.MulRat(new(big.Rat), a[0], b[1]), proofarith.MulRat(new(big.Rat), a[1], b[0])),
	}
}

func ringPointSegment2(p, a, b RingVec) *big.Rat {
	d := RingSub(b, a)
	length2 := RingDot(d, d)
	if length2.Sign() == 0 {
		v := RingSub(p, a)
		return RingDot(v, v)
	}
	v := RingSub(p, a)
	along := RingDot(v, d)
	if along.Sign() <= 0 {
		return RingDot(v, v)
	}
	if along.Cmp(length2) >= 0 {
		v = RingSub(p, b)
		return RingDot(v, v)
	}
	return proofarith.SubRat(new(big.Rat), RingDot(v, v),
		new(big.Rat).Quo(proofarith.MulRat(new(big.Rat), along, along), length2))
}

func ringPointTriangle2(p RingVec, tri [3]RingVec) *big.Rat {
	n := RingCross(RingSub(tri[1], tri[0]),
		RingSub(tri[2], tri[0]))
	n2 := RingDot(n, n)
	if n2.Sign() == 0 {
		return nil
	}
	inside := true
	for i := range tri {
		e := RingSub(tri[(i+1)%3], tri[i])
		v := RingSub(p, tri[i])
		if RingDot(RingCross(e, v), n).Sign() < 0 {
			inside = false
			break
		}
	}
	if inside {
		q := RingDot(RingSub(p, tri[0]), n)
		return new(big.Rat).Quo(proofarith.MulRat(new(big.Rat), q, q), n2)
	}
	var minimum *big.Rat
	for i := range tri {
		q := ringPointSegment2(p, tri[i], tri[(i+1)%3])
		if minimum == nil || q.Cmp(minimum) < 0 {
			minimum = q
		}
	}
	return minimum
}

// The endpoint and edge cases cover the constrained minimum of two segments.
// An interior stationary pair is added when their directions are independent.
func ringSegments2(a, b, c, d RingVec) *big.Rat {
	minimum := ringPointSegment2(a, c, d)
	for _, q := range []*big.Rat{
		ringPointSegment2(b, c, d),
		ringPointSegment2(c, a, b),
		ringPointSegment2(d, a, b),
	} {
		if q.Cmp(minimum) < 0 {
			minimum = q
		}
	}
	u, v, w := RingSub(b, a), RingSub(d, c), RingSub(a, c)
	uu, vv, uv := RingDot(u, u), RingDot(v, v), RingDot(u, v)
	det := proofarith.SubRat(new(big.Rat), proofarith.MulRat(new(big.Rat), uu, vv), proofarith.MulRat(new(big.Rat), uv, uv))
	if det.Sign() == 0 {
		return minimum
	}
	uw, vw := RingDot(u, w), RingDot(v, w)
	s := new(big.Rat).Quo(proofarith.SubRat(new(big.Rat), proofarith.MulRat(new(big.Rat), uv, vw),
		proofarith.MulRat(new(big.Rat), vv, uw)), det)
	t := new(big.Rat).Quo(proofarith.SubRat(new(big.Rat), proofarith.MulRat(new(big.Rat), uu, vw),
		proofarith.MulRat(new(big.Rat), uv, uw)), det)
	if s.Sign() <= 0 || s.Cmp(big.NewRat(1, 1)) >= 0 ||
		t.Sign() <= 0 || t.Cmp(big.NewRat(1, 1)) >= 0 {
		return minimum
	}
	for i := range w {
		w[i] = proofarith.SubRat(new(big.Rat), proofarith.AddRat(new(big.Rat), w[i], proofarith.MulRat(new(big.Rat), s, u[i])),
			proofarith.MulRat(new(big.Rat), t, v[i]))
	}
	q := RingDot(w, w)
	if q.Cmp(minimum) < 0 {
		return q
	}
	return minimum
}

// A segment/triangle minimum occurs at a segment endpoint, an edge pair,
// or a direct crossing. The zero-distance crossing is checked by the two
// segment endpoints' signed plane values and the triangle's edge planes.
func ringSegmentTriangle2(a, b RingVec,
	tri [3]RingVec) *big.Rat {
	n := RingCross(RingSub(tri[1], tri[0]),
		RingSub(tri[2], tri[0]))
	if RingDot(n, n).Sign() == 0 {
		return nil
	}
	da := RingDot(RingSub(a, tri[0]), n)
	db := RingDot(RingSub(b, tri[0]), n)
	if da.Sign() == 0 || db.Sign() == 0 || da.Sign() != db.Sign() {
		den := proofarith.SubRat(new(big.Rat), da, db)
		if den.Sign() != 0 {
			t := proofarith.QuoRat(new(big.Rat), da, den)
			if t.Sign() >= 0 && t.Cmp(big.NewRat(1, 1)) <= 0 {
				p := RingVec{}
				for i := range p {
					p[i] = proofarith.AddRat(new(big.Rat), a[i],
						proofarith.MulRat(new(big.Rat), t, proofarith.SubRat(new(big.Rat), b[i], a[i])))
				}
				if q := ringPointTriangle2(p, tri); q != nil && q.Sign() == 0 {
					return q
				}
			}
		}
	}
	minimum := ringPointTriangle2(a, tri)
	if minimum == nil {
		return nil
	}
	if q := ringPointTriangle2(b, tri); q.Cmp(minimum) < 0 {
		minimum = q
	}
	for i := range tri {
		q := ringSegments2(a, b, tri[i], tri[(i+1)%3])
		if q.Cmp(minimum) < 0 {
			minimum = q
		}
	}
	return minimum
}

// RingBox is an axis-aligned float box, one low and high pair per axis.
type RingBox [3][2]float64

// RingTriangleBox is the box of tri grown by displacement on every side.
func RingTriangleBox(tri [3]r3.Vec, displacement float64) RingBox {
	var box RingBox
	for axis := range box {
		value := [3]float64{}
		for j, p := range tri {
			switch axis {
			case 0:
				value[j] = p.X
			case 1:
				value[j] = p.Y
			default:
				value[j] = p.Z
			}
		}
		lo, hi := value[0], value[0]
		for _, x := range value[1:] {
			if x < lo {
				lo = x
			}
			if x > hi {
				hi = x
			}
		}
		box[axis] = [2]float64{
			proofbound.RatFloatDown(proofarith.SubRat(new(big.Rat), proofarith.FloatRat(lo), proofarith.FloatRat(displacement))),
			proofbound.RatFloatUp(proofarith.AddRat(new(big.Rat), proofarith.FloatRat(hi), proofarith.FloatRat(displacement))),
		}
	}
	return box
}

// RingEdgeBox is the outward-rounded box of the segment from a to b.
func RingEdgeBox(a, b RingVec) RingBox {
	var box RingBox
	for i := range box {
		lo, hi := a[i], b[i]
		if lo.Cmp(hi) > 0 {
			lo, hi = hi, lo
		}
		box[i] = [2]float64{proofbound.RatFloatDown(lo), proofbound.RatFloatUp(hi)}
	}
	return box
}

func ringBoxesMeet(a, b RingBox) bool {
	for i := range a {
		if a[i][1] < b[i][0] || b[i][1] < a[i][0] {
			return false
		}
	}
	return true
}

// RingOccupancy records only the exact checks completed for one
// connected, constant-height cutter ring. It does not certify the cutter's
// cap, collar, or entire band.
type RingOccupancy struct {
	Inner, Outer, Seam, Candidates int
	MinimumClearance2              *big.Rat
}

// RingWall is one held Loft wall triangle with its exact vertices, float box,
// normal, squared normal and plane offset.
type RingWall struct {
	Points          [3]RingVec
	Box             RingBox
	Normal          RingVec
	Normal2, Plane0 *big.Rat
}

func ringValidPoint(p proofarith.Xpt) bool {
	return p.X != nil && p.Y != nil && p.Z != nil && p.W != nil && p.W.Sign() > 0
}

// RingCycle returns the common height of edges when they form one simple
// cycle at a single height.
func RingCycle(edges [][2]proofarith.Xpt) (*big.Rat, bool) {
	if len(edges) < 3 || len(edges) > 8192 {
		return nil, false
	}
	adjacent := make(map[string][]string, len(edges))
	seen := make(map[string]struct{}, len(edges))
	var level *big.Rat
	for _, edge := range edges {
		if !ringValidPoint(edge[0]) || !ringValidPoint(edge[1]) {
			return nil, false
		}
		a, b := edge[0].Key(), edge[1].Key()
		if a == b {
			return nil, false
		}
		za := new(big.Rat).SetFrac(edge[0].Z, edge[0].W)
		zb := new(big.Rat).SetFrac(edge[1].Z, edge[1].W)
		if za.Cmp(zb) != 0 || level != nil && za.Cmp(level) != 0 {
			return nil, false
		}
		level = za
		if a > b {
			a, b = b, a
		}
		key := a + "|" + b
		if _, exists := seen[key]; exists {
			return nil, false
		}
		seen[key] = struct{}{}
		adjacent[a] = append(adjacent[a], b)
		adjacent[b] = append(adjacent[b], a)
	}
	if len(adjacent) != len(edges) {
		return nil, false
	}
	var start string
	for key, neighbors := range adjacent {
		if len(neighbors) != 2 {
			return nil, false
		}
		start = key
	}
	visited := map[string]bool{start: true}
	queue := []string{start}
	for len(queue) != 0 {
		key := queue[len(queue)-1]
		queue = queue[:len(queue)-1]
		for _, next := range adjacent[key] {
			if !visited[next] {
				visited[next] = true
				queue = append(queue, next)
			}
		}
	}
	return level, len(visited) == len(adjacent)
}

// ringSegmentPlane is a segment's ends and a bound in uncancelled fractions,
// read by every wall of one RingWallClear call.
type ringSegmentPlane struct {
	a, b   [3]proofarith.LazyRat
	bound2 proofarith.LazyRat
	usable bool
}

func ringSegmentPlaneOf(a, b RingVec, bound2 *big.Rat) ringSegmentPlane {
	if bound2 == nil || bound2.Sign() <= 0 {
		return ringSegmentPlane{}
	}
	s := ringSegmentPlane{bound2: proofarith.LazyOf(bound2), usable: true}
	for i := range a {
		s.a[i], s.b[i] = proofarith.LazyOf(a[i]), proofarith.LazyOf(b[i])
	}
	return s
}

func ringLazyDot(a [3]proofarith.LazyRat, n RingVec) proofarith.LazyRat {
	sum := proofarith.LazyMul(a[0], proofarith.LazyOf(n[0]))
	for i := 1; i < 3; i++ {
		sum = proofarith.LazyAdd(sum, proofarith.LazyMul(a[i], proofarith.LazyOf(n[i])))
	}
	return sum
}

// A same-side segment farther than bound from the entire triangle plane
// cannot meet that triangle or its parameter-matched true Loft wall. This
// only rejects pairs; unresolved pairs use the complete exact distance. The
// test multiplies the two sides of (min distance)^2 > bound2 * |normal|^2
// through, so no fraction is cancelled.
func ringPlaneApart(seg ringSegmentPlane, wall *RingWall) bool {
	if !seg.usable || wall.Normal2 == nil || wall.Normal2.Sign() == 0 || wall.Plane0 == nil {
		return false
	}
	plane0 := proofarith.LazyOf(wall.Plane0)
	da := proofarith.LazySub(ringLazyDot(seg.a, wall.Normal), plane0)
	db := proofarith.LazySub(ringLazyDot(seg.b, wall.Normal), plane0)
	if da.Sign() == 0 || da.Sign() != db.Sign() {
		return false
	}
	if da.Sign() < 0 {
		da, db = proofarith.LazyNeg(da), proofarith.LazyNeg(db)
	}
	if proofarith.LazyCmp(db, da) < 0 {
		da = db
	}
	return proofarith.LazyCmp(proofarith.LazyMul(da, da),
		proofarith.LazyMul(seg.bound2, proofarith.LazyOf(wall.Normal2))) > 0
}

// RingWallClear checks the segment from a to b, whose float box is box,
// against every wall that its box meets. It refuses when the segment may come
// within sqrt(bound2) of a wall and records the smallest clearance in result.
func RingWallClear(ctx context.Context, a, b RingVec,
	box RingBox, bound2 *big.Rat, walls []RingWall,
	result *RingOccupancy) error {
	// Most calls meet no wall, so the segment's fractions are built on the
	// first candidate.
	var seg ringSegmentPlane
	segBuilt := false
	for wallIndex := range walls {
		if !ringBoxesMeet(box, walls[wallIndex].Box) {
			continue
		}
		wall := walls[wallIndex]
		result.Candidates++
		if result.Candidates > 1000000 || result.Candidates%256 == 0 && ctx.Err() != nil {
			if err := ctx.Err(); err != nil {
				return err
			}
			return fmt.Errorf("%w: ring wall clearance work limit", decaderr.ErrUnsupported)
		}
		if !segBuilt {
			seg, segBuilt = ringSegmentPlaneOf(a, b, bound2), true
		}
		if ringPlaneApart(seg, &wall) {
			continue
		}
		q := ringSegmentTriangle2(a, b, wall.Points)
		if q == nil || q.Cmp(bound2) <= 0 {
			gap2 := math.NaN()
			if q != nil {
				gap2, _ = q.Float64()
			}
			bound2, _ := bound2.Float64()
			return fmt.Errorf("%w: ring may meet true Loft wall %d (gap²≈%g, bound²≈%g)",
				decaderr.ErrUnsupported, wallIndex, gap2, bound2)
		}
		if result.MinimumClearance2 == nil || q.Cmp(result.MinimumClearance2) < 0 {
			result.MinimumClearance2 = q
		}
	}
	return nil
}

// RingMappedPoint maps p to height level, scaling its homogeneous weight.
func RingMappedPoint(p proofarith.Xpt, level *big.Rat) proofarith.Xpt {
	den := new(big.Int).Set(level.Denom())
	return proofarith.Xpt{
		X: new(big.Int).Mul(p.X, den), Y: new(big.Int).Mul(p.Y, den),
		Z: new(big.Int).Mul(p.W, level.Num()), W: new(big.Int).Mul(p.W, den),
	}
}
