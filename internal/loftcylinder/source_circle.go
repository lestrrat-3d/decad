package loftcylinder

import (
	"fmt"
	"math/big"
	"runtime"
	"sync"
	"sync/atomic"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// SourceCircle certifies where one authenticated Loft tooth meets
// the exact blank circle in the open axial interval (safeBottom, safeTop). Source
// caps, possible root collars, and the rest of the tooth's interior are
// outside this record's claim.
type SourceCircle struct {
	Z0, Z1, SafeBottom, SafeTop *big.Rat
	Radius                      *big.Rat
	GapAxis                     XY
	Connector                   [2]Connector
}

// Connector is one LineSeg source wall at both caps, through the triangle
// diagonal A, D, F.
type Connector struct {
	// At each cap, the source LineSeg wall is the two triangles through A,D,F.
	a, d, f [2]XY
}

// XYBox is an exact axis-aligned box in the cross-section plane.
type XYBox struct {
	lo, hi XY
}

// The exact section point applies the dyadic axial placement basis to the
// recorded, untrimmed LineSeg endpoint. The section frames are checked above.
func SourcePoint(ex r3.Vec, p sectionrecord.Point2) XY {
	c, s := proofarith.FloatRat(ex.X), proofarith.FloatRat(ex.Y)
	u, v := proofarith.FloatRat(p.U), proofarith.FloatRat(p.V)
	return XY{
		X: proofarith.SubRat(new(big.Rat), proofarith.MulRat(new(big.Rat), c, u), proofarith.MulRat(new(big.Rat), s, v)),
		Y: proofarith.AddRat(new(big.Rat), proofarith.MulRat(new(big.Rat), s, u), proofarith.MulRat(new(big.Rat), c, v)),
	}
}

// SourceConnectorOf reads the true source connector of one wall cell from the section records.
func SourceConnectorOf(verts []r3.Vec, tris [][3]int, segments [][]int,
	profiles [2]momentinput.Profile, ex r3.Vec, cell, root int) (Connector, bool) {
	n := len(verts) / 2
	near, far := 0, 1
	segment := 0
	if cell != 0 {
		near, far = root, root-1
		segment = 4
	}
	if segments[0][cell] != segment {
		return Connector{}, false
	}
	var shared []int
	for _, v := range tris[2*cell] {
		for _, w := range tris[2*cell+1] {
			if v == w {
				shared = append(shared, v)
			}
		}
	}
	if len(shared) != 2 {
		return Connector{}, false
	}
	diagonal := [2]int{-1, -1}
	for _, index := range shared {
		switch index {
		case near, far:
			diagonal[0] = index
		case n + near, n + far:
			diagonal[1] = index - n
		default:
			return Connector{}, false
		}
	}
	if diagonal[0] < 0 || diagonal[1] < 0 {
		return Connector{}, false
	}
	var result Connector
	for cap, profile := range profiles {
		line, ok := profile.Outer.Segments[segment].(sectionrecord.LineSeg)
		if !ok || !proofbound.FiniteVec(r3.Vec{X: line.Start.U, Y: line.Start.V}) ||
			!proofbound.FiniteVec(r3.Vec{X: line.End.U, Y: line.End.V}) {
			return Connector{}, false
		}
		points := [2]sectionrecord.Point2{line.Start, line.End}
		if cell != 0 {
			points[0], points[1] = points[1], points[0]
		}
		result.a[cap] = SourcePoint(ex, points[0])
		result.f[cap] = SourcePoint(ex, points[1])
		switch diagonal[cap] {
		case near:
			result.d[cap] = result.a[cap]
		case far:
			result.d[cap] = result.f[cap]
		default:
			return Connector{}, false
		}
	}
	return result, true
}

// SourceDot returns the exact dot product of a and b.
func SourceDot(a, b XY) *big.Rat {
	return proofarith.AddRat(new(big.Rat), proofarith.MulRat(new(big.Rat), a.X, b.X), proofarith.MulRat(new(big.Rat), a.Y, b.Y))
}

// sourceSub returns a - b.
func sourceSub(a, b XY) XY {
	return XY{X: proofarith.SubRat(new(big.Rat), a.X, b.X), Y: proofarith.SubRat(new(big.Rat), a.Y, b.Y)}
}

// sourceLerp returns the exact point a + t*(b-a).
func sourceLerp(a, b XY, t *big.Rat) XY {
	d := sourceSub(b, a)
	return XY{
		X: proofarith.AddRat(new(big.Rat), a.X, proofarith.MulRat(new(big.Rat), t, d.X)),
		Y: proofarith.AddRat(new(big.Rat), a.Y, proofarith.MulRat(new(big.Rat), t, d.Y)),
	}
}

// Nonnegative quadratic Bernstein coefficients prove that each actual
// two-triangle source slice moves radially outward. An interior crossing is
// transverse because a nonzero segment has strictly increasing squared
// radius after its starting point.
func (c Connector) RadialMonotone() bool {
	for _, pair := range [][2][2]XY{{c.a, c.d}, {c.d, c.f}} {
		u0, u1 := pair[0][0], pair[0][1]
		d0 := sourceSub(pair[1][0], u0)
		d1 := sourceSub(pair[1][1], u1)
		b0 := SourceDot(u0, d0)
		b1 := proofarith.AddRat(new(big.Rat), SourceDot(u0, d1), SourceDot(u1, d0))
		b2 := SourceDot(u1, d1)
		if b0.Sign() < 0 || b1.Sign() < 0 || b2.Sign() < 0 {
			return false
		}
	}
	return true
}

// sourceRadiusMin2 returns the exact minimum squared radius over segment ab.
func sourceRadiusMin2(a, b XY) *big.Rat {
	d := sourceSub(b, a)
	den := SourceDot(d, d)
	if den.Sign() == 0 {
		return SourceDot(a, a)
	}
	t := proofarith.QuoRat(new(big.Rat), new(big.Rat).Neg(SourceDot(a, d)), den)
	if t.Sign() < 0 {
		t.SetInt64(0)
	} else if t.Cmp(big.NewRat(1, 1)) > 0 {
		t.SetInt64(1)
	}
	p := sourceLerp(a, b, t)
	return SourceDot(p, p)
}

func (c Connector) FarOutside(radius *big.Rat) bool {
	r2 := proofarith.MulRat(new(big.Rat), radius, radius)
	return sourceRadiusMin2(c.f[0], c.f[1]).Cmp(r2) > 0
}

// A positive half-plane containing every matched source-boundary point
// leaves its opposite circle ray outside the bounded tooth solid. This
// witnesses the exterior gap used to choose the occupied arc between the
// two connector crossings.
func SourceGapAxis(verts []r3.Vec, root int, facetDeparture float64) (XY, bool) {
	n := len(verts) / 2
	axis := XY{X: new(big.Rat), Y: new(big.Rat)}
	for _, index := range []int{0, root, n, n + root} {
		p := verts[index]
		proofarith.AddRat(axis.X, axis.X, proofarith.FloatRat(p.X))
		proofarith.AddRat(axis.Y, axis.Y, proofarith.FloatRat(p.Y))
	}
	norm2 := SourceDot(axis, axis)
	if norm2.Sign() == 0 {
		return XY{}, false
	}
	delta2 := proofarith.MulRat(new(big.Rat), proofarith.FloatRat(facetDeparture),
		proofarith.FloatRat(facetDeparture))
	margin2 := proofarith.MulRat(new(big.Rat), delta2, norm2)
	for _, p := range verts {
		dot := proofarith.AddRat(new(big.Rat), proofarith.MulRat(new(big.Rat), axis.X, proofarith.FloatRat(p.X)),
			proofarith.MulRat(new(big.Rat), axis.Y, proofarith.FloatRat(p.Y)))
		if dot.Sign() <= 0 || proofarith.MulRat(new(big.Rat), dot, dot).Cmp(margin2) <= 0 {
			return XY{}, false
		}
	}
	return axis, true
}

func sourceBoxOf(points ...XY) XYBox {
	box := XYBox{
		lo: XY{X: new(big.Rat).Set(points[0].X), Y: new(big.Rat).Set(points[0].Y)},
		hi: XY{X: new(big.Rat).Set(points[0].X), Y: new(big.Rat).Set(points[0].Y)},
	}
	for _, p := range points[1:] {
		if p.X.Cmp(box.lo.X) < 0 {
			box.lo.X.Set(p.X)
		}
		if p.Y.Cmp(box.lo.Y) < 0 {
			box.lo.Y.Set(p.Y)
		}
		if p.X.Cmp(box.hi.X) > 0 {
			box.hi.X.Set(p.X)
		}
		if p.Y.Cmp(box.hi.Y) > 0 {
			box.hi.Y.Set(p.Y)
		}
	}
	return box
}

func sourceUnionBox(a, b XYBox) XYBox {
	return sourceBoxOf(a.lo, a.hi, b.lo, b.hi)
}

// sourceRay is one connector half at a fixed height parameter t:
// the point at wall parameter u is origin + u*direction, the same rational
// point(u, t) forms from its three nested lerps.
type sourceRay struct{ origin, direction XY }

// rays fixes t for both halves (a->d, then d->f) of the connector.
func (c Connector) rays(t *big.Rat) [2]sourceRay {
	var out [2]sourceRay
	for half, pair := range [2][2][2]XY{{c.a, c.d}, {c.d, c.f}} {
		lo, hi := pair[0], pair[1]
		d0 := sourceSub(hi[0], lo[0])
		d1 := sourceSub(hi[1], lo[1])
		out[half] = sourceRay{
			origin:    sourceLerp(lo[0], lo[1], t),
			direction: sourceLerp(d0, d1, t),
		}
	}
	return out
}

// sourcePointAt is point(s, t) read off rays(t).
func sourcePointAt(rays [2]sourceRay, s *big.Rat) XY {
	u := s
	ray := rays[0]
	if s.Cmp(big.NewRat(1, 1)) > 0 {
		ray = rays[1]
		u = proofarith.SubRat(new(big.Rat), s, big.NewRat(1, 1))
	}
	return XY{
		X: proofarith.AddRat(new(big.Rat), ray.origin.X, proofarith.MulRat(new(big.Rat), u, ray.direction.X)),
		Y: proofarith.AddRat(new(big.Rat), ray.origin.Y, proofarith.MulRat(new(big.Rat), u, ray.direction.Y)),
	}
}

// sourceIntRay is a connector half at fixed height parameter t
// with integer coordinates over the shared positive denominator den:
// point(u) = (origin + u*direction) / den.
type sourceIntRay struct {
	ox, oy, dx, dy *big.Int
	den            *big.Int
}

func lcm(a, b *big.Int) *big.Int {
	g := new(big.Int).GCD(nil, nil, a, b)
	return new(big.Int).Mul(a, new(big.Int).Quo(b, g))
}

func sourceIntRayOf(ray sourceRay) sourceIntRay {
	den := lcm(lcm(new(big.Int).Set(ray.origin.X.Denom()), new(big.Int).Set(ray.origin.Y.Denom())),
		lcm(new(big.Int).Set(ray.direction.X.Denom()), new(big.Int).Set(ray.direction.Y.Denom())))
	scale := func(q *big.Rat) *big.Int {
		return new(big.Int).Mul(q.Num(), new(big.Int).Quo(den, q.Denom()))
	}
	return sourceIntRay{ox: scale(ray.origin.X), oy: scale(ray.origin.Y),
		dx: scale(ray.direction.X), dy: scale(ray.direction.Y), den: den}
}

// pointInt is the point at s = m / 2^k over denominator den*2^k.
func (r sourceIntRay) pointInt(m, pow *big.Int) (*big.Int, *big.Int) {
	x := new(big.Int).Mul(r.ox, pow)
	x.Add(x, new(big.Int).Mul(m, r.dx))
	y := new(big.Int).Mul(r.oy, pow)
	y.Add(y, new(big.Int).Mul(m, r.dy))
	return x, y
}

// crossingBox bisects the wall parameter in integers: low and high are
// L/2^k and H/2^k, every comparison is an exact comparison of rationals
// cross-multiplied over positive denominators, and the box is formed from the
// final dyadic bounds with the exact rational lerps.
func (c Connector) crossingBox(tLo, tHi, radius2 *big.Rat) XYBox {
	raysLo, raysHi := c.rays(tLo), c.rays(tHi)
	var intLo, intHi [2]sourceIntRay
	for half := range 2 {
		intLo[half] = sourceIntRayOf(raysLo[half])
		intHi[half] = sourceIntRayOf(raysHi[half])
	}
	rn, rd := new(big.Int).Set(radius2.Num()), new(big.Int).Set(radius2.Denom())
	sameT := tLo.Cmp(tHi) == 0
	// low = L/2^k, high = H/2^k.
	low, high := big.NewInt(0), big.NewInt(2)
	pow := big.NewInt(1)
	point := func(rays *[2]sourceIntRay, m, pow *big.Int) (*big.Int, *big.Int, *big.Int) {
		ray := &rays[0]
		u := m
		if m.Cmp(pow) > 0 {
			ray = &rays[1]
			u = new(big.Int).Sub(m, pow)
		}
		x, y := ray.pointInt(u, pow)
		return x, y, new(big.Int).Mul(ray.den, pow)
	}
	norm2 := func(x, y *big.Int) *big.Int {
		return new(big.Int).Add(new(big.Int).Mul(x, x), new(big.Int).Mul(y, y))
	}
	// cmpFrac compares n/d (d > 0) against radius2: sign of n*rd - rn*d.
	cmpFrac := func(n, d *big.Int) int {
		return new(big.Int).Mul(n, rd).Cmp(new(big.Int).Mul(rn, d))
	}
	for range 52 {
		// mid = (low+high)/2 = (L+H)/2^(k+1).
		low.Lsh(low, 1)
		high.Lsh(high, 1)
		pow.Lsh(pow, 1)
		mid := new(big.Int).Add(low, high)
		mid.Rsh(mid, 1)
		x0, y0, e0 := point(&intLo, mid, pow)
		x1, y1, e1 := point(&intHi, mid, pow)
		q0, q1 := norm2(x0, y0), norm2(x1, y1)
		e0sq, e1sq := new(big.Int).Mul(e0, e0), new(big.Int).Mul(e1, e1)
		if cmpFrac(q0, e0sq) < 0 && cmpFrac(q1, e1sq) < 0 {
			low = mid
			continue
		}
		// Common denominator E = e0*e1 for the segment p0 -> p1.
		px0, py0 := new(big.Int).Mul(x0, e1), new(big.Int).Mul(y0, e1)
		px1, py1 := new(big.Int).Mul(x1, e0), new(big.Int).Mul(y1, e0)
		esq := new(big.Int).Mul(e0, e1)
		esq.Mul(esq, esq)
		dx, dy := new(big.Int).Sub(px1, px0), new(big.Int).Sub(py1, py0)
		den := norm2(dx, dy)
		num := new(big.Int).Add(new(big.Int).Mul(px0, dx), new(big.Int).Mul(py0, dy))
		num.Neg(num)
		var minNum, minDen *big.Int
		switch {
		case den.Sign() == 0 || num.Sign() <= 0:
			minNum, minDen = norm2(px0, py0), esq
		case num.Cmp(den) >= 0:
			minNum, minDen = norm2(px1, py1), esq
		default:
			minNum = new(big.Int).Sub(new(big.Int).Mul(norm2(px0, py0), den), new(big.Int).Mul(num, num))
			minDen = new(big.Int).Mul(den, esq)
		}
		if cmpFrac(minNum, minDen) > 0 {
			high = mid
			continue
		}
		if sameT && cmpFrac(q0, e0sq) == 0 {
			low, high = mid, new(big.Int).Set(mid)
		}
		break
	}
	lowRat := new(big.Rat).SetFrac(low, pow)
	highRat := new(big.Rat).SetFrac(high, pow)
	box := sourceBoxOf(
		sourcePointAt(raysLo, lowRat), sourcePointAt(raysHi, lowRat),
		sourcePointAt(raysLo, highRat), sourcePointAt(raysHi, highRat))
	if lowRat.Cmp(big.NewRat(1, 1)) < 0 && highRat.Cmp(big.NewRat(1, 1)) > 0 {
		middle := big.NewRat(1, 1)
		box = sourceUnionBox(box, sourceBoxOf(
			sourcePointAt(raysLo, middle), sourcePointAt(raysHi, middle)))
	}
	return box
}

func sourceProductRange(aLo, aHi, bLo, bHi *big.Rat) (*big.Rat, *big.Rat) {
	products := [4]*big.Rat{
		proofarith.MulRat(new(big.Rat), aLo, bLo), proofarith.MulRat(new(big.Rat), aLo, bHi),
		proofarith.MulRat(new(big.Rat), aHi, bLo), proofarith.MulRat(new(big.Rat), aHi, bHi),
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

func sourceCrossLower(a, b XYBox) *big.Rat {
	xyLo, _ := sourceProductRange(a.lo.X, a.hi.X, b.lo.Y, b.hi.Y)
	_, yxHi := sourceProductRange(a.lo.Y, a.hi.Y, b.lo.X, b.hi.X)
	return proofarith.SubRat(new(big.Rat), xyLo, yxHi)
}

func sourceDotLower(axis XY, box XYBox) *big.Rat {
	xLo, _ := sourceProductRange(axis.X, axis.X, box.lo.X, box.hi.X)
	yLo, _ := sourceProductRange(axis.Y, axis.Y, box.lo.Y, box.hi.Y)
	return proofarith.AddRat(new(big.Rat), xLo, yLo)
}

// CrossingBox encloses both true source connector/circle crossings for every
// height in the rational closed interval. A returned pair has positive CCW
// order and lies in the source tooth's positive gap-witness half-plane. A
// wide interval can refuse; the caller may subdivide it with a fixed budget.
// Cap planes are included only as limiting crossing positions. The occupancy
// claim applies to open heights safeBottom < z < safeTop.
func (r SourceCircle) CrossingBox(zLo, zHi *big.Rat) ([2]XYBox, error) {
	var boxes [2]XYBox
	if zLo == nil || zHi == nil || zLo.Cmp(r.SafeBottom) < 0 || zHi.Cmp(r.SafeTop) > 0 ||
		zLo.Cmp(zHi) > 0 {
		return boxes, fmt.Errorf("%w: source circle query leaves its strict root interval", decaderr.ErrUnsupported)
	}
	height := proofarith.SubRat(new(big.Rat), r.Z1, r.Z0)
	tLo := new(big.Rat).Quo(proofarith.SubRat(new(big.Rat), zLo, r.Z0), height)
	tHi := new(big.Rat).Quo(proofarith.SubRat(new(big.Rat), zHi, r.Z0), height)
	r2 := proofarith.MulRat(new(big.Rat), r.Radius, r.Radius)
	for i, connector := range r.Connector {
		boxes[i] = connector.crossingBox(tLo, tHi, r2)
		if sourceDotLower(r.GapAxis, boxes[i]).Sign() <= 0 {
			return [2]XYBox{},
				fmt.Errorf("%w: source crossing leaves its certified tooth half-plane", decaderr.ErrUnsupported)
		}
	}
	if sourceCrossLower(boxes[0], boxes[1]).Sign() <= 0 {
		return [2]XYBox{},
			fmt.Errorf("%w: source crossing interval loses tooth angular order", decaderr.ErrUnsupported)
	}
	return boxes, nil
}

// OrderedOverSafeInterval proves the same source crossing order throughout
// the open root-safe height. Each closed rational slab retains one positive
// order and the slabs cover the interval without a gap.
func (r SourceCircle) OrderedOverSafeInterval() bool {
	const slabs = 256
	span := proofarith.SubRat(new(big.Rat), r.SafeTop, r.SafeBottom)
	if span.Sign() <= 0 {
		return false
	}
	check := func(i int) bool {
		lo := proofarith.AddRat(new(big.Rat), r.SafeBottom,
			new(big.Rat).Quo(proofarith.MulRat(new(big.Rat), span, big.NewRat(int64(i), 1)), big.NewRat(slabs, 1)))
		hi := proofarith.AddRat(new(big.Rat), r.SafeBottom,
			new(big.Rat).Quo(proofarith.MulRat(new(big.Rat), span, big.NewRat(int64(i+1), 1)), big.NewRat(slabs, 1)))
		_, err := r.CrossingBox(lo, hi)
		return err == nil
	}
	workers := min(runtime.GOMAXPROCS(0), 4)
	if workers == 1 {
		for i := range slabs {
			if !check(i) {
				return false
			}
		}
		return true
	}
	// Each slab reads the immutable source rationals and owns its box proof.
	var failed atomic.Bool
	var group sync.WaitGroup
	group.Add(workers)
	for worker := range workers {
		go func() {
			defer group.Done()
			for i := worker; i < slabs; i += workers {
				if failed.Load() {
					return
				}
				if !check(i) {
					failed.Store(true)
					return
				}
			}
		}()
	}
	group.Wait()
	return !failed.Load()
}

// ContainsTriangleSector proves that every projected point of an exact
// triangle lies strictly between both source connector/circle crossings at
// every height in [zLo,zHi]. The entire triangle must lie in that axial
// interval. The result concerns angle only: it does not prove radial tooth
// occupancy or clearance from another source wall. Endpoints on z0 or
// safeBottom and safeTop are limiting coordinates; cap planes and both collars are outside
// the occupancy claim.
func (r SourceCircle) ContainsTriangleSector(zLo, zHi *big.Rat,
	tri [3]proofarith.Xpt) bool {
	boxes, err := r.CrossingBox(zLo, zHi)
	if err != nil {
		return false
	}
	return triangleInSector(boxes, zLo, zHi, tri)
}

// triangleInSector is ContainsTriangleSector after its CrossingBox(zLo, zHi)
// call returned boxes without an error.
func triangleInSector(boxes [2]XYBox, zLo, zHi *big.Rat, tri [3]proofarith.Xpt) bool {
	for _, p := range tri {
		if p.X == nil || p.Y == nil || p.Z == nil || p.W == nil || p.W.Sign() <= 0 {
			return false
		}
		z := new(big.Rat).SetFrac(p.Z, p.W)
		if z.Cmp(zLo) < 0 || z.Cmp(zHi) > 0 {
			return false
		}
		xy := XY{
			X: new(big.Rat).SetFrac(p.X, p.W),
			Y: new(big.Rat).SetFrac(p.Y, p.W),
		}
		point := sourceBoxOf(xy)
		if sourceCrossLower(boxes[0], point).Sign() <= 0 ||
			sourceCrossLower(point, boxes[1]).Sign() <= 0 {
			return false
		}
	}
	return true
}
