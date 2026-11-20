package booleaneval

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

type stationPoint struct{ x, z *big.Rat }

func stationR(x float64) *big.Rat       { return new(big.Rat).SetFloat64(x) }
func stationAdd(a, b *big.Rat) *big.Rat { return new(big.Rat).Add(a, b) }
func stationSub(a, b *big.Rat) *big.Rat { return new(big.Rat).Sub(a, b) }
func stationMul(a, b *big.Rat) *big.Rat { return new(big.Rat).Mul(a, b) }
func stationCoords(p sectionrecord.Point2) stationPoint {
	return stationPoint{stationR(p.U), stationR(p.V)}
}
func (p stationPoint) radius2() *big.Rat {
	return stationAdd(stationMul(p.x, p.x), stationMul(p.z, p.z))
}
func (p stationPoint) minus(q stationPoint) stationPoint {
	return stationPoint{stationSub(p.x, q.x), stationSub(p.z, q.z)}
}
func (p stationPoint) along(d stationPoint, t *big.Rat) stationPoint {
	return stationPoint{stationAdd(p.x, stationMul(d.x, t)), stationAdd(p.z, stationMul(d.z, t))}
}
func stationOrdered(a, b *big.Rat) (*big.Rat, *big.Rat) {
	if a.Cmp(b) <= 0 {
		return a, b
	}
	return b, a
}
func stationLevels(p prismrecord.Payload) (*big.Rat, *big.Rat) {
	o := stationR(p.Frame.Origin().Y)
	return stationOrdered(stationSub(o, p.DenotedZ0()), stationSub(o, p.DenotedZ1()))
}

func stationLevelError(p prismrecord.Payload) float64 {
	lo, hi := stationLevels(p)
	h0, h1 := stationOrdered(stationR(p.Point(0, 0, p.Z0).Y), stationR(p.Point(0, 0, p.Z1).Y))
	d0, d1 := stationSub(h0, lo), stationSub(h1, hi)
	return max(proofbound.RatFloatUp(d0.Abs(d0)), proofbound.RatFloatUp(d1.Abs(d1)))
}

func stationRole(ctx context.Context, b *topo.Body, role string, budget *proofbound.ReplayBudget) (*topo.Face, error) {
	var match *topo.Face
	for _, f := range topobuild.Faces(b) {
		if len(f.Origins) != 1 || f.Origins[0].Role != role {
			continue
		}
		if match != nil {
			return nil, errStationsInapplicable
		}
		match = f
	}
	if match == nil {
		return nil, errStationsInapplicable
	}
	ok, err := b.ReplayPrismBuiltFace(ctx, match, budget)
	if !ok || err != nil {
		return nil, err
	}
	return match, nil
}

// A remote edge cannot introduce another intersection hidden by the paired
// chording. For an arc use its entire support circle; for a line use the
// closest point of the finite segment.
func remoteStationArc(s sectionrecord.ArcSeg, r2 *big.Rat) bool {
	center := stationCoords(s.Center)
	rs := stationCoords(s.Start).minus(center).radius2()
	re := stationCoords(s.End).minus(center).radius2()
	_, radius := stationOrdered(rs, re)
	d := stationSub(stationSub(center.radius2(), r2), radius)
	return d.Sign() > 0 && stationMul(d, d).Cmp(stationMul(stationR(4), stationMul(r2, radius))) > 0
}
func stationLineOutside(p, q stationPoint, r2 *big.Rat) bool {
	v := q.minus(p)
	den := v.radius2()
	if den.Sign() == 0 {
		return false
	}
	t := new(big.Rat).Neg(stationAdd(stationMul(p.x, v.x), stationMul(p.z, v.z)))
	if t.Sign() <= 0 {
		return p.radius2().Cmp(r2) > 0
	}
	if t.Cmp(den) >= 0 {
		return q.radius2().Cmp(r2) > 0
	}
	t.Quo(t, den)
	return p.along(v, t).radius2().Cmp(r2) > 0
}

// A segment with one end strictly inside and one strictly outside a disk
// has one transverse crossing. Bisection keeps opposite signs; both end
// coordinates therefore enclose that root even when its exact value is
// irrational. A wider interval can only cost precision.
func stationRoot(p, q stationPoint, r2 *big.Rat, origin r3.Vec) meshbool.AxialRim {
	lo, hi := stationR(0), stationR(1)
	d := q.minus(p)
	sign := p.radius2().Cmp(r2)
	for range 64 {
		mid := stationMul(stationAdd(lo, hi), stationR(.5))
		if p.along(d, mid).radius2().Cmp(r2) == sign {
			lo = mid
		} else {
			hi = mid
		}
	}
	a, b := p.along(d, lo), p.along(d, hi)
	xl, xh := stationOrdered(stationAdd(a.x, stationR(origin.X)), stationAdd(b.x, stationR(origin.X)))
	zl, zh := stationOrdered(stationAdd(a.z, stationR(origin.Z)), stationAdd(b.z, stationR(origin.Z)))
	return meshbool.AxialRim{Xlo: xl, Xhi: xh, Zlo: zl, Zhi: zh}
}

func stationRoots(ctx context.Context, plate, disk *topo.Body, p, q prismrecord.Payload, idx int, arc sectionrecord.ArcSeg, radius float64, budget *proofbound.ReplayBudget) ([]meshbool.AxialRim, bool, error) {
	r2 := stationMul(stationR(radius), stationR(radius))
	arcRadius := stationCoords(arc.Start).radius2()
	lower := stationR(math.Nextafter(radius, 0))
	if arc.Center != (sectionrecord.Point2{}) || arcRadius.Cmp(stationCoords(arc.End).radius2()) != 0 || arcRadius.Cmp(r2) >= 0 || arcRadius.Cmp(stationMul(lower, lower)) <= 0 {
		return nil, false, nil
	}
	y0, y1 := stationLevels(p)
	yy0, yy1 := stationLevels(q)
	if y0.Cmp(yy0) >= 0 || yy0.Cmp(y1) >= 0 || y1.Cmp(yy1) >= 0 {
		return nil, false, nil
	}
	ph0, ph1 := p.Point(0, 0, p.Z0).Y, p.Point(0, 0, p.Z1).Y
	qh0, qh1 := q.Point(0, 0, q.Z0).Y, q.Point(0, 0, q.Z1).Y
	if min(ph0, ph1) >= min(qh0, qh1) || min(qh0, qh1) >= max(ph0, ph1) || max(ph0, ph1) >= max(qh0, qh1) {
		return nil, false, nil
	}
	wall, err := stationRole(ctx, disk, "side(0,0)", budget)
	if wall == nil || err != nil {
		return nil, false, err
	}
	var rims []meshbool.AxialRim
	segs := p.Profile.Outer.Segments
	for i, seg := range segs {
		if err := budget.ReserveBundle(ctx, 4096, 16, 64<<10); err != nil {
			return nil, false, err
		}
		switch s := seg.(type) {
		case sectionrecord.ArcSeg:
			if i != idx && !remoteStationArc(s, r2) {
				return nil, false, nil
			}
		case sectionrecord.LineSeg:
			a, b := stationCoords(s.Start), stationCoords(s.End)
			if a.radius2().Cmp(r2)*b.radius2().Cmp(r2) != -1 {
				if !stationLineOutside(a, b, r2) {
					return nil, false, nil
				}
				continue
			}
			adjacent := (i+1)%len(segs) == idx || (idx+1)%len(segs) == i
			if !adjacent {
				return nil, false, nil
			}
			root := stationRoot(a, b, r2, p.Frame.Origin())
			root.Side, err = stationRole(ctx, plate, fmt.Sprintf("side(0,%d)", i), budget)
			if root.Side == nil || err != nil {
				return nil, false, err
			}
			root.Disk, root.Ylo, root.Yhi = wall, yy0, y1
			root.LevelError = max(stationLevelError(p), stationLevelError(q))
			rims = append(rims, root)
		default:
			return nil, false, nil
		}
	}
	return rims, len(rims) == 2, nil
}

// stationOrient uses the allocation-free exact expansion when supported. A
// binary64 coordinate has at most 1074 fractional bits, bounding the exact
// differences and products used by its rational fallback.
func stationOrient(ctx context.Context, budget *proofbound.ReplayBudget, errp *error) func(r3.Vec, r3.Vec, r3.Vec) (int, bool) {
	return func(a, b, c r3.Vec) (int, bool) {
		if sign, ok := proof.Orient2SignExpansion(a.X, a.Z, b.X, b.Z, c.X, c.Z); ok {
			return sign, true
		}
		if *errp != nil {
			return 0, false
		}
		if err := budget.ReserveBundle(ctx, 128, 24, 32768); err != nil {
			*errp = err
			return 0, false
		}
		x1 := stationSub(stationR(b.X), stationR(a.X))
		z1 := stationSub(stationR(b.Z), stationR(a.Z))
		x2 := stationSub(stationR(c.X), stationR(a.X))
		z2 := stationSub(stationR(c.Z), stationR(a.Z))
		return stationSub(stationMul(x1, z2), stationMul(z1, x2)).Sign(), true
	}
}

// The two held rings must cross once at each of the same two native walls.
// Reject equal vertices and tangencies. The common arc starts inside the
// disk, so its connected chord walk stays inside it all the way to its end.
func stationHeldCrossings(ctx context.Context, a, b *topo.Mesh, arc *topo.Face, rims []meshbool.AxialRim, meter *proofbound.ReplayBudget) (bool, error) {
	var orientErr error
	na, nb := len(a.Vertices)/2, len(b.Vertices)/2
	if len(a.Vertices) != na*2 || len(b.Vertices) != nb*2 || len(a.Source) < 2*na || len(rims) != 2 {
		return false, orientErr
	}
	count := [2]int{}
	inside := false

	orient := stationOrient(ctx, meter, &orientErr)
	for i := range na {
		if err := meter.SpendWork(ctx, uint64(nb)); err != nil {
			return false, err
		}
		p, q := a.Vertices[2*i], a.Vertices[2*((i+1)%na)]
		face := a.Source[2*i]
		checkInside := face == arc && !inside
		for j := range nb {
			r, s := b.Vertices[2*j], b.Vertices[2*((j+1)%nb)]
			if i == 0 {
				turn, ok := orient(r, s, b.Vertices[2*((j+2)%nb)])
				if !ok || turn <= 0 {
					return false, orientErr
				}
			}
			if checkInside {
				side, ok := orient(r, s, p)
				if !ok || side <= 0 {
					return false, orientErr
				}
			}
			if max(p.X, q.X) < min(r.X, s.X) || max(r.X, s.X) < min(p.X, q.X) || max(p.Z, q.Z) < min(r.Z, s.Z) || max(r.Z, s.Z) < min(p.Z, q.Z) {
				continue
			}
			if err := meter.SpendWork(ctx, 4); err != nil {
				return false, err
			}
			v, ok := orient(p, q, r)
			w, ok2 := orient(p, q, s)
			x, ok3 := orient(r, s, p)
			y, ok4 := orient(r, s, q)
			if !ok || !ok2 || !ok3 || !ok4 || v == 0 || w == 0 || x == 0 || y == 0 {
				return false, orientErr
			}
			if v == w || x == y {
				continue
			}
			switch face {
			case rims[0].Side:
				count[0]++
			case rims[1].Side:
				count[1]++
			default:
				return false, orientErr
			}
		}
		inside = inside || checkInside
	}
	return inside && count == [2]int{1, 1}, nil
}

// stationEmbedded verifies the simple projected boundary and oriented cap
// triangulations. A consistently oriented triangulated disk bounded by a
// simple ring covers each interior point once by its boundary winding.
// Its two copies at distinct heights and their vertical walls are embedded.
func stationEmbedded(ctx context.Context, m *topo.Mesh, budget *proofbound.ReplayBudget) (bool, error) {
	var orientErr error
	n := len(m.Vertices) / 2
	if n < 3 || len(m.Vertices) != 2*n || len(m.Triangles) != 4*n-4 {
		return false, orientErr
	}
	point := func(i int) r3.Vec { return m.Vertices[2*(i%n)] }
	orient := stationOrient(ctx, budget, &orientErr)
	if m.Vertices[0].Y == m.Vertices[1].Y {
		return false, orientErr
	}
	for i := range n {
		if err := budget.SpendWork(ctx, uint64(n+4)); err != nil {
			return false, err
		}
		a, b := point(i), point(i+1)
		h := m.Vertices[2*i+1]
		if a.Y != m.Vertices[0].Y || h.Y != m.Vertices[1].Y || a.X != h.X || a.Z != h.Z || a == b {
			return false, orientErr
		}
		c := point(i + 2)
		turn, ok := orient(a, b, c)
		if !ok {
			return false, orientErr
		}
		if turn == 0 && (b.X < min(a.X, c.X) || b.X > max(a.X, c.X) || b.Z < min(a.Z, c.Z) || b.Z > max(a.Z, c.Z)) {
			return false, nil
		}
		for j := i + 2; j < n; j++ {
			if i == 0 && j == n-1 {
				continue
			}
			c, d := point(j), point(j+1)
			if max(a.X, b.X) < min(c.X, d.X) || max(c.X, d.X) < min(a.X, b.X) || max(a.Z, b.Z) < min(c.Z, d.Z) || max(c.Z, d.Z) < min(a.Z, b.Z) {
				continue
			}
			s1, ok1 := orient(a, b, c)
			s2, ok2 := orient(a, b, d)
			if ok1 && ok2 && s1 != 0 && s1 == s2 {
				continue
			}
			s3, ok3 := orient(c, d, a)
			s4, ok4 := orient(c, d, b)
			if !ok3 || !ok4 || s3 == 0 || s3 != s4 {
				return false, orientErr
			}
		}
	}
	var sign [2]int
	for _, t := range m.Triangles[2*n:] {
		end := t[0] % 2
		if t[1]%2 != end || t[2]%2 != end {
			return false, orientErr
		}
		s, ok := orient(m.Vertices[t[0]], m.Vertices[t[1]], m.Vertices[t[2]])
		if !ok || s == 0 || sign[end] != 0 && sign[end] != s {
			return false, orientErr
		}
		sign[end] = s
	}
	return orientErr == nil && sign[0]*sign[1] == -1, orientErr
}
