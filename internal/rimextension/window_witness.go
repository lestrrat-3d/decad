package rimextension

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// WindowWitnesses locates points on authenticated held facets. The complete
// displacement ball of each chosen point must remain in its transverse cell.
// Cap interior lies on the positive Y side for materialY=1 and the negative
// side for materialY=-1.
func WindowWitnesses(ctx context.Context, mesh Mesh, beta []float64, g Groups, w SourceWindow, center r3.Vec, radius, capY float64, outwardZ, materialY int, meter *proofbound.ReplayBudget) (bool, error) {
	if meter == nil || len(mesh.Vertices) != len(beta) || len(mesh.Triangles) != len(mesh.Sources) || g.Cap == g.Boss ||
		(outwardZ != 1 && outwardZ != -1) || (materialY != 1 && materialY != -1) {
		return false, nil
	}
	if err := meter.ReserveBytes(ctx, 32768); err != nil {
		return false, err
	}
	x0, x1, y0, y1 := scalar(w.XMin), scalar(w.XMax), scalar(w.YMin), scalar(w.YMax)
	a, b, z, v := scalar(w.Side0.X), scalar(w.Side1.X), scalar(w.Side0.Z), scalar(w.Side1.Z)
	cx, cz, r, cy := scalar(center.X), scalar(center.Z), scalar(radius), scalar(capY)
	ol, oh := scalar(w.OffsetMin), scalar(w.OffsetMax)
	if x0 == nil || x1 == nil || y0 == nil || y1 == nil ||
		a == nil || b == nil || z == nil || v == nil || cx == nil || cz == nil || r == nil || cy == nil ||
		ol == nil || oh == nil || a.Cmp(b) == 0 || r.Sign() <= 0 || x0.Cmp(x1) >= 0 || cy.Cmp(y0) <= 0 || cy.Cmp(y1) >= 0 || ol.Cmp(oh) >= 0 {
		return false, nil
	}
	m := divide(minus(v, z), minus(b, a))
	amp := plus(scalar(1), new(big.Rat).Abs(m))
	xProbe := plus(x0, divide(minus(x1, x0), scalar(8)))
	yEdge := y0
	if materialY < 0 {
		yEdge = y1
	}
	yProbe := divide(plus(yEdge, cy), scalar(2))
	r2 := times(r, r)
	xHeld, _ := xProbe.Float64()
	yHeld, _ := yProbe.Float64()
	haveCap, haveBoss := false, false
	for i, t := range mesh.Triangles {
		if err := meter.SpendWork(ctx, 1); err != nil {
			return false, err
		}
		capFacet := mesh.Sources[i] == g.Cap
		if (!capFacet || haveCap) && (mesh.Sources[i] != g.Boss || haveBoss) {
			continue
		}
		axis, station, held := 0, xProbe, xHeld
		if !capFacet {
			axis, station, held = 1, yProbe, yHeld
		}
		low, high := false, false
		for _, id := range t {
			if id < 0 || id >= len(beta) {
				return false, nil
			}
			coord := mesh.Vertices[id].X
			if axis == 1 {
				coord = mesh.Vertices[id].Y
			}
			// Include nearest-float equality on both sides of the broad phase.
			low = low || coord <= held
			high = high || coord >= held
		}
		if !low || !high {
			continue
		}
		if err := meter.ReserveBytes(ctx, 262144); err != nil {
			return false, err
		}
		if err := meter.SpendWork(ctx, 192); err != nil {
			return false, err
		}
		var corners [3]point
		var e big.Rat
		for j, id := range t {
			corners[j] = lift(mesh.Vertices[id])
			bound := scalar(beta[id])
			if bound == nil || bound.Sign() < 0 || corners[j][0] == nil || corners[j][1] == nil || corners[j][2] == nil {
				return false, nil
			}
			if e.Cmp(bound) < 0 {
				e.Set(bound)
			}
		}
		var endpoints [6]point
		n := 0
		for j, p := range corners {
			q := corners[(j+1)%3]
			s1, s2 := p[axis].Cmp(station), q[axis].Cmp(station)
			if s1 == 0 {
				endpoints[n] = p
				n++
			}
			if s1*s2 < 0 {
				endpoints[n] = interpolate(p, q, divide(minus(station, p[axis]), minus(q[axis], p[axis])))
				n++
			}
		}
		for j := range n {
			q := endpoints[j]
			for k := j; k < n; k++ {
				p := q
				if k != j {
					p = interpolate(q, endpoints[k], scalar(.5))
				}
				in := func(q, lo, hi, err *big.Rat) bool { return minus(q, err).Cmp(lo) > 0 && plus(q, err).Cmp(hi) < 0 }
				if !in(p[0], x0, x1, &e) || !in(p[1], y0, y1, &e) {
					continue
				}
				off := minus(minus(p[2], z), times(m, minus(p[0], a)))
				de := times(&e, amp)
				if !in(off, ol, oh, de) {
					continue
				}
				zdist := signed(minus(p[2], cz), outwardZ)
				if zdist.Cmp(&e) <= 0 {
					continue
				}
				if !capFacet {
					if signed(minus(cy, p[1]), materialY).Cmp(&e) > 0 {
						haveBoss = true
					}
					continue
				}
				sideSlack := de
				if w.sideError != nil {
					sideSlack = plus(de, w.sideError)
				}
				if signed(new(big.Rat).Neg(off), outwardZ).Cmp(sideSlack) <= 0 {
					continue
				}
				dx := minus(new(big.Rat).Abs(minus(p[0], cx)), &e)
				if dx.Sign() < 0 {
					dx.SetInt64(0)
				}
				dz := minus(zdist, &e)
				if plus(times(dx, dx), times(dz, dz)).Cmp(r2) > 0 {
					haveCap = true
				}
			}
		}
		if haveCap && haveBoss {
			return true, nil
		}
	}
	return false, nil
}
