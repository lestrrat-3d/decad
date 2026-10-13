package revolvemesh

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// revolveCornerAxes holds the candidate axes RevolveVertexIsolated builds from
// one triangle and one of its corners: the two edge-fan axes and the chord
// axis. They depend on nothing but the triangle, the corner and delta.
type revolveCornerAxes struct {
	fan   [2]RevolveSepAxis
	chord RevolveSepAxis
}

// revolveAxisCache memoises those axes, and each triangle's normal axis, for
// one RevolveContactAudit call, so a pole fan's triangles build them once
// rather than once per partner.
type revolveAxisCache struct {
	normal   []RevolveSepAxis
	normalOK []bool
	corners  [][3]revolveCornerAxes
	cornerOK [][3]bool
}

func newRevolveAxisCache(n int) *revolveAxisCache {
	return &revolveAxisCache{
		normal:   make([]RevolveSepAxis, n),
		normalOK: make([]bool, n),
		corners:  make([][3]revolveCornerAxes, n),
		cornerOK: make([][3]bool, n),
	}
}

func (c *revolveAxisCache) normalAxis(i int, a RevolveAuditTri, delta float64) RevolveSepAxis {
	if !c.normalOK[i] {
		c.normal[i] = RevolveNormalAxis(a, delta)
		c.normalOK[i] = true
	}
	return c.normal[i]
}

func (c *revolveAxisCache) corner(i int, a RevolveAuditTri, at int, delta float64) *revolveCornerAxes {
	if !c.cornerOK[i][at] {
		off := RevolveCornerOffsets(a, at)
		out := &c.corners[i][at]
		out.fan[0] = RevolveEdgeFanAxis(a, off[0], delta)
		out.fan[1] = RevolveEdgeFanAxis(a, off[1], delta)
		chord := RevolveOffsetOf(proofarith.DvSub(off[0].V, off[1].V), RevIvSubVec(off[0].F, off[1].F))
		out.chord = RevolveEdgeFanAxis(a, chord, delta)
		c.cornerOK[i][at] = true
	}
	return &c.corners[i][at]
}

// auditRevolvePairCached is AuditRevolvePair with the vertex case reading
// both triangles' axes from cache.
func auditRevolvePairCached(data []RevolveAuditTri, tris [][3]int, shared [3]int, count, i, j int, delta float64,
	cache *revolveAxisCache) error {
	if count != 1 {
		return AuditRevolvePair(data, tris, shared, count, i, j, delta)
	}
	if revolveVertexIsolated(data[i], tris[i], data[j], tris[j], shared[0], delta, cache, i) ||
		revolveVertexIsolated(data[j], tris[j], data[i], tris[i], shared[0], delta, cache, j) {
		return nil
	}
	return fmt.Errorf(`%w: revolve facets %d and %d do not provably meet only at the vertex they share`, decaderr.ErrUnsupported, i, j)
}
