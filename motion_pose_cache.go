package decad

const poseBodyCacheSize = 32

// poseBodyCache bounds the transient bodies retained for one joint-box mover.
// Each entry is one completed payload placement at one bit-identical float
// transform. A hit reuses its body and any completed geometry or mesh proofs;
// the caller still computes the cell's own ideal-pose allowance.
type poseBodyCache struct {
	bodies map[[12]uint64]*Body
	order  [poseBodyCacheSize][12]uint64
	next   int
}

func (c *poseBodyCache) get(key [12]uint64) *Body {
	return c.bodies[key]
}

// put returns the body evicted from the cache, if any, so its clearance
// carriers can be removed from the call's bodyGeomCache too.
func (c *poseBodyCache) put(key [12]uint64, body *Body) *Body {
	if c.bodies == nil {
		c.bodies = make(map[[12]uint64]*Body, poseBodyCacheSize)
	}
	var evicted *Body
	if len(c.bodies) == poseBodyCacheSize {
		old := c.order[c.next]
		evicted = c.bodies[old]
		delete(c.bodies, old)
	}
	c.order[c.next] = key
	c.bodies[key] = body
	c.next = (c.next + 1) % poseBodyCacheSize
	return evicted
}
