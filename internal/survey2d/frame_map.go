package survey2d

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// PlacedFrameMap is a prism payload's plane-local to world map, evaluated in
// EXACT arithmetic on the held frame and placement numbers alone.
// prismPayload.point rounds that map twice — once through the frame, once
// through the placement — and this is the map those roundings approximate,
// which is the map the payload DENOTES: a placement re-evaluates the record and
// stores its own coordinates, so the held numbers ARE what they denote (the same
// rule internal/proofbound/interval_vector.go states).
type PlacedFrameMap struct {
	Origin, Du, Dv, Dn proofbound.IvVec3
}

// point is the exact image of a plane-local (u, v) at height z.
func (m PlacedFrameMap) Point(u, v, z *big.Rat) proofbound.IvVec3 {
	return proofbound.IvVec3Add(m.Origin, proofbound.IvVec3Add(
		proofbound.IvVec3Mul(m.Du, proofbound.PointInterval(u)),
		proofbound.IvVec3Add(proofbound.IvVec3Mul(m.Dv, proofbound.PointInterval(v)), proofbound.IvVec3Mul(m.Dn, proofbound.PointInterval(z))),
	))
}
