package survey2d

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
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

// NewPlacedFrameMap reads the held plane frame and placement into one exact
// interval map. It refuses a component that cannot be represented exactly.
func NewPlacedFrameMap(frame r3.Frame, xform r3.Transform) (PlacedFrameMap, bool) {
	basis := xform.Basis()
	ex, okX := proofbound.IvVec3Of(basis.EX)
	ey, okY := proofbound.IvVec3Of(basis.EY)
	ez, okZ := proofbound.IvVec3Of(basis.EZ)
	translation, okT := proofbound.IvVec3Of(xform.Translation())
	origin, okO := proofbound.IvVec3Of(frame.Origin())
	u, okU := proofbound.IvVec3Of(frame.U())
	v, okV := proofbound.IvVec3Of(frame.V())
	n, okN := proofbound.IvVec3Of(frame.N())
	if !okX || !okY || !okZ || !okT || !okO || !okU || !okV || !okN {
		return PlacedFrameMap{}, false
	}
	place := func(local proofbound.IvVec3) proofbound.IvVec3 {
		return proofbound.IvVec3Add(
			proofbound.IvVec3Mul(ex, local[0]),
			proofbound.IvVec3Add(proofbound.IvVec3Mul(ey, local[1]), proofbound.IvVec3Mul(ez, local[2])),
		)
	}
	return PlacedFrameMap{
		Origin: proofbound.IvVec3Add(place(origin), translation),
		Du:     place(u),
		Dv:     place(v),
		Dn:     place(n),
	}, true
}

// point is the exact image of a plane-local (u, v) at height z.
func (m PlacedFrameMap) Point(u, v, z *big.Rat) proofbound.IvVec3 {
	return proofbound.IvVec3Add(m.Origin, proofbound.IvVec3Add(
		proofbound.IvVec3Mul(m.Du, proofbound.PointInterval(u)),
		proofbound.IvVec3Add(proofbound.IvVec3Mul(m.Dv, proofbound.PointInterval(v)), proofbound.IvVec3Mul(m.Dn, proofbound.PointInterval(z))),
	))
}
