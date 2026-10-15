package decad

import (
	"context"
	"math"
	"math/big"
	"slices"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// loftCylinderProofTooth uses an axial-level proxy only when the placement's
// XY map survives r3's normalization bit for bit. The original Body remains
// the Boolean operand. The private proxy has the same authenticated sections,
// source identity, XY source points, and held XY vertices; its two caps land
// exactly on the blank planes. Its mesh proof also covers the original source.
func loftCylinderProofTooth(ctx context.Context, d *Document, tooth *Body,
	z0, z1 float64) (*Body, float64, bool, error) {
	lp, ok := tooth.n.Payload.(loftPayload)
	if !ok {
		return nil, 0, false, nil
	}
	if loftCylinderAxialPlacement(lp.Xform) {
		return tooth, 0, true, nil
	}
	basis := lp.Xform.Basis()
	if z0 != 0 || z1 <= 0 || proofbound.IsNonFinite(z1) ||
		lp.Xform.Translation() != (r3.Vec{}) ||
		basis.EZ != (r3.Vec{Z: math.Nextafter(1, 0)}) ||
		basis.EX.Z != 0 || basis.EY.Z != 0 ||
		basis.EX.X != basis.EY.Y || basis.EX.Y != -basis.EY.X {
		return nil, 0, false, nil
	}
	normalized, err := r3.FromBasis(r3.Basis{
		EX: basis.EX, EY: basis.EY, EZ: r3.Vec{Z: 1},
	}, r3.Vec{})
	if err != nil {
		return nil, 0, true, err
	}
	if !loftCylinderAxialPlacement(normalized) ||
		normalized.Basis().EX != basis.EX || normalized.Basis().EY != basis.EY {
		return nil, 0, false, nil
	}
	proxy, err := placePayload(ctx, lp, d, tooth.n.Origin.Producer, normalized)
	if err != nil {
		return nil, 0, true, err
	}
	proof, ok := proxy.n.Payload.(loftPayload)
	if !ok || proof.SourceIdentity != lp.SourceIdentity || len(proof.Verts) != len(lp.Verts) ||
		!slices.Equal(proof.Tris, lp.Tris) || proof.Walls != lp.Walls ||
		len(proof.Segment) != len(lp.Segment) {
		return nil, 0, false, nil
	}
	for i := range lp.Segment {
		if !slices.Equal(proof.Segment[i], lp.Segment[i]) {
			return nil, 0, false, nil
		}
	}
	gap := new(big.Rat).Sub(big.NewRat(1, 1), proofarith.FloatRat(basis.EZ.Z))
	shift := proofbound.RatFloatUp(new(big.Rat).Mul(gap, proofarith.FloatRat(z1)))
	for i, old := range lp.Verts {
		current := proof.Verts[i]
		if old.X != current.X || old.Y != current.Y {
			return nil, 0, false, nil
		}
		delta := new(big.Rat).Abs(new(big.Rat).Sub(
			proofarith.FloatRat(old.Z), proofarith.FloatRat(current.Z)))
		shift = max(shift, proofbound.RatFloatUp(delta))
	}
	if shift <= 0 || proofbound.IsNonFinite(shift) {
		return nil, 0, false, nil
	}
	area, err := proxy.Area()
	if err != nil {
		return nil, 0, true, err
	}
	areaUpper := proofbound.AbsSumUpper(area.Value.Base(), area.Bound.Base())
	areaBridge := proofbound.ProductUpper(proofbound.RatFloatUp(gap), areaUpper)
	volumeBridge := proofbound.SweptVolumeAllow(shift, areaUpper)
	proof.Proof.FacetDeparture = proofbound.AbsSumUpper(proof.Proof.FacetDeparture, shift)
	proof.Proof.AreaSlack = proofbound.AbsSumUpper(proof.Proof.AreaSlack, areaBridge)
	proof.Proof.VolSymDiff = proofbound.AbsSumUpper(proof.Proof.VolSymDiff, volumeBridge)
	if proofbound.IsNonFinite(proof.Proof.FacetDeparture) ||
		proofbound.IsNonFinite(proof.Proof.AreaSlack) ||
		proofbound.IsNonFinite(proof.Proof.VolSymDiff) {
		return nil, 0, false, nil
	}
	proxy.n.Payload = proof
	return proxy, shift, true, nil
}
