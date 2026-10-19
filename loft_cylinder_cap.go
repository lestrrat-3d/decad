package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
)

// loftCylinderTagCaps gives the two already audited cap groups the result's
// own cap roles and the source blank's exact planar surface. The joined cap
// audit proves each disk tiles its outline; the bore path audits each annulus
// before calling this helper. The mesh's remaining groups keep their
// faceted surface identity.
func loftCylinderTagCaps(pp *facetedPayload, blank *Body, groups [2]int,
	levels [2]float64, ref producerID) error {
	if blank == nil || len(pp.Tris) != len(pp.Src) || groups[0] == groups[1] {
		return fmt.Errorf("%w: loft/cylinder caps have no separate source groups", ErrUnsupported)
	}
	roles := [2]string{provenance.RoleCapStart, provenance.RoleCapEnd}
	for cap, group := range groups {
		if group < 0 || group >= len(pp.Groups) {
			return fmt.Errorf("%w: loft/cylinder cap names no source group", ErrUnsupported)
		}
		var source *Face
		for _, face := range blank.Faces() {
			for _, origin := range face.originRefs() {
				if origin != (provenance.Ref{Producer: blank.n.Origin.Producer, Role: roles[cap]}) {
					continue
				}
				if source != nil {
					return fmt.Errorf("%w: loft/cylinder blank has duplicate cap roles", ErrUnsupported)
				}
				source = face
			}
		}
		if source == nil {
			return fmt.Errorf("%w: loft/cylinder blank has no cap source", ErrUnsupported)
		}
		plane, ok := source.n.Surface.(surfacegeom.Plane)
		if !ok || plane.Frame.Origin().Z != levels[cap] {
			return fmt.Errorf("%w: loft/cylinder blank cap has no matching plane", ErrUnsupported)
		}
		want := float64(1)
		if cap == 0 {
			want = -1
		}
		if plane.Frame.N() != (r3.Vec{Z: want}) {
			return fmt.Errorf("%w: loft/cylinder blank cap has no axial normal", ErrUnsupported)
		}
		count := 0
		for i, tri := range pp.Tris {
			if pp.Src[i] != group {
				continue
			}
			count++
			for _, index := range tri {
				if index < 0 || index >= len(pp.Verts) || pp.Verts[index].Z != levels[cap] {
					return fmt.Errorf("%w: loft/cylinder cap facet leaves its source plane", ErrUnsupported)
				}
			}
		}
		if count == 0 {
			return fmt.Errorf("%w: loft/cylinder cap group has no facets", ErrUnsupported)
		}
		g := &pp.Groups[group]
		g.Origins = append(append([]provenance.Ref(nil), g.Origins...),
			provenance.Ref{Producer: ref, Role: roles[cap]})
		g.Surface = plane
		g.Planar = true
		g.NormalBound = source.n.NormalBound
		g.Reversed = source.n.Reversed
	}
	return nil
}

// loftCylinderTagBridgedTop replaces the blank's exact top Plane tag when
// an original tooth ends below the held cap. The cap role remains intact;
// facetedrecord.BuildWithProof supplies the source-to-held displacement on its
// surface.
func loftCylinderTagBridgedTop(pp *facetedPayload, topGroup int, axialBridge []float64) {
	for _, shift := range axialBridge {
		if shift > 0 {
			top := &pp.Groups[topGroup]
			top.Surface = nil
			top.Planar = false
			return
		}
	}
}

// loftCylinderAuditAnnulus checks the exact planar area and directed edge
// balance of one rebuilt cap. The old disk audit supplies the outer loop; the
// bore ring and side-clearance gates supply a disjoint inner loop. The whole
// shell crossing audit excludes overlapping or folded cap facets.
func loftCylinderAuditAnnulus(ctx context.Context, mesh *joinedMesh,
	outer, inner []int, group int, z float64, bottom bool) bool {
	return loftcylinder.AuditAnnulus(ctx, mesh.Verts, mesh.Tris, mesh.Src, outer, inner, group, z, bottom)
}

// The cap group must tile exactly one simple CCW outline, with the origin
// strictly inside. A later centered bore may use that outline as its source.
func loftCylinderAuditCap(ctx context.Context, mesh *joinedMesh,
	outline []int, group int, z float64, bottom bool) bool {
	return loftcylinder.AuditCap(ctx, mesh.Verts, mesh.Tris, mesh.Src, outline, group, z, bottom)
}

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
	if loftcylinder.AxialPlacement(lp.Xform) {
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
	if !loftcylinder.AxialPlacement(normalized) ||
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
