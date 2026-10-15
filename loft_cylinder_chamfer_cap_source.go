package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// Every true tooth cap contour outside the centered root arc is matched to a
// retained held chord. Union copied these chords, in order, into its audited
// outer cap loop. A later inset whose exact clearance from that whole loop
// exceeds the Loft departure cannot be crossed by any such source contour.
func loftCylinderChamferExteriorLineage(fp facetedPayload,
	source loftCylinderChamferSource, capIndex int) error {
	if capIndex < 0 || capIndex > 1 || len(source.outer[capIndex]) < 3 ||
		fp.meshBound < 0 || proofbound.IsNonFinite(fp.meshBound) {
		return fmt.Errorf("%w: cap has no exterior contour correspondence", ErrUnsupported)
	}
	outer := source.outer[capIndex]
	edges := make(map[[2]r3.Vec]struct{}, len(outer))
	for i, index := range outer {
		next := outer[(i+1)%len(outer)]
		if index < 0 || index >= len(fp.Verts) || next < 0 || next >= len(fp.Verts) {
			return fmt.Errorf("%w: cap outer loop has invalid indices", ErrUnsupported)
		}
		edges[[2]r3.Vec{fp.Verts[index], fp.Verts[next]}] = struct{}{}
	}
	for _, body := range source.teeth {
		if body == nil {
			return fmt.Errorf("%w: cap lacks an original tooth", ErrUnsupported)
		}
		lp, ok := body.n.Payload.(loftPayload)
		if !ok || !lp.AuthenticatedReconstruction || lp.Proof.FacetDeparture < 0 ||
			lp.Proof.FacetDeparture > fp.meshBound || len(lp.Verts)%2 != 0 ||
			len(lp.Segment) != 1 {
			return fmt.Errorf("%w: cap tooth has no bounded source contour", ErrUnsupported)
		}
		n := len(lp.Verts) / 2
		if len(lp.Segment[0]) != n {
			return fmt.Errorf("%w: cap tooth has no station-to-curve map", ErrUnsupported)
		}
		root := -1
		for j, segment := range lp.Segment[0] {
			if segment == 5 {
				root = j
				break
			}
			if segment < 0 || segment > 4 {
				return fmt.Errorf("%w: cap tooth exterior has an unexpected source curve", ErrUnsupported)
			}
		}
		if root < 5 || root >= n-1 {
			return fmt.Errorf("%w: cap tooth has no separate root arc", ErrUnsupported)
		}
		first := capIndex * n
		for j := range root {
			if _, ok := edges[[2]r3.Vec{lp.Verts[first+j], lp.Verts[first+j+1]}]; !ok {
				return fmt.Errorf("%w: a source tooth exterior chord is absent from the held cap", ErrUnsupported)
			}
		}
	}
	return nil
}

func loftCylinderChamferCapToothOf(body *Body, capIndex int, z float64) (loftcylinder.ChamferCapTooth, error) {
	if body == nil {
		return loftcylinder.ChamferCapTooth{}, fmt.Errorf("%w: cap lacks a source tooth", ErrUnsupported)
	}
	lp, ok := body.n.Payload.(loftPayload)
	if !ok || !lp.AuthenticatedReconstruction || len(lp.Verts)%2 != 0 ||
		lp.Proof.FacetDeparture < 0 || proofbound.IsNonFinite(lp.Proof.FacetDeparture) {
		return loftcylinder.ChamferCapTooth{}, fmt.Errorf("%w: cap tooth has no matched Loft contour", ErrUnsupported)
	}
	return loftcylinder.ChamferCapToothOf(lp.Verts, lp.Proof.FacetDeparture, capIndex, z)
}

// Each source tooth's held cap chords and exact cap curve have the same
// authenticated parameters and a two-sided FacetDeparture bound. Strict
// chord clearance keeps a complete inset edge away from the moving source
// curve, and winding at its midpoint fixes which side remains occupied.
// This checks the inset rim only. The cap interior and root band are separate.
func loftCylinderChamferExteriorRimInside(ctx context.Context, source loftCylinderChamferSource,
	verts []r3.Vec, ring []int, capIndex int, outerRadius float64) error {
	if len(ring) < 3 || len(ring) > 4096 || capIndex < 0 || capIndex > 1 ||
		len(source.teeth) == 0 || len(source.teeth) > 64 ||
		source.blankRadius <= 0 || proofbound.IsNonFinite(source.blankRadius) ||
		outerRadius <= source.blankRadius || proofbound.IsNonFinite(outerRadius) {
		return fmt.Errorf("%w: cap exterior source has invalid limits", ErrUnsupported)
	}
	z := source.z0
	if capIndex == 1 {
		z = source.z1
	}
	teeth := make([]loftcylinder.ChamferCapTooth, len(source.teeth))
	for i, body := range source.teeth {
		tooth, err := loftCylinderChamferCapToothOf(body, capIndex, z)
		if err != nil {
			return err
		}
		teeth[i] = tooth
	}
	return loftcylinder.ChamferExteriorRimInside(ctx, teeth, verts, ring, source.blankRadius, outerRadius)
}
