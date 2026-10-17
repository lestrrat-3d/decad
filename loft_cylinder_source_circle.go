package decad

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// loftCylinderSourceCircleOf rechecks the source claims needed by a later
// radial occupancy query. It refuses when the true root wall may reach the
// cylinder in the admitted open interval, a fitted wall may enter it, either
// true connector lacks one transverse crossing, or no exterior angular gap
// can be certified. The caller must retain the tooth from an audited,
// unplaced loft/cylinder Union or its centered-bore Cut.
func loftCylinderSourceCircleOf(lp loftPayload, radius float64) (loftcylinder.SourceCircle, error) {
	refuse := func(reason string) (loftcylinder.SourceCircle, error) {
		return loftcylinder.SourceCircle{}, fmt.Errorf("%w: %s", ErrUnsupported, reason)
	}
	if radius <= 0 || proofbound.IsNonFinite(radius) || !lp.AuthenticatedReconstruction ||
		lp.SourceIdentity == nil || lp.SurfaceResult || !loftCylinderAxialPlacement(lp.Xform) ||
		lp.Proof.FacetDeparture < 0 || proofbound.IsNonFinite(lp.Proof.FacetDeparture) ||
		len(lp.Profile0.Outer.Segments) != 6 || len(lp.Profile1.Outer.Segments) != 6 ||
		!loftCylinderSectionKinds(lp) || len(lp.Verts)%2 != 0 || len(lp.Verts) < 16 ||
		len(lp.Segment) != 1 {
		return refuse("source circle lacks an authenticated axial Loft")
	}
	n := len(lp.Verts) / 2
	if len(lp.Segment[0]) != n || len(lp.Tris) != 4*n-4 ||
		lp.Frame0.U() != (r3.Vec{X: 1}) || lp.Frame0.V() != (r3.Vec{Y: 1}) ||
		lp.Frame1.U() != (r3.Vec{X: 1}) || lp.Frame1.V() != (r3.Vec{Y: 1}) {
		return refuse("source circle has no paired planar sections")
	}
	z0, z1 := lp.Frame0.Origin().Z, lp.Frame1.Origin().Z
	if z1 <= z0 || !proofbound.FiniteVec(r3.Vec{Z: z0}) || !proofbound.FiniteVec(r3.Vec{Z: z1}) ||
		lp.Frame0.Origin() != (r3.Vec{Z: z0}) || lp.Frame1.Origin() != (r3.Vec{Z: z1}) ||
		lp.Verts[0].Z != z0 || lp.Verts[n].Z != z1 {
		return refuse("source circle has no exact axial interval")
	}
	root := -1
	for j, segment := range lp.Segment[0] {
		if segment == 5 && root < 0 {
			root = j
		}
		if root >= 0 && segment != 5 {
			return refuse("source circle has a nonterminal root arc")
		}
	}
	if root < 3 || root >= n-1 {
		return refuse("source circle has no paired root connectors")
	}
	gap, ok := loftcylinder.RootRadiusGap(lp.Xform, [2]momentinput.Profile{lp.Profile0, lp.Profile1}, radius)
	if !ok {
		return refuse("source root has no radial gap bound")
	}
	if _, ok := loftcylinder.RadialGate(lp.Verts, lp.Tris, lp.Segment, root, radius, lp.Proof.FacetDeparture, gap); !ok {
		return refuse("source fitted walls may meet the circle")
	}
	height := proofbound.RatFloatUp(new(big.Rat).Sub(proofarith.FloatRat(z1), proofarith.FloatRat(z0)))
	collar, ok := loftcylinder.SourceStrictRootCollar(lp.Xform,
		[2]momentinput.Profile{lp.Profile0, lp.Profile1}, radius, height)
	if !ok {
		return refuse("source root lacks a strictly interior open interval")
	}
	record := loftcylinder.SourceCircle{
		Z0: proofarith.FloatRat(z0), Z1: proofarith.FloatRat(z1), Radius: proofarith.FloatRat(radius),
	}
	record.SafeBottom = new(big.Rat).Add(record.Z0, proofarith.FloatRat(collar))
	record.SafeTop = new(big.Rat).Sub(record.Z1, proofarith.FloatRat(collar))
	if record.SafeTop.Cmp(record.SafeBottom) <= 0 {
		return refuse("source root collar covers the axial interval")
	}
	for i, cell := range [2]int{0, root - 1} {
		connector, valid := loftcylinder.SourceConnectorOf(lp.Verts, lp.Tris, lp.Segment,
			[2]profileRecord{lp.Profile0, lp.Profile1}, lp.Xform.Basis().EX, cell, root)
		if !valid || !connector.RadialMonotone() || !connector.FarOutside(record.Radius) {
			return refuse("true source connector lacks one transverse circle crossing")
		}
		record.Connector[i] = connector
	}
	axis, ok := loftcylinder.SourceGapAxis(lp.Verts, root, lp.Proof.FacetDeparture)
	if !ok {
		return refuse("source tooth has no exterior gap witness")
	}
	record.GapAxis = axis
	return record, nil
}
