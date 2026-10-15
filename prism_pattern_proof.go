package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/prismpattern"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// prismPatternJoinedBody charges the only changed source boundary to the
// circular root band. The authenticated source root arcs lie strictly inside
// the blank. Each straight connector crosses its circle once, and every
// other source wall stays outside it. Thus the true and held exteriors agree
// outside the connector reach and ring-chord band, apart from each source
// mesh's already certified boundary and occupied-volume error.
func prismPatternJoinedBody(ctx context.Context, d *Document, ref producerID,
	blank *Body, source *prismPatternSource, parts []prismPatternPart,
	mesh *joinedMesh, outlines [2][]int, groups [3]int, maxStep, ringRound float64) (*Body, error) {
	radius := source.blankRadius
	toothBound, toothVolume, toothArea := 0.0, 0.0, 0.0
	connectorReach, rootHeldExcess := 0.0, 0.0
	for i := range parts {
		part := &parts[i]
		toothBound = max(toothBound, part.mesh.Bound().Base())
		toothVolume = proofbound.AbsSumUpper(toothVolume, part.mesh.n.VolSymDiff)
		area, err := part.body.Area()
		if err != nil {
			return nil, err
		}
		toothArea = proofbound.AbsSumUpper(toothArea, area.Value.Base(), area.Bound.Base())
		reach, err := prismpattern.ConnectorReach(part.prism, source.rootArc, radius,
			source.rootIndex)

		if err != nil {
			return nil, err
		}
		connectorReach = max(connectorReach, reach)
		n := len(part.mesh.n.Vertices) / 2
		for j := range n {
			if j < part.rootStart {
				continue
			}
			v := part.mesh.n.Vertices[2*j]
			r2 := prismpattern.RadialSq(planePoint{U: v.X, V: v.Y})
			excess := new(big.Rat).Sub(proofarith.FloatRat(proofbound.RatSqrtUp(r2)),
				proofarith.FloatRat(radius))
			if excess.Sign() > 0 {
				rootHeldExcess = max(rootHeldExcess, proofbound.RatFloatUp(excess))
			}
		}
		// The retained fitted and tip facets stay strictly outside the blank
		// even after their source-to-held departure. The first and last
		// connector walls are the only retained pieces that enter the band.
		for j := 1; j < part.rootStart-1; j++ {
			for side := range 2 {
				tri := part.mesh.n.Triangles[2*j+side]
				minimum := loftcylinder.TriangleMinRadius(part.mesh.n.Vertices, tri)
				if minimum <= proofbound.AbsSumUpper(radius, part.mesh.Bound().Base()) {
					return nil, fmt.Errorf("%w: copied-prism exterior wall %d lacks circular clearance",
						ErrUnsupported, j)
				}
			}
		}
	}
	ringSag := proofbound.ProductUpper(radius,
		proofbound.ProductUpper(maxStep, maxStep)/8)
	seamBand := proofbound.AbsSumUpper(source.rootGap, connectorReach,
		rootHeldExcess, ringSag, ringRound)
	bound := proofbound.AbsSumUpper(toothBound, seamBand)
	if proofbound.IsNonFinite(bound) || bound > 0.1 {
		return nil, fmt.Errorf("%w: copied-prism join exceeds the 0.1 mm boundary proof: bound %g tooth %g root %g reach %g ring %g",
			ErrUnsupported, bound, toothBound, source.rootGap, connectorReach, ringSag)
	}
	if seamBand >= radius {
		return nil, fmt.Errorf("%w: copied-prism join band reaches the axis", ErrUnsupported)
	}
	// The symmetric annulus encloses every face changed by removal of the
	// root wall and replacement of the cylinder wall. Its volume is exactly
	// 4*pi*radius*band*height. Each original tooth mesh contributes its own
	// independently verified occupied-volume error once.
	annulusArea := proofbound.ProductUpper(
		proofbound.RatFloatUp(proofbound.PiUpper),
		proofbound.ProductUpper(proofbound.ProductUpper(4, radius), seamBand))
	height := proofbound.RatFloatUp(new(big.Rat).Sub(
		proofarith.FloatRat(source.blank.Z1), proofarith.FloatRat(source.blank.Z0)))
	volumeGap := proofbound.AbsSumUpper(toothVolume, proofbound.ProductUpper(annulusArea, height))
	areaUpper, err := proofbound.PerturbedAreaUpperContext(ctx, mesh.verts, mesh.tris, bound)
	if err != nil {
		return nil, err
	}
	blankArea, err := blank.Area()
	if err != nil {
		return nil, err
	}
	areaSlack := proofbound.AbsSumUpper(areaUpper,
		blankArea.Value.Base(), blankArea.Bound.Base(), toothArea)
	if proofbound.IsNonFinite(volumeGap) || proofbound.IsNonFinite(areaSlack) {
		return nil, fmt.Errorf("%w: copied-prism join has no finite occupied-volume proof", ErrUnsupported)
	}
	vertexBound := make([]float64, len(mesh.verts))
	for i := range vertexBound {
		vertexBound[i] = bound
	}
	fp := facetedPayload{Verts: mesh.verts, Tris: mesh.tris, Src: mesh.src, groups: mesh.groups,
		axialSectionJoin: true,
		prismPatternJoin: &prismPatternJoinRecord{blank: blank, source: source, parts: parts,
			outlines: outlines, groups: groups, maxStep: maxStep, ringRound: ringRound,
			mesh: mesh},
		vertexBound: vertexBound, contactAudited: true,
		meshBound: bound, volSymDiff: volumeGap, areaSlack: areaSlack,
		dPair: facetedSourceDiameterUpper(mesh.verts, bound), xform: r3.Identity()}
	if proofbound.IsNonFinite(fp.dPair) {
		return nil, fmt.Errorf("%w: copied-prism join has no finite diameter proof", ErrUnsupported)
	}
	if err := loftCylinderTagCaps(&fp, blank, [2]int{groups[1], groups[2]},
		[2]float64{source.blank.Z0, source.blank.Z1}, ref); err != nil {
		return nil, err
	}
	return buildFacetedBody(ctx, d, ref, fp)
}
