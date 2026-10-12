package decad

import (
	"context"
	"fmt"
	"math/big"

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
	mesh *joinedMesh, groups [3]int, maxStep, ringRound float64) (*Body, error) {
	radius := source.blankRadius
	toothBound, toothVolume, toothArea := 0.0, 0.0, 0.0
	connectorReach, rootHeldExcess := 0.0, 0.0
	for i := range parts {
		part := &parts[i]
		toothBound = max(toothBound, part.mesh.Bound().Base())
		toothVolume = proofbound.AbsSumUpper(toothVolume, part.mesh.volSymDiff)
		area, err := part.body.Area()
		if err != nil {
			return nil, err
		}
		toothArea = proofbound.AbsSumUpper(toothArea, area.Value.Base(), area.Bound.Base())
		reach, err := prismPatternConnectorReach(part.prism, source.rootArc, radius,
			source.rootIndex)
		if err != nil {
			return nil, err
		}
		connectorReach = max(connectorReach, reach)
		n := len(part.mesh.vertices) / 2
		for j := range n {
			if j < part.rootStart {
				continue
			}
			v := part.mesh.vertices[2*j]
			r2 := prismPatternRadialSq(Point2{U: v.X, V: v.Y})
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
				tri := part.mesh.triangles[2*j+side]
				minimum := loftCylinderTriangleMinRadius(part.mesh.vertices, tri)
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
		proofarith.FloatRat(source.blank.z1), proofarith.FloatRat(source.blank.z0)))
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
	fp := facetedPayload{verts: mesh.verts, tris: mesh.tris, src: mesh.src, groups: mesh.groups,
		vertexBound: vertexBound, contactAudited: true,
		meshBound: bound, volSymDiff: volumeGap, areaSlack: areaSlack,
		dPair: facetedSourceDiameterUpper(mesh.verts, bound), xform: r3.Identity()}
	if proofbound.IsNonFinite(fp.dPair) {
		return nil, fmt.Errorf("%w: copied-prism join has no finite diameter proof", ErrUnsupported)
	}
	if err := loftCylinderTagCaps(&fp, blank, [2]int{groups[1], groups[2]},
		[2]float64{source.blank.z0, source.blank.z1}, ref); err != nil {
		return nil, err
	}
	return buildFacetedBody(ctx, d, ref, fp)
}

// A connector begins strictly inside the circle, ends strictly outside, and
// increases squared radius at its inner endpoint. Convexity of squared
// radius bounds its unique crossing parameter by (R²-r0²)/(2 near·step).
func prismPatternConnectorReach(p prismPayload, root arcSeg, radius float64,
	rootIndex int) (float64, error) {
	segments := p.profile.Outer.Segments
	if rootIndex != len(segments)-1 || rootIndex < 2 {
		return 0, fmt.Errorf("%w: copied-prism connectors have no trailing root arc", ErrUnsupported)
	}
	scale2 := prismPatternScale2(p.xform)
	radiusSq := prismPatternRadiusSq(radius)
	reach := 0.0
	for _, index := range []int{0, rootIndex - 1} {
		line, ok := segments[index].(lineSeg)
		if !ok {
			return 0, fmt.Errorf("%w: copied-prism root connector is not a line", ErrUnsupported)
		}
		near, far := line.Start, line.End
		if near != root.Start && near != root.End {
			near, far = far, near
		}
		if near != root.Start && near != root.End {
			return 0, fmt.Errorf("%w: copied-prism connector misses its root arc", ErrUnsupported)
		}
		nu, nv := proofarith.FloatRat(near.U), proofarith.FloatRat(near.V)
		du := new(big.Rat).Sub(proofarith.FloatRat(far.U), nu)
		dv := new(big.Rat).Sub(proofarith.FloatRat(far.V), nv)
		derivative := new(big.Rat).Mul(big.NewRat(2, 1), scale2)
		derivative.Mul(derivative, new(big.Rat).Add(new(big.Rat).Mul(nu, du), new(big.Rat).Mul(nv, dv)))
		if derivative.Sign() <= 0 {
			return 0, fmt.Errorf("%w: copied-prism connector has no outward crossing", ErrUnsupported)
		}
		nearSq := new(big.Rat).Mul(prismPatternRadialSq(near), scale2)
		gap := new(big.Rat).Sub(radiusSq, nearSq)
		if gap.Sign() <= 0 {
			return 0, fmt.Errorf("%w: copied-prism connector starts outside the blank", ErrUnsupported)
		}
		tUpper := new(big.Rat).Quo(gap, derivative)
		if tUpper.Cmp(big.NewRat(1, 1)) > 0 {
			tUpper.SetInt64(1)
		}
		stepSq := new(big.Rat).Mul(scale2,
			new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv)))
		length := proofbound.ProductUpper(proofbound.RatSqrtUp(stepSq), proofbound.RatFloatUp(tUpper))
		reach = max(reach, length)
	}
	return reach, nil
}
