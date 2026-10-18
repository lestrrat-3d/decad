package decad

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/pointcone"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/surfacenormal"
	"github.com/lestrrat-3d/r3"
)

// exactPointCone recognizes the unplaced, full, straight-meridian cone whose
// axis is world X. Every parameter is read from the authenticated meridian;
// the rounded public Cone tag is used only for surface reporting.
func exactPointCone(b *Body) (*pointConeLimit, bool) {
	rp, ok := b.n.Payload.(revolvePayload)
	if !ok || !rp.Full || rp.SurfaceResult || rp.SectionDelta != 0 || rp.Xform != r3.Identity() ||
		len(rp.Profile.Holes) != 0 || len(rp.Profile.Outer.Segments) != 3 ||
		rp.Frame.Origin() != (r3.Vec{}) || rp.Frame.U() != (r3.Vec{X: 1}) ||
		rp.Frame.V() != (r3.Vec{Y: 1}) || rp.Ax.AU != 0 || rp.Ax.AV != 0 ||
		rp.Ax.DU != 1 || rp.Ax.DV != 0 || rp.Ax.AUBound != 0 || rp.Ax.AVBound != 0 ||
		rp.Ax.DUBound != 0 || rp.Ax.DVBound != 0 {
		return nil, false
	}
	var apex, far, radius float64
	far = math.Inf(1)
	axisCount, rimCount := 0, 0
	for _, segment := range rp.Profile.Outer.Segments {
		line, ok := segment.(lineSeg)
		if !ok {
			return nil, false
		}
		for _, pt := range [...]planePoint{line.Start, line.End} {
			if pt.V == 0 {
				axisCount++
				apex = max(apex, pt.U)
				far = min(far, pt.U)
			} else if pt.V > 0 {
				rimCount++
				radius = max(radius, pt.V)
				far = min(far, pt.U)
			} else {
				return nil, false
			}
		}
	}
	if axisCount != 4 || rimCount != 2 || !proofbound.FiniteVec(r3.Vec{X: apex, Y: far, Z: radius}) ||
		apex <= 0 || far >= 0 || radius <= 0 {
		return nil, false
	}
	for _, segment := range rp.Profile.Outer.Segments {
		line, ok := segment.(lineSeg)
		if !ok {
			return nil, false
		}
		for _, pt := range [...]planePoint{line.Start, line.End} {
			if pt.V == 0 && pt.U != apex && pt.U != far || pt.V > 0 &&
				(pt.U != far || pt.V != radius) {
				return nil, false
			}
		}
	}
	var surface surfacegeom.Cone
	var origins []provenance.Ref
	var denoted *surfacenormal.Revolved
	var reversed bool
	for _, face := range b.Faces() {
		if cone, ok := face.n.Surface.(surfacegeom.Cone); ok {
			surface, origins = cone, face.originRefs()
			denoted, reversed = face.n.Denoted, face.n.Reversed
			break
		}
	}
	if len(origins) == 0 || denoted == nil {
		return nil, false
	}
	a := proofarith.FloatRat(apex)
	k := new(big.Rat).Quo(new(big.Rat).Sub(a, proofarith.FloatRat(far)), proofarith.FloatRat(radius))
	return &pointConeLimit{Apex: a, Far: proofarith.FloatRat(far), Slope: k,
		Surface: surface, Origins: origins, Denoted: denoted, Reversed: reversed}, true
}

// tryPointConeBoolean is an accept-only structural arm. It does not change
// the mesh Boolean fallback for any pair outside this exact source class.
func tryPointConeBoolean(ctx context.Context, op meshbool.OperationKind, d *Document,
	ref producerID, target, tool *Body) (*Body, bool, error) {
	if op != meshbool.OpCut && op != meshbool.OpIntersect {
		return nil, false, nil
	}
	cone, ok := exactPointCone(tool)
	if !ok {
		return nil, false, nil
	}
	fp, ok := target.n.Payload.(facetedPayload)
	if !ok || fp.Xform != r3.Identity() {
		return nil, false, nil
	}
	record := pointConeTrimRecord{Base: fp}
	if fp.PointCone != nil {
		record = *fp.PointCone
	} else if fp.PointSection == nil || fp.PointSection.Apex != (r3.Vec{}) {
		return nil, false, nil
	}
	if !pointcone.AboveFarCap(record.Base.Verts, record.Base.MeshBound, cone.Far) {
		return nil, false, nil
	}
	if op == meshbool.OpCut {
		if record.Lower != nil && !facetedrecord.PointConeLimitLater(record.Lower, cone, record.Base) {
			return nil, false, nil
		}
		record.Lower = cone
	} else {
		if record.Upper != nil && !facetedrecord.PointConeLimitEarlier(record.Upper, cone, record.Base) {
			return nil, false, nil
		}
		record.Upper = cone
	}
	if record.Lower != nil && record.Upper != nil &&
		!facetedrecord.PointConeLimitOrdered(record.Lower, record.Upper, record.Base) {
		return nil, false, nil
	}
	body, err := bodyResult(facetedrecord.BuildPointConeTrim(ctx,
		envOf(documentNode(d), ref), record))
	return body, true, err
}
