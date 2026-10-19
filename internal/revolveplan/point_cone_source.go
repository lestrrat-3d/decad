package revolveplan

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/surfacenormal"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// PointConeSource recognizes the unplaced, full, straight-meridian cone whose
// axis is world X. Every parameter is read from the authenticated meridian;
// the rounded public Cone tag is used only for surface reporting.
func PointConeSource(b *topo.Body) (*facetedrecord.PointConeLimit, bool) {
	rp, ok := b.Payload.(Payload)
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
		line, ok := segment.(sectionrecord.LineSeg)
		if !ok {
			return nil, false
		}
		for _, pt := range [...]sectionrecord.Point2{line.Start, line.End} {
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
		line, ok := segment.(sectionrecord.LineSeg)
		if !ok {
			return nil, false
		}
		for _, pt := range [...]sectionrecord.Point2{line.Start, line.End} {
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
	for _, face := range topobuild.Faces(b) {
		if cone, ok := face.Surface.(surfacegeom.Cone); ok {
			surface, origins = cone, append([]provenance.Ref(nil), face.Origins...)
			denoted, reversed = face.Denoted, face.Reversed
			break
		}
	}
	if len(origins) == 0 || denoted == nil {
		return nil, false
	}
	a := proofarith.FloatRat(apex)
	k := new(big.Rat).Quo(new(big.Rat).Sub(a, proofarith.FloatRat(far)), proofarith.FloatRat(radius))
	return &facetedrecord.PointConeLimit{Apex: a, Far: proofarith.FloatRat(far), Slope: k,
		Surface: surface, Origins: origins, Denoted: denoted, Reversed: reversed}, true
}
