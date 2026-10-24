package revolveplan

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/coneaxis"
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

// PointConeSource recognizes the unplaced, full, straight-meridian cone about
// an axis through the world origin. Every parameter is read from the
// authenticated meridian; the rounded public Cone tag is used only for surface
// reporting.
//
// A record that passes the frame predicate (coneaxis.FrameCanonical) is read
// as a cone about world X, and its limit gets the canonical axis. Any other
// record needs coneaxis.OfRevolve and is read in exact plane-scaled
// coordinates about its recorded AxisVec A (agent-docs/revolve-axis-design.md
// §4.3): a point is on the axis only when its radial reading is exactly zero,
// the apex and far plane are A-scaled scalars, and the slope is the exact
// homogeneous ratio of the meridian.
func PointConeSource(b *topo.Body) (*facetedrecord.PointConeLimit, bool) {
	rp, ok := b.Payload.(Payload)
	if !ok || !rp.Full || rp.SurfaceResult || rp.SectionDelta != 0 || rp.Xform != r3.Identity() ||
		len(rp.Profile.Holes) != 0 || len(rp.Profile.Outer.Segments) != 3 {
		return nil, false
	}
	if !coneaxis.FrameCanonical(rp.Frame, rp.Ax) {
		return pointConeSourceAxis(b, rp)
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
	a := proofarith.FloatRat(apex)
	k := new(big.Rat).Quo(new(big.Rat).Sub(a, proofarith.FloatRat(far)), proofarith.FloatRat(radius))
	return pointConeLimit(b, coneaxis.X(), a, proofarith.FloatRat(far), k)
}

// pointConeLimit binds the cone limit to the tool's one Cone face, which
// supplies the reported surface, origins and denoted normal.
func pointConeLimit(b *topo.Body, axis coneaxis.Axis, apex, far, slope *big.Rat) (*facetedrecord.PointConeLimit, bool) {
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
	return &facetedrecord.PointConeLimit{Apex: apex, Far: far, Slope: slope, Axis: axis,
		Surface: surface, Origins: origins, Denoted: denoted, Reversed: reversed}, true
}

// pointConeSourceAxis is PointConeSource for a record outside the frame
// predicate. Each meridian point is read as its exact plane-scaled pair
// (z, r) = (p·A, cross2D(A, p)) (coneaxis.Axis.PlaneScaled). The four on-axis
// endpoints have r = 0 exactly and z equal to the apex c or the far plane
// c_far, the two rim endpoints share z = c_far and one r > 0, and c > 0 > c_far.
// A point the Revolve left a rounding off the axis has r != 0 and refuses.
func pointConeSourceAxis(b *topo.Body, rp Payload) (*facetedrecord.PointConeLimit, bool) {
	axis, ok := coneaxis.OfRevolve(rp.Frame, rp.Ax, rp.AxisVec)
	if !ok {
		return nil, false
	}
	if lo, hi := axis.NormBracket(); !(lo > 0) || proofbound.IsNonFinite(hi) {
		return nil, false
	}
	type reading struct{ z, r proofarith.Dyadic }
	readings := make([]reading, 0, 6)
	for _, segment := range rp.Profile.Outer.Segments {
		line, ok := segment.(sectionrecord.LineSeg)
		if !ok {
			return nil, false
		}
		for _, pt := range [...]sectionrecord.Point2{line.Start, line.End} {
			if !proofbound.FiniteVec(r3.Vec{X: pt.U, Y: pt.V}) {
				return nil, false
			}
			z, r := axis.PlaneScaled(rp.Frame, pt.U, pt.V)
			readings = append(readings, reading{z, r})
		}
	}
	var apex, far, radius proofarith.Dyadic
	axisCount, rimCount := 0, 0
	for _, pt := range readings {
		switch pt.r.Sign() {
		case 0:
			if axisCount == 0 || proofarith.DyCmp(pt.z, apex) > 0 {
				apex = pt.z
			}
			axisCount++
		case 1:
			if rimCount == 0 || proofarith.DyCmp(pt.r, radius) > 0 {
				radius = pt.r
			}
			rimCount++
		default:
			return nil, false
		}
		if axisCount+rimCount == 1 || proofarith.DyCmp(pt.z, far) < 0 {
			far = pt.z
		}
	}
	if axisCount != 4 || rimCount != 2 || apex.Sign() <= 0 || far.Sign() >= 0 || radius.Sign() <= 0 {
		return nil, false
	}
	for _, pt := range readings {
		onAxis := pt.r.Sign() == 0
		if onAxis && proofarith.DyCmp(pt.z, apex) != 0 && proofarith.DyCmp(pt.z, far) != 0 ||
			!onAxis && (proofarith.DyCmp(pt.z, far) != 0 || proofarith.DyCmp(pt.r, radius) != 0) {
			return nil, false
		}
	}
	c, cFar := apex.Rat(), far.Rat()
	k := new(big.Rat).Quo(new(big.Rat).Sub(c, cFar), radius.Rat())
	return pointConeLimit(b, axis, c, cFar, k)
}
