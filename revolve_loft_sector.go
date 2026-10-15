package decad

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// certifyRevolveLoftSector tries each authenticated finite conical face of
// the Revolve section. The source must certify exactly one face as the
// contact surface; a held geometric resemblance never selects one.
func certifyRevolveLoftSector(blank, sector *Body) (*revolveLoftSectorCertificate, bool) {
	if sector == nil {
		return nil, false
	}
	fp, ok := sector.n.Payload.(facetedPayload)
	if !ok || fp.Xform != r3.Identity() || fp.PointCone == nil ||
		fp.PointCone.Lower == nil || fp.PointCone.Upper == nil ||
		fp.PointCone.Base.PointSection == nil ||
		fp.PointCone.Base.PointSection.Apex != (r3.Vec{}) ||
		fp.PointCone.Base.Xform != r3.Identity() {
		return nil, false
	}
	trim := fp.PointCone
	if !pointSourceCurrent(trim.Base.PointSection) {
		return nil, false
	}
	var selected *revolveLoftSectorCertificate
	for _, section := range revolveRootCandidates(blank) {
		boundary, ok := certifyRevolveConnectorBoundary(trim.Base.PointSection,
			section, trim.Base)
		if !ok {
			boundary, ok = certifyRevolveFitBoundary(trim.Base.PointSection,
				section, trim.Base)
			if !ok {
				continue
			}
		}
		if selected != nil {
			return nil, false
		}
		selected = &revolveLoftSectorCertificate{Blank: bodyNode(blank), Section: section,
			Trimmed: fp, Trim: trim, Base: trim.Base, Boundary: boundary}
	}
	return selected, selected != nil
}

func certifyRevolveConnectorBoundary(record *pointSectionRecord,
	section revolveSectionCertificate, base facetedPayload) (revolveSectorBoundary, bool) {
	var empty revolveSectorBoundary
	if !pointSourceCurrent(record) ||
		len(record.Profile.Outer.Segments) < 5 || len(record.Profile.Outer.Segments) > 128 {
		return empty, false
	}
	implicitCost, ok := revolveImplicitDisplacement(section, base)
	if !ok {
		return empty, false
	}
	cone := section.ConeSection()
	src := record.Source
	result := revolveSectorBoundary{InsideSegment: -1, OutsideWitness: -1,
		Sides: make([]sketch.ConeCurveSide, len(src.Entities)), SourceRevision: src.Revision}
	count := 0
	for i, entity := range src.Entities {
		// Sketch's approximate Partial has no admissible exact endpoint. A
		// certified partial needs a separate restricted-entity side proof.
		if src.Profile.Outer[i].Partial {
			return empty, false
		}
		switch typed := entity.(type) {
		case *sketch.Line:
			if _, ok := record.Profile.Outer.Segments[i].(lineSeg); !ok {
				return empty, false
			}
			cert, err := src.Sketch.CertifyConeLineCrossing(typed, cone)
			if err == nil {
				if count >= len(result.Contacts) || cert.Line() != typed ||
					cert.Cone() != cone || cert.SketchRevision() != src.Revision ||
					cert.IsStale() || !cert.TExact() {
					return empty, false
				}
				lo, hi := cert.TBounds()
				point := cert.Point()
				if !(0 < lo && lo < hi && hi < 1) ||
					!proofbound.FiniteVec(r3.Vec{X: cert.T(), Y: point[0], Z: point[1]}) {
					return empty, false
				}
				result.Contacts[count] = revolveSectorContact{Segment: i, Line: cert}
				count++
				continue
			}
		case *sketch.FitSpline:
			if _, ok := record.Profile.Outer.Segments[i].(fitSplineSeg); !ok {
				return empty, false
			}
		case *sketch.Arc:
			if _, ok := record.Profile.Outer.Segments[i].(arcSeg); !ok {
				return empty, false
			}
		default:
			return empty, false
		}
		cert, err := src.Sketch.CertifyConeCurveSide(entity, cone)
		if err != nil || cert.Entity() != entity || cert.Cone() != cone || cert.IsStale() ||
			cert.ImplicitMargin() == nil ||
			cert.ImplicitMargin().Cmp(proofarith.FloatRat(implicitCost)) <= 0 {
			return empty, false
		}
		switch cert.Side() {
		case sketch.ConeInside:
			if _, isArc := entity.(*sketch.Arc); !isArc || result.InsideSegment >= 0 {
				return empty, false
			}
			result.InsideSegment = i
		case sketch.ConeOutside:
			result.OutsideWitness = i
		default:
			return empty, false
		}
		result.Sides[i] = cert
	}
	if count != 2 || result.InsideSegment < 0 || result.OutsideWitness < 0 {
		return empty, false
	}
	left := (result.InsideSegment + len(src.Entities) - 1) % len(src.Entities)
	right := (result.InsideSegment + 1) % len(src.Entities)
	if (result.Contacts[0].Segment != left || result.Contacts[1].Segment != right) &&
		(result.Contacts[0].Segment != right || result.Contacts[1].Segment != left) {
		return empty, false
	}
	return result, true
}

// revolveImplicitDisplacement bounds the source fan's two-sided held-mesh
// departure against the reported cone, including the finite meridian gap.
func revolveImplicitDisplacement(section revolveSectionCertificate,
	base facetedPayload) (float64, bool) {
	if !proofbound.FiniteVec(r3.Vec{X: base.MeshBound}) || base.MeshBound < 0 ||
		section.RootStart.U <= 0 || len(base.Verts) < 4 {
		return 0, false
	}
	minX, maxX, reach := math.Inf(1), math.Inf(-1), 0.0
	for _, world := range base.Verts[1:] {
		if !proofbound.FiniteVec(world) || world.X <= 0 {
			return 0, false
		}
		minX, maxX = min(minX, world.X), max(maxX, world.X)
		reach = max(reach, proofbound.DvLenUpper(proofbound.HeldDelta(world, r3.Vec{})))
	}
	minX = math.Nextafter(minX-base.MeshBound, math.Inf(-1))
	maxX = proofbound.AbsSumUpper(maxX, base.MeshBound)
	if !(minX > section.RootCone.Origin.X) {
		return 0, false
	}
	slope := section.ConeSection().Slope
	d := proofbound.AbsSumUpper(base.MeshBound,
		revolveMeridianExtensionGap(section, minX, maxX))
	r := proofbound.AbsSumUpper(reach, d)
	cost := proofbound.ProductUpper(proofbound.AbsSumUpper(1,
		proofbound.ProductUpper(slope, slope)),
		proofbound.ProductUpper(d, proofbound.AbsSumUpper(proofbound.ProductUpper(2, r), d)))
	return cost, !proofbound.IsNonFinite(cost)
}

func revolveMeridianExtensionGap(section revolveSectionCertificate, xLo, xHi float64) float64 {
	x0, x1 := proofarith.FloatRat(section.RootStart.U), proofarith.FloatRat(section.RootEnd.U)
	r0, r1 := proofarith.FloatRat(section.RootStart.V), proofarith.FloatRat(section.RootEnd.V)
	m := new(big.Rat).Quo(new(big.Rat).Sub(r1, r0), new(big.Rat).Sub(x1, x0))
	b := new(big.Rat).Sub(r0, new(big.Rat).Mul(m, x0))
	slope := proofarith.FloatRat(section.ConeSection().Slope)
	originX := proofarith.FloatRat(section.RootCone.Origin.X)
	gap := func(x float64) *big.Rat {
		value := new(big.Rat).Add(new(big.Rat).Mul(new(big.Rat).Sub(m, slope), proofarith.FloatRat(x)),
			new(big.Rat).Add(b, new(big.Rat).Mul(slope, originX)))
		return value.Abs(value)
	}
	a, z := gap(xLo), gap(xHi)
	if a.Cmp(z) < 0 {
		a = z
	}
	return proofbound.RatFloatUp(a)
}
