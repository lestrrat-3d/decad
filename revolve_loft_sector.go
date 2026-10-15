package decad

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

type revolveSectorContact struct {
	segment int
	line    sketch.ConeLineCrossing
	fit     sketch.ConeFitCrossing
}

type revolveSectorBoundary struct {
	insideSegment, outsideWitness int
	contacts                      [2]revolveSectorContact
	fitRoute                      bool
	sides                         []sketch.ConeCurveSide
	sourceRevision                uint64
}

// revolveLoftSectorCertificate keeps the immutable source records and the
// Sketch answers that identify one connected exposed sector of a point loft.
// Mesh stations and their displacement remain a separate construction proof.
type revolveLoftSectorCertificate struct {
	blank         *Body
	section       revolveSectionCertificate
	trimmed       facetedPayload
	trim          *pointConeTrimRecord
	base          facetedPayload
	boundary      revolveSectorBoundary
	sector        [2]float64
	loop          []revolveSectorNode
	crossingBound float64
}

type revolveSectorNode struct {
	point        r3.Vec
	groupToNext  int
	crossing     int
	rootBoundary bool
	angle        float64
}

// certifyRevolveLoftSector tries each authenticated finite conical face of
// the Revolve section. The source must certify exactly one face as the
// contact surface; a held geometric resemblance never selects one.
func certifyRevolveLoftSector(blank, sector *Body) (*revolveLoftSectorCertificate, bool) {
	if sector == nil {
		return nil, false
	}
	fp, ok := sector.n.Payload.(facetedPayload)
	if !ok || fp.xform != r3.Identity() || fp.pointCone == nil ||
		fp.pointCone.lower == nil || fp.pointCone.upper == nil ||
		fp.pointCone.base.pointSection == nil ||
		fp.pointCone.base.pointSection.apex != (r3.Vec{}) ||
		fp.pointCone.base.xform != r3.Identity() {
		return nil, false
	}
	trim := fp.pointCone
	if !pointSourceCurrent(trim.base.pointSection) {
		return nil, false
	}
	var selected *revolveLoftSectorCertificate
	for _, section := range revolveRootCandidates(blank) {
		boundary, ok := certifyRevolveConnectorBoundary(trim.base.pointSection,
			section, trim.base)
		if !ok {
			boundary, ok = certifyRevolveFitBoundary(trim.base.pointSection,
				section, trim.base)
			if !ok {
				continue
			}
		}
		if selected != nil {
			return nil, false
		}
		selected = &revolveLoftSectorCertificate{blank: blank, section: section,
			trimmed: fp, trim: trim, base: trim.base, boundary: boundary}
	}
	return selected, selected != nil
}

func certifyRevolveConnectorBoundary(record *pointSectionRecord,
	section revolveSectionCertificate, base facetedPayload) (revolveSectorBoundary, bool) {
	var empty revolveSectorBoundary
	if !pointSourceCurrent(record) ||
		len(record.profile.Outer.Segments) < 5 || len(record.profile.Outer.Segments) > 128 {
		return empty, false
	}
	implicitCost, ok := revolveImplicitDisplacement(section, base)
	if !ok {
		return empty, false
	}
	cone := section.coneSection()
	src := record.source
	result := revolveSectorBoundary{insideSegment: -1, outsideWitness: -1,
		sides: make([]sketch.ConeCurveSide, len(src.entities)), sourceRevision: src.revision}
	count := 0
	for i, entity := range src.entities {
		// Sketch's approximate Partial has no admissible exact endpoint. A
		// certified partial needs a separate restricted-entity side proof.
		if src.profile.Outer[i].Partial {
			return empty, false
		}
		switch typed := entity.(type) {
		case *sketch.Line:
			if _, ok := record.profile.Outer.Segments[i].(lineSeg); !ok {
				return empty, false
			}
			cert, err := src.sketch.CertifyConeLineCrossing(typed, cone)
			if err == nil {
				if count >= len(result.contacts) || cert.Line() != typed ||
					cert.Cone() != cone || cert.SketchRevision() != src.revision ||
					cert.IsStale() || !cert.TExact() {
					return empty, false
				}
				lo, hi := cert.TBounds()
				point := cert.Point()
				if !(0 < lo && lo < hi && hi < 1) ||
					!proofbound.FiniteVec(r3.Vec{X: cert.T(), Y: point[0], Z: point[1]}) {
					return empty, false
				}
				result.contacts[count] = revolveSectorContact{segment: i, line: cert}
				count++
				continue
			}
		case *sketch.FitSpline:
			if _, ok := record.profile.Outer.Segments[i].(fitSplineSeg); !ok {
				return empty, false
			}
		case *sketch.Arc:
			if _, ok := record.profile.Outer.Segments[i].(arcSeg); !ok {
				return empty, false
			}
		default:
			return empty, false
		}
		cert, err := src.sketch.CertifyConeCurveSide(entity, cone)
		if err != nil || cert.Entity() != entity || cert.Cone() != cone || cert.IsStale() ||
			cert.ImplicitMargin() == nil ||
			cert.ImplicitMargin().Cmp(proofarith.FloatRat(implicitCost)) <= 0 {
			return empty, false
		}
		switch cert.Side() {
		case sketch.ConeInside:
			if _, isArc := entity.(*sketch.Arc); !isArc || result.insideSegment >= 0 {
				return empty, false
			}
			result.insideSegment = i
		case sketch.ConeOutside:
			result.outsideWitness = i
		default:
			return empty, false
		}
		result.sides[i] = cert
	}
	if count != 2 || result.insideSegment < 0 || result.outsideWitness < 0 {
		return empty, false
	}
	left := (result.insideSegment + len(src.entities) - 1) % len(src.entities)
	right := (result.insideSegment + 1) % len(src.entities)
	if (result.contacts[0].segment != left || result.contacts[1].segment != right) &&
		(result.contacts[0].segment != right || result.contacts[1].segment != left) {
		return empty, false
	}
	return result, true
}

// revolveImplicitDisplacement bounds the source fan's two-sided held-mesh
// departure against the reported cone, including the finite meridian gap.
func revolveImplicitDisplacement(section revolveSectionCertificate,
	base facetedPayload) (float64, bool) {
	if !proofbound.FiniteVec(r3.Vec{X: base.meshBound}) || base.meshBound < 0 ||
		section.rootStart.U <= 0 || len(base.Verts) < 4 {
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
	minX = math.Nextafter(minX-base.meshBound, math.Inf(-1))
	maxX = proofbound.AbsSumUpper(maxX, base.meshBound)
	if !(minX > section.rootCone.Origin.X) {
		return 0, false
	}
	slope := section.coneSection().Slope
	d := proofbound.AbsSumUpper(base.meshBound,
		revolveMeridianExtensionGap(section, minX, maxX))
	r := proofbound.AbsSumUpper(reach, d)
	cost := proofbound.ProductUpper(proofbound.AbsSumUpper(1,
		proofbound.ProductUpper(slope, slope)),
		proofbound.ProductUpper(d, proofbound.AbsSumUpper(proofbound.ProductUpper(2, r), d)))
	return cost, !proofbound.IsNonFinite(cost)
}

func revolveMeridianExtensionGap(section revolveSectionCertificate, xLo, xHi float64) float64 {
	x0, x1 := proofarith.FloatRat(section.rootStart.U), proofarith.FloatRat(section.rootEnd.U)
	r0, r1 := proofarith.FloatRat(section.rootStart.V), proofarith.FloatRat(section.rootEnd.V)
	m := new(big.Rat).Quo(new(big.Rat).Sub(r1, r0), new(big.Rat).Sub(x1, x0))
	b := new(big.Rat).Sub(r0, new(big.Rat).Mul(m, x0))
	slope := proofarith.FloatRat(section.coneSection().Slope)
	originX := proofarith.FloatRat(section.rootCone.Origin.X)
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
