package decad

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/r3"
)

// revolveLoftJoinRecord retains one proved section/sector source and the
// bounds published by its joined mesh. Later placed sectors and cap-ring
// rewrites can authenticate against the same immutable source.
type revolveLoftJoinRecord struct {
	certificate                           *revolveLoftSectorCertificate
	blank                                 *Body
	first                                 facetedPayload
	boundaryBound, volumeBound, areaBound float64
}

type revolveLoftToothPlacement struct {
	source *pointConeTrimRecord
	motion r3.Transform
}

// tryRevolveLoftSectorUnion authenticates source geometry before the older
// shape-specific routes. The six-side conversion is a temporary construction
// adapter; the certificate itself admits an ordered section of any supported
// size and does not use a tooth-count or fixed profile-role matcher.
func tryRevolveLoftSectorUnion(ctx context.Context, op meshbool.OperationKind,
	d *Document, ref producerID, a, b *Body) (*Body, bool, error) {
	if op != meshbool.OpUnion {
		return nil, false, nil
	}
	blank, tooth := a, b
	cert, ok := certifyRevolveLoftSector(blank, tooth)
	if !ok {
		blank, tooth = b, a
		cert, ok = certifyRevolveLoftSector(blank, tooth)
		if !ok {
			return nil, false, nil
		}
	}
	legacyBlank, ok := legacyRevolveLoftBlank(cert.section)
	if !ok {
		return nil, false, nil
	}
	legacyBoundary, ok := legacyRevolveLoftBoundary(cert.boundary)
	if !ok {
		return nil, false, nil
	}
	path, bound, err := revolveExteriorPath(cert.base, cert.boundary,
		cert.trim.base.pointSection.source, cert.section)
	if err != nil {
		return nil, true, err
	}
	lo, hi, ok := revolveRootAngles(path)
	if !ok {
		return nil, true, ErrUnsupported
	}
	cert.sector, cert.crossingBound = [2]float64{lo, hi}, bound
	cert.loop = path
	body, err := buildBevelOneToothJoin(ctx, d, ref, blank, legacyBlank,
		cert.trimmed, cert.trim, legacyBoundary)
	if err != nil {
		return nil, true, err
	}
	payload, ok := body.payload.(facetedPayload)
	if !ok {
		return nil, true, ErrUnsupported
	}
	payload.revolveLoftJoin = &revolveLoftJoinRecord{certificate: cert, blank: blank,
		first: cert.trimmed, boundaryBound: payload.meshBound,
		volumeBound: payload.volSymDiff, areaBound: payload.areaSlack}
	body.payload = payload
	return body, true, nil
}

// legacyRevolveLoftBlank maps the first bounded construction class into the
// existing audited six-side mesh builder. Source admission happened above.
func legacyRevolveLoftBlank(section revolveSectionCertificate) (*bevelBlankSource, bool) {
	if len(section.spans) != 6 || section.rootCone.Origin.X != 0 {
		return nil, false
	}
	var axis, outer []Point2
	for _, span := range section.spans {
		point := span.start
		if point.V == 0 {
			axis = append(axis, point)
		} else {
			outer = append(outer, point)
		}
	}
	if len(axis) != 2 || len(outer) != 4 {
		return nil, false
	}
	if axis[0].U > axis[1].U {
		axis[0], axis[1] = axis[1], axis[0]
	}
	result := &bevelBlankSource{toeAxis: axis[0], heelAxis: axis[1]}
	var interior []Point2
	for _, point := range outer {
		switch point.U {
		case result.toeAxis.U:
			if result.toeOuter != (Point2{}) {
				return nil, false
			}
			result.toeOuter = point
		case result.heelAxis.U:
			if result.heelOuter != (Point2{}) {
				return nil, false
			}
			result.heelOuter = point
		default:
			interior = append(interior, point)
		}
	}
	if len(interior) != 2 || result.toeOuter == (Point2{}) || result.heelOuter == (Point2{}) {
		return nil, false
	}
	if interior[0].U > interior[1].U {
		interior[0], interior[1] = interior[1], interior[0]
	}
	result.toeRoot, result.ded = interior[0], interior[1]
	if !(result.toeRoot.U < result.toeAxis.U && result.toeAxis.U < result.ded.U &&
		result.ded.U < result.heelAxis.U) ||
		section.rootStart != result.toeRoot || section.rootEnd != result.ded {
		return nil, false
	}
	cycle := [6]Point2{result.toeAxis, result.heelAxis, result.heelOuter,
		result.ded, result.toeRoot, result.toeOuter}
	var seen [6]bool
	for _, span := range section.spans {
		matched := false
		for i := range cycle {
			a, z := cycle[i], cycle[(i+1)%len(cycle)]
			if span.start != a || span.end != z {
				if span.start != z || span.end != a {
					continue
				}
			}
			if seen[i] || len(span.sourceEdges) == 0 {
				return nil, false
			}
			seen[i], matched = true, true
			result.edgeIndex[i] = span.sourceEdges[0]
			if i == 3 {
				result.rootFace = span.face
			}
			break
		}
		if !matched {
			return nil, false
		}
	}
	if result.rootFace == nil {
		return nil, false
	}
	result.rootCone, result.coneGap = section.rootCone, section.coneGap
	return result, !math.IsNaN(result.coneGap) && !math.IsInf(result.coneGap, 0)
}

func legacyRevolveLoftBoundary(boundary revolveSectorBoundary) (*bevelToothBoundary, bool) {
	if len(boundary.sides) != 6 ||
		boundary.insideSegment < 0 || boundary.outsideWitness < 0 {
		return nil, false
	}
	result := &bevelToothBoundary{insideArc: boundary.insideSegment,
		fitRoute: boundary.fitRoute}
	for i := range result.crossing {
		if boundary.fitRoute {
			result.fitIndex[i], result.fitCross[i] = boundary.contacts[i].segment,
				boundary.contacts[i].fit
		} else {
			result.lineIndex[i], result.crossing[i] = boundary.contacts[i].segment,
				boundary.contacts[i].line
		}
	}
	copy(result.sides[:], boundary.sides)
	return result, true
}
