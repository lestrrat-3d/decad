package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/sketch"
)

// The source root can move along a whole adjacent span inside its isolated
// time bracket. The certified one-sided joint cell, not the Arc endpoint,
// supplies the source-parameter interval for its mapped XYZ enclosure.
func revolveLoftFacesHeelRootBrackets(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate, meridian revolveSectionSpan,
	band int, root loftFaceInterval, arc *sketch.Arc,
	segments [2]int, ends [2]*sketch.Point, fitted bool,
	budget *big.Rat) ([2]*revolveLoftFacesHeelRootEnclosure, bool) {
	var out [2]*revolveLoftFacesHeelRootEnclosure
	if ctx == nil || proof == nil || proof.trim == nil || root.lo == nil ||
		root.hi == nil || budget == nil || budget.Sign() <= 0 ||
		root.lo.Cmp(root.hi) > 0 || arc == nil ||
		band < 0 || band+1 >= len(proof.trim.maps) ||
		band >= len(proof.sourceRefs) {
		return out, false
	}
	arcIndex := -1
	for index, entity := range proof.trim.source.source.entities {
		if entity == arc {
			arcIndex = index
		}
	}
	if arcIndex < 0 {
		return out, false
	}
	for neighbor, segment := range segments {
		if err := ctx.Err(); err != nil {
			return [2]*revolveLoftFacesHeelRootEnclosure{}, false
		}
		if segment < 0 || segment >= len(proof.sourceRefs[band]) ||
			ends[neighbor] == nil {
			return [2]*revolveLoftFacesHeelRootEnclosure{}, false
		}
		atWalkEnd := neighbor == 0
		var span freeform.BezierSpan
		var sourceSpan int
		var roots []revolveLoftFacesEndpointRoot
		var cells []revolveLoftFacesJointCell
		var atSourceEnd bool
		if fitted {
			joint, ok := certifyRevolveLoftFacesRadialJoint(ctx, proof.trim,
				meridian, segment, arcIndex, band, atWalkEnd)
			if !ok || joint.vertex.point != ends[neighbor] ||
				joint.radial.arc != arc {
				return [2]*revolveLoftFacesHeelRootEnclosure{}, false
			}
			span, roots, cells = joint.vertex.fitSpan, joint.roots, joint.fitCells
			atSourceEnd = joint.vertex.fitAtEnd
			spans, ok := revolveLoftFacesStraightOrFitSpans(proof.trim, segment,
				proof.trim.source.profile.Outer.Segments[segment])
			if !ok {
				return [2]*revolveLoftFacesHeelRootEnclosure{}, false
			}
			sourceSpan = -1
			for index, candidate := range spans {
				if revolveLoftFacesSameSpan(candidate, span) {
					if sourceSpan >= 0 {
						return [2]*revolveLoftFacesHeelRootEnclosure{}, false
					}
					sourceSpan = index
				}
			}
		} else {
			joint, ok := certifyRevolveLoftFacesLineArcJoint(ctx, proof.trim,
				meridian, segment, arcIndex, band, atWalkEnd)
			if !ok || joint.vertex.point != ends[neighbor] ||
				joint.vertex.arc != arc {
				return [2]*revolveLoftFacesHeelRootEnclosure{}, false
			}
			span, roots, cells = joint.vertex.lineSpan, joint.roots, joint.lineCells
			atSourceEnd = joint.vertex.lineAtEnd
		}
		if sourceSpan < 0 || len(cells) != len(roots) {
			return [2]*revolveLoftFacesHeelRootEnclosure{}, false
		}
		matched := -1
		for index, candidate := range roots {
			if candidate.finite && candidate.time.lo.Cmp(root.lo) == 0 &&
				candidate.time.hi.Cmp(root.hi) == 0 {
				if matched >= 0 {
					return [2]*revolveLoftFacesHeelRootEnclosure{}, false
				}
				matched = index
			}
		}
		if matched < 0 {
			return [2]*revolveLoftFacesHeelRootEnclosure{}, false
		}
		cell := cells[matched]
		box, source, bounded := revolveLoftFacesHeelJointSliverBox(
			proof.trim.maps[band], proof.trim.maps[band+1], span,
			meridian, cell, atSourceEnd, budget)
		if !bounded {
			return [2]*revolveLoftFacesHeelRootEnclosure{}, false
		}
		ref := proof.sourceRefs[band][segment]
		face := revolveLoftFacesVerticalLiveFace(proof.tooth, ref)
		if face == nil {
			return [2]*revolveLoftFacesHeelRootEnclosure{}, false
		}
		event := -1
		if box[2].lo.Sign() > 0 {
			event = 0
		} else if box[2].hi.Sign() < 0 {
			event = 1
		}
		if event < 0 || out[event] != nil || box[1].lo.Sign() <= 0 {
			return [2]*revolveLoftFacesHeelRootEnclosure{}, false
		}
		member := revolveLoftFacesHeelRootMember{segment: segment, span: sourceSpan,
			band: band, ref: ref, face: face,
			source: source, box: box}
		out[event] = &revolveLoftFacesHeelRootEnclosure{
			time: root, members: []revolveLoftFacesHeelRootMember{member}, box: box}
	}
	return out, out[0] != nil && out[1] != nil
}

func revolveLoftFacesHeelJointSliverBox(first, last loftFaceMap,
	span freeform.BezierSpan, meridian revolveSectionSpan,
	cell revolveLoftFacesJointCell, atEnd bool,
	budget *big.Rat) ([3]loftFaceInterval, loftFaceInterval, bool) {
	var empty [3]loftFaceInterval
	if len(span) == 0 || cell.sLo == nil || cell.sHi == nil ||
		cell.time.lo == nil || cell.time.hi == nil || budget == nil || budget.Sign() <= 0 ||
		!cell.finite || cell.sLo.Sign() < 0 || cell.sLo.Cmp(cell.sHi) >= 0 ||
		cell.sHi.Cmp(lfOne()) > 0 || cell.time.lo.Cmp(cell.time.hi) > 0 ||
		cell.outsideSide != -1 && cell.outsideSide != 1 ||
		cell.sDerivative != -1 && cell.sDerivative != 1 {
		return empty, loftFaceInterval{}, false
	}
	m, b := revolveLoftFacesMeridian(meridian)
	q := spiralRootPolynomial(first, last, span, m, b)
	for depth := range 33 {
		width := big.NewRat(1, int64(1)<<depth)
		sLo, sHi := lfZero(), width
		if atEnd {
			sLo, sHi = lfSub(lfOne(), width), lfOne()
		}
		if sLo.Cmp(cell.sLo) < 0 || sHi.Cmp(cell.sHi) > 0 {
			continue
		}
		sliver := revolveLoftFacesRestrictBernstein(q, sLo, sHi,
			cell.time.lo, cell.time.hi)
		var derivative [3][]*big.Rat
		for row := range sliver {
			derivative[row] = spiralDeriv(sliver[row])
		}
		if cell.sDerivative > 0 && !spiralAllPositive(derivative) ||
			cell.sDerivative < 0 &&
				!spiralAllPositive(revolveLoftFacesNegateRows(derivative)) {
			continue
		}
		fixed := lfZero()
		if !atEnd {
			fixed = lfOne()
		}
		if revolveLoftFacesTimePointSign(sliver, fixed) != cell.outsideSide {
			continue
		}
		if depth > 0 {
			outerLo, outerHi := lfZero(), sLo
			if !atEnd {
				outerLo, outerHi = sHi, lfOne()
			}
			outer := revolveLoftFacesRestrictBernstein(q, outerLo, outerHi,
				cell.time.lo, cell.time.hi)
			if cell.outsideSide > 0 && !spiralAllPositive(outer) ||
				cell.outsideSide < 0 &&
					!spiralAllPositive(revolveLoftFacesNegateRows(outer)) {
				continue
			}
		}
		box, ok := spiralJunctionBox(first, last, span,
			sLo, sHi, cell.time.lo, cell.time.hi)
		if !ok {
			continue
		}
		radius := revolveLoftFacesHeelBoxRadius(box)
		if radius != nil && radius.Cmp(budget) <= 0 {
			return box, loftFaceInterval{sLo, sHi}, true
		}
	}
	return empty, loftFaceInterval{}, false
}
