package decad

import (
	"context"
	"math/big"
)

// A sloped finite Revolve generator intersects the heel tool cone at one
// constant-X circle. The live root-side face and both cone limits supply the
// exact radial interval; source cells then locate the true Sketch crossings.
func revolveLoftFacesHeelSlopedOccupiedArc(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate,
	circle revolveLoftFacesToolCapCircle) (
	*revolveLoftFacesToolCapOccupiedArc, bool, error,
) {
	if ctx == nil || proof == nil || proof.blank == nil || proof.tooth == nil ||
		proof.trim == nil || proof.trim.toe == nil || proof.trim.heel == nil ||
		circle.tool != 1 || circle.sourceMiss || circle.x == nil ||
		circle.radius == nil || circle.radius.Sign() <= 0 ||
		circle.span < 0 || circle.span >= len(proof.section.spans) ||
		circle.blankFace == nil || circle.capFace == nil ||
		circle.capFace != proof.heelFace ||
		!revolveLoftFacesSectionCurrent(proof) ||
		!revolveLoftFacesSectionFacesComplete(proof) {
		return nil, false, nil
	}
	if err := ctx.Err(); err != nil {
		return nil, false, err
	}
	trim := proof.trim
	payload, ok := proof.tooth.n.Payload.(facetedPayload)
	if !ok || payload.loftFacesTrim != trim ||
		!revolveLoftFacesTrimCurrent(proof.tooth, trim) ||
		trim.heelCapGroup < 0 || trim.heelCapGroup >= len(payload.groups) ||
		len(payload.groups[trim.heelCapGroup].origins) != 1 ||
		payload.groups[trim.heelCapGroup].origins[0] != circle.capOrigin ||
		!revolveLoftFacesToolCapFaceOriginCurrent(circle) ||
		uniqueResultSourceFace(proof.tooth, trim.heelCapGroup) != circle.capFace {
		return nil, false, nil
	}
	span := proof.section.spans[circle.span]
	if span.start.U == span.end.U || span.face != circle.blankFace ||
		span.face.n.Body != bodyNode(proof.blank) || len(span.origins) == 0 {
		return nil, false, nil
	}
	face, live := revolveSpanFace(proof.blank, span)
	if !live || face != circle.blankFace ||
		!verticalOriginsMatch(face.originRefs(), span.origins) {
		return nil, false, nil
	}
	x, radius, crosses, decided := revolveLoftFacesToolCircleOnSpan(trim.heel, span)
	if !decided || !crosses || x.Cmp(circle.x) != 0 ||
		radius.Cmp(circle.radius) != 0 {
		return nil, false, nil
	}
	if !revolveLoftFacesSlopedRadialInside(proof.section, circle.span, x, radius) {
		return nil, false, nil
	}
	if trim.toe.apex == nil || trim.toe.far == nil || trim.toe.slope == nil ||
		trim.toe.slope.Sign() <= 0 || x.Cmp(trim.toe.far) <= 0 ||
		x.Cmp(trim.heel.far) <= 0 || x.Cmp(trim.heel.apex) >= 0 {
		return nil, false, nil
	}
	lower := lfZero()
	if x.Cmp(trim.toe.apex) < 0 {
		lower = lfDiv(lfSub(trim.toe.apex, x), trim.toe.slope)
	}
	if lower.Sign() < 0 || lower.Cmp(radius) >= 0 {
		return nil, false, nil
	}
	section, ok, err := revolveLoftFacesConstantXSourceCells(ctx, proof,
		x, lower, radius)
	if err != nil || !ok {
		return nil, false, err
	}
	return revolveLoftFacesHeelOccupiedFromSource(ctx, proof, circle, section)
}

// At an X value away from every section vertex, two and only two transverse
// boundary crossings certify the local radial slice of a whole-line profile.
// The axis walks toward +X and the selected outer side walks toward -X.
// Their signed crossings make the open interval (0,radius) the filled side;
// an extra side crossing, vertex touch, or reversed walk is undecided.
func revolveLoftFacesSlopedRadialInside(section revolveSectionCertificate,
	spanIndex int, x, radius *big.Rat) bool {
	if x == nil || radius == nil || radius.Sign() <= 0 ||
		spanIndex < 0 || spanIndex >= len(section.spans) ||
		section.axisSpan < 0 || section.axisSpan >= len(section.spans) ||
		spanIndex == section.axisSpan {
		return false
	}
	axis := section.spans[section.axisSpan]
	if axis.start.V != 0 || axis.end.V != 0 ||
		lfF(axis.start.U).Cmp(x) >= 0 || lfF(axis.end.U).Cmp(x) <= 0 {
		return false
	}
	found := false
	for i, span := range section.spans {
		startX, endX := lfF(span.start.U), lfF(span.end.U)
		if startX.Cmp(x) == 0 || endX.Cmp(x) == 0 {
			return false
		}
		if i == section.axisSpan {
			continue
		}
		if startX.Cmp(x) == endX.Cmp(x) {
			continue
		}
		if i != spanIndex || startX.Cmp(x) <= 0 || endX.Cmp(x) >= 0 ||
			span.start.V < 0 || span.end.V < 0 {
			return false
		}
		fraction := lfDiv(lfSub(x, startX), lfSub(endX, startX))
		atX := lfAdd(lfF(span.start.V), lfMul(fraction,
			lfSub(lfF(span.end.V), lfF(span.start.V))))
		if atX.Cmp(radius) != 0 {
			return false
		}
		found = true
	}
	return found
}
