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
		proof.trim == nil || proof.trim.Toe == nil || proof.trim.Heel == nil ||
		circle.tool != 1 || circle.sourceMiss || circle.x == nil ||
		circle.radius == nil || circle.radius.Sign() <= 0 ||
		circle.span < 0 || circle.span >= len(proof.section.Spans) ||
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
	if !ok || payload.LoftFacesTrim != trim ||
		!revolveLoftFacesTrimCurrent(proof.tooth, trim) ||
		trim.HeelCapGroup < 0 || trim.HeelCapGroup >= len(payload.Groups) ||
		len(payload.Groups[trim.HeelCapGroup].Origins) != 1 ||
		payload.Groups[trim.HeelCapGroup].Origins[0] != circle.capOrigin ||
		!revolveLoftFacesToolCapFaceOriginCurrent(circle) ||
		uniqueResultSourceFace(proof.tooth, trim.HeelCapGroup) != circle.capFace {
		return nil, false, nil
	}
	span := proof.section.Spans[circle.span]
	if span.Start.U == span.End.U || span.Face != circle.blankFace ||
		span.Face.n.Body != bodyNode(proof.blank) || len(span.Origins) == 0 {
		return nil, false, nil
	}
	face, live := revolveSpanFace(proof.blank, span)
	if !live || face != circle.blankFace ||
		!verticalOriginsMatch(face.originRefs(), span.Origins) {
		return nil, false, nil
	}
	x, radius, crosses, decided := revolveLoftFacesToolCircleOnSpan(trim.Heel, span)
	if !decided || !crosses || x.Cmp(circle.x) != 0 ||
		radius.Cmp(circle.radius) != 0 {
		return nil, false, nil
	}
	if !revolveLoftFacesSlopedRadialInside(proof.section, circle.span, x, radius) {
		return nil, false, nil
	}
	if trim.Toe.apex == nil || trim.Toe.far == nil || trim.Toe.slope == nil ||
		trim.Toe.slope.Sign() <= 0 || x.Cmp(trim.Toe.far) <= 0 ||
		x.Cmp(trim.Heel.far) <= 0 || x.Cmp(trim.Heel.apex) >= 0 {
		return nil, false, nil
	}
	lower := lfZero()
	if x.Cmp(trim.Toe.apex) < 0 {
		lower = lfDiv(lfSub(trim.Toe.apex, x), trim.Toe.slope)
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
		spanIndex < 0 || spanIndex >= len(section.Spans) ||
		section.AxisSpan < 0 || section.AxisSpan >= len(section.Spans) ||
		spanIndex == section.AxisSpan {
		return false
	}
	axis := section.Spans[section.AxisSpan]
	if axis.Start.V != 0 || axis.End.V != 0 ||
		lfF(axis.Start.U).Cmp(x) >= 0 || lfF(axis.End.U).Cmp(x) <= 0 {
		return false
	}
	found := false
	for i, span := range section.Spans {
		startX, endX := lfF(span.Start.U), lfF(span.End.U)
		if startX.Cmp(x) == 0 || endX.Cmp(x) == 0 {
			return false
		}
		if i == section.AxisSpan {
			continue
		}
		if startX.Cmp(x) == endX.Cmp(x) {
			continue
		}
		if i != spanIndex || startX.Cmp(x) <= 0 || endX.Cmp(x) >= 0 ||
			span.Start.V < 0 || span.End.V < 0 {
			return false
		}
		fraction := lfDiv(lfSub(x, startX), lfSub(endX, startX))
		atX := lfAdd(lfF(span.Start.V), lfMul(fraction,
			lfSub(lfF(span.End.V), lfF(span.Start.V))))
		if atX.Cmp(radius) != 0 {
			return false
		}
		found = true
	}
	return found
}
