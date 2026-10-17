package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
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
	if span.Start.U == span.End.U || span.Face != faceNode(circle.blankFace) ||
		span.Face.Body != bodyNode(proof.blank) || len(span.Origins) == 0 {
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
	if !revolveloftfaces.SlopedRadialInside(proof.section, circle.span, x, radius) {
		return nil, false, nil
	}
	if trim.Toe.Apex == nil || trim.Toe.Far == nil || trim.Toe.Slope == nil ||
		trim.Toe.Slope.Sign() <= 0 || x.Cmp(trim.Toe.Far) <= 0 ||
		x.Cmp(trim.Heel.Far) <= 0 || x.Cmp(trim.Heel.Apex) >= 0 {
		return nil, false, nil
	}
	lower := lfZero()
	if x.Cmp(trim.Toe.Apex) < 0 {
		lower = lfDiv(lfSub(trim.Toe.Apex, x), trim.Toe.Slope)
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
