package decad

import (
	"context"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/sketch"
)

// A boundary point is the exact circle/source intersection enclosed by its
// rational source parameter and mapped XYZ boxes. The true point need not
// have rational coordinates. The referenced LoftFaces face is live.
type revolveLoftFacesToolCapBoundary struct {
	segment, span int
	ref           provenance.Ref
	face          *Face
	parameter     loftFaceInterval
	box           [3]loftFaceInterval
	outward       bool // radial residual increases in the Sketch outer-loop walk.
}

// Between two transverse source-boundary crossings, the circle has one
// occupied open arc. Both endpoints lie on positive Y, one on each side of
// Z=0; throughPositiveY selects the short arc through that axis.
type revolveLoftFacesToolCapOccupiedArc struct {
	circle           revolveLoftFacesToolCapCircle
	band             int
	time             *big.Rat // exact constant-X time within the authenticated cap band
	negativeZ        revolveLoftFacesToolCapBoundary
	positiveZ        revolveLoftFacesToolCapBoundary
	throughPositiveY bool
}

// A separately certified exact simple, counter-clockwise source loop and
// orientation-preserving affine cap map keep the true interior on the left
// of the mapped walk. At a circle crossing, the mapped radial derivative
// tells which counter-clockwise circle arc lies inside. Exactly two
// transverse crossings therefore certify one occupied arc.
func revolveLoftFacesHeelVerticalOccupiedArc(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate,
	circle revolveLoftFacesToolCapCircle) (
	*revolveLoftFacesToolCapOccupiedArc, bool, error,
) {
	if ctx == nil || proof == nil || proof.tooth == nil || proof.trim == nil || proof.trim.Source == nil ||
		proof.trim.Source.Source == nil || proof.trim.Source.Source.Profile == nil ||
		circle.tool != 1 || circle.sourceMiss || circle.x == nil || circle.radius == nil ||
		circle.span < 0 || circle.span >= len(proof.section.Spans) ||
		circle.blankFace == nil || circle.capFace != proof.heelFace ||
		!revolveLoftFacesSectionFacesComplete(proof) ||
		!proof.trim.Source.Source.Profile.Valid ||
		len(proof.trim.Source.Source.Profile.Holes) != 0 {
		return nil, false, nil
	}
	x, radius, crosses, decided := revolveLoftFacesToolCircleOnSpan(
		proof.trim.Heel, proof.section.Spans[circle.span])
	if !decided || !crosses || x.Cmp(circle.x) != 0 ||
		radius.Cmp(circle.radius) != 0 {
		return nil, false, nil
	}
	payload, ok := proof.tooth.n.Payload.(facetedPayload)
	if !ok || payload.LoftFacesTrim != proof.trim ||
		proof.trim.HeelCapGroup < 0 || proof.trim.HeelCapGroup >= len(payload.Groups) ||
		len(payload.Groups[proof.trim.HeelCapGroup].Origins) != 1 ||
		payload.Groups[proof.trim.HeelCapGroup].Origins[0] != circle.capOrigin ||
		!revolveLoftFacesToolCapFaceOriginCurrent(circle) ||
		uniqueResultSourceFace(proof.tooth, proof.trim.HeelCapGroup) != circle.capFace {
		return nil, false, nil
	}
	vertical, outcome, err := revolveLoftFacesVerticalSideClassify(ctx, proof, circle.span)
	if err != nil || outcome != revolveLoftFacesVerticalContact || vertical == nil {
		return nil, false, err
	}
	if vertical.face != circle.blankFace || vertical.x.Cmp(circle.x) != 0 ||
		vertical.upper.Cmp(circle.radius) != 0 ||
		vertical.lower.Cmp(circle.radius) >= 0 ||
		vertical.yzAffine == nil || vertical.yzAffine.Det == nil ||
		vertical.yzAffine.Det.Sign() <= 0 {
		return nil, false, nil
	}
	return revolveLoftFacesHeelOccupiedFromSource(ctx, proof, circle,
		vertical.revolveLoftFacesConstantXSource)
}

// Both vertical and sloped blank generators meet a heel tool cap at a
// constant-X circle. Exact source cells and a simple true boundary determine
// the occupied angular arc independently of the generator slope.
func revolveLoftFacesHeelOccupiedFromSource(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate, circle revolveLoftFacesToolCapCircle,
	section *revolveLoftFacesConstantXSource) (
	*revolveLoftFacesToolCapOccupiedArc, bool, error,
) {
	if proof == nil || proof.trim == nil || circle.x == nil || circle.radius == nil ||
		section == nil || section.x == nil || section.upper == nil ||
		section.band < 0 || section.band >= len(proof.sourceRefs) ||
		section.x.Cmp(circle.x) != 0 ||
		section.upper.Cmp(circle.radius) != 0 || !section.hasRetained ||
		section.yzAffine == nil || section.yzAffine.Det == nil ||
		section.yzAffine.Det.Sign() <= 0 {
		return nil, false, nil
	}
	current, err := revolveLoftFacesToolCapSourceCurrent(ctx, proof, circle, section)
	if err != nil || !current {
		return nil, false, err
	}
	loop, simple, err := revolveLoftFacesSimpleSourceLoop(ctx, proof.trim)
	if err != nil || !simple || loop == nil || !loop.ccw {
		return nil, false, err
	}
	outer := proof.trim.Source.Source.Profile.Outer
	var negative, positive *revolveLoftFacesToolCapBoundary
	for _, cell := range section.cells {
		if cell.segment < 0 || cell.segment >= len(outer) {
			return nil, false, nil
		}
		if cell.upper.Root.Lo == nil {
			if cell.upper.Side != -1 && cell.upper.Side != 1 {
				return nil, false, nil
			}
			continue
		}
		root := cell.upper.Root
		if root.Hi == nil || root.Lo.Sign() <= 0 || root.Lo.Cmp(root.Hi) >= 0 ||
			root.Hi.Cmp(lfOne()) >= 0 || cell.face == nil ||
			cell.upper.Before*cell.upper.After != -1 ||
			cell.upper.Box[0].Lo == nil || cell.upper.Box[0].Hi == nil ||
			cell.upper.Box[0].Lo.Cmp(circle.x) != 0 ||
			cell.upper.Box[0].Hi.Cmp(circle.x) != 0 ||
			cell.upper.Box[1].Lo == nil || cell.upper.Box[1].Lo.Sign() <= 0 ||
			cell.upper.Box[2].Lo == nil || cell.upper.Box[2].Hi == nil {
			return nil, false, nil
		}
		edge := outer[cell.segment]
		if proof.sourceRefs[section.band][cell.segment] != cell.ref {
			return nil, false, nil
		}
		outward, oriented := revolveLoftFacesToolBoundaryOutward(edge, cell.upper)
		if !oriented {
			return nil, false, nil
		}
		boundary := &revolveLoftFacesToolCapBoundary{
			segment: cell.segment, span: cell.span, ref: cell.ref,
			face: cell.face, parameter: root, box: cell.upper.Box,
			outward: outward}
		switch {
		case boundary.box[2].Hi.Sign() < 0 && negative == nil:
			negative = boundary
		case boundary.box[2].Lo.Sign() > 0 && positive == nil:
			positive = boundary
		default:
			return nil, false, nil
		}
	}
	if negative == nil || positive == nil || negative.outward == positive.outward {
		return nil, false, nil
	}
	return &revolveLoftFacesToolCapOccupiedArc{circle: circle,
		band: section.band, time: new(big.Rat).Set(section.time),
		negativeZ: *negative, positiveZ: *positive,
		throughPositiveY: negative.outward}, true, nil
}

// An occupied arc must use the current circle and the full live source
// classification. Checking only the two root boxes would miss changed side
// signs, which can reverse the selected occupied arc.
func revolveLoftFacesToolCapSourceCurrent(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate, circle revolveLoftFacesToolCapCircle,
	section *revolveLoftFacesConstantXSource) (bool, error) {
	if ctx == nil || proof == nil || proof.trim == nil || section == nil ||
		circle.x == nil || circle.radius == nil || circle.tool != 1 {
		return false, nil
	}
	sides, ok := revolveLoftFacesToolCapSidesOf(proof)
	if !ok {
		return false, nil
	}
	matched := false
	for _, live := range sides.circles {
		if live.blankFace == circle.blankFace && live.capFace == circle.capFace &&
			live.capOrigin == circle.capOrigin && live.span == circle.span &&
			live.tool == circle.tool && live.sourceMiss == circle.sourceMiss &&
			live.x.Cmp(circle.x) == 0 && live.radius.Cmp(circle.radius) == 0 {
			matched = true
			break
		}
	}
	if !matched || circle.sourceMiss || circle.span < 0 ||
		circle.span >= len(proof.section.Spans) {
		return false, nil
	}
	span := proof.section.Spans[circle.span]
	var fresh *revolveLoftFacesConstantXSource
	if span.Start.U == span.End.U {
		vertical, outcome, err := revolveLoftFacesVerticalSideClassify(ctx, proof, circle.span)
		if err != nil || outcome != revolveLoftFacesVerticalContact || vertical == nil ||
			vertical.face != circle.blankFace || vertical.x.Cmp(circle.x) != 0 ||
			vertical.upper.Cmp(circle.radius) != 0 {
			return false, err
		}
		fresh = vertical.revolveLoftFacesConstantXSource
	} else {
		if !revolveLoftFacesSlopedRadialInside(proof.section, circle.span,
			circle.x, circle.radius) || proof.trim.Toe == nil ||
			proof.trim.Toe.Apex == nil || proof.trim.Toe.Slope == nil ||
			proof.trim.Toe.Slope.Sign() <= 0 {
			return false, nil
		}
		lower := lfZero()
		if circle.x.Cmp(proof.trim.Toe.Apex) < 0 {
			lower = lfDiv(lfSub(proof.trim.Toe.Apex, circle.x), proof.trim.Toe.Slope)
		}
		var err error
		fresh, ok, err = revolveLoftFacesConstantXSourceCells(ctx, proof,
			circle.x, lower, circle.radius)
		if err != nil || !ok {
			return false, err
		}
	}
	return revolveLoftFacesToolCapSourceCellsEqual(section, fresh), nil
}

func revolveLoftFacesToolCapFaceOriginCurrent(circle revolveLoftFacesToolCapCircle) bool {
	return circle.capFace != nil && slices.Contains(circle.capFace.originRefs(), circle.capOrigin)
}

func revolveLoftFacesToolBoundaryOutward(edge sketch.BoundaryEdge,
	limit loftface.VerticalLimit) (bool, bool) {
	if edge.Partial || limit.Before*limit.After != -1 {
		return false, false
	}
	before, after := limit.Before, limit.After
	if edge.Reversed {
		before, after = after, before
	}
	return before < 0 && after > 0, true
}
