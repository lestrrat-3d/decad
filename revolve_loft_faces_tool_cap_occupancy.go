package decad

import (
	"context"
	"slices"

	"github.com/lestrrat-3d/sketch"
)

// A boundary point is the exact circle/source intersection enclosed by its
// rational source parameter and mapped XYZ boxes. The true point need not
// have rational coordinates. The referenced LoftFaces face is live.
type revolveLoftFacesToolCapBoundary struct {
	segment, span int
	ref           FeatureRef
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
	if ctx == nil || proof == nil || proof.tooth == nil || proof.trim == nil || proof.trim.source == nil ||
		proof.trim.source.source == nil || proof.trim.source.source.profile == nil ||
		circle.tool != 1 || circle.sourceMiss || circle.x == nil || circle.radius == nil ||
		circle.span < 0 || circle.span >= len(proof.section.spans) ||
		circle.blankFace == nil || circle.capFace != proof.heelFace ||
		!revolveLoftFacesSectionFacesComplete(proof) ||
		!proof.trim.source.source.profile.Valid ||
		len(proof.trim.source.source.profile.Holes) != 0 {
		return nil, false, nil
	}
	x, radius, crosses, decided := revolveLoftFacesToolCircleOnSpan(
		proof.trim.heel, proof.section.spans[circle.span])
	if !decided || !crosses || x.Cmp(circle.x) != 0 ||
		radius.Cmp(circle.radius) != 0 {
		return nil, false, nil
	}
	payload, ok := proof.tooth.payload.(facetedPayload)
	if !ok || payload.loftFacesTrim != proof.trim ||
		proof.trim.heelCapGroup < 0 || proof.trim.heelCapGroup >= len(payload.groups) ||
		len(payload.groups[proof.trim.heelCapGroup].origins) != 1 ||
		payload.groups[proof.trim.heelCapGroup].origins[0] != circle.capOrigin ||
		!revolveLoftFacesToolCapFaceOriginCurrent(circle) ||
		uniqueResultSourceFace(proof.tooth, proof.trim.heelCapGroup) != circle.capFace {
		return nil, false, nil
	}
	vertical, outcome, err := revolveLoftFacesVerticalSideClassify(ctx, proof, circle.span)
	if err != nil || outcome != revolveLoftFacesVerticalContact || vertical == nil {
		return nil, false, err
	}
	if vertical.face != circle.blankFace || vertical.x.Cmp(circle.x) != 0 ||
		vertical.upper.Cmp(circle.radius) != 0 ||
		vertical.lower.Cmp(circle.radius) >= 0 ||
		vertical.yzAffine == nil || vertical.yzAffine.det == nil ||
		vertical.yzAffine.det.Sign() <= 0 {
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
		section.yzAffine == nil || section.yzAffine.det == nil ||
		section.yzAffine.det.Sign() <= 0 {
		return nil, false, nil
	}
	loop, simple, err := revolveLoftFacesSimpleSourceLoop(ctx, proof.trim)
	if err != nil || !simple || loop == nil || !loop.ccw {
		return nil, false, err
	}
	outer := proof.trim.source.source.profile.Outer
	var negative, positive *revolveLoftFacesToolCapBoundary
	for _, cell := range section.cells {
		if cell.segment < 0 || cell.segment >= len(outer) {
			return nil, false, nil
		}
		if cell.upper.root.lo == nil {
			if cell.upper.side != -1 && cell.upper.side != 1 {
				return nil, false, nil
			}
			continue
		}
		root := cell.upper.root
		if root.hi == nil || root.lo.Sign() <= 0 || root.lo.Cmp(root.hi) >= 0 ||
			root.hi.Cmp(lfOne()) >= 0 || cell.face == nil ||
			cell.upper.before*cell.upper.after != -1 ||
			cell.upper.box[0].lo == nil || cell.upper.box[0].hi == nil ||
			cell.upper.box[0].lo.Cmp(circle.x) != 0 ||
			cell.upper.box[0].hi.Cmp(circle.x) != 0 ||
			cell.upper.box[1].lo == nil || cell.upper.box[1].lo.Sign() <= 0 ||
			cell.upper.box[2].lo == nil || cell.upper.box[2].hi == nil {
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
			face: cell.face, parameter: root, box: cell.upper.box,
			outward: outward}
		switch {
		case boundary.box[2].hi.Sign() < 0 && negative == nil:
			negative = boundary
		case boundary.box[2].lo.Sign() > 0 && positive == nil:
			positive = boundary
		default:
			return nil, false, nil
		}
	}
	if negative == nil || positive == nil || negative.outward == positive.outward {
		return nil, false, nil
	}
	return &revolveLoftFacesToolCapOccupiedArc{circle: circle,
		negativeZ: *negative, positiveZ: *positive,
		throughPositiveY: negative.outward}, true, nil
}

func revolveLoftFacesToolCapFaceOriginCurrent(circle revolveLoftFacesToolCapCircle) bool {
	return circle.capFace != nil && slices.Contains(circle.capFace.Origins(), circle.capOrigin)
}

func revolveLoftFacesToolBoundaryOutward(edge sketch.BoundaryEdge,
	limit revolveLoftFacesVerticalLimit) (bool, bool) {
	if edge.Partial || limit.before*limit.after != -1 {
		return false, false
	}
	before, after := limit.before, limit.after
	if edge.Reversed {
		before, after = after, before
	}
	return before < 0 && after > 0, true
}
