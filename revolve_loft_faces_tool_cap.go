package decad

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/provenance"
)

// A tool-cap circle is the exact intersection of one authenticated finite
// Revolve generator and one cone used to trim the source LoftFaces body. It
// says where a contact could occur; a separate source-region proof must show
// which part of the circle belongs to the surviving tool cap.
type revolveLoftFacesToolCapCircle struct {
	blankFace  *Face
	capFace    *Face
	capOrigin  provenance.Ref
	span       int
	tool       int // 0 is the Cut cone, 1 is the Intersect cone.
	x, radius  *big.Rat
	sourceMiss bool
}

type revolveLoftFacesToolCapSides struct {
	circles   []revolveLoftFacesToolCapCircle
	separated map[[2]*Face]struct{}
}

// The exact affine cone residual on a straight meridian can vanish once.
// Strict endpoint signs separate a complete Revolve face from the cone. A
// zero strictly inside the finite tool cone identifies a possible contact
// circle, including on a vertical section generator.
func revolveLoftFacesToolCapSidesOf(proof *revolveLoftFacesSourceCertificate) (
	revolveLoftFacesToolCapSides, bool,
) {
	var out revolveLoftFacesToolCapSides
	if proof == nil || proof.blank == nil || proof.tooth == nil ||
		proof.section.blank != proof.blank || proof.trim == nil ||
		!revolveLoftFacesSectionCurrent(proof) ||
		!revolveLoftFacesSectionFacesComplete(proof) ||
		proof.toeFace == nil || proof.heelFace == nil ||
		proof.toeFace == proof.heelFace ||
		proof.toeFace.n.Body != bodyNode(proof.tooth) || proof.heelFace.n.Body != bodyNode(proof.tooth) {
		return out, false
	}
	p, ok := proof.tooth.n.Payload.(facetedPayload)
	if !ok || p.loftFacesTrim != proof.trim ||
		!revolveLoftFacesTrimCurrent(proof.tooth, proof.trim) ||
		uniqueResultSourceFace(proof.tooth, proof.trim.toeCapGroup) != proof.toeFace ||
		uniqueResultSourceFace(proof.tooth, proof.trim.heelCapGroup) != proof.heelFace {
		return out, false
	}
	tools := [...]struct {
		limit *pointConeLimit
		face  *Face
		group int
	}{{proof.trim.toe, proof.toeFace, proof.trim.toeCapGroup},
		{proof.trim.heel, proof.heelFace, proof.trim.heelCapGroup}}
	out.separated = make(map[[2]*Face]struct{})
	for toolIndex, tool := range tools {
		if tool.limit == nil || tool.limit.apex == nil || tool.limit.far == nil ||
			tool.limit.slope == nil || tool.limit.slope.Sign() <= 0 ||
			tool.limit.far.Cmp(tool.limit.apex) >= 0 ||
			tool.group < 0 || tool.group >= len(p.groups) ||
			len(p.groups[tool.group].origins) != 1 {
			return revolveLoftFacesToolCapSides{}, false
		}
		origin := p.groups[tool.group].origins[0]
		found := false
		for _, faceOrigin := range tool.face.originRefs() {
			found = found || faceOrigin == origin
		}
		if !found {
			return revolveLoftFacesToolCapSides{}, false
		}
		for spanIndex, span := range proof.section.spans {
			if span.face == nil {
				continue // The axis span has no boundary face.
			}
			if span.face.n.Body != bodyNode(proof.blank) || span.start.V < 0 || span.end.V < 0 ||
				span.start == span.end {
				return revolveLoftFacesToolCapSides{}, false
			}
			x, radius, crosses, decided := revolveLoftFacesToolCircleOnSpan(tool.limit, span)
			if !decided {
				return revolveLoftFacesToolCapSides{}, false
			}
			if !crosses {
				out.separated[[2]*Face{span.face, tool.face}] = struct{}{}
				continue
			}
			out.circles = append(out.circles, revolveLoftFacesToolCapCircle{
				blankFace: span.face, capFace: tool.face, capOrigin: origin,
				span: spanIndex, tool: toolIndex, x: x, radius: radius,
				sourceMiss: revolveLoftFacesToolCircleOutsideSource(proof.trim, x, radius)})
		}
	}
	return out, true
}

func revolveLoftFacesToolCircleOnSpan(tool *pointConeLimit, span revolveSectionSpan) (
	*big.Rat, *big.Rat, bool, bool,
) {
	if tool == nil || tool.apex == nil || tool.far == nil || tool.slope == nil ||
		tool.slope.Sign() <= 0 || tool.far.Cmp(tool.apex) >= 0 ||
		span.start == span.end || span.start.V < 0 || span.end.V < 0 {
		return nil, nil, false, false
	}
	a := revolveLoftFacesToolResidual(tool, span.start)
	b := revolveLoftFacesToolResidual(tool, span.end)
	if a.Sign() != 0 && a.Sign() == b.Sign() {
		return nil, nil, false, true
	}
	if a.Sign() == 0 || b.Sign() == 0 || a.Cmp(b) == 0 {
		return nil, nil, false, false
	}
	fraction := lfDiv(a, lfSub(a, b))
	if fraction.Sign() <= 0 || fraction.Cmp(lfOne()) >= 0 {
		return nil, nil, false, false
	}
	x := lfAdd(lfF(span.start.U), lfMul(fraction,
		lfSub(lfF(span.end.U), lfF(span.start.U))))
	radius := lfAdd(lfF(span.start.V), lfMul(fraction,
		lfSub(lfF(span.end.V), lfF(span.start.V))))
	if radius.Sign() <= 0 || x.Cmp(tool.far) <= 0 || x.Cmp(tool.apex) >= 0 {
		return nil, nil, false, false
	}
	return x, radius, true, true
}

func revolveLoftFacesToolResidual(tool *pointConeLimit, point planePoint) *big.Rat {
	return lfSub(lfAdd(lfF(point.U), lfMul(tool.slope, lfF(point.V))), tool.apex)
}

// Every true LoftFaces band is inside the image of the authenticated source
// rectangle. A circle missed by every full band image cannot meet the
// surviving Cut/Intersect cap. Exact coordinate hulls intentionally include
// more than the actual Sketch region and therefore only prove misses.
func revolveLoftFacesToolCircleOutsideSource(trim *loftFacesTrimRecord,
	x, radius *big.Rat) bool {
	if trim == nil || trim.source == nil || x == nil || radius == nil ||
		radius.Sign() <= 0 {
		return false
	}
	box, err := lfSourceBox(trim.source)
	if err != nil {
		return false
	}
	return loftface.CircleOutsideMaps(trim.maps, box, x, radius)
}
