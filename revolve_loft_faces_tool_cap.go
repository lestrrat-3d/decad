package decad

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
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
		proof.section.Blank != bodyNode(proof.blank) || proof.trim == nil ||
		!revolveLoftFacesSectionCurrent(proof) ||
		!revolveLoftFacesSectionFacesComplete(proof) ||
		proof.toeFace == nil || proof.heelFace == nil ||
		proof.toeFace == proof.heelFace ||
		proof.toeFace.n.Body != bodyNode(proof.tooth) || proof.heelFace.n.Body != bodyNode(proof.tooth) {
		return out, false
	}
	p, ok := proof.tooth.n.Payload.(facetedPayload)
	if !ok || p.LoftFacesTrim != proof.trim ||
		!revolveLoftFacesTrimCurrent(proof.tooth, proof.trim) ||
		uniqueResultSourceFace(proof.tooth, proof.trim.ToeCapGroup) != proof.toeFace ||
		uniqueResultSourceFace(proof.tooth, proof.trim.HeelCapGroup) != proof.heelFace {
		return out, false
	}
	tools := [...]struct {
		limit *pointConeLimit
		face  *Face
		group int
	}{{proof.trim.Toe, proof.toeFace, proof.trim.ToeCapGroup},
		{proof.trim.Heel, proof.heelFace, proof.trim.HeelCapGroup}}
	out.separated = make(map[[2]*Face]struct{})
	for toolIndex, tool := range tools {
		if tool.limit == nil || tool.limit.Apex == nil || tool.limit.Far == nil ||
			tool.limit.Slope == nil || tool.limit.Slope.Sign() <= 0 ||
			tool.limit.Far.Cmp(tool.limit.Apex) >= 0 ||
			tool.group < 0 || tool.group >= len(p.Groups) ||
			len(p.Groups[tool.group].Origins) != 1 {
			return revolveLoftFacesToolCapSides{}, false
		}
		origin := p.Groups[tool.group].Origins[0]
		found := false
		for _, faceOrigin := range tool.face.originRefs() {
			found = found || faceOrigin == origin
		}
		if !found {
			return revolveLoftFacesToolCapSides{}, false
		}
		for spanIndex, span := range proof.section.Spans {
			if span.Face == nil {
				continue // The axis span has no boundary face.
			}
			if span.Face.Body != bodyNode(proof.blank) || span.Start.V < 0 || span.End.V < 0 ||
				span.Start == span.End {
				return revolveLoftFacesToolCapSides{}, false
			}
			x, radius, crosses, decided := revolveLoftFacesToolCircleOnSpan(tool.limit, span)
			if !decided {
				return revolveLoftFacesToolCapSides{}, false
			}
			if !crosses {
				out.separated[[2]*Face{faceHandle(span.Face), tool.face}] = struct{}{}
				continue
			}
			out.circles = append(out.circles, revolveLoftFacesToolCapCircle{
				blankFace: faceHandle(span.Face), capFace: tool.face, capOrigin: origin,
				span: spanIndex, tool: toolIndex, x: x, radius: radius,
				sourceMiss: revolveLoftFacesToolCircleOutsideSource(proof.trim, x, radius)})
		}
	}
	return out, true
}

func revolveLoftFacesToolCircleOnSpan(tool *pointConeLimit, span revolveSectionSpan) (
	*big.Rat, *big.Rat, bool, bool,
) {
	if tool == nil || tool.Apex == nil || tool.Far == nil || tool.Slope == nil ||
		tool.Slope.Sign() <= 0 || tool.Far.Cmp(tool.Apex) >= 0 ||
		span.Start == span.End || span.Start.V < 0 || span.End.V < 0 {
		return nil, nil, false, false
	}
	a := revolveLoftFacesToolResidual(tool, span.Start)
	b := revolveLoftFacesToolResidual(tool, span.End)
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
	x := lfAdd(lfF(span.Start.U), lfMul(fraction,
		lfSub(lfF(span.End.U), lfF(span.Start.U))))
	radius := lfAdd(lfF(span.Start.V), lfMul(fraction,
		lfSub(lfF(span.End.V), lfF(span.Start.V))))
	if radius.Sign() <= 0 || x.Cmp(tool.Far) <= 0 || x.Cmp(tool.Apex) >= 0 {
		return nil, nil, false, false
	}
	return x, radius, true, true
}

func revolveLoftFacesToolResidual(tool *pointConeLimit, point planePoint) *big.Rat {
	return lfSub(lfAdd(lfF(point.U), lfMul(tool.Slope, lfF(point.V))), tool.Apex)
}

// Every true LoftFaces band is inside the image of the authenticated source
// rectangle. A circle missed by every full band image cannot meet the
// surviving Cut/Intersect cap. Exact coordinate hulls intentionally include
// more than the actual Sketch region and therefore only prove misses.
func revolveLoftFacesToolCircleOutsideSource(trim *loftFacesTrimRecord,
	x, radius *big.Rat) bool {
	if trim == nil || trim.Source == nil || x == nil || radius == nil ||
		radius.Sign() <= 0 {
		return false
	}
	box, err := revolveloftfaces.LfSourceBox(trim.Source)
	if err != nil {
		return false
	}
	return loftface.CircleOutsideMaps(trim.Maps, box, x, radius)
}
