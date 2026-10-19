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
			x, radius, crosses, decided := revolveloftfaces.ToolCircleOnSpan(tool.limit, span)
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
				sourceMiss: revolveloftfaces.ToolCircleOutsideSource(proof.trim, x, radius)})
		}
	}
	return out, true
}

// A constant-X source section is private mutable data. Compare every field
// with a fresh classification before its root boxes become cap endpoints.
func revolveLoftFacesToolCapSourceCellsEqual(a, b *revolveLoftFacesConstantXSource) bool {
	if a == nil || b == nil || a.band != b.band ||
		!loftface.RatEqual(a.time, b.time) ||
		!loftface.RatEqual(a.x, b.x) ||
		!loftface.RatEqual(a.lower, b.lower) ||
		!loftface.RatEqual(a.upper, b.upper) ||
		a.hasRetained != b.hasRetained || len(a.cells) != len(b.cells) ||
		(a.cells == nil) != (b.cells == nil) ||
		!loftface.AffineEqual(a.yzAffine, b.yzAffine) {
		return false
	}
	for i := range a.cells {
		if !revolveLoftFacesToolCapCellEqual(a.cells[i], b.cells[i]) {
			return false
		}
	}
	return true
}

func revolveLoftFacesToolCapCellEqual(a, b revolveLoftFacesVerticalCell) bool {
	if a.segment != b.segment || a.span != b.span || a.band != b.band ||
		a.ref != b.ref || a.face != b.face || len(a.source) != len(b.source) ||
		(a.source == nil) != (b.source == nil) ||
		len(a.retained) != len(b.retained) ||
		(a.retained == nil) != (b.retained == nil) ||
		!loftface.LimitEqual(a.lower, b.lower) ||
		!loftface.LimitEqual(a.upper, b.upper) {
		return false
	}
	for i := range a.source {
		if !loftface.RatEqual(a.source[i].U, b.source[i].U) ||
			!loftface.RatEqual(a.source[i].V, b.source[i].V) {
			return false
		}
	}
	for i := range a.retained {
		if !loftface.PieceEqual(a.retained[i], b.retained[i]) {
			return false
		}
	}
	return true
}
