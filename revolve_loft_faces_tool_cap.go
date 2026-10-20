package decad

import (
	"context"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/decad/internal/revolveshell"
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

// A consumer must rederive a stored occupied arc before using its source
// endpoints. This binds its exact band time and both root brackets to the
// current Sketch, Revolve section, cone trims, and retained source faces.
func revolveLoftFacesToolCapOccupiedArcCurrent(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate,
	recorded *revolveLoftFacesToolCapOccupiedArc) (bool, error) {
	if ctx == nil || proof == nil || recorded == nil || recorded.circle.tool != 1 {
		return false, nil
	}
	if err := ctx.Err(); err != nil {
		return false, err
	}
	circles, current := revolveLoftFacesToolCapSidesOf(proof)
	if !current {
		return false, nil
	}
	for _, circle := range circles.circles {
		if !revolveLoftFacesToolCapCircleEqual(circle, recorded.circle) {
			continue
		}
		span := proof.section.Spans[circle.span]
		var fresh *revolveLoftFacesToolCapOccupiedArc
		var proved bool
		var err error
		if span.Start.U == span.End.U {
			fresh, proved, err = revolveLoftFacesHeelVerticalOccupiedArc(ctx, proof, circle)
		} else {
			fresh, proved, err = revolveLoftFacesHeelSlopedOccupiedArc(ctx, proof, circle)
		}
		if err != nil || !proved {
			return false, err
		}
		return revolveLoftFacesToolCapOccupiedArcEqual(fresh, recorded), nil
	}
	return false, nil
}

// revolveLoftFacesToolCapOccupiedArcCurrentIn is
// revolveLoftFacesToolCapOccupiedArcCurrent for a caller that holds the
// census of this proof from earlier in its call. For an arc that census
// derived itself, it compares the arc with the census's private snapshot
// of that derivation; any other arc is rederived from the live source.
func revolveLoftFacesToolCapOccupiedArcCurrentIn(ctx context.Context,
	proof *revolveLoftFacesSourceCertificate, census *revolveLoftFacesPairCensus,
	recorded *revolveLoftFacesToolCapOccupiedArc) (bool, error) {
	if ctx != nil && proof != nil && census != nil && recorded != nil {
		for i := range census.ownArcs {
			if census.ownArcs[i].arc != recorded {
				continue
			}
			if err := ctx.Err(); err != nil {
				return false, err
			}
			return revolveLoftFacesToolCapOccupiedArcEqual(&census.ownArcs[i].derived, recorded), nil
		}
	}
	return revolveLoftFacesToolCapOccupiedArcCurrent(ctx, proof, recorded)
}

func revolveLoftFacesToolCapOccupiedArcEqual(a, b *revolveLoftFacesToolCapOccupiedArc) bool {
	return a != nil && b != nil &&
		revolveLoftFacesToolCapCircleEqual(a.circle, b.circle) &&
		a.band == b.band && loftface.RatEqual(a.time, b.time) &&
		revolveLoftFacesToolCapBoundaryEqual(a.negativeZ, b.negativeZ) &&
		revolveLoftFacesToolCapBoundaryEqual(a.positiveZ, b.positiveZ) &&
		a.throughPositiveY == b.throughPositiveY
}

func revolveLoftFacesToolCapCircleEqual(a, b revolveLoftFacesToolCapCircle) bool {
	return a.blankFace == b.blankFace && a.capFace == b.capFace &&
		a.capOrigin == b.capOrigin && a.span == b.span && a.tool == b.tool &&
		a.sourceMiss == b.sourceMiss &&
		loftface.RatEqual(a.x, b.x) &&
		loftface.RatEqual(a.radius, b.radius)
}

func revolveLoftFacesToolCapBoundaryEqual(a, b revolveLoftFacesToolCapBoundary) bool {
	if a.segment != b.segment || a.span != b.span || a.ref != b.ref ||
		a.face != b.face || a.outward != b.outward ||
		!loftface.IntervalEqual(a.parameter, b.parameter) {
		return false
	}
	for axis := range a.box {
		if !loftface.IntervalEqual(a.box[axis], b.box[axis]) {
			return false
		}
	}
	return true
}

// A field added to the arc, circle or boundary must be copied here and
// compared in the Equal functions above.
func revolveLoftFacesCloneToolCapOccupiedArc(
	a *revolveLoftFacesToolCapOccupiedArc) revolveLoftFacesToolCapOccupiedArc {
	out := *a
	out.circle.x = revolveLoftFacesCloneRat(a.circle.x)
	out.circle.radius = revolveLoftFacesCloneRat(a.circle.radius)
	out.time = revolveLoftFacesCloneRat(a.time)
	out.negativeZ = revolveLoftFacesCloneToolCapBoundary(a.negativeZ)
	out.positiveZ = revolveLoftFacesCloneToolCapBoundary(a.positiveZ)
	return out
}

func revolveLoftFacesCloneToolCapBoundary(
	b revolveLoftFacesToolCapBoundary) revolveLoftFacesToolCapBoundary {
	b.parameter = loftFaceInterval{Lo: revolveLoftFacesCloneRat(b.parameter.Lo), Hi: revolveLoftFacesCloneRat(b.parameter.Hi)}
	for i := range b.box {
		b.box[i] = loftFaceInterval{Lo: revolveLoftFacesCloneRat(b.box[i].Lo), Hi: revolveLoftFacesCloneRat(b.box[i].Hi)}
	}
	return b
}

func revolveLoftFacesCloneRat(x *big.Rat) *big.Rat {
	if x == nil {
		return nil
	}
	return new(big.Rat).Set(x)
}

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
	x, radius, crosses, decided := revolveloftfaces.ToolCircleOnSpan(
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
	if err != nil || outcome != revolveloftfaces.VerticalContact || vertical == nil {
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
	loop, simple, err := revolveloftfaces.SimpleSourceLoop(ctx, proof.trim)
	if err != nil || !simple || loop == nil || !loop.CCW {
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
		outward, oriented := revolveloftfaces.ToolBoundaryOutward(edge, cell.upper)
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
		if err != nil || outcome != revolveloftfaces.VerticalContact || vertical == nil ||
			vertical.face != circle.blankFace || vertical.x.Cmp(circle.x) != 0 ||
			vertical.upper.Cmp(circle.radius) != 0 {
			return false, err
		}
		fresh = vertical.revolveLoftFacesConstantXSource
	} else {
		if !revolveloftfaces.SlopedRadialInside(proof.section, circle.span,
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
	face, live := revolveshell.SpanFace(bodyNode(proof.blank), span)
	if !live || face != faceNode(circle.blankFace) ||
		!revolveloftfaces.OriginsMatch(face.Origins, span.Origins) {
		return nil, false, nil
	}
	x, radius, crosses, decided := revolveloftfaces.ToolCircleOnSpan(trim.Heel, span)
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
