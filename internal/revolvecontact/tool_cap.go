package revolvecontact

import (
	"context"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/decad/internal/revolveshell"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// A tool-cap circle is the exact intersection of one authenticated finite
// Revolve generator and one cone used to trim the source LoftFaces body. It
// says where a contact could occur; a separate source-region proof must show
// which part of the circle belongs to the surviving tool cap.
type ToolCapCircle struct {
	BlankFace  *topo.Face
	CapFace    *topo.Face
	CapOrigin  provenance.Ref
	Span       int
	Tool       int // 0 is the Cut cone, 1 is the Intersect cone.
	X, Radius  *big.Rat
	SourceMiss bool
}

type ToolCapSides struct {
	Circles   []ToolCapCircle
	Separated map[[2]*topo.Face]struct{}
}

// The exact affine cone residual on a straight meridian can vanish once.
// Strict endpoint signs separate a complete Revolve face from the cone. A
// zero strictly inside the finite tool cone identifies a possible contact
// circle, including on a vertical section generator.
func ToolCapSidesOf(proof *SourceCertificate) (
	ToolCapSides, bool,
) {
	var out ToolCapSides
	if proof == nil || proof.Blank == nil || proof.Tooth == nil ||
		proof.Section.Blank != proof.Blank || proof.Trim == nil ||
		!SectionCurrent(proof) ||
		!SectionFacesComplete(proof) ||
		proof.ToeFace == nil || proof.HeelFace == nil ||
		proof.ToeFace == proof.HeelFace ||
		proof.ToeFace.Body != proof.Tooth || proof.HeelFace.Body != proof.Tooth {
		return out, false
	}
	p, ok := proof.Tooth.Payload.(facetedrecord.Payload)
	if !ok || p.LoftFacesTrim != proof.Trim ||
		!TrimCurrent(proof.Tooth, proof.Trim) ||
		UniqueResultSourceFace(proof.Tooth, proof.Trim.ToeCapGroup) != proof.ToeFace ||
		UniqueResultSourceFace(proof.Tooth, proof.Trim.HeelCapGroup) != proof.HeelFace {
		return out, false
	}
	tools := [...]struct {
		limit *facetedrecord.PointConeLimit
		face  *topo.Face
		group int
	}{{proof.Trim.Toe, proof.ToeFace, proof.Trim.ToeCapGroup},
		{proof.Trim.Heel, proof.HeelFace, proof.Trim.HeelCapGroup}}
	out.Separated = make(map[[2]*topo.Face]struct{})
	for toolIndex, tool := range tools {
		if tool.limit == nil || tool.limit.Apex == nil || tool.limit.Far == nil ||
			tool.limit.Slope == nil || tool.limit.Slope.Sign() <= 0 ||
			tool.limit.Far.Cmp(tool.limit.Apex) >= 0 ||
			tool.group < 0 || tool.group >= len(p.Groups) ||
			len(p.Groups[tool.group].Origins) != 1 {
			return ToolCapSides{}, false
		}
		origin := p.Groups[tool.group].Origins[0]
		found := false
		for _, faceOrigin := range tool.face.Origins {
			found = found || faceOrigin == origin
		}
		if !found {
			return ToolCapSides{}, false
		}
		for spanIndex, span := range proof.Section.Spans {
			if span.Face == nil {
				continue // The axis span has no boundary face.
			}
			if span.Face.Body != proof.Blank || span.Start.V < 0 || span.End.V < 0 ||
				span.Start == span.End {
				return ToolCapSides{}, false
			}
			x, radius, crosses, decided := revolveloftfaces.ToolCircleOnSpan(tool.limit, span)
			if !decided {
				return ToolCapSides{}, false
			}
			if !crosses {
				out.Separated[[2]*topo.Face{span.Face, tool.face}] = struct{}{}
				continue
			}
			out.Circles = append(out.Circles, ToolCapCircle{
				BlankFace: span.Face, CapFace: tool.face, CapOrigin: origin,
				Span: spanIndex, Tool: toolIndex, X: x, Radius: radius,
				SourceMiss: revolveloftfaces.ToolCircleOutsideSource(proof.Trim, x, radius)})
		}
	}
	return out, true
}

// A constant-X source section is private mutable data. Compare every field
// with a fresh classification before its root boxes become cap endpoints.
func ToolCapSourceCellsEqual(a, b *ConstantXSource) bool {
	if a == nil || b == nil || a.Band != b.Band ||
		!loftface.RatEqual(a.Time, b.Time) ||
		!loftface.RatEqual(a.X, b.X) ||
		!loftface.RatEqual(a.Lower, b.Lower) ||
		!loftface.RatEqual(a.Upper, b.Upper) ||
		a.HasRetained != b.HasRetained || len(a.Cells) != len(b.Cells) ||
		(a.Cells == nil) != (b.Cells == nil) ||
		!loftface.AffineEqual(a.YzAffine, b.YzAffine) {
		return false
	}
	for i := range a.Cells {
		if !ToolCapCellEqual(a.Cells[i], b.Cells[i]) {
			return false
		}
	}
	return true
}

func ToolCapCellEqual(a, b VerticalCell) bool {
	if a.Segment != b.Segment || a.Span != b.Span || a.Band != b.Band ||
		a.Ref != b.Ref || a.Face != b.Face || len(a.Source) != len(b.Source) ||
		(a.Source == nil) != (b.Source == nil) ||
		len(a.Retained) != len(b.Retained) ||
		(a.Retained == nil) != (b.Retained == nil) ||
		!loftface.LimitEqual(a.Lower, b.Lower) ||
		!loftface.LimitEqual(a.Upper, b.Upper) {
		return false
	}
	for i := range a.Source {
		if !loftface.RatEqual(a.Source[i].U, b.Source[i].U) ||
			!loftface.RatEqual(a.Source[i].V, b.Source[i].V) {
			return false
		}
	}
	for i := range a.Retained {
		if !loftface.PieceEqual(a.Retained[i], b.Retained[i]) {
			return false
		}
	}
	return true
}

// A consumer must rederive a stored occupied arc before using its source
// endpoints. This binds its exact band time and both root brackets to the
// current Sketch, Revolve section, cone trims, and retained source faces.
func ToolCapOccupiedArcCurrent(ctx context.Context,
	proof *SourceCertificate,
	recorded *ToolCapOccupiedArc) (bool, error) {
	if ctx == nil || proof == nil || recorded == nil || recorded.Circle.Tool != 1 {
		return false, nil
	}
	if err := ctx.Err(); err != nil {
		return false, err
	}
	circles, current := ToolCapSidesOf(proof)
	if !current {
		return false, nil
	}
	for _, circle := range circles.Circles {
		if !ToolCapCircleEqual(circle, recorded.Circle) {
			continue
		}
		span := proof.Section.Spans[circle.Span]
		var fresh *ToolCapOccupiedArc
		var proved bool
		var err error
		if span.Start.U == span.End.U {
			fresh, proved, err = HeelVerticalOccupiedArc(ctx, proof, circle)
		} else {
			fresh, proved, err = HeelSlopedOccupiedArc(ctx, proof, circle)
		}
		if err != nil || !proved {
			return false, err
		}
		return ToolCapOccupiedArcEqual(fresh, recorded), nil
	}
	return false, nil
}

// ToolCapOccupiedArcCurrentIn is
// ToolCapOccupiedArcCurrent for a caller that holds the
// census of this proof from earlier in its call. For an arc that census
// derived itself, it compares the arc with the census's private snapshot
// of that derivation; any other arc is rederived from the live source.
func ToolCapOccupiedArcCurrentIn(ctx context.Context,
	proof *SourceCertificate, census *PairCensus,
	recorded *ToolCapOccupiedArc) (bool, error) {
	if ctx != nil && proof != nil && census != nil && recorded != nil {
		for i := range census.OwnArcs {
			if census.OwnArcs[i].Arc != recorded {
				continue
			}
			if err := ctx.Err(); err != nil {
				return false, err
			}
			return ToolCapOccupiedArcEqual(&census.OwnArcs[i].Derived, recorded), nil
		}
	}
	return ToolCapOccupiedArcCurrent(ctx, proof, recorded)
}

func ToolCapOccupiedArcEqual(a, b *ToolCapOccupiedArc) bool {
	return a != nil && b != nil &&
		ToolCapCircleEqual(a.Circle, b.Circle) &&
		a.Band == b.Band && loftface.RatEqual(a.Time, b.Time) &&
		ToolCapBoundaryEqual(a.NegativeZ, b.NegativeZ) &&
		ToolCapBoundaryEqual(a.PositiveZ, b.PositiveZ) &&
		a.ThroughPositiveY == b.ThroughPositiveY
}

func ToolCapCircleEqual(a, b ToolCapCircle) bool {
	return a.BlankFace == b.BlankFace && a.CapFace == b.CapFace &&
		a.CapOrigin == b.CapOrigin && a.Span == b.Span && a.Tool == b.Tool &&
		a.SourceMiss == b.SourceMiss &&
		loftface.RatEqual(a.X, b.X) &&
		loftface.RatEqual(a.Radius, b.Radius)
}

func ToolCapBoundaryEqual(a, b ToolCapBoundary) bool {
	if a.Segment != b.Segment || a.Span != b.Span || a.Ref != b.Ref ||
		a.Face != b.Face || a.Outward != b.Outward ||
		!loftface.IntervalEqual(a.Parameter, b.Parameter) {
		return false
	}
	for axis := range a.Box {
		if !loftface.IntervalEqual(a.Box[axis], b.Box[axis]) {
			return false
		}
	}
	return true
}

// A field added to the arc, circle or boundary must be copied here and
// compared in the Equal functions above.
func CloneToolCapOccupiedArc(
	a *ToolCapOccupiedArc) ToolCapOccupiedArc {
	out := *a
	out.Circle.X = CloneRat(a.Circle.X)
	out.Circle.Radius = CloneRat(a.Circle.Radius)
	out.Time = CloneRat(a.Time)
	out.NegativeZ = CloneToolCapBoundary(a.NegativeZ)
	out.PositiveZ = CloneToolCapBoundary(a.PositiveZ)
	return out
}

func CloneToolCapBoundary(
	b ToolCapBoundary) ToolCapBoundary {
	b.Parameter = loftface.Interval{Lo: CloneRat(b.Parameter.Lo), Hi: CloneRat(b.Parameter.Hi)}
	for i := range b.Box {
		b.Box[i] = loftface.Interval{Lo: CloneRat(b.Box[i].Lo), Hi: CloneRat(b.Box[i].Hi)}
	}
	return b
}

func CloneRat(x *big.Rat) *big.Rat {
	if x == nil {
		return nil
	}
	return new(big.Rat).Set(x)
}

// A boundary point is the exact circle/source intersection enclosed by its
// rational source parameter and mapped XYZ boxes. The true point need not
// have rational coordinates. The referenced LoftFaces face is live.
type ToolCapBoundary struct {
	Segment, Span int
	Ref           provenance.Ref
	Face          *topo.Face
	Parameter     loftface.Interval
	Box           [3]loftface.Interval
	Outward       bool // radial residual increases in the Sketch outer-loop walk.
}

// Between two transverse source-boundary crossings, the circle has one
// occupied open arc. Both endpoints lie on positive Y, one on each side of
// Z=0; throughPositiveY selects the short arc through that axis.
type ToolCapOccupiedArc struct {
	Circle           ToolCapCircle
	Band             int
	Time             *big.Rat // exact constant-X time within the authenticated cap band
	NegativeZ        ToolCapBoundary
	PositiveZ        ToolCapBoundary
	ThroughPositiveY bool
}

// A separately certified exact simple, counter-clockwise source loop and
// orientation-preserving affine cap map keep the true interior on the left
// of the mapped walk. At a circle crossing, the mapped radial derivative
// tells which counter-clockwise circle arc lies inside. Exactly two
// transverse crossings therefore certify one occupied arc.
func HeelVerticalOccupiedArc(ctx context.Context,
	proof *SourceCertificate,
	circle ToolCapCircle) (
	*ToolCapOccupiedArc, bool, error,
) {
	if ctx == nil || proof == nil || proof.Tooth == nil || proof.Trim == nil || proof.Trim.Source == nil ||
		proof.Trim.Source.Source == nil || proof.Trim.Source.Source.Profile == nil ||
		circle.Tool != 1 || circle.SourceMiss || circle.X == nil || circle.Radius == nil ||
		circle.Span < 0 || circle.Span >= len(proof.Section.Spans) ||
		circle.BlankFace == nil || circle.CapFace != proof.HeelFace ||
		!SectionFacesComplete(proof) ||
		!proof.Trim.Source.Source.Profile.Valid ||
		len(proof.Trim.Source.Source.Profile.Holes) != 0 {
		return nil, false, nil
	}
	x, radius, crosses, decided := revolveloftfaces.ToolCircleOnSpan(
		proof.Trim.Heel, proof.Section.Spans[circle.Span])
	if !decided || !crosses || x.Cmp(circle.X) != 0 ||
		radius.Cmp(circle.Radius) != 0 {
		return nil, false, nil
	}
	payload, ok := proof.Tooth.Payload.(facetedrecord.Payload)
	if !ok || payload.LoftFacesTrim != proof.Trim ||
		proof.Trim.HeelCapGroup < 0 || proof.Trim.HeelCapGroup >= len(payload.Groups) ||
		len(payload.Groups[proof.Trim.HeelCapGroup].Origins) != 1 ||
		payload.Groups[proof.Trim.HeelCapGroup].Origins[0] != circle.CapOrigin ||
		!ToolCapFaceOriginCurrent(circle) ||
		UniqueResultSourceFace(proof.Tooth, proof.Trim.HeelCapGroup) != circle.CapFace {
		return nil, false, nil
	}
	vertical, outcome, err := VerticalSideClassify(ctx, proof, circle.Span)
	if err != nil || outcome != revolveloftfaces.VerticalContact || vertical == nil {
		return nil, false, err
	}
	if vertical.Face != circle.BlankFace || vertical.X.Cmp(circle.X) != 0 ||
		vertical.Upper.Cmp(circle.Radius) != 0 ||
		vertical.Lower.Cmp(circle.Radius) >= 0 ||
		vertical.YzAffine == nil || vertical.YzAffine.Det == nil ||
		vertical.YzAffine.Det.Sign() <= 0 {
		return nil, false, nil
	}
	return HeelOccupiedFromSource(ctx, proof, circle,
		vertical.ConstantXSource)
}

// Both vertical and sloped blank generators meet a heel tool cap at a
// constant-X circle. Exact source cells and a simple true boundary determine
// the occupied angular arc independently of the generator slope.
func HeelOccupiedFromSource(ctx context.Context,
	proof *SourceCertificate, circle ToolCapCircle,
	section *ConstantXSource) (
	*ToolCapOccupiedArc, bool, error,
) {
	if proof == nil || proof.Trim == nil || circle.X == nil || circle.Radius == nil ||
		section == nil || section.X == nil || section.Upper == nil ||
		section.Band < 0 || section.Band >= len(proof.SourceRefs) ||
		section.X.Cmp(circle.X) != 0 ||
		section.Upper.Cmp(circle.Radius) != 0 || !section.HasRetained ||
		section.YzAffine == nil || section.YzAffine.Det == nil ||
		section.YzAffine.Det.Sign() <= 0 {
		return nil, false, nil
	}
	current, err := ToolCapSourceCurrent(ctx, proof, circle, section)
	if err != nil || !current {
		return nil, false, err
	}
	loop, simple, err := revolveloftfaces.SimpleSourceLoop(ctx, proof.Trim)
	if err != nil || !simple || loop == nil || !loop.CCW {
		return nil, false, err
	}
	outer := proof.Trim.Source.Source.Profile.Outer
	var negative, positive *ToolCapBoundary
	for _, cell := range section.Cells {
		if cell.Segment < 0 || cell.Segment >= len(outer) {
			return nil, false, nil
		}
		if cell.Upper.Root.Lo == nil {
			if cell.Upper.Side != -1 && cell.Upper.Side != 1 {
				return nil, false, nil
			}
			continue
		}
		root := cell.Upper.Root
		if root.Hi == nil || root.Lo.Sign() <= 0 || root.Lo.Cmp(root.Hi) >= 0 ||
			root.Hi.Cmp(lfOne()) >= 0 || cell.Face == nil ||
			cell.Upper.Before*cell.Upper.After != -1 ||
			cell.Upper.Box[0].Lo == nil || cell.Upper.Box[0].Hi == nil ||
			cell.Upper.Box[0].Lo.Cmp(circle.X) != 0 ||
			cell.Upper.Box[0].Hi.Cmp(circle.X) != 0 ||
			cell.Upper.Box[1].Lo == nil || cell.Upper.Box[1].Lo.Sign() <= 0 ||
			cell.Upper.Box[2].Lo == nil || cell.Upper.Box[2].Hi == nil {
			return nil, false, nil
		}
		edge := outer[cell.Segment]
		if proof.SourceRefs[section.Band][cell.Segment] != cell.Ref {
			return nil, false, nil
		}
		outward, oriented := revolveloftfaces.ToolBoundaryOutward(edge, cell.Upper)
		if !oriented {
			return nil, false, nil
		}
		boundary := &ToolCapBoundary{
			Segment: cell.Segment, Span: cell.Span, Ref: cell.Ref,
			Face: cell.Face, Parameter: root, Box: cell.Upper.Box,
			Outward: outward}
		switch {
		case boundary.Box[2].Hi.Sign() < 0 && negative == nil:
			negative = boundary
		case boundary.Box[2].Lo.Sign() > 0 && positive == nil:
			positive = boundary
		default:
			return nil, false, nil
		}
	}
	if negative == nil || positive == nil || negative.Outward == positive.Outward {
		return nil, false, nil
	}
	return &ToolCapOccupiedArc{Circle: circle,
		Band: section.Band, Time: new(big.Rat).Set(section.Time),
		NegativeZ: *negative, PositiveZ: *positive,
		ThroughPositiveY: negative.Outward}, true, nil
}

// An occupied arc must use the current circle and the full live source
// classification. Checking only the two root boxes would miss changed side
// signs, which can reverse the selected occupied arc.
func ToolCapSourceCurrent(ctx context.Context,
	proof *SourceCertificate, circle ToolCapCircle,
	section *ConstantXSource) (bool, error) {
	if ctx == nil || proof == nil || proof.Trim == nil || section == nil ||
		circle.X == nil || circle.Radius == nil || circle.Tool != 1 {
		return false, nil
	}
	sides, ok := ToolCapSidesOf(proof)
	if !ok {
		return false, nil
	}
	matched := false
	for _, live := range sides.Circles {
		if live.BlankFace == circle.BlankFace && live.CapFace == circle.CapFace &&
			live.CapOrigin == circle.CapOrigin && live.Span == circle.Span &&
			live.Tool == circle.Tool && live.SourceMiss == circle.SourceMiss &&
			live.X.Cmp(circle.X) == 0 && live.Radius.Cmp(circle.Radius) == 0 {
			matched = true
			break
		}
	}
	if !matched || circle.SourceMiss || circle.Span < 0 ||
		circle.Span >= len(proof.Section.Spans) {
		return false, nil
	}
	span := proof.Section.Spans[circle.Span]
	var fresh *ConstantXSource
	if span.Start.U == span.End.U {
		vertical, outcome, err := VerticalSideClassify(ctx, proof, circle.Span)
		if err != nil || outcome != revolveloftfaces.VerticalContact || vertical == nil ||
			vertical.Face != circle.BlankFace || vertical.X.Cmp(circle.X) != 0 ||
			vertical.Upper.Cmp(circle.Radius) != 0 {
			return false, err
		}
		fresh = vertical.ConstantXSource
	} else {
		if !revolveloftfaces.SlopedRadialInside(proof.Section, circle.Span,
			circle.X, circle.Radius) || proof.Trim.Toe == nil ||
			proof.Trim.Toe.Apex == nil || proof.Trim.Toe.Slope == nil ||
			proof.Trim.Toe.Slope.Sign() <= 0 {
			return false, nil
		}
		lower := lfZero()
		if circle.X.Cmp(proof.Trim.Toe.Apex) < 0 {
			lower = lfDiv(lfSub(proof.Trim.Toe.Apex, circle.X), proof.Trim.Toe.Slope)
		}
		var err error
		fresh, ok, err = ConstantXSourceCells(ctx, proof,
			circle.X, lower, circle.Radius)
		if err != nil || !ok {
			return false, err
		}
	}
	return ToolCapSourceCellsEqual(section, fresh), nil
}

func ToolCapFaceOriginCurrent(circle ToolCapCircle) bool {
	return circle.CapFace != nil && slices.Contains(circle.CapFace.Origins, circle.CapOrigin)
}

// A sloped finite Revolve generator intersects the heel tool cone at one
// constant-X circle. The live root-side face and both cone limits supply the
// exact radial interval; source cells then locate the true Sketch crossings.
func HeelSlopedOccupiedArc(ctx context.Context,
	proof *SourceCertificate,
	circle ToolCapCircle) (
	*ToolCapOccupiedArc, bool, error,
) {
	if ctx == nil || proof == nil || proof.Blank == nil || proof.Tooth == nil ||
		proof.Trim == nil || proof.Trim.Toe == nil || proof.Trim.Heel == nil ||
		circle.Tool != 1 || circle.SourceMiss || circle.X == nil ||
		circle.Radius == nil || circle.Radius.Sign() <= 0 ||
		circle.Span < 0 || circle.Span >= len(proof.Section.Spans) ||
		circle.BlankFace == nil || circle.CapFace == nil ||
		circle.CapFace != proof.HeelFace ||
		!SectionCurrent(proof) ||
		!SectionFacesComplete(proof) {
		return nil, false, nil
	}
	if err := ctx.Err(); err != nil {
		return nil, false, err
	}
	trim := proof.Trim
	payload, ok := proof.Tooth.Payload.(facetedrecord.Payload)
	if !ok || payload.LoftFacesTrim != trim ||
		!TrimCurrent(proof.Tooth, trim) ||
		trim.HeelCapGroup < 0 || trim.HeelCapGroup >= len(payload.Groups) ||
		len(payload.Groups[trim.HeelCapGroup].Origins) != 1 ||
		payload.Groups[trim.HeelCapGroup].Origins[0] != circle.CapOrigin ||
		!ToolCapFaceOriginCurrent(circle) ||
		UniqueResultSourceFace(proof.Tooth, trim.HeelCapGroup) != circle.CapFace {
		return nil, false, nil
	}
	span := proof.Section.Spans[circle.Span]
	if span.Start.U == span.End.U || span.Face != circle.BlankFace ||
		span.Face.Body != proof.Blank || len(span.Origins) == 0 {
		return nil, false, nil
	}
	face, live := revolveshell.SpanFace(proof.Blank, span)
	if !live || face != circle.BlankFace ||
		!revolveloftfaces.OriginsMatch(face.Origins, span.Origins) {
		return nil, false, nil
	}
	x, radius, crosses, decided := revolveloftfaces.ToolCircleOnSpan(trim.Heel, span)
	if !decided || !crosses || x.Cmp(circle.X) != 0 ||
		radius.Cmp(circle.Radius) != 0 {
		return nil, false, nil
	}
	if !revolveloftfaces.SlopedRadialInside(proof.Section, circle.Span, x, radius) {
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
	section, ok, err := ConstantXSourceCells(ctx, proof,
		x, lower, radius)
	if err != nil || !ok {
		return nil, false, err
	}
	return HeelOccupiedFromSource(ctx, proof, circle, section)
}
