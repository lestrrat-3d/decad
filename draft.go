package decad

import (
	"context"
	"fmt"
	"slices"
	"strings"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is Body.Draft (docs/draft-design.md §10): the face draft of an
// existing prism. It resolves the neutral cap and the wall selection against
// the receiver, then builds the same draftPayload a tapered Extrude builds
// (draft_build.go) over the receiver's recorded section.

// roleSidePrefix opens every wall role a sweep mints, side(i, j). Walls(b)
// and the complete-wall-set test of Draft read it.
const roleSidePrefix = "side("

// isWallOf reports whether f carries a side(i, j) role minted by producer.
func (f *Face) isWallOf(producer producerID) bool {
	for _, o := range f.origins {
		if o.producer == producer && strings.HasPrefix(o.Role, roleSidePrefix) {
			return true
		}
	}
	return false
}

// NeutralPlane is the plane a draft tilts faces about. The set is sealed
// (docs/draft-design.md §10).
type NeutralPlane interface{ neutralPlane() }

// NeutralFace names a planar face of Body, selected and never pointed at
// (core §9), under [MirrorFace]'s rules: Face resolves through
// SelectFaces(Body) under the implicit exactly-one rule, so zero or several
// faces is [ErrCardinality] with Expected "exactly 1"; a curved face is
// [ErrDegenerate]; a flat face with no analytic [Plane] surface, such as a
// mesh-boolean [Faceted] face, is [ErrUnsupported]. Body MUST be a live body
// of the receiver's document ([ErrForeignBody], [ErrRetiredBody]). This
// evaluator drafts about a cap of the receiver only, so Body must be the
// receiver and the face its capStart or capEnd; any other face is
// [ErrUnsupported].
type NeutralFace struct {
	Body *Body
	Face FaceSelector
}

// NeutralFrame names the plane through Frame's origin spanned by its U and V
// axes. Staged: a draft about it is [ErrUnsupported] until the increment that
// lifts draft SD20.
type NeutralFrame struct {
	Frame r3.Frame
}

// The sealed set.
func (NeutralFace) neutralPlane()  {}
func (NeutralFrame) neutralPlane() {}

// errNilNeutralPlane rejects a nil plane, or a nil variant pointer: either
// names no plane to tilt about.
var errNilNeutralPlane = fmt.Errorf(`%w: nil neutral plane`, ErrDegenerate)

// Draft returns a new body whose walls lean by angle from the sweep axis about
// the neutral plane, retiring the receiver (docs/draft-design.md §10). A
// positive angle narrows the body with distance from the neutral plane and a
// negative one widens it; the angle is a signed [units.Angle] outside
// [ErrNegativeMagnitude] (core §12). sel names the walls to lean, about one
// of the receiver's caps. Faces(Walls(b)) names every wall, and the result is
// the body a tapered [Document.Extrude] of the same section and sweep builds.
// A subset leans only the walls it names: each moved wall's far trace is its
// own moved line or circle, a kept wall stays vertical, and a corner between a
// moved and a kept line is the intersection of the two
// (docs/draft-design.md §10.2).
//
// The receiver is a live straight prism: an extrude, a filleted or chamfered
// extrude, a tube, or any of these placed, whose recorded section is the
// section it denotes. The gates run in docs/draft-design.md §5's order, and a
// refused call leaves the document unchanged, including on ctx.Err():
//
//   - an angle that is not an angle is [ErrUnitKind], one that is not finite
//     [ErrNotFinite], and one at or past a right angle [ErrDegenerate];
//   - a retired or foreign receiver is [ErrRetiredBody] or [ErrForeignBody],
//     and a sheet is [ErrUnsupported];
//   - a zero angle is [ErrDegenerate] (the draft is the receiver);
//   - a neutral selector resolving to zero or several faces is [ErrCardinality],
//     a curved neutral face [ErrDegenerate];
//   - a receiver that is not a prism, a section carrying a displacement, an
//     already drafted body, a [NeutralFrame], a neutral face that is not a cap
//     of the receiver, and a selected face that is no wall of the receiver
//     are each [ErrUnsupported];
//   - a selected cap is [ErrDegenerate], and an empty selection is the
//     selector's [ErrNoMatch];
//   - a subset that moves one of two walls meeting at a circular corner (a
//     G1 join of a line and an arc, or of two arcs) is [ErrUnsupported]: the
//     moved wall and the kept one no longer meet tangentially, and their
//     corner moves along a conic. A whole circle is one wall and moves or
//     stays whole.
//
// The far section's own gates (draft SD5 to SD16) are those of a tapered
// extrude. ctx, sel and neutral MUST NOT be nil; a nil one is [ErrDegenerate].
// Every measurement of the result is Approximate: the tangent of the angle is
// only ever enclosed.
func (b *Body) Draft(ctx context.Context, sel FaceSelector, neutral NeutralPlane, angle units.Value) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a draft`, ErrDegenerate)
	}
	if b == nil || b.doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	d := b.doc

	// Stage 1: SD1, SD2, SD17, SD18 and SD19's cardinality.
	if angle.Kind() != units.Angle {
		return nil, fmt.Errorf(`%w: a draft angle must be an angle, got %s`, ErrUnitKind, angle.Kind())
	}
	if _, err := angle.In(units.Radian); err != nil {
		return nil, fmt.Errorf(`%w: the draft angle is not representable: %s`, ErrNotFinite, err)
	}
	var alpha, alphaDelta float64
	if angle.Mag() != 0 {
		var err error
		if alpha, alphaDelta, err = draftAngle(angle); err != nil {
			return nil, err
		}
	}
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if err := refuseSheetOperand(b, "Draft"); err != nil {
		return nil, err
	}
	if angle.Mag() == 0 {
		return nil, fmt.Errorf(`%w: a zero draft angle leaves the receiver as it is (draft SD18)`, ErrDegenerate)
	}
	if sel == nil {
		return nil, errNilSelector
	}
	neutralFace, isFrame, err := d.resolveNeutralPlane(neutral)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	// Stage 2: SD23, SD20, SD19's kind rules, SD22, SD21.
	pp, err := draftReceiver(b)
	if err != nil {
		return nil, err
	}
	if isFrame {
		return nil, fmt.Errorf(`%w: this evaluator drafts about a cap of the receiver; a NeutralFrame is staged (draft SD20)`, ErrUnsupported)
	}
	if err := requireFlatAnalyticNeutral(neutralFace); err != nil {
		return nil, err
	}
	producer := b.origin.producer
	nearStart, err := neutralCap(neutralFace, b)
	if err != nil {
		return nil, err
	}
	faces, err := sel.SelectFaces(b)
	if err != nil {
		return nil, err
	}
	kept, err := draftKeptWalls(b, producer, faces)
	if err != nil {
		return nil, err
	}

	// Stages 3 to 7 are the tapered extrude's, over the receiver's section.
	ref := d.nextProducerID()
	body, err := evalDraftContext(ctx, d, ref, draftPayload{
		profile:    pp.profile,
		frame:      pp.frame,
		z0:         pp.z0,
		z1:         pp.z1,
		z0Delta:    pp.z0Delta,
		z1Delta:    pp.z1Delta,
		nearStart:  nearStart,
		taper:      alpha,
		taperDelta: alphaDelta,
		xform:      pp.xform,
		kept:       kept,
	})
	if err != nil {
		return nil, err
	}
	// Keep the consumed input aligned with document liveness at the commit edge.
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(body, b)
	return body, nil
}

// resolveNeutralPlane returns the neutral face for a NeutralFace, resolved
// against its body under the implicit exactly-one rule. A NeutralFrame
// resolves to no face and reports isFrame, since its refusal belongs to stage
// 2 (SD20). The variants seal with value receivers, so a pointer to either is
// accepted and a nil pointer is rejected like a nil plane.
func (d *Document) resolveNeutralPlane(neutral NeutralPlane) (*Face, bool, error) {
	switch p := neutral.(type) {
	case NeutralFrame:
		return nil, true, nil
	case *NeutralFrame:
		if p == nil {
			return nil, false, errNilNeutralPlane
		}
		return nil, true, nil
	case NeutralFace:
		face, err := d.resolveNeutralFace(p)
		return face, false, err
	case *NeutralFace:
		if p == nil {
			return nil, false, errNilNeutralPlane
		}
		face, err := d.resolveNeutralFace(*p)
		return face, false, err
	default:
		// The interface is sealed, so the only value left is nil.
		return nil, false, errNilNeutralPlane
	}
}

func (d *Document) resolveNeutralFace(nf NeutralFace) (*Face, error) {
	if nf.Body == nil {
		return nil, fmt.Errorf(`%w: a neutral face names no body to resolve against`, ErrDegenerate)
	}
	if err := d.requireLive(nf.Body); err != nil {
		return nil, err
	}
	return selectImpliedOneFace(nf.Body, nf.Face, "the neutral face")
}

// draftReceiver is SD23: the receiver's payload must be a straight prism whose
// recorded section is the section it denotes. A draft body is refused by name,
// since a second draft is SX10's composition rule.
func draftReceiver(b *Body) (prismPayload, error) {
	if _, ok := b.payload.(draftPayload); ok {
		return prismPayload{}, fmt.Errorf(`%w: this evaluator drafts a straight prism once; the receiver is already a draft body (draft SD23)`, ErrUnsupported)
	}
	pp, ok := b.payload.(prismPayload)
	if !ok {
		return prismPayload{}, fmt.Errorf(`%w: this evaluator drafts a straight prism only (an extrude, its filleted or chamfered result, a tube, or any of these placed) (draft SD23)`, ErrUnsupported)
	}
	if err := requireExactSection(pp, "drafts"); err != nil {
		return prismPayload{}, err
	}
	return pp, nil
}

// requireFlatAnalyticNeutral is SD19's kind rules, MirrorFace's own: a curved
// neutral face is ErrDegenerate and a flat one without an analytic Plane
// surface is ErrUnsupported.
func requireFlatAnalyticNeutral(f *Face) error {
	if _, ok := f.Surface().(Plane); ok {
		return nil
	}
	if f.isPlanar() {
		return fmt.Errorf(`%w: the neutral face is flat but its surface is %T, which states no exact plane to tilt about (draft SD19)`, ErrUnsupported, f.Surface())
	}
	return fmt.Errorf(`%w: the neutral face is not planar (its surface is %T) (draft SD19)`, ErrDegenerate, f.Surface())
}

// neutralCap is SD20 for a NeutralFace: the face must be a cap of the
// receiver itself. It reports whether the neutral cap is the start cap, which
// is the end of the sweep the draft's near section sits at.
func neutralCap(f *Face, b *Body) (bool, error) {
	if f.body != b {
		return false, fmt.Errorf(`%w: the neutral face must be a cap of the receiver, and it belongs to another body (draft SD20)`, ErrUnsupported)
	}
	switch {
	case f.hasRole(b.origin.producer, roleCapStart):
		return true, nil
	case f.hasRole(b.origin.producer, roleCapEnd):
		return false, nil
	}
	return false, fmt.Errorf(`%w: the neutral face must be the receiver's capStart or capEnd; a wall or any other face is staged (draft SD20)`, ErrUnsupported)
}

func (f *Face) hasRole(producer producerID, role string) bool {
	return slices.Contains(f.origins, FeatureRef{producer: producer, Role: role})
}

// draftKeptWalls is SD22 then SD21 (docs/draft-design.md §10.2): a selected
// cap has no trace on the neutral plane to tilt about (ErrDegenerate), and a
// selected face that is no wall of the receiver names nothing to tilt
// (ErrUnsupported). It returns the recorded segments of every wall the
// selection leaves out, read from their side(i, j) roles under producer,
// empty when the selection is the receiver's complete wall set. Whether a
// subset's corners admit the draft is stage 3's question (SD4).
func draftKeptWalls(b *Body, producer producerID, selected []*Face) (map[draftWall]struct{}, error) {
	for _, f := range selected {
		if f.hasRole(producer, roleCapStart) || f.hasRole(producer, roleCapEnd) {
			return nil, fmt.Errorf(`%w: the selection contains a cap of the receiver, which has no trace on the neutral plane to tilt about (draft SD22)`, ErrDegenerate)
		}
	}
	for _, f := range selected {
		if !f.isWallOf(producer) {
			return nil, fmt.Errorf(`%w: the selection contains a face that is no wall of the receiver (it carries no side(i, j) role of the receiver's own), so it names no wall to tilt (draft SD21)`, ErrUnsupported)
		}
	}
	kept := map[draftWall]struct{}{}
	for _, f := range b.Faces() {
		if !f.isWallOf(producer) || slices.Contains(selected, f) {
			continue
		}
		for _, o := range f.origins {
			if o.producer != producer {
				continue
			}
			var li, j int
			if n, err := fmt.Sscanf(o.Role, "side(%d,%d)", &li, &j); err != nil || n != 2 || o.Role != fmt.Sprintf("side(%d,%d)", li, j) {
				continue
			}
			kept[draftWall{loop: li, seg: j}] = struct{}{}
		}
	}
	return kept, nil
}
