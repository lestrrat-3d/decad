package decad

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/draftrecord"
	"github.com/lestrrat-3d/decad/internal/extent"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// This file is Draft (agent-docs/draft-design.md §10): the face draft of an
// existing prism. It resolves the neutral cap and the wall selection against
// b, then builds the same draftPayload a tapered Extrude builds
// (internal/draftrecord) over b's recorded section.

// NeutralPlane is the plane a draft tilts faces about. The set is sealed
// (agent-docs/draft-design.md §10).
type NeutralPlane interface{ neutralPlane() }

// NeutralFace names a planar face of Body, selected and never pointed at
// (core §9), under [MirrorFace]'s rules: Face resolves through
// SelectFaces(Body) under the implicit exactly-one rule, so zero or several
// faces is [ErrCardinality] with Expected "exactly 1"; a curved face is
// [ErrDegenerate]; a flat face with no analytic [Plane] surface, such as a
// mesh-boolean [Faceted] face, is [ErrUnsupported]. Body MUST be a live body
// of b's document ([ErrForeignBody], [ErrRetiredBody]). This
// evaluator drafts about a cap of b only, so Body must be
// b and the face its capStart or capEnd; any other face is
// [ErrUnsupported].
type NeutralFace struct {
	Body *Body
	Face FaceSelector
}

// NeutralFrame is a placeholder for drafting about the plane through Frame's
// origin spanned by its U and V axes. A draft about it is [ErrUnsupported]
// until the increment that lifts draft SD20.
type NeutralFrame struct {
	Frame r3.Frame
}

// The sealed set.
func (NeutralFace) neutralPlane()  {}
func (NeutralFrame) neutralPlane() {}

// DraftOption is a placeholder option tier for [Draft]. It carries no
// option today and reserves the signature for a later draft option.
type DraftOption interface {
	option.Interface
	draftOption()
}

// errNilNeutralPlane rejects a nil plane, or a nil variant pointer: either
// names no plane to tilt about.
var errNilNeutralPlane = fmt.Errorf(`%w: nil neutral plane`, ErrDegenerate)

// Draft returns a new body whose walls lean by angle from the sweep axis about
// the neutral plane, retiring b (agent-docs/draft-design.md §10). A
// positive angle narrows the body with distance from the neutral plane and a
// negative one widens it; the angle is a signed [units.Angle] outside
// [ErrNegativeMagnitude] (core §12). sel names the walls to lean, about one
// of b's caps. Faces(Walls(b)) names every wall, and the result is
// the body a tapered [Extrude] of the same section and sweep builds.
// A subset leans only the walls it names: each moved wall's far trace is its
// own moved line or circle, a kept wall stays vertical, and a corner between a
// moved and a kept line is the intersection of the two
// (agent-docs/draft-design.md §10.2).
//
// A faceted solid with exact vertices that is a straight prism along a world axis is also drafted, on all its walls,
// about one of its two caps (agent-docs/orthogonal-boolean-design.md B21).
//
// b is a live straight prism: an extrude, a filleted or chamfered
// extrude, a tube, or any of these placed, whose recorded section is the
// section it denotes. The gates run in agent-docs/draft-design.md §5's order, and a
// refused call leaves the document unchanged, including on ctx.Err():
//
//   - an angle that is not an angle is [ErrUnitKind], one that is not finite
//     [ErrNotFinite], and one at or past a right angle [ErrDegenerate];
//   - a retired or foreign b is [ErrRetiredBody] or [ErrForeignBody],
//     and a sheet is [ErrUnsupported];
//   - a zero angle is [ErrDegenerate] (the draft is b);
//   - a neutral selector resolving to zero or several faces is [ErrCardinality],
//     a curved neutral face [ErrDegenerate];
//   - b that is not a prism, a section carrying a displacement, an
//     already drafted body, a [NeutralFrame], a neutral face that is not a cap
//     of b, and a selected face that is no wall of b
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
// extrude. ctx, b, sel and neutral MUST NOT be nil; a nil one is [ErrDegenerate].
// Every measurement of the result is Approximate: the tangent of the angle is
// only ever enclosed.
func Draft(ctx context.Context, b *Body, sel FaceSelector, neutral NeutralPlane, angle units.Value,
	opts ...DraftOption) (result *Body, retErr error) {
	defer func() { retErr = operationError(OperationDraft, retErr) }()
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a draft`, ErrDegenerate)
	}
	if b == nil || b.n.Doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	for _, o := range opts {
		if o == nil {
			return nil, fmt.Errorf(`%w: a nil option names nothing to apply`, ErrDegenerate)
		}
	}
	d := documentHandle(b.n.Doc)

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
	pp, err := draftrecord.Receiver(bodyNode(b))
	if err != nil {
		// A straight prism read from a mesh, drafted on all its walls, is a tapered extrude of its section
		// (agent-docs/orthogonal-boolean-design.md B21).
		if errors.Is(err, ErrUnsupported) && !isFrame {
			if selected, serr := sel.SelectFaces(b); serr == nil {
				if out, handled, derr := draftMeshPrism(ctx, d, b, neutralFace, selected, angle); derr != nil {
					return nil, derr
				} else if handled {
					return out, nil
				}
			}
		}
		return nil, err
	}
	if isFrame {
		return nil, fmt.Errorf(`%w: this evaluator drafts about a cap of the receiver; a NeutralFrame is staged (draft SD20)`, ErrUnsupported)
	}
	if err := requireFlatAnalyticNeutral(neutralFace); err != nil {
		return nil, err
	}
	producer := b.n.Origin.Producer
	nearStart, err := draftrecord.NeutralCap(faceNode(neutralFace), bodyNode(b))
	if err != nil {
		return nil, err
	}
	faces, err := sel.SelectFaces(b)
	if err != nil {
		return nil, err
	}
	kept, err := draftrecord.KeptWalls(bodyNode(b), producer, faceNodes(faces))
	if err != nil {
		return nil, err
	}

	// Stages 3 to 7 are the tapered extrude's, over b's section.
	ref := d.nextProducerID()
	body, err := bodyResult(draftrecord.FromPrism(ctx, envOf(documentNode(d), ref), pp, nearStart, alpha, alphaDelta, kept))
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
	inputs := []OperationInput{{Body: b, Role: operationRoleSource, Use: InputConsumed}}
	if neutralFace != nil {
		inputs = append(inputs, OperationInput{Body: bodyHandle(neutralFace.n.Body), Role: "neutral", Use: InputReferenced})
	}
	if err := d.commitOperationIn(ctx, nil, OperationDraft, inputs, body, b); err != nil {
		return nil, err
	}
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

// requireFlatAnalyticNeutral is SD19's kind rules, MirrorFace's own: a curved
// neutral face is ErrDegenerate and a flat one without an analytic Plane
// surface is ErrUnsupported.
func requireFlatAnalyticNeutral(f *Face) error {
	if _, ok := f.n.Surface.(surfacegeom.Plane); ok {
		return nil
	}
	if f.isPlanar() {
		return fmt.Errorf(`%w: the neutral face is flat but its surface is %T, which states no exact plane to tilt about (draft SD19)`, ErrUnsupported, f.Surface())
	}
	return fmt.Errorf(`%w: the neutral face is not planar (its surface is %T) (draft SD19)`, ErrDegenerate, f.Surface())
}

// This file is the tapered Extrude's entry (agent-docs/draft-design.md §7): the
// taper's angle gate (SD2), the SD12 surface-result gate and extent
// resolution, then the one or two slabs internal/draftrecord builds over
// topology nodes (draftrecord.Build, draftrecord.BuildTwoSided).

// draftAngle is SD2 (agent-docs/draft-design.md Table SD) on a nonzero taper that
// has passed SD1: the angle in radians beside its conversion rounding, or
// ErrDegenerate for an angle at or past a right angle to the sweep. The test
// reads the stated angle in degrees as well as in radians, because a stated
// 90 degrees converts to a radian float just below π/2.
func draftAngle(taper units.Value) (float64, float64, error) {
	alpha, alphaDelta, err := extent.DisplacementIn(taper, units.Angle, units.Radian, "the taper")
	if err != nil {
		return 0, 0, err
	}
	if deg, err := taper.In(units.Degree); err != nil || math.Abs(deg) >= 90 || math.Abs(alpha) >= math.Pi/2 {
		return 0, 0, fmt.Errorf(`%w: a taper of %s leans the walls at or past a right angle to the sweep, so they sweep no solid; a draft angle lies strictly between -90 and 90 degrees`, ErrDegenerate, taper)
	}
	return alpha, alphaDelta, nil
}

// extrudeDraft is Extrude's path for a nonzero taper alpha (radians, within
// alphaDelta of the stated angle, past SD1 and SD2): the SD12 surface-result
// gate, extent resolution, then one or two draft slabs. A one-sided stop
// resolves its far level and axial bound before the single-slab build.
// ctx is checked between those steps and polled inside the slab builds; a
// canceled ctx returns ctx.Err() before the commit. Every gate runs before the
// document changes.
func (d *Document) extrudeDraft(ctx context.Context, profile profileRecord, frame r3.Frame, e Extent, alpha, alphaDelta float64, surfaceResult bool) (*Body, error) {
	switch e.(type) {
	case Distance, ThroughAll, ToFace, Symmetric, TwoSided:
	default:
		return nil, fmt.Errorf(`%w: a tapered extrude does not support extent %T`, ErrUnsupported, e)
	}
	if surfaceResult {
		return nil, fmt.Errorf(`%w: this evaluator builds a tapered extrude as a solid only; omit WithSurfaceResult or the taper (draft SD12)`, ErrUnsupported)
	}
	sweep, err := d.resolveLinearExtent(e, frame)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	ref := d.nextProducerID()
	base := draftPayload{
		Profile:    profile,
		Frame:      frame,
		Z0:         sweep.z0,
		Z1:         sweep.z1,
		Z0Delta:    sweep.z0Delta,
		Z1Delta:    sweep.z1Delta,
		NearStart:  sweep.z0 == 0,
		Taper:      alpha,
		TaperDelta: alphaDelta,
		Xform:      r3.Identity(),
	}
	body, err := bodyResult(draftrecord.Extrude(ctx, envOf(documentNode(d), ref), base))
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if err := d.commitOperationIn(ctx, nil, OperationExtrude, sweep.operationInputs, body); err != nil {
		return nil, err
	}
	return body, nil
}
