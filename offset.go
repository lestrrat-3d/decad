package decad

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/extent"
	"github.com/lestrrat-3d/decad/internal/opcommand"
	"github.com/lestrrat-3d/decad/internal/orthoshell"
	"github.com/lestrrat-3d/decad/internal/thickenbuild"

	"github.com/lestrrat-3d/units"
)

// This file is Offset of agent-docs/surface-design.md §17: a SECOND sheet whose
// every face lies at a stated normal distance from the source face it came
// from. It admits §16's two receiver families and reads the same recorded
// generator Thicken reads, and it differs from Thicken in exactly two ways.
//
// The receiver stays LIVE. An offset depends on its source and consumes
// nothing — the two are the pair a caller stitches, thickens or measures a
// clearance between — so the result registers on the non-consuming terms
// Copy already takes (document.go's copyUnder,
// agent-docs/api-design.md §6). Nothing sums the pair: Volume and Centroid are
// ErrNotSolid on a sheet by kind (§8), and Union/Cut/Intersect refuse a sheet
// permanently (Table X), so the two live bodies can never be combined into a
// reading that counts the same material twice.
//
// The result holds ONE region, bounded by the offset loop alone. Thicken
// assembles an annulus from an outer and an inner loop, so it must first prove
// the source and offset loops disjoint and strictly nested; this build
// publishes no such relation and denotes nothing whatever about the source, so
// that audit has no subject here and offset2d.ReverseLoopRecordContext and
// prismshell.BuildTube are not on this path (§17.2). Everything else §16.2 states —
// offset2d.OffsetProfile, offset2d.AuditOffsetSectionBudget, the exact-generation gate and the
// whole-interval certification of thicken_axis.go — runs unchanged.

// OffsetSide names the side of the source sheet the offset sheet is placed on.
// It is its own two-value type rather than a reuse of [ThickenSide]: an offset
// publishes ONE surface and two surfaces are two calls, so there is no centered
// value, and ThickenCentered can never enter an operation that would have to
// refuse it at run time (agent-docs/surface-design.md §17.1).
type OffsetSide int

const (
	// OffsetPositive places the offset along the sheet's positive normal and is
	// the default.
	OffsetPositive OffsetSide = iota
	// OffsetNegative places the offset opposite the sheet's positive normal.
	OffsetNegative
)

// Offset returns a second sheet whose every face lies at the stated normal
// distance from the source face of b it came from, LEAVING b LIVE.
//
// distance is a strictly positive [units.Value] of kind Length; its sign never
// chooses a side, and [OffsetCommand.Side] does, the last call winning. A
// recorded planar patch offsets by translation along its own frame normal; a
// profile-fed prism sheet offsets by the certified per-feature section offset
// of agent-docs/modify-design.md §8, growing a radius-distance arc at each convex
// corner and mitering each concave one. Every other sheet family is ErrUnsupported (agent-docs/surface-design.md Table R row R37). A solid
// offsets only when it is a faceted solid whose faces all square to a world axis, such as a mesh solid
// (agent-docs/orthogonal-design.md): each face moves along its outward normal, outward for OffsetPositive
// and inward for OffsetNegative, the neighbouring faces extended or trimmed to meet, and the result is a second
// solid; any other solid is ErrUnsupported.
//
// A nil context given to Do, a nil b or a side that is neither value is
// ErrDegenerate, as is a zero distance; a wrong-kind, non-finite or negative distance keeps
// sectionrecord.MagnitudeIn's own sentinel (R40). A retired b is ErrRetiredBody (R41).
// A refusal registers nothing and retires nothing, so b is live
// after a refusal exactly as it is after a success.
func Offset(b *Body, distance units.Value) *OffsetCommand {
	return &OffsetCommand{b: b, distance: distance}
}

// OffsetCommand offsets a sheet or an axis-square solid. Build it with
// [Offset].
type OffsetCommand struct {
	state    opcommand.State            // consumed flag and first recorded fault
	b        *Body                      // stored as given
	distance units.Value                // the normal distance, gated at Do
	side     opcommand.Slot[OffsetSide] // set by Side
}

// Side selects the side the offset sheet is placed on. Without it the side is
// [OffsetPositive]. A value that is neither side is ErrDegenerate, returned by
// Do. A second call replaces the first.
func (c *OffsetCommand) Side(side OffsetSide) *OffsetCommand {
	var err error
	if side < OffsetPositive || side > OffsetNegative {
		err = fmt.Errorf(`%w: unknown offset side %d`, ErrDegenerate, side)
	}
	c.side.Set(side, err)
	return c
}

// Do builds the offset and leaves the body live, or returns the error of the
// first gate that refuses. A nil ctx is ErrDegenerate, and a second call
// returns ErrConsumed.
func (c *OffsetCommand) Do(ctx context.Context) (*Body, error) {
	return runCommand(ctx, &c.state, OperationOffset, c.run)
}

func (c *OffsetCommand) run(ctx context.Context) (*Body, error) {
	side, sideErr := c.side.Get()
	return offset(ctx, c.b, c.distance, side, sideErr)
}

// offset is Offset's implementation. ctx is not nil, and the errors it
// returns carry no operation tag (agent-docs/command-api-design.md §11).
// sideErr is the fault of the Side call.
func offset(ctx context.Context, b *Body, distance units.Value, side OffsetSide, sideErr error) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if b == nil || b.n.Doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	d := documentHandle(b.n.Doc)
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if sideErr != nil {
		return nil, subjectError(OperationOffset, "Side", sideErr)
	}
	dmm, dDelta, err := extent.MagnitudeInBounded(distance, units.Length, units.Millimeter, "the offset distance")
	if err != nil {
		return nil, err
	}
	if dmm == 0 {
		return nil, fmt.Errorf(`%w: a zero distance names no offset surface`, ErrDegenerate)
	}
	if b.Kind() == BodySolid {
		return offsetSolid(ctx, d, b, side, dmm)
	}
	if b.Kind() != BodySheet {
		return nil, fmt.Errorf(`%w: Offset requires a sheet body`, ErrUnsupported)
	}
	node, err := thickenbuild.Offset(ctx, envOf(documentNode(d), d.producerAt(ctx)), bodyNode(b), side == OffsetNegative, dmm, dDelta)
	if err != nil {
		return nil, err
	}
	result := bodyHandle(node)
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	// The source is DEPENDED ON, never consumed: commit with no consumed input,
	// exactly as copyUnder does, so the receiver stays in Document.Bodies()
	// beside its offset (agent-docs/surface-design.md §17.1).
	if err := d.commitOperationIn(ctx, d.bodyContainer(b), OperationOffset,
		[]OperationInput{{Body: b, Role: operationRoleSource, Use: InputReferenced}}, result); err != nil {
		return nil, err
	}
	return result, nil
}

// offsetSolid offsets a faceted solid every face of which squares to a world axis (agent-docs/orthogonal-design.md):
// each face moves along its outward normal by the distance, outward for OffsetPositive and inward for OffsetNegative, and
// the result is a second solid, the receiver staying live as for a sheet. Any other solid is ErrUnsupported.
func offsetSolid(ctx context.Context, d *Document, b *Body, side OffsetSide, dmm float64) (*Body, error) {
	payload, ok := b.n.Payload.(facetedPayload)
	if !ok || len(payload.Tris) == 0 {
		return nil, fmt.Errorf(`%w: Offset requires a sheet body (or a faceted solid whose faces all square to an axis, such as a mesh solid)`, ErrUnsupported)
	}
	for _, beta := range payload.VertexBound {
		if beta != 0 {
			return nil, fmt.Errorf(`%w: Offset takes a faceted solid whose held vertices carry no bound`, ErrUnsupported)
		}
	}
	distance := dmm
	if side == OffsetNegative {
		distance = -dmm
	}
	verts, tris, err := orthoshell.Offset(payload.Verts, payload.Tris, distance)
	switch {
	case errors.Is(err, orthoshell.ErrNotOrthogonal):
		return nil, fmt.Errorf(`%w: Offset takes a solid whose faces all square to an axis: %s`, ErrUnsupported, err)
	case errors.Is(err, orthoshell.ErrDegenerate):
		return nil, fmt.Errorf("%w: %s", ErrDegenerate, err)
	case errors.Is(err, orthoshell.ErrUnsupported):
		return nil, fmt.Errorf("%w: %s", ErrUnsupported, err)
	case err != nil:
		return nil, err
	}
	mesh, err := NewMeshBody(ctx, verts, tris)
	if err != nil {
		return nil, err
	}
	result, err := solidFromMeshBody(ctx, d, mesh)
	if err != nil {
		return nil, fmt.Errorf("%w: the offset solid is not one the evaluator can prove: %s", ErrUnsupported, err)
	}
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if err := d.commitOperationIn(ctx, d.bodyContainer(b), OperationOffset,
		[]OperationInput{{Body: b, Role: operationRoleSource, Use: InputReferenced}}, result); err != nil {
		return nil, err
	}
	return result, nil
}
