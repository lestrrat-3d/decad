package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/embossplan"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/ribplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// EmbossMode says whether a relief stands out of a face or is cut into it.
type EmbossMode int

const (
	// EmbossRaised adds the relief on the outward side of the face.
	EmbossRaised EmbossMode = iota
	// EmbossEngraved cuts the relief into the body from the face.
	EmbossEngraved
)

// String names the mode.
func (m EmbossMode) String() string {
	switch m {
	case EmbossRaised:
		return "EmbossRaised"
	case EmbossEngraved:
		return "EmbossEngraved"
	default:
		return fmt.Sprintf("EmbossMode(%d)", int(m))
	}
}

// Emboss raises or engraves the profiles on a planar face of b and returns the
// result, retiring b.
//
// face selects the planar face and must resolve to exactly one face. s is a
// sketch whose plane is the plane of that face, and profiles are closed
// profiles of s, such as the outlines of letters or a logo; a profile with
// holes keeps them. The relief is height tall: a raised relief stands height
// out of the face, and an engraved one is cut height into the body. The tool
// extends one extra millimetre to the other side of the face so that it
// overlaps the body instead of touching it (agent-docs/emboss-design.md). The
// result has the exactness Union or Cut gives for the same bodies.
//
// b MUST be a live solid. A nil context, body, sketch or selector, an empty
// or nil profile list, a non-positive height, a sketch plane that is more than
// 1e-6 mm off the face plane or not parallel to it, and a relief that
// certifiably misses b or changes no volume are ErrDegenerate. A face that is
// not an analytic plane is ErrUnsupported. Profiles must not touch each other
// in a shared plane. A boolean the evaluator cannot decide is the boolean's
// own error. Every failure leaves the document unchanged and b live.
func Emboss(ctx context.Context, b *Body, s *sketch.Sketch, profiles []*sketch.Profile, face FaceSelector,
	mode EmbossMode, height units.Value) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control an emboss`, ErrDegenerate)
	}
	if b == nil || b.n.Doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	d := documentHandle(b.n.Doc)
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if b.Kind() != BodySolid {
		return nil, fmt.Errorf(`%w: an emboss needs a solid body`, ErrNotSolid)
	}
	if mode != EmbossRaised && mode != EmbossEngraved {
		return nil, fmt.Errorf(`%w: unknown emboss mode %d`, ErrDegenerate, int(mode))
	}
	if s == nil || len(profiles) == 0 {
		return nil, fmt.Errorf(`%w: an emboss needs a sketch and at least one profile`, ErrDegenerate)
	}
	if face == nil {
		return nil, errNilSelector
	}
	h, err := sectionrecord.MagnitudeIn(height, units.Length, units.Millimeter, "the relief height")
	if err != nil {
		return nil, err
	}
	plane, outward, err := resolvePlanarFace(b, face, "Emboss")
	if err != nil {
		return nil, err
	}
	frame, err := s.Plane().Frame()
	if err != nil {
		return nil, fmt.Errorf(`%w: the sketch plane has no frame: %s`, ErrDegenerate, err)
	}
	sweep, err := embossplan.Plan(frame, plane.Frame, outward.Value, h, mode == EmbossRaised)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	extent := TwoSided{
		One: DistanceSide{D: units.Millimeters(sweep.Along)},
		Two: DistanceSide{D: units.Millimeters(sweep.Against)},
	}
	var tools []*Body
	defer func() { removeTools(tools) }()
	for _, p := range profiles {
		if p == nil {
			return nil, fmt.Errorf(`%w: a nil profile names no relief`, ErrDegenerate)
		}
		tool, err := Extrude(ctx, d, s, p, extent)
		if err != nil {
			return nil, err
		}
		tools = append(tools, tool)
		disjoint, err := boundsDisjoint(b, tool)
		if err != nil {
			return nil, err
		}
		if disjoint {
			return nil, fmt.Errorf(`%w: a profile lies outside the body, so its relief would not reach it`, ErrDegenerate)
		}
	}
	op := meshbool.OpUnion
	if mode == EmbossEngraved {
		op = meshbool.OpCut
	}
	return combineTools(ctx, d, b, op, tools, false, func(result *Body) error {
		return requireReliefChangesVolume(b, result, mode)
	})
}

// requireReliefChangesVolume refuses a result whose volume certifiably did not
// move the way the mode says: a raised relief that adds nothing or an
// engraving that removes nothing.
func requireReliefChangesVolume(before, after *Body, mode EmbossMode) error {
	vb, err := before.Volume()
	if err != nil {
		return err
	}
	va, err := after.Volume()
	if err != nil {
		return err
	}
	b0, bb, a0, ab := vb.Value.Base(), vb.Bound.Base(), va.Value.Base(), va.Bound.Base()
	if mode == EmbossRaised && ribplan.AddsNothing(b0, bb, a0, ab) {
		return fmt.Errorf(`%w: the raised relief lies inside the body and adds no material`, ErrDegenerate)
	}
	if mode == EmbossEngraved && embossplan.RemovesNothing(b0, bb, a0, ab) {
		return fmt.Errorf(`%w: the engraving lies outside the material and removes none`, ErrDegenerate)
	}
	return nil
}
