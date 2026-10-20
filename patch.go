package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/patchchain"
	"github.com/lestrrat-3d/decad/internal/sectioncert"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// This file is PatchProfile of agent-docs/surface-design.md §5.1: a single planar
// face built from a recorded profile, on the sketch plane's own frame. It is
// the seam call (sectioncert.RecordProfile) plus one evaluation, patchchain.Build,
// that mirrors prismrecord.Build's order without ever building a solid.
//
// patchPayload (record.go) is the patchchain.Payload that records what the body
// was built from, and patchchain.Build is its evaluation.

// PatchProfile records p through the seam exactly as Extrude does — the same
// authentication, staleness, foreign-entity and TExact admission gates of
// docs/api-design.md §7 and agent-docs/sketch-seam-design.md, with no relaxation —
// and builds a single planar face on s.Plane().Frame(), carrying the
// profile's outer loop and every hole loop (agent-docs/surface-design.md §5.1). The
// result is a one-face sheet body: Kind() == BodySheet, one lump, one open
// shell, and one free edge per coalesced boundary walk. p MUST be a profile
// of s (ErrForeignProfile) and a current, unaltered snapshot (ErrStaleProfile
// or ErrInvalidProfile); an invalid profile is also ErrInvalidProfile, and a
// boundary decad cannot record exactly is ErrUnrecordableProfile. A recorded
// boundary carrying a free-form segment this evaluator cannot integrate is
// ErrUnsupported (Table R row R3). A failed evaluation leaves the document
// untouched. A nil context or a nil d is ErrDegenerate.
func PatchProfile(ctx context.Context, d *Document, s *sketch.Sketch, p *sketch.Profile) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a patch`, ErrDegenerate)
	}
	if d == nil {
		return nil, fmt.Errorf(`%w: a nil document owns no model`, ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	profile, plane, profileArea, err := sectioncert.RecordProfile(s, p)
	if err != nil {
		return nil, err
	}
	// ONE free-form work counter for this whole call, same convention as
	// Extrude: the area falsifier's preflight opens it and patchchain.Build's
	// own walk resolution spends what is left (agent-docs/spline-design.md §5.2).
	work := freeform.NewFreeformWork()
	if _, err := profile.FalsifyRecordedArea(profileArea, work); err != nil {
		return nil, err
	}
	frame, err := r3.NewFrame(plane.Origin, plane.U, plane.V)
	if err != nil {
		return nil, fmt.Errorf(`%w: the recorded plane is degenerate: %s`, ErrDegenerate, err)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	ref := d.nextProducerID()
	body, err := bodyResult(patchchain.Build(ctx, envOf(&d.n, ref), patchPayload{
		Profile: profile,
		Frame:   frame,
		Xform:   r3.Identity(),
	}, work))
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(body)
	return body, nil
}
