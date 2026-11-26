package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/opcommand"
	"github.com/lestrrat-3d/decad/internal/pointloft"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectioncert"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// LoftFromPoint builds a solid from one exact world point to a closed Sketch
// profile. The apex is in millimetres. This operation accepts one outer loop
// and no holes; every fitted flank remains the recorded Sketch curve. The
// boundary is a certified chorded fan and a cap (agent-docs/loft-design.md §20).
func LoftFromPoint(d *Document, apex r3.Vec, s *sketch.Sketch, p *sketch.Profile) *LoftFromPointCommand {
	c := &LoftFromPointCommand{d: d, apex: apex, s: s, p: p}
	recordNilFaults(&c.state, "LoftFromPoint", d, sketchArg(s), profileArg(p))
	return c
}

// LoftFromPointCommand builds a solid from a point to a profile. Build it with
// [LoftFromPoint]. It has no method.
type LoftFromPointCommand struct {
	state opcommand.State // consumed flag and first recorded fault
	d     *Document       // stored as given
	apex  r3.Vec          // gated at Do
	s     *sketch.Sketch  // stored as given
	p     *sketch.Profile // stored as given
}

// Do builds the point loft, registers the new body and returns it, or returns
// the error of the first gate that refuses, leaving the document unchanged. A
// nil ctx is ErrDegenerate, and a second call returns ErrConsumed.
func (c *LoftFromPointCommand) Do(ctx context.Context) (*Body, error) {
	return runCommand(ctx, &c.state, OperationLoftFromPoint, c.run)
}

func (c *LoftFromPointCommand) run(ctx context.Context) (*Body, error) {
	return loftFromPoint(ctx, c.d, c.apex, c.s, c.p)
}

// loftFromPoint is LoftFromPoint's implementation. ctx is not nil, and the
// errors it returns carry no operation tag (agent-docs/command-api-design.md §11).
func loftFromPoint(ctx context.Context, d *Document, apex r3.Vec,
	s *sketch.Sketch, p *sketch.Profile) (*Body, error) {
	if d == nil || s == nil || p == nil {
		return nil, fmt.Errorf("%w: point loft needs a document, sketch and profile", ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if !proofbound.FiniteVec(apex) {
		return nil, fmt.Errorf("%w: the point loft apex must be finite", ErrNotFinite)
	}
	profile, plane, sketchArea, err := sectioncert.RecordProfile(s, p)
	if err != nil {
		return nil, err
	}
	if len(profile.Holes) != 0 {
		return nil, fmt.Errorf("%w: a point loft cannot cone hole loops to one manifold apex", ErrUnsupported)
	}
	// The exact dyadic determinant is the plane-side existence gate. A float
	// dot product can round a shallow but valid point/plane separation to 0.
	separation := proofarith.XdotRat(
		proofarith.Xsub(proofarith.XptOf(apex), proofarith.XptOf(plane.Origin)),
		proofarith.Xcross(proofarith.XptOf(plane.U), proofarith.XptOf(plane.V)),
	)
	if separation.Sign() == 0 {
		return nil, fmt.Errorf("%w: the point loft apex lies in the profile plane", ErrDegenerate)
	}
	frame, err := r3.NewFrame(plane.Origin, plane.U, plane.V)
	if err != nil {
		return nil, fmt.Errorf("%w: the profile plane has no frame: %s", ErrDegenerate, err)
	}
	work0, work1 := freeform.NewFreeformWork(), freeform.NewFreeformWork()
	segmentCount := uint64(len(profile.Outer.Segments))
	limit := loftmesh.StationWorkLimit(0, segmentCount)
	work0.RaiseLimit(limit)
	work1.RaiseLimit(limit)
	work0.RaiseReconstructionLimit(freeform.LoftReconstructionWorkLimit)
	work1.RaiseReconstructionLimit(freeform.LoftReconstructionWorkLimit)
	area, err := profile.FalsifyRecordedArea(sketchArea, work0)
	if err != nil {
		return nil, err
	}
	source := &pointSectionSource{
		Sketch: s, Profile: p, Revision: s.Revision(), Frame: frame,
		Entities: make([]sketch.Entity, len(p.Outer)),
	}
	for i, edge := range p.Outer {
		source.Entities[i] = edge.Entity
	}
	record := pointSectionRecord{Apex: apex, Profile: profile, Plane: plane, Source: source}
	ref := d.producerAt(ctx)
	payload, err := pointloft.Build(ctx, record, frame, area, work0, work1, ref)
	if err != nil {
		return nil, err
	}
	body, err := bodyResult(facetedrecord.Build(ctx, envOf(documentNode(d), ref), payload))
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if err := d.commitOperationIn(ctx, d.ContainerOfSketch(s), OperationLoftFromPoint, nil, body); err != nil {
		return nil, err
	}
	return body, nil
}
