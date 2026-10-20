package loftrecord

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/featureoption"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectioncert"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// FromProfiles records the two profiles and builds their Loft under the
// decoded options cfg. The result is not yet committed.
func FromProfiles(ctx context.Context, env topo.Env, s0 *sketch.Sketch, p0 *sketch.Profile,
	s1 *sketch.Sketch, p1 *sketch.Profile, cfg featureoption.LoftConfig) (*topo.Body, error) {
	profile0, plane0, area0, err := sectioncert.RecordProfile(s0, p0)
	if err != nil {
		return nil, err
	}
	profile1, plane1, area1, err := sectioncert.RecordProfile(s1, p1)
	if err != nil {
		return nil, err
	}

	work0 := freeform.NewFreeformWork()
	work1 := freeform.NewFreeformWork()
	p := uint64(len(profile0.Outer.Segments))
	for _, hole := range profile0.Holes {
		p += uint64(len(hole.Segments))
	}
	limit := loftmesh.StationWorkLimit(0, p)
	work0.RaiseLimit(limit)
	work1.RaiseLimit(limit)
	work0.RaiseReconstructionLimit(freeform.LoftReconstructionWorkLimit)
	work1.RaiseReconstructionLimit(freeform.LoftReconstructionWorkLimit)
	recordArea0, err := profile0.FalsifyLoftRecordedArea(area0, work0)
	if err != nil {
		return nil, err
	}
	recordArea1, err := profile1.FalsifyLoftRecordedArea(area1, work1)
	if err != nil {
		return nil, err
	}

	frame0, err := r3.NewFrame(plane0.Origin, plane0.U, plane0.V)
	if err != nil {
		return nil, fmt.Errorf(`%w: the first recorded plane is degenerate: %s`, decaderr.ErrDegenerate, err)
	}
	frame1, err := r3.NewFrame(plane1.Origin, plane1.U, plane1.V)
	if err != nil {
		return nil, fmt.Errorf(`%w: the second recorded plane is degenerate: %s`, decaderr.ErrDegenerate, err)
	}

	body, err := Build(ctx, env, Payload{
		Profile0: profile0, Profile1: profile1,
		Plane0: plane0, Plane1: plane1,
		Frame0: frame0, Frame1: frame1,
		Alignment:                   cfg.Alignment,
		Xform:                       r3.Identity(),
		AuthenticatedReconstruction: true,
		SurfaceResult:               cfg.SurfaceResult,
		RecordArea:                  [2]float64{recordArea0, recordArea1},
	}, proofbound.NewWorkBudget(ctx), work0, work1)
	if err != nil {
		return nil, err
	}
	return body, ctx.Err()
}

// FromChains builds the LoftChain of two recorded open chains. The result is
// not yet committed.
func FromChains(ctx context.Context, env topo.Env, chain0 sectionrecord.ChainRecord, plane0 sectionrecord.PlaneRecord,
	chain1 sectionrecord.ChainRecord, plane1 sectionrecord.PlaneRecord) (*topo.Body, error) {
	frame0, err := r3.NewFrame(plane0.Origin, plane0.U, plane0.V)
	if err != nil {
		return nil, fmt.Errorf(`%w: the first recorded plane is degenerate: %s`, decaderr.ErrDegenerate, err)
	}
	frame1, err := r3.NewFrame(plane1.Origin, plane1.U, plane1.V)
	if err != nil {
		return nil, fmt.Errorf(`%w: the second recorded plane is degenerate: %s`, decaderr.ErrDegenerate, err)
	}

	body, err := BuildChain(ctx, env, ChainPayload{
		Chain0: chain0, Chain1: chain1,
		Plane0: plane0, Plane1: plane1,
		Frame0: frame0, Frame1: frame1,
		Xform: r3.Identity(),
	}, proofbound.NewWorkBudget(ctx), freeform.NewFreeformWork(), freeform.NewFreeformWork())
	if err != nil {
		return nil, err
	}
	return body, ctx.Err()
}
