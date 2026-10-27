package loftrecord

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/featureoption"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/loftsplit"
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
	return FromProfilesReusing(ctx, env, s0, p0, s1, p1, cfg, nil)
}

// FromProfilesReusing is FromProfiles that reads and fills reuse
// (PairingReuse). A nil reuse builds exactly as FromProfiles.
func FromProfilesReusing(ctx context.Context, env topo.Env, s0 *sketch.Sketch, p0 *sketch.Profile,
	s1 *sketch.Sketch, p1 *sketch.Profile, cfg featureoption.LoftConfig, reuse *PairingReuse) (*topo.Body, error) {
	profile0, plane0, area0, err := sectioncert.RecordProfile(s0, p0)
	if err != nil {
		return nil, err
	}
	profile1, plane1, area1, err := sectioncert.RecordProfile(s1, p1)
	if err != nil {
		return nil, err
	}

	// The splits cut recorded segments into pieces of the same curves, so the
	// split records denote the same sections (internal/loftsplit).
	if profile0, err = loftsplit.Apply(profile0, splitsOf(cfg.Splits[0])); err != nil {
		return nil, err
	}
	if profile1, err = loftsplit.Apply(profile1, splitsOf(cfg.Splits[1])); err != nil {
		return nil, err
	}

	held := reuse.lookup(profile0, profile1, area0, area1, cfg.Alignment)
	work0 := freeform.NewFreeformWork()
	work1 := freeform.NewFreeformWork()
	var recordArea [2]float64
	if held != nil {
		// The area falsifiers already passed on these records and sketch areas
		// from counters in this same state (PairingReuse).
		recordArea = held.recordArea
	} else {
		p := uint64(len(profile0.Outer.Segments))
		for _, hole := range profile0.Holes {
			p += uint64(len(hole.Segments))
		}
		limit := loftmesh.StationWorkLimit(0, p)
		work0.RaiseLimit(limit)
		work1.RaiseLimit(limit)
		work0.RaiseReconstructionLimit(freeform.LoftReconstructionWorkLimit)
		work1.RaiseReconstructionLimit(freeform.LoftReconstructionWorkLimit)
		if recordArea[0], err = profile0.FalsifyLoftRecordedArea(area0, work0); err != nil {
			return nil, err
		}
		if recordArea[1], err = profile1.FalsifyLoftRecordedArea(area1, work1); err != nil {
			return nil, err
		}
	}

	frame0, err := r3.NewFrame(plane0.Origin, plane0.U, plane0.V)
	if err != nil {
		return nil, fmt.Errorf(`%w: the first recorded plane is degenerate: %s`, decaderr.ErrDegenerate, err)
	}
	frame1, err := r3.NewFrame(plane1.Origin, plane1.U, plane1.V)
	if err != nil {
		return nil, fmt.Errorf(`%w: the second recorded plane is degenerate: %s`, decaderr.ErrDegenerate, err)
	}

	pl := Payload{
		Profile0: profile0, Profile1: profile1,
		Plane0: plane0, Plane1: plane1,
		Frame0: frame0, Frame1: frame1,
		Alignment:                   cfg.Alignment,
		Xform:                       r3.Identity(),
		AuthenticatedReconstruction: true,
		SurfaceResult:               cfg.SurfaceResult,
		RecordArea:                  recordArea,
	}
	budget := proofbound.NewWorkBudget(ctx)

	// What follows is Build, with its record-only half read from reuse when
	// reuse holds these records.
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	var paired recordPairing
	var next *heldPairing
	if held != nil {
		// The S5 gate is the only step of pairPayload that reads the planes.
		// The steps before it passed on these same inputs and the steps after
		// it pass again, so it is the one step that runs again.
		if err := loftmesh.ProfilePlaneGate(plane0, plane1); err != nil {
			return nil, err
		}
		paired = held.paired.clone()
		reuse.reused++
	} else {
		if paired, err = pairPayload(pl, work0, work1); err != nil {
			return nil, err
		}
		if reuse != nil {
			// buildPaired may write to paired.pairs, so reuse keeps its own copy.
			next = &heldPairing{recordArea: recordArea, paired: paired.clone()}
		}
	}
	body, err := buildPaired(ctx, env, pl, budget, paired)
	if err != nil {
		return nil, err
	}
	reuse.hold(next, profile0, profile1, area0, area1, cfg.Alignment)
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

// splitsOf converts the option's splits to loftsplit's.
func splitsOf(in []featureoption.LoftSplit) []loftsplit.Split {
	out := make([]loftsplit.Split, len(in))
	for i, s := range in {
		out[i] = loftsplit.Split{Loop: s.Loop, Segment: s.Segment, At: s.At}
	}
	return out
}
