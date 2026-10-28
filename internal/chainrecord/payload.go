package chainrecord

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// Payload is chain Extrude's own record of a ribbon body: the recorded
// walk SET, the plane frame it lifts through, the signed sweep interval, and
// the accumulated rigid placement — Payload is to chain Extrude what
// prismrecord.Payload is to Extrude (agent-docs/surface-design.md §13.4). A plain
// chain Extrude builds the one-walk case; agent-docs/surface-intersection-design.md
// §3.4 is what first builds more than one, one lump per surviving walk, and
// sets SectionDelta beside them. Build evaluates it over topology nodes and it
// implements topo.Payload (agent-docs/evaluator-design.md §8).
type Payload struct {
	Chains  []sectionrecord.ChainRecord
	Frame   r3.Frame
	Z0, Z1  float64
	Z0Delta float64
	Z1Delta float64
	// Z0Denoted and Z1Denoted are prismrecord.Payload's own level
	// denotations (agent-docs/stacked-prism-design.md §2.1).
	Z0Denoted, Z1Denoted prismplacement.LevelDenotation
	Xform                r3.Transform
	// SectionDelta is prismrecord.Payload's own §7 term
	// (agent-docs/prism-boolean-design.md), carried here on the identical terms: the
	// proven upper bound on how far any recorded boundary coordinate sits from
	// the section its construction denotes. Zero for every walk a caller draws
	// through chain Extrude.
	SectionDelta float64
}

// Z0Scalar is the first sweep level with its own displacement.
func (pp Payload) Z0Scalar() proofbound.BoundedScalar {
	return proofbound.MeasuredScalar(pp.Z0, pp.Z0Delta)
}

// Z1Scalar is the second sweep level with its own displacement.
func (pp Payload) Z1Scalar() proofbound.BoundedScalar {
	return proofbound.MeasuredScalar(pp.Z1, pp.Z1Delta)
}

// Prism is a *view* of pp as a zero-section-delta prismrecord.Payload — never
// a body this evaluator builds — used only to feed pp's point/dir/reflected
// readings and prismrecord.Bounds, none of which cares whether the section it
// reads closes. SectionDelta stays zero in THIS view deliberately:
// prismrecord.Bounds charges a nonzero SectionDelta as a
// BLANKET term, δ outward on every face regardless of which candidate wins
// each extreme, which is sound for an ordinary prism but would widen a
// trimmed ribbon's UNTOUCHED extremes too — exactly the ones T170 requires to
// stay bit-identical to the untrimmed sheet's own. The tight, per-extreme
// charge this design actually needs is carried instead by the WALKS
// Build resolves through prismcells.TrimProfileWalks
// (internal/prismcells/trim_walks.go) when pp.SectionDelta != 0, never by this view's own
// field. The first chain stands in for a profile's outer loop, the rest for
// its holes — the extent reading treats every one the same way, caring only
// about the segments, never about winding or closure.
func (pp Payload) Prism() prismrecord.Payload {
	profile := momentinput.Profile{Outer: sectionrecord.LoopRecord(pp.Chains[0])}
	for _, c := range pp.Chains[1:] {
		profile.Holes = append(profile.Holes, sectionrecord.LoopRecord(c))
	}
	return prismrecord.Payload{
		Profile: profile,
		Frame:   pp.Frame,
		Z0:      pp.Z0, Z1: pp.Z1,
		Z0Delta: pp.Z0Delta, Z1Delta: pp.Z1Delta,
		Z0Denoted: pp.Z0Denoted, Z1Denoted: pp.Z1Denoted,
		Xform: pp.Xform,
	}
}

// Transform is the accumulated rigid placement.
func (pp Payload) Transform() r3.Transform { return pp.Xform }

// ProvesSimple implements topo.SimpleProver. A chain-fed prism earns leg 4 on
// the profile argument with one word changed (agent-docs/surface-design.md §13.4):
// `sketch` already proved the recorded walk a simple planar curve —
// RecordChain's own ch.Valid gate (§13.3) — Build already refuses a
// non-positive sweep height, and a simple planar curve crossed with a positive
// interval cannot self-intersect. A chain-fed body is always a sheet by
// construction (Table G), so SurfaceResult carries no separate condition here;
// SectionDelta is the one term that can still break the argument, on the
// identical prism reading.
func (pp Payload) ProvesSimple(context.Context) bool { return pp.SectionDelta == 0 }

// Placed re-evaluates the same record under the composed motion (core §8). A
// re-evaluation path: no moments preflight has run on this record within the
// call, so the build opens the record's one free-form work counter itself
// (agent-docs/spline-design.md §5.2), exactly as prismrecord.Payload.Placed does.
func (pp Payload) Placed(ctx context.Context, env topo.Env, composed r3.Transform) (*topo.Body, error) {
	pp.Xform = composed
	return Build(ctx, env, pp, freeform.NewFreeformWork())
}

// SurveyClass names the payload class a staged survey diagnostic reports
// (internal/verifypublish): the root type name this payload had before it moved.
func (Payload) SurveyClass() string { return "chainPayload" }
