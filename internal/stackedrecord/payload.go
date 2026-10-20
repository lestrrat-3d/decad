package stackedrecord

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// Payload is the evaluator's own record of a stacked prism: the slabs and
// the interface patches between them, the section's frame and accumulated
// placement, and the section displacement every slab carries. Build
// evaluates it over topology nodes, and it implements topo.Payload
// (docs/stacked-prism-design.md §1, docs/evaluator-design.md §8).
type Payload struct {
	Slabs        []Slab
	Interfaces   []Interface
	Frame        r3.Frame
	Xform        r3.Transform
	SectionDelta float64
}

// Transform is the accumulated rigid placement.
func (sp Payload) Transform() r3.Transform { return sp.Xform }

// OpposedSectionExact reports whether the payload's section displacement is
// zero or no interface carries opposed holes.
func (sp Payload) OpposedSectionExact() bool {
	return sp.SectionDelta == 0 || !HasOpposed(sp.Interfaces)
}

// Placed re-evaluates the same record under the composed motion through
// Build with the payload's own plan.
func (sp Payload) Placed(ctx context.Context, env topo.Env, composed r3.Transform) (*topo.Body, error) {
	sp.Xform = composed
	return Build(ctx, env, sp, OwnPlan{})
}

// AxialDelta is the largest displacement any slab's end carries.
func (sp Payload) AxialDelta() float64 {
	var delta float64
	for _, slab := range sp.Slabs {
		delta = max(delta, slab.Z0Delta, slab.Z1Delta)
	}
	return delta
}

// OuterPrism is the outer-only prism over the whole stack's interval.
func (sp Payload) OuterPrism() prismrecord.Payload {
	first, last := sp.Slabs[0], sp.Slabs[len(sp.Slabs)-1]
	return prismrecord.Payload{
		Profile: momentinput.Profile{Outer: first.Regions[0].Outer},
		Frame:   sp.Frame, Xform: sp.Xform, SectionDelta: sp.SectionDelta,
		Z0: first.Z0, Z0Delta: first.Z0Delta,
		Z1: last.Z1, Z1Delta: last.Z1Delta,
	}
}

// OuterRuns is one outer-only prism per maximal run of consecutive slabs that
// carry one outer loop record, over that run's own interval. A cut-built stack
// is one run, the outer-only prism on the full interval. A union-built stack
// has one run per distinct outer, and every run's prism is material of the
// body, since a union-built slab region is hole-free (§2.2's union reading).
// The union of the runs' prisms is the body's outer envelope, which is what
// Bounds, ExtentAlong, the centroid's geometric cap and the tolerance gate's
// witnesses read.
func (sp Payload) OuterRuns() []prismrecord.Payload {
	base := sp.OuterPrism()
	planned := OuterRuns(sp.Slabs)
	runs := make([]prismrecord.Payload, len(planned))
	for i, slab := range planned {
		run := base
		run.Profile = slab.Regions[0]
		run.Z0, run.Z0Delta = slab.Z0, slab.Z0Delta
		run.Z1, run.Z1Delta = slab.Z1, slab.Z1Delta
		runs[i] = run
	}
	return runs
}

// ExtentAlong is the union of every outer run's extent along g, with the
// largest run bound.
func (sp Payload) ExtentAlong(g r3.Vec) (float64, float64, float64, error) {
	runs := sp.OuterRuns()
	var lo, hi, bound float64
	for i, run := range runs {
		rlo, rhi, rbound, err := run.ExtentAlong(g)
		if err != nil {
			return 0, 0, 0, err
		}
		if i == 0 {
			lo, hi, bound = rlo, rhi, rbound
			continue
		}
		lo, hi, bound = min(lo, rlo), max(hi, rhi), max(bound, rbound)
	}
	return lo, hi, bound, nil
}

// IsGroup reports whether the payload is a prism group
// (docs/mirror-pattern-design.md §6.3): one slab holding two or more regions.
func (sp Payload) IsGroup() bool {
	return len(sp.Slabs) == 1 && len(sp.Slabs[0].Regions) >= 2
}

// Record is the payload's slab and interface record.
func (sp Payload) Record() Record {
	return Record{Slabs: sp.Slabs, Interfaces: sp.Interfaces}
}

// VecL1 is a proven upper bound of v's L1 norm.
func VecL1(v r3.Vec) float64 {
	return proofbound.AbsSumUpper(v.X, v.Y, v.Z)
}

// SurveyClass names the payload class a staged survey diagnostic reports
// (internal/verifypublish): the root type name this payload had before it moved.
func (Payload) SurveyClass() string { return "stackedPrismPayload" }
