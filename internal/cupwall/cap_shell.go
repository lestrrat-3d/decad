package cupwall

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismshell"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// CapShell is the offset stage of a cap-removing shell of a straight prism:
// the cavity's section and floor limits, S11a as the offset is built, then
// the offset section's audit (agent-docs/modify-design.md §4/§5). It exists so a
// caller can mint the result's producer identity only after that stage has
// passed, and spend it in Build.
type CapShell struct {
	in         prismshell.CapShellInput
	offset     momentinput.Profile
	budget     *proofbound.WorkBudget
	removedEnd bool
}

// PlanCapShell runs the offset stage for in. removedEnd says which cap a
// one-cap shell opens: the end cap when true, the start cap when false.
func PlanCapShell(ctx context.Context, in prismshell.CapShellInput, removedEnd bool) (*CapShell, error) {
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, err
	}
	// Stages 3 and 4 (§4/§5): S18 and S10, the cavity's section and floor
	// limits (inward only), S11a as the offset is built, then the offset
	// section's audit.
	offset, err := in.OffsetSection(budget)
	if err != nil {
		return nil, err
	}
	return &CapShell{in: in, offset: offset, budget: budget, removedEnd: removedEnd}, nil
}

// Build builds the shell: a tube when both caps go and the section has no
// hole, one band around the outer loop plus one lining each hole (B4 / reach
// BX8) when both go otherwise, and a cup (B5/B6) when one cap stays.
func (c *CapShell) Build(ctx context.Context, env topo.Env) (*topo.Body, error) {
	bothCaps := c.in.KeptCaps == 0
	if bothCaps && len(c.in.Prism.Profile.Holes) == 0 {
		return prismshell.BuildTube(ctx, env, c.in, c.offset)
	}
	offsetDelta, err := c.in.SectionDelta(c.budget)
	if err != nil {
		return nil, err
	}
	if bothCaps {
		return prismshell.BuildBands(ctx, env, c.in, c.offset, offsetDelta)
	}
	// A one-cap shell is a cup, for any k ≥ 0: the offset section's loops are
	// proven simple and correctly nested by the §5 audit, and the cup wraps a
	// wall band around each post, all hanging off the one floor slab (one
	// lump). It records the offset section's displacement beside it (§9).
	cup, err := ViewFor(c.in.Prism, c.offset, c.in.Sense, c.in.HeldThickness, c.in.ThicknessDelta,
		offsetDelta, c.removedEnd).Payload(ctx)
	if err != nil {
		return nil, err
	}
	return Build(ctx, env, cup)
}
