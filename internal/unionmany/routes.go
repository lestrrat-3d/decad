// Package unionmany orders the certified many-operand Union routes and
// evaluates fallback prism pair chains over nodes.
package unionmany

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/axialsection"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/prismboolean"
	"github.com/lestrrat-3d/decad/internal/prismpattern"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveshell"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/verifiedmesh"
)

// SectionUnionError wraps the refusal Routes returns when every route
// declined a set and the union of its prism sections was among the refusals
// (agent-docs/axial-section-boolean-design.md §2). The caller keeps that refusal
// as the answer rather than trying a pair chain. Unwrap returns the refusal.
type SectionUnionError struct{ Err error }

func (e *SectionUnionError) Error() string { return e.Err.Error() }

func (e *SectionUnionError) Unwrap() error { return e.Err }

// Routes tries the certified prism, axial, mirrored-Loft, circular-Loft,
// Revolve-sector, prism-section union, copied-prism and straight-source Loft
// routes in that order. Their owning designs define each admitted class and
// its proofs. A route's error stops the chain, except the prism-section union's,
// which counts as a miss so the copied-prism route still takes what it
// certifies; exhaustion then returns a SectionUnionError. A miss tries the
// next route. Exhaustion returns ErrUnsupported. A completed result uses ref and
// spans one producer identity. It commits no operation and leaves every input
// body unchanged.
// ctx, env.Doc, env.Alloc, mesher and every operand must not be nil, and
// operands must contain at least three bodies belonging to env.Doc.
func Routes(ctx context.Context, env topo.Env, ref provenance.Producer,
	operands []*topo.Body, mesher verifiedmesh.Mesher) (*topo.Body, provenance.Producer, error) {
	env.Ref = ref
	a := operands[0]
	if node, matched, err := prismboolean.UnionMany(ctx, env, operands); matched || err != nil {
		if err != nil {
			return nil, 0, err
		}
		return node, 1, ctx.Err()
	}
	if node, matched, err := axialsection.UnionMany(ctx, env, operands); matched || err != nil {
		if err != nil {
			return nil, 0, err
		}
		return node, 1, ctx.Err()
	}
	tools := operands[1:]
	body, matched, err := loftcylinder.MirrorPairs(ctx, env, mesher, a, tools, nil)
	if !matched && err == nil {
		body, matched, err = loftcylinder.UnionTeeth(ctx, env, mesher, a, tools)
	}
	if !matched && err == nil {
		body, matched, err = revolveshell.SectorUnionMany(ctx, env, operands)
	}
	// A set of prisms with a free-form section joins through the union of its
	// sections ahead of the copied-prism route, so whether the copies' placements
	// round strictly inside the blank does not pick the route.
	var sectionErr error
	if !matched && err == nil {
		body, matched, sectionErr = axialsection.UnionManyPrisms(ctx, env, operands)
		if sectionErr != nil {
			body, matched = nil, false
		}
	}
	if !matched && err == nil {
		body, matched, err = prismpattern.UnionMany(ctx, env, mesher, operands)
	}
	if !matched && err == nil {
		body, matched, err = prismboolean.StraightLoftUnionMany(ctx, env, operands)
	}
	if !matched && err == nil {
		err = fmt.Errorf("%w: Union of three or more bodies needs a certified prism, "+
			"axial or copied-tooth set; join them two at a time", decaderr.ErrUnsupported)
	}
	if err != nil {
		if sectionErr != nil {
			return nil, 0, &SectionUnionError{Err: err}
		}
		return nil, 0, err
	}
	return body, 1, ctx.Err()
}
