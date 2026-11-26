package decad

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/revolveshell"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
)

// unionRefinedAdvice closes every refusal UnionRefined makes because the
// direct build does not apply.
const unionRefinedAdvice = "RefineTo builds only a straight-bevel sector join of three or more bodies " +
	"directly at the bound; call Union, then Refine"

// refinementBound validates the bound RefineTo takes and returns it in
// millimetres.
func refinementBound(maxBound units.Value) (float64, error) {
	target, err := sectionrecord.MagnitudeIn(maxBound, units.Length, units.Millimeter, "the refinement bound")
	if err != nil {
		return 0, err
	}
	if target == 0 {
		return 0, fmt.Errorf(`%w: a zero bound admits no held mesh`, ErrDegenerate)
	}
	return target, nil
}

// unionRefined is the implementation of Union(bodies...).RefineTo: it joins a
// straight-bevel sector join directly at the bound target millimetres
// (agent-docs/body-refine-design.md §7.1). Its errors carry no operation tag.
func unionRefined(ctx context.Context, bodies []*Body, target float64) (*Body, error) {
	if len(bodies) == 2 {
		return nil, fmt.Errorf("%w: a pair of bodies is no sector join of a blank and two or more teeth: %s",
			ErrUnsupported, unionRefinedAdvice)
	}
	d, ctx, err := prepareUnionMany(ctx, bodies)
	if err != nil {
		return nil, err
	}
	node, matched, err := revolveshell.SectorUnionManyAt(ctx, envOf(documentNode(d), d.producerAt(ctx)),
		bodyNodes(bodies), min(target, revolveshell.SectorJoinDefaultCeiling))
	if err == nil && !matched {
		err = fmt.Errorf("%w: the bodies are no sector join of a revolved blank and copies of one point-Loft tooth",
			ErrUnsupported)
	}
	if err != nil {
		return nil, withUnionRefinedAdvice(err)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	body := bodyHandle(node)
	if err := commitUnionMany(ctx, d, bodies, body, 1); err != nil {
		return nil, err
	}
	return body, nil
}

// withUnionRefinedAdvice adds the fall-back advice to a refusal of the direct
// build and leaves every other error, a cancellation included, as it is.
func withUnionRefinedAdvice(err error) error {
	if !errors.Is(err, ErrUnsupported) {
		return err
	}
	return fmt.Errorf("%w: %s", err, unionRefinedAdvice)
}
