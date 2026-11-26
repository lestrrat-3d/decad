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
const unionRefinedAdvice = "UnionRefined builds only a straight-bevel sector join of three or more bodies " +
	"directly at the bound; call Union, then Refine"

// UnionRefined joins bodies, a straight-bevel sector join, directly at the
// refinement bound maxBound, in one build (agent-docs/body-refine-design.md
// §7.1). It returns the join [Union] followed by [Refine] returns.
//
// bodies MUST hold a revolved blank and two or more copies of one point-Loft
// tooth, as [Union] takes a bevel gear join, and none may be nil. The join is
// built once at the bound min(maxBound, 0.1 mm). Union builds and audits the
// join at 0.1 mm, and Refine then discards that join and builds it again from
// the same sources at maxBound, so the single build saves the first. The
// result has the vertices, triangles, topology, bounds, Volume and Area that
// the pair returns, bit for bit. It differs from the pair's result in these
// ways. Its faces keep the producer and face identities the Union gives,
// where Refine makes fresh ones. The document's operation history holds one
// Union entry and no Refine entry, so the producer identities of later
// operations are one lower than after the pair. LocatePoint, a two-body
// Section and the regroup of a later Cut or Union read the result as a Union
// of its operands through that history, where they read the pair's refined
// body as a leaf. A join that cannot meet maxBound is refused and commits
// nothing, where the pair keeps the 0.1 mm join of its Union.
//
// UnionRefined returns the joined body, or an error with nothing committed:
// the document is unchanged and every operand stays live. A canceled ctx
// returns ctx.Err() unchanged. A set that is not a sector join the direct
// build takes is ErrUnsupported, refused before anything is built, and its
// message says to call Union, then Refine. That covers a pair of bodies, a
// union of any other kind, and a join that Union folds two bodies at a time.
// A join that cannot meet maxBound is ErrUnsupported too, with the same
// advice. Union's own gates on the operands apply.
//
// maxBound is a length: wrong kind is ErrUnitKind, non-finite ErrNotFinite,
// negative ErrNegativeMagnitude and zero ErrDegenerate, each returned before
// anything is built. A nil context, or fewer than two bodies, is ErrDegenerate.
func UnionRefined(ctx context.Context, bodies []*Body, maxBound units.Value) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf("%w: a nil context cannot control a boolean", ErrDegenerate)
	}
	if len(bodies) < 2 {
		return nil, fmt.Errorf("%w: UnionRefined needs at least two bodies", ErrDegenerate)
	}
	target, err := sectionrecord.MagnitudeIn(maxBound, units.Length, units.Millimeter, "the refinement bound")
	if err != nil {
		return nil, operationError(OperationRefine, err)
	}
	if target == 0 {
		return nil, operationError(OperationRefine, fmt.Errorf(`%w: a zero bound admits no held mesh`, ErrDegenerate))
	}
	if len(bodies) == 2 {
		return nil, operationError(OperationUnion,
			fmt.Errorf("%w: a pair of bodies is no sector join of a blank and two or more teeth: %s",
				ErrUnsupported, unionRefinedAdvice))
	}
	d, ctx, err := prepareUnionMany(ctx, bodies)
	if err != nil {
		return nil, operationError(OperationUnion, err)
	}
	node, matched, err := revolveshell.SectorUnionManyAt(ctx, envOf(documentNode(d), d.producerAt(ctx)),
		bodyNodes(bodies), min(target, revolveshell.SectorJoinDefaultCeiling))
	if err == nil && !matched {
		err = fmt.Errorf("%w: the bodies are no sector join of a revolved blank and copies of one point-Loft tooth",
			ErrUnsupported)
	}
	if err != nil {
		return nil, operationError(OperationUnion, withUnionRefinedAdvice(err))
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	body := bodyHandle(node)
	if err := commitUnionMany(ctx, d, bodies, body, 1); err != nil {
		return nil, operationError(OperationUnion, err)
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
