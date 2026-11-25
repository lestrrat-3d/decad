package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/revolveshell"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
)

// UnionRefined joins first and others as [Union] does and returns the result
// refined as [Refine] would refine it to maxBound, in one step
// (agent-docs/body-refine-design.md §7).
//
// For a set of three or more bodies that is a straight-bevel sector join (a
// revolved blank and copies of one point-Loft tooth), the join is built once,
// directly at the bound min(maxBound, 0.1 mm). The result has the vertices,
// triangles, topology, bounds, Volume and Area that Union followed by Refine
// returns, bit for bit. Union builds and audits the join at 0.1 mm, and Refine
// then discards that join and builds it again from the same sources at
// maxBound, so the single build saves the first. The result differs from the
// pair's in two ways only: its faces carry Union's producer and face
// identities, where Refine gives fresh ones, and the document's operation
// history holds one Union entry and no Refine entry. A join that cannot meet
// maxBound is ErrUnsupported and changes nothing: every operand stays live, so
// the caller may fall back to Union, which keeps the 0.1 mm join as the pair
// does when its Refine refuses.
//
// Every other union is Union followed by Refine. That includes a pair of
// bodies, whose join is built at 0.1 mm first, and a sector join that a
// pair-at-a-time fold built. When that Refine refuses, the union stays
// committed: UnionRefined returns it beside the Refine error, the one case
// where the body and the error are both non-nil, so the caller keeps the
// unrefined union as it would after the pair.
//
// maxBound is a length: wrong kind is ErrUnitKind, non-finite ErrNotFinite,
// negative ErrNegativeMagnitude and zero ErrDegenerate. Each is returned before
// anything is built, where the pair would report it after the Union. A nil
// context, or fewer than two bodies, is ErrDegenerate. A union Union itself
// refuses, UnionRefined refuses with the same error, except that a sector join
// whose 0.1 mm build is refused but whose build at maxBound succeeds is
// returned. Operands retired by a successful call, and every other error and
// cancellation rule, are Union's and Refine's.
func UnionRefined(ctx context.Context, maxBound units.Value, first *Body, others ...*Body) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf("%w: a nil context cannot control a boolean", ErrDegenerate)
	}
	target, err := sectionrecord.MagnitudeIn(maxBound, units.Length, units.Millimeter, "the refinement bound")
	if err != nil {
		return nil, operationError(OperationRefine, err)
	}
	if target == 0 {
		return nil, operationError(OperationRefine, fmt.Errorf(`%w: a zero bound admits no held mesh`, ErrDegenerate))
	}
	ceiling := min(target, revolveshell.SectorJoinDefaultCeiling)
	joined, err := unionAt(ctx, ceiling, first, others)
	if err != nil {
		return nil, err
	}
	if sectorJoinMeets(joined, ceiling) {
		return joined, nil
	}
	refined, err := Refine(ctx, joined, maxBound)
	if err != nil {
		return joined, err
	}
	return refined, nil
}

// sectorJoinMeets reports whether body is a straight-bevel sector join already
// built at a boundary bound of at most ceiling, so Refine at that bound would
// rebuild the same mesh.
func sectorJoinMeets(body *Body, ceiling float64) bool {
	fp, ok := body.n.Payload.(facetedPayload)
	return ok && fp.RevolveLoftJoin != nil && revolveshell.RecordCeiling(fp.RevolveLoftJoin) <= ceiling
}
