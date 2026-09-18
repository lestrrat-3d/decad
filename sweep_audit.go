package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/r3"
)

// sweepAuditSpan is one closed analytic span before composite Sweep removes
// its internal caps. The cap pointers follow path direction.
type sweepAuditSpan struct {
	body     *Body
	startCap *Face
	endCap   *Face
}

// auditCompositeSweep conservatively proves that the closed span bodies can
// be united without adding contact beyond their intended shared sections.
// The caller must first prove that every adjacent cap pair has matching rim
// topology. Adjacent spans are then certified on opposite sides of that cap's
// plane, while non-neighbours require strictly separated bounded boxes.
func auditCompositeSweep(ctx context.Context, spans []sweepAuditSpan) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if len(spans) < 2 {
		return fmt.Errorf(`%w: a composite sweep audit requires at least two spans`, ErrDegenerate)
	}
	pairs, ok := wallChoose2(uint64(len(spans)))
	if !ok || pairs > maxFacetPairTestsPerCall {
		return fmt.Errorf(`%w: the composite sweep audit exceeds its fixed pair budget`, ErrUnsupported)
	}
	for i, span := range spans {
		if span.body == nil || span.startCap == nil || span.endCap == nil {
			return fmt.Errorf(`%w: composite sweep span %d has incomplete audit geometry`, ErrUnsupported, i)
		}
		if span.startCap.body != span.body || span.endCap.body != span.body {
			return fmt.Errorf(`%w: composite sweep span %d has a cap from another body`, ErrUnsupported, i)
		}
	}

	operation := newWorkBudget(ctx)
	geometry := newFreeformWork()
	for i := range spans {
		for j := i + 1; j < len(spans); j++ {
			if err := operation.step(); err != nil {
				return err
			}
			var err error
			if j == i+1 {
				err = auditAdjacentSweepSpans(ctx, spans[i], spans[j], geometry)
			} else if !sweepAuditBoxesStrictlySeparated(spans[i].body.bounds, spans[j].body.bounds) {
				err = fmt.Errorf(`%w: sweep spans %d and %d have no strict bounded-box separation`, ErrUnsupported, i, j)
			}
			if err != nil {
				return err
			}
		}
	}
	return operation.err()
}

func auditAdjacentSweepSpans(
	ctx context.Context,
	before, after sweepAuditSpan,
	geometry *freeformWork,
) error {
	if _, ok := before.endCap.surface.(Plane); !ok {
		return fmt.Errorf(`%w: an adjacent sweep section is not planar`, ErrUnsupported)
	}
	startPlane, ok := after.startCap.surface.(Plane)
	if !ok {
		return fmt.Errorf(`%w: an adjacent sweep section is not planar`, ErrUnsupported)
	}
	// The next span starts from the certified transported frame, while a
	// preceding arc's independently evaluated end cap may hold a rounded image
	// of that frame. The rim-pairing precondition proves both caps denote one
	// section. Use the next span's exact start plane as the separating plane,
	// rather than requiring the two held cap frames to be bit-identical.
	startNormal := startPlane.Frame.N()
	direction := startNormal.Scale(-1)
	startOrigin := startPlane.Frame.Origin()
	if !finiteVec(direction) || !finiteVec(startOrigin) {
		return fmt.Errorf(`%w: an adjacent sweep section has no finite separating plane`, ErrUnsupported)
	}
	planeValue := startOrigin.Dot(direction)
	planeBound := exactIsometryDotRound(r3.Identity(), startOrigin, direction, false, planeValue)
	planeLo, planeHi := boundedEnds(measuredScalar(planeValue, planeBound))
	_, beforeHi, beforeBound, err := sweepAuditExtent(ctx, before.body, direction, geometry)
	if err != nil {
		return err
	}
	afterLo, _, afterBound, err := sweepAuditExtent(ctx, after.body, direction, geometry)
	if err != nil {
		return err
	}
	_, beforeUpper := boundedEnds(measuredScalar(beforeHi, beforeBound))
	afterLower, _ := boundedEnds(measuredScalar(afterLo, afterBound))
	beforeSupported := beforeUpper <= planeLo ||
		(beforeHi == planeValue && sweepAuditEndpointSupports(before.body))
	afterSupported := afterLower >= planeHi ||
		(afterLo == planeValue && sweepAuditEndpointSupports(after.body))
	if !beforeSupported || !afterSupported {
		return fmt.Errorf(
			`%w: adjacent sweep spans are not certified on opposite sides of their shared section `+
				`(before upper %g, plane [%g,%g], after lower %g)`,
			ErrUnsupported, beforeUpper, planeLo, planeHi, afterLower,
		)
	}
	return nil
}

// sweepAuditEndpointSupports reports whether a span's local construction
// proves each endpoint cap is a supporting plane. A prism has that property
// for both ends. For an exact-turn arc no wider than a half turn, Revolve's
// half-plane gate then keeps the whole swept section between its two endpoint
// tangent planes.
func sweepAuditEndpointSupports(body *Body) bool {
	switch payload := body.payload.(type) {
	case prismPayload:
		return true
	case revolvePayload:
		if payload.full {
			return false
		}
		from, to := payload.den.phi0, payload.den.phi1
		if !from.valid() || !to.valid() || from.span != nil || to.span != nil ||
			from.rad.Sign() != 0 || to.rad.Sign() != 0 {
			return false
		}
		width := new(big.Rat).Sub(to.turn, from.turn)
		return width.Sign() > 0 && width.Cmp(big.NewRat(1, 2)) <= 0
	default:
		return false
	}
}

func sweepAuditExtent(
	ctx context.Context,
	body *Body,
	direction r3.Vec,
	work *freeformWork,
) (float64, float64, float64, error) {
	switch payload := body.payload.(type) {
	case prismPayload:
		return payload.extentBoundedAlong(ctx, direction, work, payload.walks)
	case revolvePayload:
		return payload.extentBoundedAlong(ctx, direction, work)
	default:
		return 0, 0, 0, fmt.Errorf(`%w: a composite sweep span has no analytic extent certificate`, ErrUnsupported)
	}
}

// sweepAuditBoxesStrictlySeparated requires a positive certified gap. Equality
// is intentionally rejected because non-neighbour sweep spans may not touch.
func sweepAuditBoxesStrictlySeparated(a, b Box) bool {
	aBound, bBound := a.Bound.Base(), b.Bound.Base()
	for _, axis := range [][4]float64{
		{a.Min.X, a.Max.X, b.Min.X, b.Max.X},
		{a.Min.Y, a.Max.Y, b.Min.Y, b.Max.Y},
		{a.Min.Z, a.Max.Z, b.Min.Z, b.Max.Z},
	} {
		aMin, _ := boundedEnds(measuredScalar(axis[0], aBound))
		_, aMax := boundedEnds(measuredScalar(axis[1], aBound))
		bMin, _ := boundedEnds(measuredScalar(axis[2], bBound))
		_, bMax := boundedEnds(measuredScalar(axis[3], bBound))
		if aMax < bMin || bMax < aMin {
			return true
		}
	}
	return false
}
