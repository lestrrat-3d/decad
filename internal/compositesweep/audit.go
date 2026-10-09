package compositesweep

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
)

// AuditBox holds the bounded axis-aligned box of a closed span.
type AuditBox struct {
	Min, Max r3.Vec
	Bound    float64
}

// ExtentReader supplies a built span's analytic extent certificate.
type ExtentReader interface {
	AuditExtent(context.Context, r3.Vec, *freeform.FreeformWork) (float64, float64, float64, error)
}

// AuditSpan holds the geometry needed to certify one closed sweep span.
type AuditSpan struct {
	StartPlane       surfacegeom.Plane
	StartPlanar      bool
	EndPlanar        bool
	EndpointSupports bool
	Box              AuditBox
	Extent           ExtentReader
}

// Audit proves that adjacent spans occupy opposite sides of their shared
// section and that every non-neighbour pair has a strict bounded-box gap.
// The caller first proves matching rim topology at each shared section.
func Audit(ctx context.Context, spans []AuditSpan) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if len(spans) < 2 {
		return fmt.Errorf(`%w: a composite sweep audit requires at least two spans`, decaderr.ErrDegenerate)
	}
	pairs, ok := proofbound.WallChoose2(uint64(len(spans)))
	if !ok || pairs > proofbound.MaxFacetPairTestsPerCall {
		return fmt.Errorf(`%w: the composite sweep audit exceeds its fixed pair budget`, decaderr.ErrUnsupported)
	}
	operation := proofbound.NewWorkBudget(ctx)
	geometry := freeform.NewFreeformWork()
	for i := range spans {
		for j := i + 1; j < len(spans); j++ {
			if err := operation.Step(); err != nil {
				return err
			}
			var err error
			if j == i+1 {
				err = auditAdjacentSpans(ctx, spans[i], spans[j], geometry)
			} else if !BoxesStrictlySeparated(spans[i].Box, spans[j].Box) {
				err = fmt.Errorf(`%w: sweep spans %d and %d have no strict bounded-box separation`,
					decaderr.ErrUnsupported, i, j)
			}
			if err != nil {
				return err
			}
		}
	}
	return operation.Err()
}

func auditAdjacentSpans(
	ctx context.Context, before, after AuditSpan, geometry *freeform.FreeformWork,
) error {
	if !before.EndPlanar || !after.StartPlanar {
		return fmt.Errorf(`%w: an adjacent sweep section is not planar`, decaderr.ErrUnsupported)
	}
	// The next span starts from the certified transported frame, while a
	// preceding arc's independently evaluated end cap may hold a rounded image
	// of that frame. The rim-pairing precondition proves both caps denote one
	// section. Use the next span's exact start plane as the separator.
	startNormal := after.StartPlane.Frame.N()
	direction := startNormal.Scale(-1)
	startOrigin := after.StartPlane.Frame.Origin()
	if !proofbound.FiniteVec(direction) || !proofbound.FiniteVec(startOrigin) {
		return fmt.Errorf(`%w: an adjacent sweep section has no finite separating plane`, decaderr.ErrUnsupported)
	}
	planeValue := startOrigin.Dot(direction)
	planeBound := proofbound.ExactIsometryDotRound(r3.Identity(), startOrigin, direction, false, planeValue)
	planeLo, planeHi := proofbound.BoundedEnds(proofbound.MeasuredScalar(planeValue, planeBound))
	_, beforeHi, beforeBound, err := before.Extent.AuditExtent(ctx, direction, geometry)
	if err != nil {
		return err
	}
	afterLo, _, afterBound, err := after.Extent.AuditExtent(ctx, direction, geometry)
	if err != nil {
		return err
	}
	beforeLower, beforeUpper := proofbound.BoundedEnds(proofbound.MeasuredScalar(beforeHi, beforeBound))
	afterLower, afterUpper := proofbound.BoundedEnds(proofbound.MeasuredScalar(afterLo, afterBound))
	// An endpoint-support certificate lets the extent falsify support but not
	// establish it. Rim pairing proves the two caps denote one section.
	beforeSupported := beforeUpper <= planeLo ||
		(before.EndpointSupports && beforeLower <= planeHi)
	afterSupported := afterLower >= planeHi ||
		(after.EndpointSupports && afterUpper >= planeLo)
	if !beforeSupported || !afterSupported {
		return fmt.Errorf(
			`%w: adjacent sweep spans are not certified on opposite sides of their shared section `+
				`(before upper %g, plane [%g,%g], after lower %g)`,
			decaderr.ErrUnsupported, beforeUpper, planeLo, planeHi, afterLower,
		)
	}
	return nil
}

// BoxesStrictlySeparated requires a positive certified gap. Equality is
// rejected because non-neighbour sweep spans may not touch.
func BoxesStrictlySeparated(a, b AuditBox) bool {
	for _, axis := range [][4]float64{
		{a.Min.X, a.Max.X, b.Min.X, b.Max.X},
		{a.Min.Y, a.Max.Y, b.Min.Y, b.Max.Y},
		{a.Min.Z, a.Max.Z, b.Min.Z, b.Max.Z},
	} {
		aMin, _ := proofbound.BoundedEnds(proofbound.MeasuredScalar(axis[0], a.Bound))
		_, aMax := proofbound.BoundedEnds(proofbound.MeasuredScalar(axis[1], a.Bound))
		bMin, _ := proofbound.BoundedEnds(proofbound.MeasuredScalar(axis[2], b.Bound))
		_, bMax := proofbound.BoundedEnds(proofbound.MeasuredScalar(axis[3], b.Bound))
		if aMax < bMin || bMax < aMin {
			return true
		}
	}
	return false
}
