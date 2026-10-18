package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/extent"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sweeprecord"
	"github.com/lestrrat-3d/decad/internal/thickenbuild"

	"github.com/lestrrat-3d/decad/internal/freeform"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/units"
)

// ThickenOption configures which side of a sheet receives material.
type ThickenOption interface{ thickenOption() }

type thickenSideOption struct{ side ThickenSide }

func (thickenSideOption) thickenOption() {}

// ThickenSide names the side of the source sheet that receives material.
type ThickenSide int

const (
	// ThickenPositive grows along the sheet's positive normal and is the default.
	ThickenPositive ThickenSide = iota
	// ThickenNegative grows opposite the sheet's positive normal.
	ThickenNegative
	// ThickenCentered grows half the thickness on each side.
	ThickenCentered
)

// WithThickenSide selects the side that receives the material.
func WithThickenSide(side ThickenSide) ThickenOption { return thickenSideOption{side: side} }

// Thicken builds a solid from the admitted sheet b and retires b.
// A nil context or a nil b is ErrDegenerate.
// It extrudes a recorded planar patch through the signed thickness interval,
// builds a certified annular wall around a profile-fed prism sheet, spins a
// certified meridian annulus through a profile-fed revolve sheet's own
// interval, or sweeps a ribbon's own assembled section through the ribbon's
// interval and spins a chain shell's own through the shell's. A one-span
// sweep sheet uses its prism or revolve reduction and restores sweep roles.
// A one-arc chain sweep uses its chain Revolve reduction; a cardinal composite
// chain sweep assembles one section and sweeps both spans.
// Other sheet families are staged under docs/surface-design.md §16.8.
func Thicken(ctx context.Context, b *Body, thickness units.Value, opts ...ThickenOption) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a thicken`, ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if b == nil || b.n.Doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	d := documentHandle(b.n.Doc)
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	side := ThickenPositive
	for _, raw := range opts {
		o, ok := raw.(thickenSideOption)
		if !ok {
			return nil, fmt.Errorf(`%w: the thicken option is not a decad thicken option (%T)`, ErrDegenerate, raw)
		}
		if o.side < ThickenPositive || o.side > ThickenCentered {
			return nil, fmt.Errorf(`%w: unknown thicken side %d`, ErrDegenerate, o.side)
		}
		side = o.side
	}
	tmm, tDelta, err := extent.MagnitudeInBounded(thickness, units.Length, units.Millimeter, "the thicken thickness")
	if err != nil {
		return nil, err
	}
	if tmm == 0 {
		return nil, fmt.Errorf(`%w: a zero-thickness sheet encloses no solid`, ErrDegenerate)
	}
	if b.Kind() != BodySheet {
		return nil, fmt.Errorf(`%w: Thicken requires a sheet body`, ErrUnsupported)
	}
	var result *Body
	switch payload := b.n.Payload.(type) {
	case patchPayload:
		result, err = thickenPatch(ctx, d, payload, side, tmm, tDelta)
	case prismPayload:
		result, err = thickenPrism(ctx, d, payload, side, tmm, tDelta)
	case revolvePayload:
		result, err = bodyResult(thickenbuild.Revolve(ctx, envOf(documentNode(d), d.nextProducerID()),
			payload, thickenbuild.Side(side), tmm, tDelta))
	case sweepPayload:
		result, err = thickenSweep(ctx, d, payload, side, tmm, tDelta)
	case chainPayload:
		result, err = bodyResult(thickenbuild.ChainExtrude(ctx, envOf(documentNode(d), d.nextProducerID()),
			payload, thickenbuild.Side(side), tmm, tDelta))
	case chainRevolvePayload:
		result, err = bodyResult(thickenbuild.ChainRevolve(ctx, envOf(documentNode(d), d.nextProducerID()),
			payload, thickenbuild.Side(side), tmm, tDelta))
	case chainSweepPayload:
		result, err = thickenChainSweep(ctx, d, payload, side, tmm, tDelta)
	default:
		return nil, fmt.Errorf(`%w: this sheet has no admitted Thicken generator`, ErrUnsupported)
	}
	if err != nil {
		return nil, err
	}
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	d.commit(result, b)
	return result, nil
}

// thickenSweep builds through a one-span sheet's analytic reduction, then
// publishes the result as a Sweep. The reduced builder proves the offset and
// solid; the finishing step restores the path's wall and cap roles.
func thickenSweep(ctx context.Context, d *Document, sp sweepPayload, side ThickenSide, tmm, tDelta float64) (*Body, error) {
	if !sp.SurfaceResult || len(sp.Spans) != 0 {
		return nil, fmt.Errorf(`%w: this sweep sheet has no admitted Thicken join proof`, ErrUnsupported)
	}
	if sp.Arc {
		body, err := bodyResult(thickenbuild.Revolve(ctx, envOf(documentNode(d), d.nextProducerID()),
			sp.Revolve, thickenbuild.Side(side), tmm, tDelta))
		if err != nil {
			return nil, err
		}
		built, ok := body.n.Payload.(revolvePayload)
		if !ok {
			return nil, fmt.Errorf(`%w: a thickened arc sweep has no revolve reduction`, ErrDegenerate)
		}
		sp.Revolve = built
		sp.Prism.Profile = built.Profile
		sp.Prism.Xform = built.Xform
		sp.SurfaceResult = false
		sweeprecord.FinishArc(bodyNode(body), sp)
		return body, nil
	}
	body, err := thickenPrism(ctx, d, sp.Prism, side, tmm, tDelta)
	if err != nil {
		return nil, err
	}
	sp.SurfaceResult = false
	sweeprecord.FinishStraight(bodyNode(body), sp)
	return body, nil
}

func thickenPatch(ctx context.Context, d *Document, pp patchPayload, side ThickenSide, tmm, tDelta float64) (*Body, error) {
	level := proofbound.MeasuredScalar(tmm, tDelta)
	var z0, z1 proofbound.BoundedScalar
	switch side {
	case ThickenPositive:
		z1 = level
	case ThickenNegative:
		z0 = proofbound.BoundedNeg(level)
	case ThickenCentered:
		half := proofbound.BoundedQuotient(level.Value, level.Bound, 2, 0)
		z0, z1 = proofbound.BoundedNeg(half), half
	}
	if proofbound.AdmitAbove(proofbound.BoundedSub(z1, z0), 0) != proofbound.SurvAdmit {
		return nil, fmt.Errorf(`%w: the thicken interval has no proven positive height`, ErrUnsupported)
	}
	prism := pp.Prism()
	prism.Z0, prism.Z0Delta = z0.Value, z0.Bound
	prism.Z1, prism.Z1Delta = z1.Value, z1.Bound
	ref := d.nextProducerID()
	return bodyResult(prismrecord.Build(ctx, envOf(documentNode(d), ref), prism, freeform.NewFreeformWork()))
}
