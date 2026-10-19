package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/extent"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sweepinput"
	"github.com/lestrrat-3d/decad/internal/sweeprecord"
	"github.com/lestrrat-3d/decad/internal/thickenaxis"
	"github.com/lestrrat-3d/decad/internal/thickenbuild"
	"github.com/lestrrat-3d/r3"
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

// thickenPrism builds a through-height wall around a recorded prism sheet.
// This arm first proves that its generated section coordinates denote the
// exact offset of the source section; prismrecord.Build can then keep its
// zero-displacement section certificate.
func thickenPrism(ctx context.Context, d *Document, pp prismPayload, side ThickenSide, tmm, tDelta float64) (*Body, error) {
	if !pp.SurfaceResult || pp.SectionDelta != 0 || len(pp.Profile.Holes) != 0 {
		return nil, fmt.Errorf(`%w: this prism sheet has no admitted Thicken section`, ErrUnsupported)
	}
	if proofbound.AdmitAbove(proofbound.BoundedSub(pp.Z1Scalar(), pp.Z0Scalar()), 0) != proofbound.SurvAdmit {
		return nil, fmt.Errorf(`%w: the prism sheet has no proven positive sweep height`, ErrUnsupported)
	}
	amount, err := thickenbuild.Amount(tmm, tDelta, thickenbuild.Side(side))
	if err != nil {
		return nil, err
	}
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, err
	}
	sec, err := thickenbuild.SectionOf(ctx, pp.Profile, thickenbuild.Side(side), amount, budget, nil)
	if err != nil {
		return nil, err
	}
	if side != ThickenCentered {
		negative := side == ThickenNegative
		return evalTubeContext(ctx, d, d.nextProducerID(), pp, sec.Generated(negative), sec.Sense(negative))
	}
	annulus, err := sec.Annulus(ctx)
	if err != nil {
		return nil, err
	}
	pp.Profile = annulus
	pp.SurfaceResult = false
	pp.Walks = nil
	return bodyResult(prismrecord.Build(ctx, envOf(documentNode(d), d.nextProducerID()), pp, freeform.NewFreeformWork()))
}

func thickenChainSweep(ctx context.Context, d *Document, sp chainSweepPayload,
	side ThickenSide, tmm, tDelta float64) (*Body, error) {
	if sp.Arc && len(sp.Spans) == 0 {
		return thickenArcChainSweep(ctx, d, sp, side, tmm, tDelta)
	}
	return thickenCompositeChainSweep(ctx, d, sp, side, tmm, tDelta)
}

// thickenArcChainSweep closes the source walk through the same exact-axis
// chain Revolve offset proof, then publishes its new closed section as a
// solid one-arc Sweep. The original open-chain reduction cannot describe it.
func thickenArcChainSweep(ctx context.Context, d *Document, sp chainSweepPayload,
	side ThickenSide, tmm, tDelta float64) (*Body, error) {
	if sp.Path == nil || len(sp.Path.Records) != 1 {
		return nil, fmt.Errorf(`%w: this chain Sweep has no one-arc Thicken path`, ErrUnsupported)
	}
	reverse := sp.Revolve.Phi0 < 0 && sp.Revolve.Phi1 == 0
	if !reverse && (sp.Revolve.Phi0 != 0 || sp.Revolve.Phi1 <= 0) {
		return nil, fmt.Errorf(`%w: this chain Sweep has no path-order cap proof`, ErrUnsupported)
	}
	body, err := bodyResult(thickenbuild.ChainRevolve(ctx, envOf(documentNode(d), d.nextProducerID()),
		sp.Revolve, thickenbuild.Side(side), tmm, tDelta))
	if err != nil {
		return nil, err
	}
	built, ok := body.n.Payload.(revolvePayload)
	if !ok {
		return nil, fmt.Errorf(`%w: thickened chain arc has no solid Revolve reduction`, ErrDegenerate)
	}
	sweeprecord.FinishArc(bodyNode(body), sweepPayload{
		Prism: prismrecord.Payload{
			Profile: built.Profile, Frame: built.Frame, Xform: built.Xform,
		},
		Revolve: built, Arc: true, ReverseArcCaps: reverse, Path: sp.Path,
	})
	return body, nil
}

// thickenCompositeChainSweep closes the authenticated radial line into a
// rectangular section and sends that one section through both path spans.
func thickenCompositeChainSweep(ctx context.Context, d *Document, sp chainSweepPayload,
	side ThickenSide, tmm, tDelta float64) (*Body, error) {
	if len(sp.Spans) != 2 || sp.Spans[0].Arc || !sp.Spans[1].Arc || sp.Path == nil ||
		len(sp.Spans[0].Chain.Chains) != 1 || len(sp.Spans[1].Revolve.Chains) != 1 ||
		sp.Spans[0].Chain.SectionDelta != 0 || sp.Spans[1].Revolve.SectionDelta != 0 ||
		sp.Spans[0].Chain.Xform != r3.Identity() || sp.Spans[1].Revolve.Xform != r3.Identity() {
		return nil, fmt.Errorf(`%w: this chain sweep has no admitted Thicken section`, ErrUnsupported)
	}
	straight := sp.Spans[0].Chain
	frame := straight.Frame
	plane := planeRecord{Origin: frame.Origin(), U: frame.U(), V: frame.V()}
	if err := sweepinput.CardinalChainSweepClass(straight.Chains[0], plane, sp.Path); err != nil {
		return nil, err
	}
	amount, err := thickenbuild.Amount(tmm, tDelta, thickenbuild.Side(side))
	if err != nil {
		return nil, err
	}
	budget := proofbound.NewWorkBudget(ctx)
	work := freeform.NewFreeformWork()
	right, left := thickenbuild.RibbonSteps(thickenbuild.Side(side))
	// The path's exact half-circle rotates about U=R along +V. RibbonProfile's
	// interval scan checks every moving piece against that axis, so the whole
	// offset family has positive hoop radius before either solid span builds.
	arc, ok := sp.Path.Segments[1], sp.Path.Segments[1].Arc
	if !ok {
		return nil, fmt.Errorf(`%w: the chain sweep has no cardinal arc source`, ErrUnsupported)
	}
	radial := thickenaxis.NewRadial(proofarith.FloatRat(arc.Through.X), big.NewRat(0, 1),
		big.NewRat(0, 1), big.NewRat(1, 1))
	section, err := thickenaxis.RibbonProfile(ctx, straight.Chains[0], right, left, amount, budget, work, &radial)
	if err != nil {
		return nil, err
	}
	return bodyResult(sweeprecord.BuildComposite(ctx, envOf(documentNode(d), d.nextProducerID()), section, plane,
		frame, sp.Path, work, false))
}
