package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/extent"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/thickenaxis"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
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

// Thicken builds a solid from an admitted sheet and retires that sheet.
// It extrudes a recorded planar patch through the signed thickness interval,
// builds a certified annular wall around a profile-fed prism sheet, spins a
// certified meridian annulus through a profile-fed revolve sheet's own
// interval, or sweeps a ribbon's own assembled section through the ribbon's
// interval and spins a chain shell's own through the shell's. Other sheet
// families are staged under docs/surface-design.md §16.8.
func (b *Body) Thicken(ctx context.Context, thickness units.Value, opts ...ThickenOption) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a thicken`, ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if b == nil || b.doc == nil {
		return nil, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	d := b.doc
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
	switch payload := b.payload.(type) {
	case patchPayload:
		result, err = thickenPatch(ctx, d, payload, side, tmm, tDelta)
	case prismPayload:
		result, err = thickenPrism(ctx, d, payload, side, tmm, tDelta)
	case revolvePayload:
		result, err = thickenRevolve(ctx, d, payload, side, tmm, tDelta)
	case chainPayload:
		result, err = thickenChainExtrude(ctx, d, payload, side, tmm, tDelta)
	case chainRevolvePayload:
		result, err = thickenChainRevolve(ctx, d, payload, side, tmm, tDelta)
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
	prism := pp.prism()
	prism.z0, prism.z0Delta = z0.Value, z0.Bound
	prism.z1, prism.z1Delta = z1.Value, z1.Bound
	ref := d.nextProducerID()
	return evalPrismContext(ctx, d, ref, prism, freeform.NewFreeformWork())
}

// thickenRadial is the revolve arm's radial-axis gate (docs/surface-design.md
// §16.5): the plane-local axis every swept offset must keep a strictly
// positive radius from, held as exact rationals so each comparison below is
// decided rather than measured. A surface of revolution's own normal lies in
// its meridian plane, so offsetting the meridian IS offsetting the surface,
// and the swept offset folds exactly where the offset meridian reaches the
// axis.
type thickenRadial = thickenaxis.Radial

// thickenRadialOf reads one resolved axis frame into the gate, refusing any
// axis this arm cannot decide exactly.
func thickenRadialOf(ax axisFrame) (thickenRadial, error) {
	// An axis along one recorded plane axis is what keeps axisFrame.toAxis
	// exact: every product it forms is by 0 or ±1, so the built wall's own
	// radial coordinate is the coordinate this gate decided on. Any other
	// direction rounds that re-expression, and a rounded radius admits no
	// exact positive-radius claim. The direction is read first because it is
	// what a caller can act on: a tilted axis also arrives with a rounded
	// direction bound, and naming the tilt says which input to change.
	along := (ax.DU == 0 && (ax.DV == 1 || ax.DV == -1)) || (ax.DV == 0 && (ax.DU == 1 || ax.DU == -1))
	if !along {
		return thickenRadial{}, fmt.Errorf(`%w: the revolve axis is not parallel to a recorded plane axis`, ErrUnsupported)
	}
	if ax.AUBound != 0 || ax.AVBound != 0 || ax.DUBound != 0 || ax.DVBound != 0 {
		return thickenRadial{}, fmt.Errorf(`%w: the revolve axis is not stated exactly in the sketch plane`, ErrUnsupported)
	}
	aU, aV, dU, dV := proofarith.FloatRat(ax.AU), proofarith.FloatRat(ax.AV), proofarith.FloatRat(ax.DU), proofarith.FloatRat(ax.DV)
	if aU == nil || aV == nil || dU == nil || dV == nil {
		return thickenRadial{}, fmt.Errorf(`%w: the revolve axis has a non-finite plane-local coordinate`, ErrUnsupported)
	}
	return thickenaxis.NewRadial(aU, aV, dU, dV), nil
}

// thickenRevolve builds a solid of revolution from an admitted revolve sheet
// by offsetting its recorded meridian and spinning the annulus through the
// sheet's own interval (docs/surface-design.md §16.5).
func thickenRevolve(ctx context.Context, d *Document, rp revolvePayload, side ThickenSide, tmm, tDelta float64) (*Body, error) {
	if !rp.surfaceResult || rp.sectionDelta != 0 || len(rp.profile.Holes) != 0 {
		return nil, fmt.Errorf(`%w: this revolve sheet has no admitted Thicken section`, ErrUnsupported)
	}
	radial, err := thickenRadialOf(rp.ax)
	if err != nil {
		return nil, err
	}
	amount, err := thickenAmount(tmm, tDelta, side)
	if err != nil {
		return nil, err
	}
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, err
	}
	sec, err := thickenSectionOf(ctx, rp.profile, side, amount, budget, &radial)
	if err != nil {
		return nil, err
	}
	annulus, err := sec.Annulus(ctx)
	if err != nil {
		return nil, err
	}
	// The annulus is a section no axis resolution has seen, so its own snap
	// allowances, radial admission charge and axial envelope are proven here
	// rather than inherited from the sheet's: every one of them is an integral
	// over the region, and the region changed.
	work := freeform.NewFreeformWork()
	ax, axisSide, err := resolveAxisSide(ctx, annulus, revolveaxis.Line2{
		AU: rp.ax.AU, AV: rp.ax.AV, DU: rp.ax.DU, DV: rp.ax.DV,
	}, work)
	if err != nil {
		return nil, fmt.Errorf(`%w: the thicken offset's revolve axis side is unresolved: %v`, ErrUnsupported, err)
	}
	if axisSide < 0 {
		return nil, fmt.Errorf(`%w: the thicken offset crossed to the far side of the revolve axis`, ErrUnsupported)
	}
	rp.profile = annulus
	rp.ax = ax
	rp.radialProof = false
	rp.surfaceResult = false
	rp.blendSegs, rp.blendKind = nil, ""
	return evalRevolveContextWork(ctx, d, d.nextProducerID(), rp, work)
}

// thickenRibbonSteps maps the requested side to the right and left copies of
// the recorded walk. A centered request offsets both copies.
func thickenRibbonSteps(side ThickenSide) (int, int) {
	switch side {
	case ThickenNegative:
		return 0, 1
	case ThickenCentered:
		return 1, 1
	default:
		return 1, 0
	}
}

// thickenChainExtrude builds a solid from a ribbon: the open walk's own
// thickened section swept through the ribbon's unchanged interval
// (docs/surface-design.md §16.6).
func thickenChainExtrude(ctx context.Context, d *Document, cp chainPayload, side ThickenSide, tmm, tDelta float64) (*Body, error) {
	if len(cp.chains) != 1 || cp.sectionDelta != 0 {
		return nil, fmt.Errorf(`%w: this ribbon has no admitted Thicken walk`, ErrUnsupported)
	}
	if proofbound.AdmitAbove(proofbound.BoundedSub(cp.z1Scalar(), cp.z0Scalar()), 0) != proofbound.SurvAdmit {
		return nil, fmt.Errorf(`%w: the ribbon has no proven positive sweep height`, ErrUnsupported)
	}
	amount, err := thickenAmount(tmm, tDelta, side)
	if err != nil {
		return nil, err
	}
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, err
	}
	// ONE free-form work counter for the record: the walk resolution below and
	// the build that consumes its section both spend from it.
	work := freeform.NewFreeformWork()
	rightSteps, leftSteps := thickenRibbonSteps(side)
	section, err := thickenaxis.RibbonProfile(ctx, cp.chains[0], rightSteps, leftSteps, amount, budget, work, nil)
	if err != nil {
		return nil, err
	}
	return evalPrismContext(ctx, d, d.nextProducerID(), prismPayload{
		profile: section,
		frame:   cp.frame,
		z0:      cp.z0, z1: cp.z1,
		z0Delta: cp.z0Delta, z1Delta: cp.z1Delta,
		xform: cp.xform,
	}, work)
}

// thickenChainRevolve builds a solid of revolution from an uncapped chain
// shell: §16.6's assembled section under §16.5's sweep and radial gate
// (docs/surface-design.md §16.7).
func thickenChainRevolve(ctx context.Context, d *Document, cp chainRevolvePayload, side ThickenSide, tmm, tDelta float64) (*Body, error) {
	// The prism ribbon's own admission, re-read over the meridian walk set
	// (thickenChainExtrude above): §16.6 offsets ONE walk, and the assembled
	// section it hands the solid build has to be exact — requireExactRevolveSection
	// refuses a displaced one at that build anyway, and refusing here names the
	// ribbon rather than the revolve it was about to become.
	if len(cp.chains) != 1 || cp.sectionDelta != 0 {
		return nil, fmt.Errorf(`%w: this ribbon has no admitted Thicken walk`, ErrUnsupported)
	}
	radial, err := thickenRadialOf(cp.ax)
	if err != nil {
		return nil, err
	}
	amount, err := thickenAmount(tmm, tDelta, side)
	if err != nil {
		return nil, err
	}
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, err
	}
	// ONE free-form work counter for the record: the walk resolution, the axis
	// re-resolution and the build that consumes the section all spend from it.
	work := freeform.NewFreeformWork()
	rightSteps, leftSteps := thickenRibbonSteps(side)
	section, err := thickenaxis.RibbonProfile(ctx, cp.chains[0], rightSteps, leftSteps, amount, budget, work, &radial)
	if err != nil {
		return nil, err
	}
	// The assembled section is a region no axis resolution has seen, so its own
	// snap allowances, radial admission charge and axial envelope are proven
	// here rather than inherited from the shell's: every one of them is an
	// integral over the region, and the shell had no region at all.
	ax, axisSide, err := resolveAxisSide(ctx, section, revolveaxis.Line2{
		AU: cp.ax.AU, AV: cp.ax.AV, DU: cp.ax.DU, DV: cp.ax.DV,
	}, work)
	if err != nil {
		return nil, fmt.Errorf(`%w: the thicken offset's revolve axis side is unresolved: %v`, ErrUnsupported, err)
	}
	if axisSide < 0 {
		return nil, fmt.Errorf(`%w: the thicken offset crossed to the far side of the revolve axis`, ErrUnsupported)
	}
	rp := cp.revolve()
	rp.profile = section
	rp.ax = ax
	rp.radialProof = false
	return evalRevolveContextWork(ctx, d, d.nextProducerID(), rp, work)
}
