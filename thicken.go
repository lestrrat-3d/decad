package decad

import (
	"context"
	"fmt"
	"math/big"

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
	tmm, tDelta, err := magnitudeInBounded(thickness, units.Length, units.Millimeter, "the thicken thickness")
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
	level := measuredScalar(tmm, tDelta)
	var z0, z1 boundedScalar
	switch side {
	case ThickenPositive:
		z1 = level
	case ThickenNegative:
		z0 = boundedNeg(level)
	case ThickenCentered:
		half := boundedQuotient(level.value, level.bound, 2, 0)
		z0, z1 = boundedNeg(half), half
	}
	if admitAbove(boundedSub(z1, z0), 0) != survAdmit {
		return nil, fmt.Errorf(`%w: the thicken interval has no proven positive height`, ErrUnsupported)
	}
	prism := pp.prism()
	prism.z0, prism.z0Delta = z0.value, z0.bound
	prism.z1, prism.z1Delta = z1.value, z1.bound
	ref := d.nextProducerID()
	return evalPrismContext(ctx, d, ref, prism, newFreeformWork())
}

// thickenRadial is the revolve arm's radial-axis gate (docs/surface-design.md
// §16.5): the plane-local axis every swept offset must keep a strictly
// positive radius from, held as exact rationals so each comparison below is
// decided rather than measured. A surface of revolution's own normal lies in
// its meridian plane, so offsetting the meridian IS offsetting the surface,
// and the swept offset folds exactly where the offset meridian reaches the
// axis.
type thickenRadial struct{ aU, aV, dU, dV *big.Rat }

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
	along := (ax.dU == 0 && (ax.dV == 1 || ax.dV == -1)) || (ax.dV == 0 && (ax.dU == 1 || ax.dU == -1))
	if !along {
		return thickenRadial{}, fmt.Errorf(`%w: the revolve axis is not parallel to a recorded plane axis`, ErrUnsupported)
	}
	if ax.aUBound != 0 || ax.aVBound != 0 || ax.dUBound != 0 || ax.dVBound != 0 {
		return thickenRadial{}, fmt.Errorf(`%w: the revolve axis is not stated exactly in the sketch plane`, ErrUnsupported)
	}
	aU, aV, dU, dV := floatRat(ax.aU), floatRat(ax.aV), floatRat(ax.dU), floatRat(ax.dV)
	if aU == nil || aV == nil || dU == nil || dV == nil {
		return thickenRadial{}, fmt.Errorf(`%w: the revolve axis has a non-finite plane-local coordinate`, ErrUnsupported)
	}
	return thickenRadial{aU: aU, aV: aV, dU: dU, dV: dV}, nil
}

// rho is axisFrame.toAxis's own radial coordinate, taken over the rationals.
func (r thickenRadial) rho(u, v *big.Rat) *big.Rat {
	du := new(big.Rat).Sub(u, r.aU)
	dv := new(big.Rat).Sub(v, r.aV)
	return new(big.Rat).Sub(new(big.Rat).Mul(dv, r.dU), new(big.Rat).Mul(du, r.dV))
}

// leastOverBox is the least radius any point of one exact box reaches. ρ is
// affine in (u, v), so its minimum over a box sits at a corner.
func (r thickenRadial) leastOverBox(b thickenExactBox) *big.Rat {
	least := r.rho(b.minU, b.minV)
	for _, corner := range [][2]*big.Rat{{b.minU, b.maxV}, {b.maxU, b.minV}, {b.maxU, b.maxV}} {
		if got := r.rho(corner[0], corner[1]); got.Cmp(least) < 0 {
			least = got
		}
	}
	return least
}

// require refuses a swept offset whose least radius is not proven positive.
func (r thickenRadial) require(least *big.Rat) error {
	if least.Sign() <= 0 {
		return fmt.Errorf(`%w: the swept offset reaches the revolve axis (least radius %s mm)`,
			ErrUnsupported, least.FloatString(9))
	}
	return nil
}

// thickenRevolve builds a solid of revolution from an admitted revolve sheet
// by offsetting its recorded meridian and spinning the annulus through the
// sheet's own interval (docs/surface-design.md §16.5).
func thickenRevolve(ctx context.Context, d *Document, rp revolvePayload, side ThickenSide, tmm, tDelta float64) (*Body, error) {
	if !rp.surfaceResult || len(rp.profile.Holes) != 0 {
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
	budget := newWorkBudget(ctx)
	if err := budget.err(); err != nil {
		return nil, err
	}
	sec, err := thickenSectionOf(ctx, rp.profile, side, amount, budget, &radial)
	if err != nil {
		return nil, err
	}
	annulus, err := thickenAnnulus(ctx, sec)
	if err != nil {
		return nil, err
	}
	// The annulus is a section no axis resolution has seen, so its own snap
	// allowances, radial admission charge and axial envelope are proven here
	// rather than inherited from the sheet's: every one of them is an integral
	// over the region, and the region changed.
	work := newFreeformWork()
	ax, axisSide, err := resolveAxisSide(ctx, annulus, axisLine2{
		aU: rp.ax.aU, aV: rp.ax.aV, dU: rp.ax.dU, dV: rp.ax.dV,
	}, work)
	if err != nil {
		return nil, fmt.Errorf(`%w: the thicken offset's revolve axis side is unresolved: %v`, ErrUnsupported, err)
	}
	if axisSide < 0 {
		return nil, fmt.Errorf(`%w: the thicken offset crossed to the far side of the revolve axis`, ErrUnsupported)
	}
	rp.profile = annulus
	rp.ax = ax
	rp.surfaceResult = false
	return evalRevolveContextWork(ctx, d, d.nextProducerID(), rp, work)
}

// thickenChainExtrude builds a solid from a ribbon: the open walk's own
// thickened section swept through the ribbon's unchanged interval
// (docs/surface-design.md §16.6).
func thickenChainExtrude(ctx context.Context, d *Document, cp chainPayload, side ThickenSide, tmm, tDelta float64) (*Body, error) {
	if len(cp.chains) != 1 || cp.sectionDelta != 0 {
		return nil, fmt.Errorf(`%w: this ribbon has no admitted Thicken walk`, ErrUnsupported)
	}
	if admitAbove(boundedSub(cp.z1Scalar(), cp.z0Scalar()), 0) != survAdmit {
		return nil, fmt.Errorf(`%w: the ribbon has no proven positive sweep height`, ErrUnsupported)
	}
	amount, err := thickenAmount(tmm, tDelta, side)
	if err != nil {
		return nil, err
	}
	budget := newWorkBudget(ctx)
	if err := budget.err(); err != nil {
		return nil, err
	}
	// ONE free-form work counter for the record: the walk resolution below and
	// the build that consumes its section both spend from it.
	work := newFreeformWork()
	section, err := thickenRibbon(ctx, cp.chains[0], side, amount, budget, work, nil)
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
	budget := newWorkBudget(ctx)
	if err := budget.err(); err != nil {
		return nil, err
	}
	// ONE free-form work counter for the record: the walk resolution, the axis
	// re-resolution and the build that consumes the section all spend from it.
	work := newFreeformWork()
	section, err := thickenRibbon(ctx, cp.chains[0], side, amount, budget, work, &radial)
	if err != nil {
		return nil, err
	}
	// The assembled section is a region no axis resolution has seen, so its own
	// snap allowances, radial admission charge and axial envelope are proven
	// here rather than inherited from the shell's: every one of them is an
	// integral over the region, and the shell had no region at all.
	ax, axisSide, err := resolveAxisSide(ctx, section, axisLine2{
		aU: cp.ax.aU, aV: cp.ax.aV, dU: cp.ax.dU, dV: cp.ax.dV,
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
	return evalRevolveContextWork(ctx, d, d.nextProducerID(), rp, work)
}
