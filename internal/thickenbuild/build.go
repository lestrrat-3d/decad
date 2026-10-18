package thickenbuild

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/chainrecord"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/thickenaxis"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// Side is the validated public ThickenSide expressed for the builder.
type Side uint8

const (
	Positive Side = iota
	Negative
	Centered
)

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
func thickenRadialOf(ax revolveaxis.Frame) (thickenRadial, error) {
	// An axis along one recorded plane axis is what keeps the axis re-expression
	// exact: every product it forms is by 0 or ±1, so the built wall's own
	// radial coordinate is the coordinate this gate decided on. Any other
	// direction rounds that re-expression, and a rounded radius admits no
	// exact positive-radius claim. The direction is read first because it is
	// what a caller can act on: a tilted axis also arrives with a rounded
	// direction bound, and naming the tilt says which input to change.
	along := (ax.DU == 0 && (ax.DV == 1 || ax.DV == -1)) || (ax.DV == 0 && (ax.DU == 1 || ax.DU == -1))
	if !along {
		return thickenRadial{}, fmt.Errorf(`%w: the revolve axis is not parallel to a recorded plane axis`, decaderr.ErrUnsupported)
	}
	if ax.AUBound != 0 || ax.AVBound != 0 || ax.DUBound != 0 || ax.DVBound != 0 {
		return thickenRadial{}, fmt.Errorf(`%w: the revolve axis is not stated exactly in the sketch plane`, decaderr.ErrUnsupported)
	}
	aU, aV, dU, dV := proofarith.FloatRat(ax.AU), proofarith.FloatRat(ax.AV), proofarith.FloatRat(ax.DU), proofarith.FloatRat(ax.DV)
	if aU == nil || aV == nil || dU == nil || dV == nil {
		return thickenRadial{}, fmt.Errorf(`%w: the revolve axis has a non-finite plane-local coordinate`, decaderr.ErrUnsupported)
	}
	return thickenaxis.NewRadial(aU, aV, dU, dV), nil
}

// Revolve builds a solid of revolution from an admitted revolve sheet
// by offsetting its recorded meridian and spinning the annulus through the
// sheet's own interval (docs/surface-design.md §16.5).
func Revolve(ctx context.Context, env topo.Env, rp revolveplan.Payload, side Side, tmm, tDelta float64) (*topo.Body, error) {
	if !rp.SurfaceResult || rp.SectionDelta != 0 || len(rp.Profile.Holes) != 0 {
		return nil, fmt.Errorf(`%w: this revolve sheet has no admitted Thicken section`, decaderr.ErrUnsupported)
	}
	radial, err := thickenRadialOf(rp.Ax)
	if err != nil {
		return nil, err
	}
	amount, err := Amount(tmm, tDelta, side)
	if err != nil {
		return nil, err
	}
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, err
	}
	sec, err := SectionOf(ctx, rp.Profile, side, amount, budget, &radial)
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
	ax, axisSide, err := revolveplan.ResolveAxisSide(ctx, annulus, revolveaxis.Line2{
		AU: rp.Ax.AU, AV: rp.Ax.AV, DU: rp.Ax.DU, DV: rp.Ax.DV,
	}, work)
	if err != nil {
		return nil, fmt.Errorf(`%w: the thicken offset's revolve axis side is unresolved: %v`, decaderr.ErrUnsupported, err)
	}
	if axisSide < 0 {
		return nil, fmt.Errorf(`%w: the thicken offset crossed to the far side of the revolve axis`, decaderr.ErrUnsupported)
	}
	rp.Profile = annulus
	rp.Ax = ax
	rp.RadialProof = false
	rp.SurfaceResult = false
	rp.BlendSegs, rp.BlendKind = nil, ""
	return revolveplan.Build(ctx, env, rp, work)
}

// RibbonSteps maps the requested side to the right and left copies of
// the recorded walk. A centered request offsets both copies.
func RibbonSteps(side Side) (int, int) {
	switch side {
	case Negative:
		return 0, 1
	case Centered:
		return 1, 1
	default:
		return 1, 0
	}
}

// ChainExtrude builds a solid from a ribbon: the open walk's own
// thickened section swept through the ribbon's unchanged interval
// (docs/surface-design.md §16.6).
func ChainExtrude(ctx context.Context, env topo.Env, cp chainrecord.Payload, side Side, tmm, tDelta float64) (*topo.Body, error) {
	if len(cp.Chains) != 1 || cp.SectionDelta != 0 {
		return nil, fmt.Errorf(`%w: this ribbon has no admitted Thicken walk`, decaderr.ErrUnsupported)
	}
	if proofbound.AdmitAbove(proofbound.BoundedSub(cp.Z1Scalar(), cp.Z0Scalar()), 0) != proofbound.SurvAdmit {
		return nil, fmt.Errorf(`%w: the ribbon has no proven positive sweep height`, decaderr.ErrUnsupported)
	}
	amount, err := Amount(tmm, tDelta, side)
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
	rightSteps, leftSteps := RibbonSteps(side)
	section, err := thickenaxis.RibbonProfile(ctx, cp.Chains[0], rightSteps, leftSteps, amount, budget, work, nil)
	if err != nil {
		return nil, err
	}
	return prismrecord.Build(ctx, env, prismrecord.Payload{
		Profile: section,
		Frame:   cp.Frame,
		Z0:      cp.Z0, Z1: cp.Z1,
		Z0Delta: cp.Z0Delta, Z1Delta: cp.Z1Delta,
		Xform: cp.Xform,
	}, work)
}

// ChainRevolve builds a solid of revolution from an uncapped chain
// shell: §16.6's assembled section under §16.5's sweep and radial gate
// (docs/surface-design.md §16.7).
func ChainRevolve(ctx context.Context, env topo.Env, cp revolveplan.ChainPayload, side Side, tmm, tDelta float64) (*topo.Body, error) {
	// The prism ribbon's own admission, re-read over the meridian walk set
	// (ChainExtrude above): §16.6 offsets ONE walk, and the assembled
	// section it hands the solid build has to be exact — revolveplan.RequireExactSection
	// refuses a displaced one at that build anyway, and refusing here names the
	// ribbon rather than the revolve it was about to become.
	if len(cp.Chains) != 1 || cp.SectionDelta != 0 {
		return nil, fmt.Errorf(`%w: this ribbon has no admitted Thicken walk`, decaderr.ErrUnsupported)
	}
	radial, err := thickenRadialOf(cp.Ax)
	if err != nil {
		return nil, err
	}
	amount, err := Amount(tmm, tDelta, side)
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
	rightSteps, leftSteps := RibbonSteps(side)
	section, err := thickenaxis.RibbonProfile(ctx, cp.Chains[0], rightSteps, leftSteps, amount, budget, work, &radial)
	if err != nil {
		return nil, err
	}
	// The assembled section is a region no axis resolution has seen, so its own
	// snap allowances, radial admission charge and axial envelope are proven
	// here rather than inherited from the shell's: every one of them is an
	// integral over the region, and the shell had no region at all.
	ax, axisSide, err := revolveplan.ResolveAxisSide(ctx, section, revolveaxis.Line2{
		AU: cp.Ax.AU, AV: cp.Ax.AV, DU: cp.Ax.DU, DV: cp.Ax.DV,
	}, work)
	if err != nil {
		return nil, fmt.Errorf(`%w: the thicken offset's revolve axis side is unresolved: %v`, decaderr.ErrUnsupported, err)
	}
	if axisSide < 0 {
		return nil, fmt.Errorf(`%w: the thicken offset crossed to the far side of the revolve axis`, decaderr.ErrUnsupported)
	}
	rp := cp.Revolve()
	rp.Profile = section
	rp.Ax = ax
	rp.RadialProof = false
	return revolveplan.Build(ctx, env, rp, work)
}
