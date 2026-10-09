package massmoment

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// RevolveRecord carries the section, exact axis, sweep, and placement that
// define a revolve's mass moments. Bounds and allowances state whether the
// analytic path can charge every term.
type RevolveRecord struct {
	Profile                            momentinput.Profile
	Frame                              r3.Frame
	Transform                          r3.Transform
	Den                                revolveangle.Sweep
	Phi0, Phi1                         float64
	AU, AV, DU, DV                     float64
	AUBound, AVBound, DUBound, DVBound float64
	SectionDelta, RadialAdmitAllow     float64
	AxisSnap                           bool
}

// RevolveProfileMoments integrates the section and angular factors about the
// exact axis anchor. A displaced section or axis, axis snap, or unstated sweep
// denotation is refused before integration.
func RevolveProfileMoments(ctx context.Context, p RevolveRecord) (Moments, error) {
	if p.SectionDelta != 0 {
		return Moments{}, fmt.Errorf("%w: revolve section carries a displacement the mass path does not charge", decaderr.ErrUnsupported)
	}
	if p.AUBound != 0 || p.AVBound != 0 || p.DUBound != 0 || p.DVBound != 0 {
		return Moments{}, fmt.Errorf("%w: revolve axis is not exact", decaderr.ErrUnsupported)
	}
	if p.RadialAdmitAllow != 0 || p.AxisSnap {
		return Moments{}, fmt.Errorf("%w: revolve axis snap is not charged by the mass path", decaderr.ErrUnsupported)
	}
	if !p.Den.Phi0.Valid() || !p.Den.Phi1.Valid() {
		return Moments{}, fmt.Errorf("%w: revolve sweep has no exact denotation", decaderr.ErrUnsupported)
	}
	if err := ctx.Err(); err != nil {
		return Moments{}, err
	}
	plane, err := RevolveSectionMoments(ctx, p.Profile)
	if err != nil {
		return Moments{}, err
	}
	angular, ok := RevolveSweepFactors(p.Den, p.Phi0, p.Phi1)
	if !ok {
		return Moments{}, fmt.Errorf("%w: revolve sweep has no certified angular factors", decaderr.ErrUnsupported)
	}
	if err := ctx.Err(); err != nil {
		return Moments{}, err
	}
	return RevolveMoments(plane, p.AU, p.AV, p.DU, p.DV, angular)
}

// RevolveProperties maps the local moments through the recorded axis frame
// and placement, then publishes mass and world centroidal inertia.
func RevolveProperties(ctx context.Context, p RevolveRecord, center measurement.VecMeasurement,
	density units.Value) (MassProperties, error) {
	moments, err := RevolveProfileMoments(ctx, p)
	if err != nil {
		return MassProperties{}, err
	}
	linear, err := RevolveRotation(p.Frame, p.DU, p.DV, p.Transform)
	if err != nil {
		return MassProperties{}, err
	}
	world, massIv, err := AffineInertia(moments, linear, density)
	if err != nil {
		return MassProperties{}, err
	}
	return Publish(ctx, center, massIv, world)
}
