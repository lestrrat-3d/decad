package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/units"
)

// This file owns the mass properties of an untapered prism whose frame or
// placement basis is not a signed permutation, or whose recorded section or
// sweep levels carry a proven displacement (docs/multibody-dynamics-design.md
// §8.1 and §8.2, docs/dynamic-mass-design.md §2.1 and §3).
//
// The volume moments are integrated in the frame-local coordinates
// q = (u, v, z - zm), zm the recorded mid level, then rotated into world axes
// by the exact rational product M of the placement basis and the frame axes.
// Two certificates join the analytic integrals:
//
//   - The occupied-volume error E between the recorded prism and the prism its
//     construction denotes widens V by E, each P_i by R·E and each Q_ij by
//     R²·E, R bounding every |q_i| over both prisms (dynamic-mass §2.2).
//   - The held basis M is orthonormal only to rounding. The reading is about
//     the rigid rotation Q nearest M (its polar factor), so each world
//     component widens by 3·d·(2+d)·m, where d bounds ‖MᵀM − I‖_F and m is the
//     largest local tensor magnitude: ‖M − Q‖_F ≤ d, so
//     |(Q I Qᵀ − M I Mᵀ)_ij| ≤ ‖M − Q‖(‖I‖ + ‖M‖‖I‖) ≤ d(2+d)‖I‖_2 and
//     ‖I‖_2 ≤ ‖I‖_F ≤ 3m.
//
// prismVolumeMoments and the internal/massmoment transforms are shared
// with the sweep and cup paths, which
// combine several solids' V, P and Q about one anchor before forming a
// tensor.

// rotatedPrismMassProperties integrates pp's admitted section moments over
// its axial interval, charges any recorded displacement, and rotates the
// centroidal tensor into world axes. The caller has already admitted pp as a
// solid prism; this path takes every frame and placement basis, cardinal ones
// included.
func rotatedPrismMassProperties(ctx context.Context, pp prismPayload, center VecMeasurement, density units.Value) (MassProperties, error) {
	moments, err := prismVolumeMoments(ctx, pp)
	if err != nil {
		return MassProperties{}, err
	}
	local, rho, eigenLower, err := massmoment.PrismLocalInertia(moments, density)
	if err != nil {
		return MassProperties{}, err
	}
	basis, err := prismRotation(pp)
	if err != nil {
		return MassProperties{}, err
	}
	world := massmoment.RotatePrismInertia(basis, local)
	result := MassProperties{Center: center}
	result.Mass, err = massIntervalReading(proofbound.IntervalScale(moments.Volume, rho), units.Kilogram)
	if err != nil {
		return MassProperties{}, err
	}
	if result.Mass.Bound.Mag() >= result.Mass.Value.Mag() {
		return MassProperties{}, fmt.Errorf("%w: mass interval does not prove positive mass", ErrUnsupported)
	}
	entries := []struct {
		i, j    int
		reading *Measurement
	}{
		{0, 0, &result.Inertia.XX}, {1, 1, &result.Inertia.YY}, {2, 2, &result.Inertia.ZZ},
		{0, 1, &result.Inertia.XY}, {0, 2, &result.Inertia.XZ}, {1, 2, &result.Inertia.YZ},
	}
	for _, entry := range entries {
		*entry.reading, err = massIntervalReading(world[entry.i][entry.j], units.KilogramSquareMillimeter)
		if err != nil {
			return MassProperties{}, err
		}
	}
	// Any tensor inside the published intervals differs from the true world
	// tensor by at most twice each bound per entry, and the sum of those nine
	// widths bounds the spectral norm of the difference.
	spread := new(big.Rat)
	for _, entry := range entries {
		width := new(big.Rat).Mul(big.NewRat(2, 1), proofarith.FloatRat(entry.reading.Bound.Mag()))
		if entry.i != entry.j {
			width.Mul(width, big.NewRat(2, 1))
		}
		spread.Add(spread, width)
	}
	if eigenLower.Cmp(spread) <= 0 {
		return MassProperties{}, fmt.Errorf("%w: inertia interval does not prove positive definiteness", ErrUnsupported)
	}
	if err := ctx.Err(); err != nil {
		return MassProperties{}, err
	}
	return result, nil
}

// prismVolumeMoments integrates pp's admitted section moments over its axial
// interval in the frame-local coordinates q = (u, v, z - zm), zm the recorded
// mid level, and charges any recorded displacement as E, R·E and R²·E.
func prismVolumeMoments(ctx context.Context, pp prismPayload) (massmoment.Moments, error) {
	if !nonNegativeFinite(pp.sectionDelta) || !nonNegativeFinite(pp.z0Delta) || !nonNegativeFinite(pp.z1Delta) {
		return massmoment.Moments{}, fmt.Errorf("%w: prism displacement has no finite bound", ErrUnsupported)
	}
	section, err := prismSectionMoments(ctx, pp)
	if err != nil {
		return massmoment.Moments{}, err
	}
	return massmoment.PrismVolumeMoments(ctx, section, pp.z0, pp.z1,
		pp.sectionDelta > 0 || pp.z0Delta > 0 || pp.z1Delta > 0,
		func(area proofbound.RatInterval, h *big.Rat) (*big.Rat, *big.Rat, error) {
			return prismOccupiedVolumeError(ctx, pp, area, h)
		})
}

// prismMidLevel is zm = (z0 + z1)/2, the anchor level of prismVolumeMoments,
// as an exact rational.
func prismMidLevel(pp prismPayload) (*big.Rat, error) {
	return massmoment.PrismMidLevel(pp.z0, pp.z1)
}

// prismSectionMoments reads the section's area, first and second moments as
// rational intervals: exact when the moment engine certified every field
// exactly, else each held value widened by its own published bound.
func prismSectionMoments(ctx context.Context, pp prismPayload) ([6]proofbound.RatInterval, error) {
	ig, err := pp.profile.evaluatorIntegralsContext(ctx, freeform.MomentSecondOrder, nil)
	if err != nil {
		return [6]proofbound.RatInterval{}, err
	}
	section, err := massmoment.SectionIntervals(sectionMomentInputs(ig))
	if err != nil {
		return [6]proofbound.RatInterval{}, err
	}
	return section, nil
}

func sectionMomentInputs(ig regionIntegrals) massmoment.SectionInputs {
	input := massmoment.SectionInputs{Bounded: [6]proofbound.BoundedScalar{
		{Value: ig.area, Bound: ig.areaBound}, {Value: ig.mu, Bound: ig.muBound}, {Value: ig.mv, Bound: ig.mvBound},
		{Value: ig.muu, Bound: ig.muuBound}, {Value: ig.muv, Bound: ig.muvBound}, {Value: ig.mvv, Bound: ig.mvvBound},
	}}
	if !ig.exactDead && ig.exact.Complete() {
		input.ExactAvailable = true
		input.Exact = ig.exact.Fields()
	}
	return input
}

// prismOccupiedVolumeError bounds the volume of the symmetric difference
// between the recorded prism and the prism its construction denotes, and a
// radius R bounding every coordinate |u|, |v|, |z - zm| of both.
//
// A point in one prism and not the other either projects into the section's
// displacement tube, of area at most proofbound.SectionDisplacementArea, over the wider
// of the two axial intervals, or projects into the recorded section and lies
// in one of the two end slabs the level displacements sweep. So
//
//	E = SDA·(h + δ0 + δ1) + A_upper·(δ0 + δ1).
//
// Every recorded boundary coordinate lies within the section envelope, and
// the denoted boundary within sectionDelta of it; the denoted levels lie
// within their own displacement of h/2 from zm.
func prismOccupiedVolumeError(ctx context.Context, pp prismPayload, area proofbound.RatInterval, h *big.Rat) (*big.Rat, *big.Rat, error) {
	work := freeform.NewFreeformWork()
	walks, err := resolveProfileWalks(pp.profile, work)
	if err != nil {
		return nil, nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, nil, err
	}
	count := 0
	perimeter := 0.0
	for _, loop := range append([][]survey2d.SegmentWalk{walks.outer}, walks.holes...) {
		for _, w := range loop {
			count++
			perimeter = proofbound.AbsSumUpper(perimeter, w.Length, w.LengthBound)
		}
	}
	coordUpper, err := profileCoordinateEnvelope(pp.profile, work, walks)
	if err != nil {
		return nil, nil, err
	}
	return massmoment.PrismOccupiedError(area, h, pp.sectionDelta, pp.z0Delta, pp.z1Delta,
		count, perimeter, coordUpper)
}

// prismRotation is the exact rational matrix taking frame-local (u, v, n)
// directions to world directions: the placement basis times the frame axes,
// both read from the held floats. Column k is the image of local axis k.
func prismRotation(pp prismPayload) ([3][3]*big.Rat, error) {
	return massmoment.PrismRotation(pp.frame, pp.xform)
}

func nonNegativeFinite(value float64) bool {
	return !proofbound.IsNonFinite(value) && value >= 0
}
