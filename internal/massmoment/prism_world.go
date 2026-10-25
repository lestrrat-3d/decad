package massmoment

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// PrismWorldMoments integrates a prism about anchor in world coordinates.
// The intervals include the recorded frame and placement lift rounding.
func PrismWorldMoments(ctx context.Context, record PrismRecord, anchor r3.Vec) (Moments, error) {
	if err := ctx.Err(); err != nil {
		return Moments{}, err
	}
	if !record.Frame.IsValid() || !record.Transform.IsValid() || !proofbound.FiniteVec(anchor) ||
		!proofbound.FiniteVec(record.Frame.ToWorldUV(0, 0)) ||
		proofbound.IsNonFinite(record.Z0) || proofbound.IsNonFinite(record.Z1) {
		return Moments{}, fmt.Errorf("%w: prism world moments have invalid frame, placement, or anchor", decaderr.ErrUnsupported)
	}

	midpoint, err := PrismMidLevel(record.Z0, record.Z1)
	if err != nil {
		return Moments{}, err
	}
	local, err := PrismProfileMoments(ctx, record.Profile, record.Z0, record.Z1,
		record.SectionDelta, record.Z0Delta, record.Z1Delta)
	if err != nil {
		return Moments{}, err
	}
	local = Shift(local, [3]*big.Rat{new(big.Rat), new(big.Rat), midpoint})

	linear, err := PrismRotation(record.Frame, record.Transform)
	if err != nil {
		return Moments{}, err
	}
	if _, err := MapChargeOf(linear); err != nil {
		return Moments{}, err
	}
	world := Transform(local, linear)

	frameOrigin := record.Frame.ToWorldUV(0, 0)
	heldOrigin := record.Transform.Apply(frameOrigin)
	originBound := proofbound.ExactFrameLiftRound(record.Frame, record.Transform, 0, 0, 0, heldOrigin)
	if !proofbound.FiniteVec(heldOrigin) || proofbound.IsNonFinite(originBound) || originBound < 0 {
		return Moments{}, fmt.Errorf("%w: prism origin lift has no finite rounding bound", decaderr.ErrUnsupported)
	}
	delta := proofbound.HeldDelta(heldOrigin, anchor)
	world = Shift(world, [3]*big.Rat{delta[0].Rat(), delta[1].Rat(), delta[2].Rat()})
	world, err = WidenTranslation(world, originBound)
	if err != nil {
		return Moments{}, err
	}
	if err := ctx.Err(); err != nil {
		return Moments{}, err
	}
	return world, nil
}

// WidenTranslation encloses a common translation uncertainty in every
// coordinate. Each second-moment term reads the same unshifted moment values.
func WidenTranslation(m Moments, radius float64) (Moments, error) {
	if radius == 0 {
		return m, nil
	}
	if math.IsNaN(radius) || math.IsInf(radius, 0) || radius < 0 {
		return Moments{}, fmt.Errorf("%w: prism origin rounding radius is invalid", decaderr.ErrUnsupported)
	}
	r := proofarith.FloatRat(radius)
	if r == nil {
		return Moments{}, fmt.Errorf("%w: prism origin rounding radius is not finite", decaderr.ErrUnsupported)
	}
	var displacement [3]proofbound.RatInterval
	for i := range displacement {
		displacement[i] = proofbound.Interval(new(big.Rat).Neg(r), r)
	}

	out := Moments{Volume: m.Volume}
	for i := range out.First {
		out.First[i] = proofbound.IntervalAdd(m.First[i], proofbound.IntervalMul(m.Volume, displacement[i]))
	}
	for i := range out.Second {
		for j := i; j < len(out.Second[i]); j++ {
			value := proofbound.IntervalAdd(m.Second[i][j], proofbound.IntervalMul(displacement[i], m.First[j]))
			value = proofbound.IntervalAdd(value, proofbound.IntervalMul(m.First[i], displacement[j]))
			value = proofbound.IntervalAdd(value,
				proofbound.IntervalMul(m.Volume, proofbound.IntervalMul(displacement[i], displacement[j])))
			out.Second[i][j], out.Second[j][i] = value, value
		}
	}
	return out, nil
}
