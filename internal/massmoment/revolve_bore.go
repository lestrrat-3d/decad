package massmoment

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/units"
)

// RevolveBoreProperties subtracts prism moments from Revolve moments about a
// common world anchor, then publishes mass and centroidal inertia.
func RevolveBoreProperties(ctx context.Context, host RevolveRecord, tool PrismRecord,
	center measurement.VecMeasurement, density units.Value) (MassReadings, error) {
	local, err := RevolveProfileMoments(ctx, host)
	if err != nil {
		return MassReadings{}, err
	}
	linear, err := RevolveRotation(host.Frame, host.DU, host.DV, host.Transform)
	if err != nil {
		return MassReadings{}, err
	}
	world := Transform(local, linear)
	anchor := center.Value
	heldOrigin := host.Transform.Apply(host.Frame.ToWorldUV(host.AU, host.AV))
	originBound := proofbound.ExactFrameLiftRound(host.Frame, host.Transform, host.AU, host.AV, 0, heldOrigin)
	delta := proofbound.HeldDelta(heldOrigin, anchor)
	world = Shift(world, [3]*big.Rat{delta[0].Rat(), delta[1].Rat(), delta[2].Rat()})
	world, err = WidenTranslation(world, originBound)
	if err != nil {
		return MassReadings{}, err
	}
	void, err := PrismWorldMoments(ctx, tool, anchor)
	if err != nil {
		return MassReadings{}, err
	}
	identity := [3][3]*big.Rat{}
	for i := range 3 {
		for j := range 3 {
			identity[i][j] = new(big.Rat)
			if i == j {
				identity[i][j].SetInt64(1)
			}
		}
	}
	inertia, mass, err := AffineInertia(Sub(world, void), identity, density)
	if err != nil {
		return MassReadings{}, err
	}
	return Publish(ctx, center, mass, inertia)
}
