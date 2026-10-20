package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/massdispatch"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/units"
)

// MassProperties contains the uniform-density mass, world-space center, and
// centroidal inertia of a solid. Each scalar is accompanied by an absolute
// bound on its numerical error.
type MassProperties struct {
	Mass    Measurement
	Center  VecMeasurement
	Inertia InertiaReading
}

// InertiaReading is the symmetric inertia tensor about the mass center in
// world axes. Mixed entries include the physical minus sign.
type InertiaReading struct {
	XX, YY, ZZ Measurement
	XY, XZ, YZ Measurement
}

// massPropertiesFromReadings assembles the public record from integrator data.
func massPropertiesFromReadings(readings massmoment.MassReadings, err error) (MassProperties, error) {
	if err != nil {
		return MassProperties{}, err
	}
	tensor := readings.Tensor
	return MassProperties{
		Mass: measurementFromInternal(readings.Mass), Center: vecMeasurementFromInternal(readings.Center),
		Inertia: InertiaReading{
			XX: measurementFromInternal(tensor[0]), YY: measurementFromInternal(tensor[1]),
			ZZ: measurementFromInternal(tensor[2]), XY: measurementFromInternal(tensor[3]),
			XZ: measurementFromInternal(tensor[4]), YZ: measurementFromInternal(tensor[5]),
		},
	}, nil
}

// MassProperties computes the properties of b for a stated positive, uniform
// density. The density must have kind units.Density. The current evaluator
// admits untapered solid prisms under any frame and rigid placement, including
// those whose recorded section or levels carry a proven displacement, full
// source spheres under rigid placements, cardinal full source cylinders made
// by revolving an axis-incident rectangle, full and partial revolves of any
// integrated section about an exact in-plane axis under any frame and rigid
// placement, sweeps whose spans those prism and revolve paths admit, cups
// whose outer and cavity prisms they admit, and faceted Boolean solids with a
// certified occupied-volume bound. Every other solid, and a sweep or cup the
// analytic arms refuse, is integrated over its VerifyAll mesh when that mesh
// carries an occupied-volume proof, refining the mesh until the tensor
// interval proves positive: lofts, tapered extrudes, exact stitched solids
// and curved payloads the analytic arms refuse.
// It returns ErrUnsupported for other solids rather than estimating their inertia.
// The receiver and context must not be nil.
func (b *Body) MassProperties(ctx context.Context, density units.Value) (MassProperties, error) {
	if b == nil {
		return MassProperties{}, fmt.Errorf("%w: nil body", ErrDegenerate)
	}
	return massPropertiesFromReadings(massdispatch.Properties(ctx, verifyAllMesher{}, &b.n, density))
}
