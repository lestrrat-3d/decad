package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/r3"
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

// MassProperties computes the properties of b for a stated positive, uniform
// density. The density must have kind units.Density. The current evaluator
// admits source rectangular prisms under axis-preserving rigid placements and
// returns ErrUnsupported for other solids rather than estimating their inertia.
// The receiver and context must not be nil.
func (b *Body) MassProperties(ctx context.Context, density units.Value) (MassProperties, error) {
	if b == nil {
		return MassProperties{}, fmt.Errorf("%w: nil body", ErrDegenerate)
	}
	if density.Kind() != units.Density {
		return MassProperties{}, fmt.Errorf("%w: mass density is required", ErrUnitKind)
	}
	if isNonFinite(density.Mag()) || isNonFinite(density.Unit().Factor()) {
		return MassProperties{}, fmt.Errorf("%w: density is not finite", ErrNotFinite)
	}
	if density.Mag() < 0 {
		return MassProperties{}, fmt.Errorf("%w: density is negative", ErrNegativeMagnitude)
	}
	if density.Mag() == 0 {
		return MassProperties{}, fmt.Errorf("%w: density is zero", ErrDegenerate)
	}
	if !b.solid || b.kind != BodySolid {
		return MassProperties{}, ErrNotSolid
	}
	if b.payload == nil {
		return MassProperties{}, fmt.Errorf("%w: body has no evaluator payload", ErrUnsupported)
	}
	pp, ok := b.payload.(prismPayload)
	if !ok || pp.sectionDelta != 0 || pp.z0Delta != 0 || pp.z1Delta != 0 ||
		pp.surfaceResult || !rectangularProfile(pp.profile) ||
		!cardinalBasis(pp.frame.U(), pp.frame.V(), pp.frame.N()) {
		return MassProperties{}, fmt.Errorf("%w: certified volume moments are unavailable for this solid", ErrUnsupported)
	}
	basis := pp.xform.Basis()
	if !cardinalBasis(basis.EX, basis.EY, basis.EZ) {
		return MassProperties{}, fmt.Errorf("%w: rotated box inertia is not yet certified", ErrUnsupported)
	}
	if err := ctx.Err(); err != nil {
		return MassProperties{}, err
	}

	// The recorded rectangle and levels are the source solid. Translation does
	// not change its centroidal inertia, and a signed-permutation basis only
	// reorders its three dimensions. Work in exact dyadics until division by 12.
	first := pp.profile.Outer.Segments[0].(LineSeg).Start
	minU, maxU, minV, maxV := first.U, first.U, first.V, first.V
	for _, segment := range pp.profile.Outer.Segments {
		if err := ctx.Err(); err != nil {
			return MassProperties{}, err
		}
		line := segment.(LineSeg)
		point := line.Start
		if line.TStart == 1 {
			point = line.End
		}
		minU, maxU = math.Min(minU, point.U), math.Max(maxU, point.U)
		minV, maxV = math.Min(minV, point.V), math.Max(maxV, point.V)
	}
	u := dySubScalar(mustDyOf(maxU), mustDyOf(minU))
	v := dySubScalar(mustDyOf(maxV), mustDyOf(minV))
	z := dySubScalar(mustDyOf(pp.z1), mustDyOf(pp.z0))
	if u.sign() <= 0 || v.sign() <= 0 || z.sign() <= 0 {
		return MassProperties{}, fmt.Errorf("%w: source box has no positive volume", ErrUnsupported)
	}
	densityBase := dyMul(mustDyOf(density.Mag()), mustDyOf(density.Unit().Factor()))
	mass := dyMul(densityBase, dyMul(u, dyMul(v, z)))
	if mass.sign() <= 0 {
		return MassProperties{}, fmt.Errorf("%w: mass is not positive", ErrUnsupported)
	}

	var dimensions [3]dyadic
	for i, axis := range []r3.Vec{
		pp.xform.ApplyDir(pp.frame.U()),
		pp.xform.ApplyDir(pp.frame.V()),
		pp.xform.ApplyDir(pp.frame.N()),
	} {
		dimension := []dyadic{u, v, z}[i]
		switch {
		case math.Abs(axis.X) == 1 && axis.Y == 0 && axis.Z == 0:
			dimensions[0] = dimension
		case axis.X == 0 && math.Abs(axis.Y) == 1 && axis.Z == 0:
			dimensions[1] = dimension
		case axis.X == 0 && axis.Y == 0 && math.Abs(axis.Z) == 1:
			dimensions[2] = dimension
		default:
			return MassProperties{}, fmt.Errorf("%w: box orientation has no exact cardinal axes", ErrUnsupported)
		}
	}
	squared := [3]dyadic{}
	for i, length := range dimensions {
		squared[i] = dyMul(length, length)
	}
	inertia := func(a, c dyadic) *big.Rat {
		numerator := dyMul(mass, dyAdd(a, c))
		return new(big.Rat).Quo(numerator.rat(), big.NewRat(12, 1))
	}
	readings := [3]*big.Rat{
		inertia(squared[1], squared[2]),
		inertia(squared[0], squared[2]),
		inertia(squared[0], squared[1]),
	}
	result := MassProperties{Center: b.centroid}
	var err error
	result.Mass, err = massReading(mass.rat(), units.Kilogram)
	if err != nil {
		return MassProperties{}, err
	}
	if result.Mass.Bound.Mag() >= result.Mass.Value.Mag() {
		return MassProperties{}, fmt.Errorf("%w: mass interval does not prove positive mass", ErrUnsupported)
	}
	diagonal := [3]*Measurement{&result.Inertia.XX, &result.Inertia.YY, &result.Inertia.ZZ}
	for i, exact := range readings {
		*diagonal[i], err = massReading(exact, units.KilogramSquareMillimeter)
		if err != nil {
			return MassProperties{}, err
		}
		if diagonal[i].Bound.Mag() >= diagonal[i].Value.Mag() {
			return MassProperties{}, fmt.Errorf("%w: inertia interval does not prove positive definiteness", ErrUnsupported)
		}
	}
	zero := Measurement{
		Value:     units.KilogramSquareMillimeters(0),
		Bound:     units.KilogramSquareMillimeters(0),
		Exactness: Exact,
	}
	result.Inertia.XY, result.Inertia.XZ, result.Inertia.YZ = zero, zero, zero
	if err := ctx.Err(); err != nil {
		return MassProperties{}, err
	}
	return result, nil
}

func massReading(exact *big.Rat, unit units.Unit) (Measurement, error) {
	value, _ := exact.Float64()
	bound := rationalFloatError(exact, value)
	if isNonFinite(value) || isNonFinite(bound) {
		return Measurement{}, fmt.Errorf("%w: mass property cannot be represented finitely", ErrNotFinite)
	}
	return Measurement{
		Value:     units.New(value, unit),
		Exactness: exactnessOf(bound),
		Bound:     units.New(bound, unit),
	}, nil
}
