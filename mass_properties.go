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
// admits source prisms under axis-preserving rigid placements and returns
// ErrUnsupported for other solids rather than estimating their inertia.
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
		pp.surfaceResult ||
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
	if !rectangularProfile(pp.profile) {
		return prismMassProperties(ctx, b, pp, density)
	}

	// The recorded rectangle and levels are the source solid. Translation does
	// not change its centroidal inertia, and a signed-permutation basis only
	// reorders its three dimensions. Work in exact dyadics until division by 12.
	firstLine, ok := pp.profile.Outer.Segments[0].(LineSeg)
	if !ok {
		return MassProperties{}, fmt.Errorf("%w: box section is not a line loop", ErrUnsupported)
	}
	first := firstLine.Start
	minU, maxU, minV, maxV := first.U, first.U, first.V, first.V
	for _, segment := range pp.profile.Outer.Segments {
		if err := ctx.Err(); err != nil {
			return MassProperties{}, err
		}
		line, ok := segment.(LineSeg)
		if !ok {
			return MassProperties{}, fmt.Errorf("%w: box section is not a line loop", ErrUnsupported)
		}
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

// prismMassProperties integrates the evaluator's admitted section moments,
// then its recorded axial interval. Every interval is rational, including the
// enclosure of a curved section's published moments. The current orientation
// gate admits exact signed-permutation axes, so no rotation coefficient can
// silently lose a bound while mapping the local tensor to world axes.
func prismMassProperties(ctx context.Context, b *Body, pp prismPayload, density units.Value) (MassProperties, error) {
	ig, err := pp.profile.evaluatorIntegralsContext(ctx, momentSecondOrder, nil)
	if err != nil {
		return MassProperties{}, err
	}
	section := [6]ratInterval{}
	if !ig.exactDead && ig.exact.complete() {
		for i, value := range ig.exact.fields() {
			section[i] = pointInterval(value)
		}
	} else {
		values := [6]boundedScalar{
			{ig.area, ig.areaBound}, {ig.mu, ig.muBound}, {ig.mv, ig.mvBound},
			{ig.muu, ig.muuBound}, {ig.muv, ig.muvBound}, {ig.mvv, ig.mvvBound},
		}
		for i, value := range values {
			section[i], err = massMomentInterval(value)
			if err != nil {
				return MassProperties{}, err
			}
		}
	}
	a, mu, mv := section[0], section[1], section[2]
	if a.lo.Sign() <= 0 {
		return MassProperties{}, fmt.Errorf("%w: section area interval does not prove positive volume", ErrUnsupported)
	}
	h := new(big.Rat).Sub(floatRat(pp.z1), floatRat(pp.z0))
	if h.Sign() <= 0 {
		return MassProperties{}, fmt.Errorf("%w: axial interval does not prove positive volume", ErrUnsupported)
	}
	rho := new(big.Rat).Mul(floatRat(density.Mag()), floatRat(density.Unit().Factor()))
	rhoH := new(big.Rat).Mul(rho, h)
	massIv := intervalScale(a, rhoH)
	mu2OverA, _ := intervalQuo(intervalMul(mu, mu), a)
	mv2OverA, _ := intervalQuo(intervalMul(mv, mv), a)
	mumvOverA, _ := intervalQuo(intervalMul(mu, mv), a)
	cuu := intervalSub(section[3], mu2OverA)
	cuv := intervalSub(section[4], mumvOverA)
	cvv := intervalSub(section[5], mv2OverA)
	h2Over12 := new(big.Rat).Quo(new(big.Rat).Mul(h, h), big.NewRat(12, 1))
	axial := intervalScale(a, new(big.Rat).Mul(rhoH, h2Over12))
	local := [3][3]ratInterval{}
	local[0][0] = intervalAdd(intervalScale(cvv, rhoH), axial)
	local[1][1] = intervalAdd(intervalScale(cuu, rhoH), axial)
	local[2][2] = intervalScale(intervalAdd(cuu, cvv), rhoH)
	local[0][1] = intervalScale(cuv, new(big.Rat).Neg(rhoH))
	local[1][0] = local[0][1]
	zero := pointInterval(new(big.Rat))
	local[0][2], local[2][0], local[1][2], local[2][1] = zero, zero, zero, zero

	world := [3][3]ratInterval{}
	var worldAxis [3]int
	var sign [3]int64
	for k, axis := range []r3.Vec{
		pp.xform.ApplyDir(pp.frame.U()),
		pp.xform.ApplyDir(pp.frame.V()),
		pp.xform.ApplyDir(pp.frame.N()),
	} {
		switch {
		case math.Abs(axis.X) == 1 && axis.Y == 0 && axis.Z == 0:
			worldAxis[k], sign[k] = 0, int64(axis.X)
		case axis.X == 0 && math.Abs(axis.Y) == 1 && axis.Z == 0:
			worldAxis[k], sign[k] = 1, int64(axis.Y)
		case axis.X == 0 && axis.Y == 0 && math.Abs(axis.Z) == 1:
			worldAxis[k], sign[k] = 2, int64(axis.Z)
		default:
			return MassProperties{}, fmt.Errorf("%w: prism orientation has no exact cardinal axes", ErrUnsupported)
		}
	}
	for i := range local {
		for j := range local[i] {
			world[worldAxis[i]][worldAxis[j]] = intervalScale(local[i][j], big.NewRat(sign[i]*sign[j], 1))
		}
	}
	for i := range world {
		lower := new(big.Rat).Set(world[i][i].lo)
		for j := range world[i] {
			if i == j {
				continue
			}
			magnitude := new(big.Rat).Abs(world[i][j].lo)
			if upper := new(big.Rat).Abs(world[i][j].hi); upper.Cmp(magnitude) > 0 {
				magnitude = upper
			}
			lower.Sub(lower, magnitude)
		}
		if lower.Sign() <= 0 {
			return MassProperties{}, fmt.Errorf("%w: inertia interval does not prove positive definiteness", ErrUnsupported)
		}
	}
	result := MassProperties{Center: b.centroid}
	result.Mass, err = massIntervalReading(massIv, units.Kilogram)
	if err != nil {
		return MassProperties{}, err
	}
	entries := []struct {
		iv      ratInterval
		reading *Measurement
	}{
		{world[0][0], &result.Inertia.XX}, {world[1][1], &result.Inertia.YY},
		{world[2][2], &result.Inertia.ZZ}, {world[0][1], &result.Inertia.XY},
		{world[0][2], &result.Inertia.XZ}, {world[1][2], &result.Inertia.YZ},
	}
	for _, entry := range entries {
		*entry.reading, err = massIntervalReading(entry.iv, units.KilogramSquareMillimeter)
		if err != nil {
			return MassProperties{}, err
		}
	}
	if err := ctx.Err(); err != nil {
		return MassProperties{}, err
	}
	return result, nil
}

func massMomentInterval(value boundedScalar) (ratInterval, error) {
	if isNonFinite(value.value) || isNonFinite(value.bound) || value.bound < 0 {
		return ratInterval{}, fmt.Errorf("%w: section moment has no finite enclosure", ErrNotFinite)
	}
	held, bound := floatRat(value.value), floatRat(value.bound)
	return intervalOwned(new(big.Rat).Sub(held, bound), new(big.Rat).Add(held, bound)), nil
}

func massIntervalReading(iv ratInterval, unit units.Unit) (Measurement, error) {
	if iv.lo.Cmp(iv.hi) == 0 {
		return massReading(iv.lo, unit)
	}
	held, _ := intervalMid(iv).Float64()
	bound := intervalFloatError(iv, held)
	if isNonFinite(held) || isNonFinite(bound) {
		return Measurement{}, fmt.Errorf("%w: mass property cannot be represented finitely", ErrNotFinite)
	}
	return Measurement{Value: units.New(held, unit), Bound: units.New(bound, unit), Exactness: exactnessOf(bound)}, nil
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
