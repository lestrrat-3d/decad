package decad

import (
	"context"
	"fmt"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the general revolve's mass path (docs/dynamic-mass-design.md
// §2.1, docs/multibody-dynamics-design.md §8.6): a full or partial revolve of
// any section the moment engine integrates, from the recorded section's
// moments through third order and the sweep's certified angular factors.
//
// In the axis frame a section point (z, ρ) at sweep angle φ sits at
// a3 + z·w + ρ·(cos φ·e0 + sin φ·e1) (revolvePayload.point), and the volume
// element is ρ dA dφ. About the axis anchor a3, in the local basis (w, e0, e1):
//
//	V     = Δφ·∫ρ
//	P     = (Δφ·∫zρ,  Sφ·∫ρ²,  Cφ·∫ρ²)
//	Q_ww  = Δφ·∫z²ρ   Q_w0 = Sφ·∫zρ²   Q_w1 = Cφ·∫zρ²
//	Q_00  = ∫cos²φ·∫ρ³   Q_01 = ∫sinφcosφ·∫ρ³   Q_11 = ∫sin²φ·∫ρ³
//
// with Sφ = ∫cos φ dφ and Cφ = ∫sin φ dφ over [φ0, φ1], every section
// integral over dA. The ∫ρ³, ∫zρ² and ∫z²ρ terms are the third-order ones
// momentThirdOrder supplies. A full turn's Sφ, Cφ and ∫sinφcosφ are the exact
// zero and its ∫cos² and ∫sin² are π, through the same formulas, because the
// turn-stated endpoints have exact sine and cosine (quarterTurnSinCos).
//
// Every quantity is a rational interval: section moments exactly or from
// their published bounds, the axis frame exactly, the angular factors from
// the payload's own sweep denotation, density exactly. The centroidal tensor
// is S = Q − P·Pᵀ/V over one common V, P, Q enclosure, and the inertia
// I = ρ_m·(trace(S)·1 − S).

// revolveMassProperties integrates the general revolve. Every gate below only
// refuses: a payload whose readings carry a term this path does not charge —
// an axis-snap or admitted-band allowance, an uncertain axis, a sweep end
// without a denotation, a section displacement — returns ErrUnsupported
// rather than a tensor missing that term. The local tensor reaches world axes
// through docs/multibody-dynamics-design.md §8.1's rotation: the exact
// rational product of the placement basis and the local basis, widened by
// its orthonormality defect so the reading refers to the rigid rotation
// nearest it.
func revolveMassProperties(ctx context.Context, b *Body, rp revolvePayload, density units.Value) (MassProperties, error) {
	if rp.sectionDelta != 0 {
		return MassProperties{}, fmt.Errorf("%w: revolve section carries a displacement the mass path does not charge", ErrUnsupported)
	}
	ax := rp.ax
	if ax.aUBound != 0 || ax.aVBound != 0 || ax.dUBound != 0 || ax.dVBound != 0 {
		return MassProperties{}, fmt.Errorf("%w: revolve axis is not exact", ErrUnsupported)
	}
	if ax.radialAdmitAllow != 0 || ax.snap != (regionSnapAllow{}) {
		return MassProperties{}, fmt.Errorf("%w: revolve axis snap is not charged by the mass path", ErrUnsupported)
	}
	if !rp.den.phi0.valid() || !rp.den.phi1.valid() {
		return MassProperties{}, fmt.Errorf("%w: revolve sweep has no exact denotation", ErrUnsupported)
	}
	rotation, err := revolveRotation(rp)
	if err != nil {
		return MassProperties{}, err
	}
	if err := ctx.Err(); err != nil {
		return MassProperties{}, err
	}

	ig, err := rp.profile.evaluatorIntegralsContext(ctx, momentThirdOrder, nil)
	if err != nil {
		return MassProperties{}, err
	}
	plane, err := revolveSectionMoments(ig)
	if err != nil {
		return MassProperties{}, err
	}
	axisMoment := revolveAxisMoments(plane, ax)
	r1 := axisMoment(0, 1)
	zr := axisMoment(1, 1)
	r2 := axisMoment(0, 2)
	z2r := axisMoment(2, 1)
	zr2 := axisMoment(1, 2)
	r3m := axisMoment(0, 3)

	angular, ok := revolveAngularFactors(rp)
	if !ok {
		return MassProperties{}, fmt.Errorf("%w: revolve sweep has no certified angular factors", ErrUnsupported)
	}
	if err := ctx.Err(); err != nil {
		return MassProperties{}, err
	}

	volume := intervalMul(angular.width, r1)
	if volume.lo.Sign() <= 0 {
		return MassProperties{}, fmt.Errorf("%w: revolve volume interval does not prove positive volume", ErrUnsupported)
	}
	first := [3]ratInterval{
		intervalMul(angular.width, zr),
		intervalMul(angular.cos, r2),
		intervalMul(angular.sin, r2),
	}
	var second [3][3]ratInterval
	second[0][0] = intervalMul(angular.width, z2r)
	second[0][1] = intervalMul(angular.cos, zr2)
	second[0][2] = intervalMul(angular.sin, zr2)
	second[1][1] = intervalMul(angular.cos2, r3m)
	second[1][2] = intervalMul(angular.sinCos, r3m)
	second[2][2] = intervalMul(angular.sin2, r3m)
	var centroidal [3][3]ratInterval
	for i := range 3 {
		for j := i; j < 3; j++ {
			shift, ok := intervalQuo(intervalMul(first[i], first[j]), volume)
			if !ok {
				return MassProperties{}, fmt.Errorf("%w: revolve volume interval does not prove positive volume", ErrUnsupported)
			}
			centroidal[i][j] = intervalSub(second[i][j], shift)
			centroidal[j][i] = centroidal[i][j]
		}
	}

	rho := new(big.Rat).Mul(proofarith.FloatRat(density.Mag()), proofarith.FloatRat(density.Unit().Factor()))
	trace := intervalAdd(intervalAdd(centroidal[0][0], centroidal[1][1]), centroidal[2][2])
	var local [3][3]ratInterval
	for i := range 3 {
		for j := range 3 {
			if i == j {
				local[i][j] = intervalScale(intervalSub(trace, centroidal[i][i]), rho)
				continue
			}
			local[i][j] = intervalScale(centroidal[i][j], new(big.Rat).Neg(rho))
		}
	}
	world := rotateTensorInterval(rotation, local)
	defect := orthonormalityDefect(rotation)
	widen := new(big.Rat).Mul(big.NewRat(3, 1), defect)
	widen.Mul(widen, new(big.Rat).Add(big.NewRat(2, 1), defect))
	widen.Mul(widen, tensorMagnitude(local))
	for i := range world {
		for j := range world[i] {
			world[i][j] = intervalWiden(world[i][j], widen)
		}
	}
	if err := ctx.Err(); err != nil {
		return MassProperties{}, err
	}

	result := MassProperties{Center: b.centroid}
	result.Mass, err = massIntervalReading(intervalScale(volume, rho), units.Kilogram)
	if err != nil {
		return MassProperties{}, err
	}
	if result.Mass.Bound.Base() >= result.Mass.Value.Base() {
		return MassProperties{}, fmt.Errorf("%w: revolve mass reading is not positive", ErrUnsupported)
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
	// The proof runs on the PUBLISHED readings, so every tensor a caller can
	// read inside the six bounds is positive definite, not only the rational
	// box they were rounded from.
	if !intervalPositiveDefinite(publishedTensor(result.Inertia)) {
		return MassProperties{}, fmt.Errorf("%w: inertia interval does not prove positive definiteness", ErrUnsupported)
	}
	if err := ctx.Err(); err != nil {
		return MassProperties{}, err
	}
	return result, nil
}

// publishedTensor is the interval tensor the six readings state: each held
// value widened by its own bound, both read as exact rationals.
func publishedTensor(reading InertiaReading) [3][3]ratInterval {
	entry := func(m Measurement) ratInterval {
		return intervalWiden(pointInterval(proofarith.FloatRat(m.Value.Base())), proofarith.FloatRat(m.Bound.Base()))
	}
	xy, xz, yz := entry(reading.XY), entry(reading.XZ), entry(reading.YZ)
	return [3][3]ratInterval{
		{entry(reading.XX), xy, xz},
		{xy, entry(reading.YY), yz},
		{xz, yz, entry(reading.ZZ)},
	}
}

// revolveRotation is the exact rational matrix taking the local basis
// (w, e0, e1) to world directions, column k the image of local axis k: the
// placement basis times w = dU·U + dV·V, e0 = dU·V − dV·U and e1 = w × e0,
// every factor read from the held floats (revolvePayload.basis states the same
// basis in floats). Its departure from orthonormality is charged by the
// caller.
func revolveRotation(rp revolvePayload) ([3][3]*big.Rat, error) {
	vec := func(v r3.Vec) ([3]*big.Rat, bool) {
		out := [3]*big.Rat{proofarith.FloatRat(v.X), proofarith.FloatRat(v.Y), proofarith.FloatRat(v.Z)}
		return out, out[0] != nil && out[1] != nil && out[2] != nil
	}
	u, okU := vec(rp.frame.U())
	v, okV := vec(rp.frame.V())
	dU, dV := proofarith.FloatRat(rp.ax.dU), proofarith.FloatRat(rp.ax.dV)
	basis := rp.xform.Basis()
	ex, okX := vec(basis.EX)
	ey, okY := vec(basis.EY)
	ez, okZ := vec(basis.EZ)
	if !okU || !okV || dU == nil || dV == nil || !okX || !okY || !okZ {
		return [3][3]*big.Rat{}, fmt.Errorf("%w: revolve orientation is not finite", ErrNotFinite)
	}
	var w, e0, e1 [3]*big.Rat
	for i := range 3 {
		w[i] = ratAdd(ratMul(u[i], dU), ratMul(v[i], dV))
		e0[i] = new(big.Rat).Sub(ratMul(v[i], dU), ratMul(u[i], dV))
	}
	for i := range 3 {
		j, k := (i+1)%3, (i+2)%3
		e1[i] = new(big.Rat).Sub(ratMul(w[j], e0[k]), ratMul(w[k], e0[j]))
	}
	placement := [3][3]*big.Rat{ex, ey, ez}
	var out [3][3]*big.Rat
	for i := range 3 {
		for k, local := range [3][3]*big.Rat{w, e0, e1} {
			sum := new(big.Rat)
			for l := range 3 {
				sum.Add(sum, ratMul(placement[l][i], local[l]))
			}
			out[i][k] = sum
		}
	}
	return out, nil
}

// revolveSectionMoments returns the recorded section's plane-origin moments
// m[p][q] = ∫u^p·v^q dA for p + q ≤ 3 as rational enclosures. Orders up to
// two are the region's exact rationals where it has them and otherwise its
// published values widened by their proven bounds, as prismMassProperties
// reads them; the third order is the engine's own enclosure.
func revolveSectionMoments(ig regionIntegrals) ([4][4]ratInterval, error) {
	var m [4][4]ratInterval
	slots := [6]*ratInterval{&m[0][0], &m[1][0], &m[0][1], &m[2][0], &m[1][1], &m[0][2]}
	if !ig.exactDead && ig.exact.complete() {
		for i, value := range ig.exact.fields() {
			*slots[i] = pointInterval(value)
		}
	} else {
		values := [6]boundedScalar{
			{ig.area, ig.areaBound}, {ig.mu, ig.muBound}, {ig.mv, ig.mvBound},
			{ig.muu, ig.muuBound}, {ig.muv, ig.muvBound}, {ig.mvv, ig.mvvBound},
		}
		for i, value := range values {
			iv, err := massMomentInterval(value)
			if err != nil {
				return m, err
			}
			*slots[i] = iv
		}
	}
	third, ok := ig.thirdMoments()
	if !ok {
		return m, fmt.Errorf("%w: revolve section has no third-order moment enclosure", ErrUnsupported)
	}
	m[3][0], m[2][1], m[1][2], m[0][3] = third[0], third[1], third[2], third[3]
	return m, nil
}

// revolveAxisMoments returns a reader of ∫z^a·ρ^b dA, a + b ≤ 3, over the
// section, for the exact axis ax (axisFrame.toAxis):
// z = dU·(u − aU) + dV·(v − aV) and ρ = dU·(v − aV) − dV·(u − aU). Each is an
// affine form in (u, v) with exact rational coefficients, so z^a·ρ^b expands
// into a polynomial whose coefficients weight the plane-origin moments.
func revolveAxisMoments(m [4][4]ratInterval, ax axisFrame) func(a, b int) ratInterval {
	aU, aV := proofarith.FloatRat(ax.aU), proofarith.FloatRat(ax.aV)
	dU, dV := proofarith.FloatRat(ax.dU), proofarith.FloatRat(ax.dV)
	zForm := [3]*big.Rat{dU, dV, new(big.Rat).Neg(ratAdd(ratMul(dU, aU), ratMul(dV, aV)))}
	rhoForm := [3]*big.Rat{new(big.Rat).Neg(dV), dU, new(big.Rat).Sub(ratMul(dV, aU), ratMul(dU, aV))}
	return func(a, b int) ratInterval {
		var poly [4][4]*big.Rat
		poly[0][0] = big.NewRat(1, 1)
		for k := range a + b {
			form := zForm
			if k >= a {
				form = rhoForm
			}
			var next [4][4]*big.Rat
			add := func(i, j int, value *big.Rat) {
				if next[i][j] == nil {
					next[i][j] = new(big.Rat)
				}
				next[i][j].Add(next[i][j], value)
			}
			for i := range 4 {
				for j := range 4 - i {
					c := poly[i][j]
					if c == nil {
						continue
					}
					add(i+1, j, ratMul(form[0], c))
					add(i, j+1, ratMul(form[1], c))
					add(i, j, ratMul(form[2], c))
				}
			}
			poly = next
		}
		sum := pointInterval(new(big.Rat))
		for i := range 4 {
			for j := range 4 - i {
				if poly[i][j] != nil && poly[i][j].Sign() != 0 {
					sum = intervalAdd(sum, intervalScale(m[i][j], poly[i][j]))
				}
			}
		}
		return sum
	}
}

// revolveMassAngular is the sweep's angular factors over [φ0, φ1]: its width
// ∫dφ, cos ∫cos φ, sin ∫sin φ, and the three quadratic factors.
type revolveMassAngular struct {
	width, cos, sin    ratInterval
	cos2, sinCos, sin2 ratInterval
}

// revolveAngularFactors encloses the factors from the payload's own sweep
// denotation: the width is den.widthInterval (2π's enclosure for a full
// turn), and each end's sine and cosine come from angleDenotation.sinCosFor,
// the certified enclosure the partial-sweep centroid already reads. With
// s, c the end values,
//
//	∫cos φ = s1 − s0        ∫sin φ = c0 − c1
//	∫cos² φ = Δφ/2 + (s1c1 − s0c0)/2
//	∫sin² φ = Δφ/2 − (s1c1 − s0c0)/2
//	∫sin φ cos φ = (s1² − s0²)/2
func revolveAngularFactors(rp revolvePayload) (revolveMassAngular, bool) {
	width, ok := rp.den.widthInterval()
	if !ok {
		return revolveMassAngular{}, false
	}
	s0, c0, ok0 := rp.den.phi0.sinCosFor(rp.phi0)
	s1, c1, ok1 := rp.den.phi1.sinCosFor(rp.phi1)
	if !ok0 || !ok1 {
		return revolveMassAngular{}, false
	}
	half := big.NewRat(1, 2)
	halfWidth := intervalScale(width, half)
	doubleAngle := intervalScale(intervalSub(intervalMul(s1, c1), intervalMul(s0, c0)), half)
	return revolveMassAngular{
		width:  width,
		cos:    intervalSub(s1, s0),
		sin:    intervalSub(c0, c1),
		cos2:   intervalAdd(halfWidth, doubleAngle),
		sin2:   intervalSub(halfWidth, doubleAngle),
		sinCos: intervalScale(intervalSub(intervalMul(s1, s1), intervalMul(s0, s0)), half),
	}, true
}

// intervalPositiveDefinite proves every symmetric matrix in the interval
// tensor positive definite by Sylvester's criterion: each leading principal
// minor, evaluated in interval arithmetic, has a strictly positive lower end.
// The minors are polynomials in the entries, so their interval evaluation
// encloses the minor of every member; a positive lower end then holds for
// every member at once.
func intervalPositiveDefinite(m [3][3]ratInterval) bool {
	if m[0][0].lo.Sign() <= 0 {
		return false
	}
	minor2 := intervalSub(intervalMul(m[0][0], m[1][1]), intervalMul(m[0][1], m[1][0]))
	if minor2.lo.Sign() <= 0 {
		return false
	}
	cofactor := func(a, b, c, d ratInterval) ratInterval {
		return intervalSub(intervalMul(a, b), intervalMul(c, d))
	}
	det := intervalAdd(
		intervalSub(
			intervalMul(m[0][0], cofactor(m[1][1], m[2][2], m[1][2], m[2][1])),
			intervalMul(m[0][1], cofactor(m[1][0], m[2][2], m[1][2], m[2][0])),
		),
		intervalMul(m[0][2], cofactor(m[1][0], m[2][1], m[1][1], m[2][0])),
	)
	return det.lo.Sign() > 0
}
