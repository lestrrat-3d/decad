package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

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
	moments, err := revolveVolumeMoments(ctx, rp)
	if err != nil {
		return MassProperties{}, err
	}
	rotation, err := revolveRotation(rp)
	if err != nil {
		return MassProperties{}, err
	}
	return rigidMassProperties(ctx, b.centroid, moments, rotation, density)
}

// revolveVolumeMoments integrates the revolve's V, P and Q about the axis
// anchor a3 in the local basis (w, e0, e1), refusing every term it does not
// charge.
func revolveVolumeMoments(ctx context.Context, rp revolvePayload) (volumeMoments, error) {
	if rp.sectionDelta != 0 {
		return volumeMoments{}, fmt.Errorf("%w: revolve section carries a displacement the mass path does not charge", ErrUnsupported)
	}
	ax := rp.ax
	if ax.aUBound != 0 || ax.aVBound != 0 || ax.dUBound != 0 || ax.dVBound != 0 {
		return volumeMoments{}, fmt.Errorf("%w: revolve axis is not exact", ErrUnsupported)
	}
	if ax.radialAdmitAllow != 0 || ax.snap != (regionSnapAllow{}) {
		return volumeMoments{}, fmt.Errorf("%w: revolve axis snap is not charged by the mass path", ErrUnsupported)
	}
	if !rp.den.phi0.valid() || !rp.den.phi1.valid() {
		return volumeMoments{}, fmt.Errorf("%w: revolve sweep has no exact denotation", ErrUnsupported)
	}
	if err := ctx.Err(); err != nil {
		return volumeMoments{}, err
	}

	ig, err := rp.profile.evaluatorIntegralsContext(ctx, momentThirdOrder, nil)
	if err != nil {
		return volumeMoments{}, err
	}
	plane, err := revolveSectionMoments(ig)
	if err != nil {
		return volumeMoments{}, err
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
		return volumeMoments{}, fmt.Errorf("%w: revolve sweep has no certified angular factors", ErrUnsupported)
	}
	if err := ctx.Err(); err != nil {
		return volumeMoments{}, err
	}

	volume := proofbound.IntervalMul(angular.width, r1)
	if volume.Lo.Sign() <= 0 {
		return volumeMoments{}, fmt.Errorf("%w: revolve volume interval does not prove positive volume", ErrUnsupported)
	}
	first := [3]proofbound.RatInterval{
		proofbound.IntervalMul(angular.width, zr),
		proofbound.IntervalMul(angular.cos, r2),
		proofbound.IntervalMul(angular.sin, r2),
	}
	var second [3][3]proofbound.RatInterval
	second[0][0] = proofbound.IntervalMul(angular.width, z2r)
	second[0][1] = proofbound.IntervalMul(angular.cos, zr2)
	second[0][2] = proofbound.IntervalMul(angular.sin, zr2)
	second[1][1] = proofbound.IntervalMul(angular.cos2, r3m)
	second[1][2] = proofbound.IntervalMul(angular.sinCos, r3m)
	second[2][2] = proofbound.IntervalMul(angular.sin2, r3m)
	second[1][0], second[2][0], second[2][1] = second[0][1], second[0][2], second[1][2]
	return volumeMoments{volume: volume, first: first, second: second}, nil
}

// revolveAnchor is the axis anchor a3 = o + aU·U + aV·V before placement, the
// origin of revolveVolumeMoments' coordinates, as exact rationals read from
// the held floats.
func revolveAnchor(rp revolvePayload) ([3]*big.Rat, error) {
	aU, aV := proofarith.FloatRat(rp.ax.aU), proofarith.FloatRat(rp.ax.aV)
	if aU == nil || aV == nil {
		return [3]*big.Rat{}, fmt.Errorf("%w: revolve axis anchor is not finite", ErrNotFinite)
	}
	origin, u, v := rp.frame.Origin(), rp.frame.U(), rp.frame.V()
	var out [3]*big.Rat
	for i := range out {
		o := proofarith.FloatRat(vecComponent(origin, i))
		ui, vi := proofarith.FloatRat(vecComponent(u, i)), proofarith.FloatRat(vecComponent(v, i))
		if o == nil || ui == nil || vi == nil {
			return [3]*big.Rat{}, fmt.Errorf("%w: revolve frame is not finite", ErrNotFinite)
		}
		out[i] = proofbound.RatAdd(o, proofbound.RatMul(aU, ui), proofbound.RatMul(aV, vi))
	}
	return out, nil
}

// rigidMassProperties publishes the mass properties of a solid whose local
// moments are m and whose local axes reach world axes through the rigid
// rotation nearest the exact rational matrix rotation (its column k the
// world image of local axis k). It forms the centroidal tensor from the one
// V, P, Q enclosure, rotates it with docs/multibody-dynamics-design.md §8.1's
// orthonormality-defect widening, and proves the PUBLISHED tensor positive
// definite by its leading principal minors. center is the evaluator's own
// bounded world centroid of the same solid.
func rigidMassProperties(ctx context.Context, center VecMeasurement, m volumeMoments, rotation [3][3]*big.Rat, density units.Value) (MassProperties, error) {
	volume, first, second := m.volume, m.first, m.second
	if volume.Lo.Sign() <= 0 {
		return MassProperties{}, fmt.Errorf("%w: volume interval does not prove positive volume", ErrUnsupported)
	}
	var centroidal [3][3]proofbound.RatInterval
	for i := range 3 {
		for j := i; j < 3; j++ {
			shift, ok := survey2d.IntervalQuo(proofbound.IntervalMul(first[i], first[j]), volume)
			if !ok {
				return MassProperties{}, fmt.Errorf("%w: volume interval does not prove positive volume", ErrUnsupported)
			}
			centroidal[i][j] = proofbound.IntervalSub(second[i][j], shift)
			centroidal[j][i] = centroidal[i][j]
		}
	}

	rho := new(big.Rat).Mul(proofarith.FloatRat(density.Mag()), proofarith.FloatRat(density.Unit().Factor()))
	trace := proofbound.IntervalAdd(proofbound.IntervalAdd(centroidal[0][0], centroidal[1][1]), centroidal[2][2])
	var local [3][3]proofbound.RatInterval
	for i := range 3 {
		for j := range 3 {
			if i == j {
				local[i][j] = proofbound.IntervalScale(proofbound.IntervalSub(trace, centroidal[i][i]), rho)
				continue
			}
			local[i][j] = proofbound.IntervalScale(centroidal[i][j], new(big.Rat).Neg(rho))
		}
	}
	world := rotateTensorInterval(rotation, local)
	defect := orthonormalityDefect(rotation)
	widen := new(big.Rat).Mul(big.NewRat(3, 1), defect)
	widen.Mul(widen, new(big.Rat).Add(big.NewRat(2, 1), defect))
	widen.Mul(widen, tensorMagnitude(local))
	for i := range world {
		for j := range world[i] {
			world[i][j] = survey2d.IntervalWiden(world[i][j], widen)
		}
	}
	if err := ctx.Err(); err != nil {
		return MassProperties{}, err
	}

	result := MassProperties{Center: center}
	var err error
	result.Mass, err = massIntervalReading(proofbound.IntervalScale(volume, rho), units.Kilogram)
	if err != nil {
		return MassProperties{}, err
	}
	if result.Mass.Bound.Base() >= result.Mass.Value.Base() {
		return MassProperties{}, fmt.Errorf("%w: mass reading is not positive", ErrUnsupported)
	}
	entries := []struct {
		iv      proofbound.RatInterval
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
func publishedTensor(reading InertiaReading) [3][3]proofbound.RatInterval {
	entry := func(m Measurement) proofbound.RatInterval {
		return survey2d.IntervalWiden(proofbound.PointInterval(proofarith.FloatRat(m.Value.Base())), proofarith.FloatRat(m.Bound.Base()))
	}
	xy, xz, yz := entry(reading.XY), entry(reading.XZ), entry(reading.YZ)
	return [3][3]proofbound.RatInterval{
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
		w[i] = proofbound.RatAdd(proofbound.RatMul(u[i], dU), proofbound.RatMul(v[i], dV))
		e0[i] = new(big.Rat).Sub(proofbound.RatMul(v[i], dU), proofbound.RatMul(u[i], dV))
	}
	for i := range 3 {
		j, k := (i+1)%3, (i+2)%3
		e1[i] = new(big.Rat).Sub(proofbound.RatMul(w[j], e0[k]), proofbound.RatMul(w[k], e0[j]))
	}
	placement := [3][3]*big.Rat{ex, ey, ez}
	var out [3][3]*big.Rat
	for i := range 3 {
		for k, local := range [3][3]*big.Rat{w, e0, e1} {
			sum := new(big.Rat)
			for l := range 3 {
				sum.Add(sum, proofbound.RatMul(placement[l][i], local[l]))
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
func revolveSectionMoments(ig regionIntegrals) ([4][4]proofbound.RatInterval, error) {
	var m [4][4]proofbound.RatInterval
	slots := [6]*proofbound.RatInterval{&m[0][0], &m[1][0], &m[0][1], &m[2][0], &m[1][1], &m[0][2]}
	if !ig.exactDead && ig.exact.complete() {
		for i, value := range ig.exact.fields() {
			*slots[i] = proofbound.PointInterval(value)
		}
	} else {
		values := [6]proofbound.BoundedScalar{
			{Value: ig.area, Bound: ig.areaBound}, {Value: ig.mu, Bound: ig.muBound}, {Value: ig.mv, Bound: ig.mvBound},
			{Value: ig.muu, Bound: ig.muuBound}, {Value: ig.muv, Bound: ig.muvBound}, {Value: ig.mvv, Bound: ig.mvvBound},
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
func revolveAxisMoments(m [4][4]proofbound.RatInterval, ax axisFrame) func(a, b int) proofbound.RatInterval {
	aU, aV := proofarith.FloatRat(ax.aU), proofarith.FloatRat(ax.aV)
	dU, dV := proofarith.FloatRat(ax.dU), proofarith.FloatRat(ax.dV)
	zForm := [3]*big.Rat{dU, dV, new(big.Rat).Neg(proofbound.RatAdd(proofbound.RatMul(dU, aU), proofbound.RatMul(dV, aV)))}
	rhoForm := [3]*big.Rat{new(big.Rat).Neg(dV), dU, new(big.Rat).Sub(proofbound.RatMul(dV, aU), proofbound.RatMul(dU, aV))}
	return func(a, b int) proofbound.RatInterval {
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
					add(i+1, j, proofbound.RatMul(form[0], c))
					add(i, j+1, proofbound.RatMul(form[1], c))
					add(i, j, proofbound.RatMul(form[2], c))
				}
			}
			poly = next
		}
		sum := proofbound.PointInterval(new(big.Rat))
		for i := range 4 {
			for j := range 4 - i {
				if poly[i][j] != nil && poly[i][j].Sign() != 0 {
					sum = proofbound.IntervalAdd(sum, proofbound.IntervalScale(m[i][j], poly[i][j]))
				}
			}
		}
		return sum
	}
}

// revolveMassAngular is the sweep's angular factors over [φ0, φ1]: its width
// ∫dφ, cos ∫cos φ, sin ∫sin φ, and the three quadratic factors.
type revolveMassAngular struct {
	width, cos, sin    proofbound.RatInterval
	cos2, sinCos, sin2 proofbound.RatInterval
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
	halfWidth := proofbound.IntervalScale(width, half)
	doubleAngle := proofbound.IntervalScale(proofbound.IntervalSub(proofbound.IntervalMul(s1, c1), proofbound.IntervalMul(s0, c0)), half)
	return revolveMassAngular{
		width:  width,
		cos:    proofbound.IntervalSub(s1, s0),
		sin:    proofbound.IntervalSub(c0, c1),
		cos2:   proofbound.IntervalAdd(halfWidth, doubleAngle),
		sin2:   proofbound.IntervalSub(halfWidth, doubleAngle),
		sinCos: proofbound.IntervalScale(proofbound.IntervalSub(proofbound.IntervalMul(s1, s1), proofbound.IntervalMul(s0, s0)), half),
	}, true
}

// intervalPositiveDefinite proves every symmetric matrix in the interval
// tensor positive definite by Sylvester's criterion: each leading principal
// minor, evaluated in interval arithmetic, has a strictly positive lower end.
// The minors are polynomials in the entries, so their interval evaluation
// encloses the minor of every member; a positive lower end then holds for
// every member at once.
func intervalPositiveDefinite(m [3][3]proofbound.RatInterval) bool {
	if m[0][0].Lo.Sign() <= 0 {
		return false
	}
	minor2 := proofbound.IntervalSub(proofbound.IntervalMul(m[0][0], m[1][1]), proofbound.IntervalMul(m[0][1], m[1][0]))
	if minor2.Lo.Sign() <= 0 {
		return false
	}
	cofactor := func(a, b, c, d proofbound.RatInterval) proofbound.RatInterval {
		return proofbound.IntervalSub(proofbound.IntervalMul(a, b), proofbound.IntervalMul(c, d))
	}
	det := proofbound.IntervalAdd(
		proofbound.IntervalSub(
			proofbound.IntervalMul(m[0][0], cofactor(m[1][1], m[2][2], m[1][2], m[2][1])),
			proofbound.IntervalMul(m[0][1], cofactor(m[1][0], m[2][2], m[1][2], m[2][0])),
		),
		proofbound.IntervalMul(m[0][2], cofactor(m[1][0], m[2][1], m[1][1], m[2][0])),
	)
	return det.Lo.Sign() > 0
}
