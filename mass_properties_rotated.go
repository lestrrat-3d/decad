package decad

import (
	"context"
	"fmt"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
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

// rotatedPrismMassProperties integrates pp's admitted section moments over
// its axial interval, charges any recorded displacement, and rotates the
// centroidal tensor into world axes. The caller has already admitted pp as a
// solid prism; this path takes every frame and placement basis, cardinal ones
// included.
func rotatedPrismMassProperties(ctx context.Context, pp prismPayload, center VecMeasurement, density units.Value) (MassProperties, error) {
	if !nonNegativeFinite(pp.sectionDelta) || !nonNegativeFinite(pp.z0Delta) || !nonNegativeFinite(pp.z1Delta) {
		return MassProperties{}, fmt.Errorf("%w: prism displacement has no finite bound", ErrUnsupported)
	}
	section, err := prismSectionMoments(ctx, pp)
	if err != nil {
		return MassProperties{}, err
	}
	a, mu, mv := section[0], section[1], section[2]
	if a.lo.Sign() <= 0 {
		return MassProperties{}, fmt.Errorf("%w: section area interval does not prove positive volume", ErrUnsupported)
	}
	z0, z1 := proofarith.FloatRat(pp.z0), proofarith.FloatRat(pp.z1)
	if z0 == nil || z1 == nil {
		return MassProperties{}, fmt.Errorf("%w: prism levels are not finite", ErrNotFinite)
	}
	h := new(big.Rat).Sub(z1, z0)
	if h.Sign() <= 0 {
		return MassProperties{}, fmt.Errorf("%w: axial interval does not prove positive volume", ErrUnsupported)
	}

	// Volume moments about (0, 0, zm). The axial interval is symmetric about
	// zm, so every first or mixed moment in z vanishes exactly.
	zero := pointInterval(new(big.Rat))
	volume := intervalScale(a, h)
	first := [3]ratInterval{intervalScale(mu, h), intervalScale(mv, h), zero}
	h3Over12 := new(big.Rat).Quo(new(big.Rat).Mul(h, new(big.Rat).Mul(h, h)), big.NewRat(12, 1))
	second := [3][3]ratInterval{
		{intervalScale(section[3], h), intervalScale(section[4], h), zero},
		{intervalScale(section[4], h), intervalScale(section[5], h), zero},
		{zero, zero, intervalScale(a, h3Over12)},
	}
	if pp.sectionDelta > 0 || pp.z0Delta > 0 || pp.z1Delta > 0 {
		e, r, err := prismOccupiedVolumeError(ctx, pp, a, h)
		if err != nil {
			return MassProperties{}, err
		}
		re := new(big.Rat).Mul(r, e)
		r2e := new(big.Rat).Mul(r, re)
		volume = intervalWiden(volume, e)
		for i := range first {
			first[i] = intervalWiden(first[i], re)
			for j := range second[i] {
				second[i][j] = intervalWiden(second[i][j], r2e)
			}
		}
	}
	if volume.lo.Sign() <= 0 {
		return MassProperties{}, fmt.Errorf("%w: volume interval does not prove positive volume", ErrUnsupported)
	}
	if err := ctx.Err(); err != nil {
		return MassProperties{}, err
	}

	// Centroidal second moment S = Q - P Pᵀ/V and the local inertia
	// ρ(trace(S)δ - S), all from the one V, P, Q enclosure (dynamic-mass §3).
	var central [3][3]ratInterval
	for i := range central {
		for j := range central[i] {
			shift, _ := intervalQuo(intervalMul(first[i], first[j]), volume)
			central[i][j] = intervalSub(second[i][j], shift)
		}
	}
	trace := intervalAdd(intervalAdd(central[0][0], central[1][1]), central[2][2])
	rho := new(big.Rat).Mul(proofarith.FloatRat(density.Mag()), proofarith.FloatRat(density.Unit().Factor()))
	var local [3][3]ratInterval
	for i := range local {
		for j := range local[i] {
			term := intervalNeg(central[i][j])
			if i == j {
				term = intervalSub(trace, central[i][j])
			}
			local[i][j] = intervalScale(term, rho)
		}
	}
	// Every tensor in the local box has its smallest eigenvalue at or above
	// the Gershgorin lower bound; a rotation keeps the eigenvalues.
	eigenLower := gershgorinLower(local)
	if eigenLower.Sign() <= 0 {
		return MassProperties{}, fmt.Errorf("%w: inertia interval does not prove positive definiteness", ErrUnsupported)
	}

	basis, err := prismRotation(pp)
	if err != nil {
		return MassProperties{}, err
	}
	world := rotateTensorInterval(basis, local)
	widen := new(big.Rat).Mul(big.NewRat(3, 1), orthonormalityDefect(basis))
	widen.Mul(widen, new(big.Rat).Add(big.NewRat(2, 1), orthonormalityDefect(basis)))
	widen.Mul(widen, tensorMagnitude(local))
	for i := range world {
		for j := range world[i] {
			world[i][j] = intervalWiden(world[i][j], widen)
		}
	}

	result := MassProperties{Center: center}
	result.Mass, err = massIntervalReading(intervalScale(volume, rho), units.Kilogram)
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

// prismSectionMoments reads the section's area, first and second moments as
// rational intervals: exact when the moment engine certified every field
// exactly, else each held value widened by its own published bound.
func prismSectionMoments(ctx context.Context, pp prismPayload) ([6]ratInterval, error) {
	ig, err := pp.profile.evaluatorIntegralsContext(ctx, momentSecondOrder, nil)
	if err != nil {
		return [6]ratInterval{}, err
	}
	section := [6]ratInterval{}
	if !ig.exactDead && ig.exact.complete() {
		for i, value := range ig.exact.fields() {
			section[i] = pointInterval(value)
		}
		return section, nil
	}
	values := [6]boundedScalar{
		{ig.area, ig.areaBound}, {ig.mu, ig.muBound}, {ig.mv, ig.mvBound},
		{ig.muu, ig.muuBound}, {ig.muv, ig.muvBound}, {ig.mvv, ig.mvvBound},
	}
	for i, value := range values {
		section[i], err = massMomentInterval(value)
		if err != nil {
			return [6]ratInterval{}, err
		}
	}
	return section, nil
}

// prismOccupiedVolumeError bounds the volume of the symmetric difference
// between the recorded prism and the prism its construction denotes, and a
// radius R bounding every coordinate |u|, |v|, |z - zm| of both.
//
// A point in one prism and not the other either projects into the section's
// displacement tube, of area at most sectionDisplacementArea, over the wider
// of the two axial intervals, or projects into the recorded section and lies
// in one of the two end slabs the level displacements sweep. So
//
//	E = SDA·(h + δ0 + δ1) + A_upper·(δ0 + δ1).
//
// Every recorded boundary coordinate lies within the section envelope, and
// the denoted boundary within sectionDelta of it; the denoted levels lie
// within their own displacement of h/2 from zm.
func prismOccupiedVolumeError(ctx context.Context, pp prismPayload, area ratInterval, h *big.Rat) (*big.Rat, *big.Rat, error) {
	work := newFreeformWork()
	walks, err := resolveProfileWalks(pp.profile, work)
	if err != nil {
		return nil, nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, nil, err
	}
	count := 0
	perimeter := 0.0
	for _, loop := range append([][]segmentWalk{walks.outer}, walks.holes...) {
		for _, w := range loop {
			count++
			perimeter = absSumUpper(perimeter, w.length, w.lengthBound)
		}
	}
	coordUpper, err := profileCoordinateEnvelope(pp.profile, work, walks)
	if err != nil {
		return nil, nil, err
	}
	displaced := sectionDisplacementArea(pp.sectionDelta, count, perimeter)
	if !nonNegativeFinite(displaced) || !nonNegativeFinite(coordUpper) {
		return nil, nil, fmt.Errorf("%w: prism displacement has no finite occupied-volume bound", ErrUnsupported)
	}
	d0, d1, delta := proofarith.FloatRat(pp.z0Delta), proofarith.FloatRat(pp.z1Delta), proofarith.FloatRat(pp.sectionDelta)
	axial := new(big.Rat).Add(d0, d1)
	e := new(big.Rat).Mul(proofarith.FloatRat(displaced), new(big.Rat).Add(h, axial))
	e.Add(e, new(big.Rat).Mul(intervalAbsUpper(area), axial))

	inPlane := new(big.Rat).Add(proofarith.FloatRat(coordUpper), delta)
	alongAxis := new(big.Rat).Quo(h, big.NewRat(2, 1))
	alongAxis.Add(alongAxis, ratMax(d0, d1))
	return e, ratMax(inPlane, alongAxis), nil
}

// prismRotation is the exact rational matrix taking frame-local (u, v, n)
// directions to world directions: the placement basis times the frame axes,
// both read from the held floats. Column k is the image of local axis k.
func prismRotation(pp prismPayload) ([3][3]*big.Rat, error) {
	basis := pp.xform.Basis()
	placement := [3]r3.Vec{basis.EX, basis.EY, basis.EZ}
	frame := [3]r3.Vec{pp.frame.U(), pp.frame.V(), pp.frame.N()}
	var out [3][3]*big.Rat
	for i := range out {
		for k := range out[i] {
			sum := new(big.Rat)
			for l := range placement {
				entry := proofarith.FloatRat(vecComponent(placement[l], i))
				axis := proofarith.FloatRat(vecComponent(frame[k], l))
				if entry == nil || axis == nil {
					return out, fmt.Errorf("%w: prism orientation is not finite", ErrNotFinite)
				}
				sum.Add(sum, entry.Mul(entry, axis))
			}
			out[i][k] = sum
		}
	}
	return out, nil
}

// rotateTensorInterval forms M T Mᵀ entry by entry over exact rational
// coefficients, so every tensor inside the local box maps inside the result.
func rotateTensorInterval(m [3][3]*big.Rat, local [3][3]ratInterval) [3][3]ratInterval {
	var out [3][3]ratInterval
	for i := range out {
		for j := range out[i] {
			sum := pointInterval(new(big.Rat))
			for k := range local {
				for l := range local[k] {
					coefficient := new(big.Rat).Mul(m[i][k], m[j][l])
					sum = intervalAdd(sum, intervalScale(local[k][l], coefficient))
				}
			}
			out[i][j] = sum
		}
	}
	return out
}

// orthonormalityDefect is the entrywise absolute sum of MᵀM - I, an upper
// bound on its Frobenius norm that needs no square root.
func orthonormalityDefect(m [3][3]*big.Rat) *big.Rat {
	defect := new(big.Rat)
	for i := range m {
		for j := range m {
			dot := new(big.Rat)
			for k := range m {
				dot.Add(dot, new(big.Rat).Mul(m[k][i], m[k][j]))
			}
			if i == j {
				dot.Sub(dot, big.NewRat(1, 1))
			}
			defect.Add(defect, dot.Abs(dot))
		}
	}
	return defect
}

// tensorMagnitude is the largest magnitude any entry of the box allows.
func tensorMagnitude(t [3][3]ratInterval) *big.Rat {
	largest := new(big.Rat)
	for i := range t {
		for j := range t[i] {
			largest = ratMax(largest, intervalAbsUpper(t[i][j]))
		}
	}
	return largest
}

// gershgorinLower is a lower bound on the smallest eigenvalue of every
// symmetric tensor inside the box: each diagonal lower end minus the largest
// magnitudes its row's off-diagonal entries allow, minimized over rows.
func gershgorinLower(t [3][3]ratInterval) *big.Rat {
	var lower *big.Rat
	for i := range t {
		row := new(big.Rat).Set(t[i][i].lo)
		for j := range t[i] {
			if i != j {
				row.Sub(row, intervalAbsUpper(t[i][j]))
			}
		}
		if lower == nil || row.Cmp(lower) < 0 {
			lower = row
		}
	}
	return lower
}

func nonNegativeFinite(value float64) bool {
	return !isNonFinite(value) && value >= 0
}
