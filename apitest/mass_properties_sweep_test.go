package apitest_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The sweep mass path (docs/multibody-dynamics-design.md §8.7) is checked
// against the sum of its spans: each span's V, P and Q about the world origin
// is integrated in this file in exact rationals (a box in closed form, a
// quarter-turn sector from its separable ρ, y and θ integrals with a
// rational π accurate to 1e-79), the spans are summed, and only then is the
// centroidal tensor formed. A placed fixture rotates that tensor by the
// rigid rotation nearest the held placement basis.
//
// Legs shown to fail (each deleted in mass_properties_rotated.go or
// mass_properties_revolve.go, the fixture watched go red, then restored):
//   - The placement's orthonormality-defect widening in rigidMassProperties:
//     with it zeroed, TestMassPropertiesCompositeSweepPlaced misses a
//     reference component.
//   - The span frame's defect widening in massmoment.Rotate, on P and on Q:
//     recorded in mass_properties_sweep_internal_test.go. A held sweep frame
//     is orthonormal to within an ulp, so TestMassPropertiesCompositeSweep-
//     TurnedFrame stays green without it; that test checks the turned frame's
//     rotation, and the internal test makes each widening visible.
//   - The exact re-anchoring in massmoment.Shift: dropping its V·sᵢ·sⱼ or
//     sᵢ·Pⱼ terms turns TestMassPropertiesCompositeSweepSumsSpans red.

// massMoments is a solid's V, P and Q about the world origin, exact.
type massMoments struct {
	volume *big.Rat
	first  [3]*big.Rat
	second [3][3]*big.Rat
}

// exactBoxMoments integrates the axis-aligned box lo..hi in closed form.
func exactBoxMoments(lo, hi [3]*big.Rat) massMoments {
	power := func(axis, k int) *big.Rat { // ∫ x^k dx over the box's axis
		a := new(big.Rat).Set(big.NewRat(1, 1))
		b := new(big.Rat).Set(big.NewRat(1, 1))
		for range k + 1 {
			a.Mul(a, lo[axis])
			b.Mul(b, hi[axis])
		}
		return new(big.Rat).Quo(new(big.Rat).Sub(b, a), big.NewRat(int64(k+1), 1))
	}
	var m massMoments
	m.volume = ratProduct(power(0, 0), power(1, 0), power(2, 0))
	for i := range 3 {
		factors := [3]*big.Rat{power(0, 0), power(1, 0), power(2, 0)}
		factors[i] = power(i, 1)
		m.first[i] = ratProduct(factors[:]...)
		for j := range 3 {
			factors := [3]*big.Rat{power(0, 0), power(1, 0), power(2, 0)}
			if i == j {
				factors[i] = power(i, 2)
			} else {
				factors[i], factors[j] = power(i, 1), power(j, 1)
			}
			m.second[i][j] = ratProduct(factors[:]...)
		}
	}
	return m
}

// shifted moves the anchor from the origin to −c: the moments of the same
// solid translated by +c.
func (m massMoments) shifted(c [3]*big.Rat) massMoments {
	out := massMoments{volume: m.volume}
	for i := range 3 {
		out.first[i] = new(big.Rat).Add(m.first[i], new(big.Rat).Mul(m.volume, c[i]))
	}
	for i := range 3 {
		for j := range 3 {
			out.second[i][j] = ratSum(m.second[i][j], ratProduct(c[i], m.first[j]),
				ratProduct(m.first[i], c[j]), ratProduct(m.volume, c[i], c[j]))
		}
	}
	return out
}

// rotated carries the moments through the linear map q about the anchor.
func (m massMoments) rotated(q [3][3]*big.Rat) massMoments {
	out := massMoments{volume: m.volume}
	for i := range 3 {
		out.first[i] = new(big.Rat)
		for k := range 3 {
			out.first[i].Add(out.first[i], ratProduct(q[i][k], m.first[k]))
		}
		for j := range 3 {
			out.second[i][j] = new(big.Rat)
			for k := range 3 {
				for l := range 3 {
					out.second[i][j].Add(out.second[i][j], ratProduct(q[i][k], q[j][l], m.second[k][l]))
				}
			}
		}
	}
	return out
}

// image is m's exact image under q: rotated, then every moment scaled by
// |det q|, the factor q scales each volume element by.
func (m massMoments) image(q [3][3]*big.Rat) massMoments {
	det := new(big.Rat)
	for i := range 3 {
		j, k := (i+1)%3, (i+2)%3
		minor := new(big.Rat).Sub(ratProduct(q[1][j], q[2][k]), ratProduct(q[1][k], q[2][j]))
		det.Add(det, ratProduct(q[0][i], minor))
	}
	det.Abs(det)
	out := m.rotated(q)
	out.volume = ratProduct(out.volume, det)
	for i := range 3 {
		out.first[i] = ratProduct(out.first[i], det)
		for j := range 3 {
			out.second[i][j] = ratProduct(out.second[i][j], det)
		}
	}
	return out
}

func (m massMoments) plus(o massMoments, sign int64) massMoments {
	s := big.NewRat(sign, 1)
	out := massMoments{volume: ratSum(m.volume, ratProduct(s, o.volume))}
	for i := range 3 {
		out.first[i] = ratSum(m.first[i], ratProduct(s, o.first[i]))
		for j := range 3 {
			out.second[i][j] = ratSum(m.second[i][j], ratProduct(s, o.second[i][j]))
		}
	}
	return out
}

// inertia is the centroidal tensor ρ(trace(S)·1 − S), S = Q − P·Pᵀ/V, with
// the physical sign on its products.
func (m massMoments) inertia(rho *big.Rat) [3][3]*big.Rat {
	var s [3][3]*big.Rat
	for i := range 3 {
		for j := range 3 {
			shift := new(big.Rat).Quo(ratProduct(m.first[i], m.first[j]), m.volume)
			s[i][j] = new(big.Rat).Sub(m.second[i][j], shift)
		}
	}
	trace := ratSum(s[0][0], s[1][1], s[2][2])
	var out [3][3]*big.Rat
	for i := range 3 {
		for j := range 3 {
			out[i][j] = ratProduct(new(big.Rat).Neg(rho), s[i][j])
			if i == j {
				out[i][j] = ratProduct(rho, new(big.Rat).Sub(trace, s[i][i]))
			}
		}
	}
	return out
}

// requireMomentsReadings checks the mass, the center and all six inertia
// components of got against the exact moments carried into world axes as
// their exact image under the held map q (nil for the identity) about the
// placed center (docs/multibody-dynamics-design.md §8.1): mass ρ·|det q|·V
// and the image's inertia (affineInertia).
func requireMomentsReadings(t *testing.T, got decad.MassProperties, m massMoments, rho *big.Rat, pose r3.Transform, q *[3][3]*big.Float) {
	t.Helper()
	if q == nil {
		requireReadingCovers(t, got.Mass, ratProduct(rho, m.volume))
	} else {
		det := wideDet(*q)
		det.Abs(det)
		requireWideCovers(t, got.Mass, newWide().Mul(newWide().SetRat(ratProduct(rho, m.volume)), det))
	}
	var center [3]float64
	for i := range center {
		center[i], _ = new(big.Rat).Quo(m.first[i], m.volume).Float64()
	}
	placed := pose.Apply(r3.NewVec(center[0], center[1], center[2]))
	// The evaluator's centroid bound encloses the exact center; 1e-12 covers
	// this test's float evaluation of the center and of pose.Apply.
	slack := got.Center.Bound.Base() + 1e-12
	require.InDelta(t, placed.X, got.Center.Value.X, slack)
	require.InDelta(t, placed.Y, got.Center.Value.Y, slack)
	require.InDelta(t, placed.Z, got.Center.Value.Z, slack)

	local := m.inertia(rho)
	var image [3][3]*big.Float
	if q != nil {
		var wide [3][3]*big.Float
		for i := range 3 {
			for j := range 3 {
				wide[i][j] = newWide().SetRat(local[i][j])
			}
		}
		image = affineInertia(*q, wide, newWide().SetRat(rho))
	}
	for _, entry := range []struct {
		reading decad.Measurement
		i, j    int
	}{
		{got.Inertia.XX, 0, 0}, {got.Inertia.YY, 1, 1}, {got.Inertia.ZZ, 2, 2},
		{got.Inertia.XY, 0, 1}, {got.Inertia.XZ, 0, 2}, {got.Inertia.YZ, 1, 2},
	} {
		require.Equal(t, units.MomentOfInertia, entry.reading.Value.Kind())
		want := newWide().SetRat(local[entry.i][entry.j])
		if q != nil {
			want = image[entry.i][entry.j]
		}
		deviation := newWide().Sub(want, newWide().SetFloat64(entry.reading.Value.Base()))
		deviation.Abs(deviation)
		require.LessOrEqual(t, deviation.Cmp(newWide().SetFloat64(entry.reading.Bound.Base())), 0,
			"component (%d,%d) = %s ± %s misses %s", entry.i, entry.j,
			entry.reading.Value, entry.reading.Bound, want.Text('g', 20))
		// A reading that covers its value only because its bound is wide
		// would hide a missing term. 1e-9 of the tensor scale is far above
		// float rounding here and far below any dropped term.
		scale, _ := local[0][0].Float64()
		require.Less(t, entry.reading.Bound.Base(), 1e-9*scale)
	}
}

func ratSum(values ...*big.Rat) *big.Rat {
	out := new(big.Rat)
	for _, value := range values {
		out.Add(out, value)
	}
	return out
}

func ratProduct(values ...*big.Rat) *big.Rat {
	out := big.NewRat(1, 1)
	for _, value := range values {
		out.Mul(out, value)
	}
	return out
}

func ratVec(x, y, z int64) [3]*big.Rat {
	return [3]*big.Rat{big.NewRat(x, 1), big.NewRat(y, 1), big.NewRat(z, 1)}
}

// compositeSweep sweeps the square [-1, 1]² of the XY plane along a straight
// rise to z = 4, a quarter turn of radius 5 about the line x = 5, z = 4 (its
// points a Pythagorean triple from the center), and a straight run along +X:
// three spans, two straight and one arc.
func compositeSweep(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(-1, -1, 1, 1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	path, err := decad.NewPath(r3.NewVec(0, 0, 0),
		decad.LineTo{End: r3.NewVec(0, 0, 4)},
		decad.ArcThrough{Through: r3.NewVec(2, 0, 8), End: r3.NewVec(5, 0, 9)},
		decad.LineTo{End: r3.NewVec(15, 0, 9)},
	)
	require.NoError(t, err)
	body, err := doc.Sweep(t.Context(), s, s.Profiles()[0], path)
	require.NoError(t, err)
	return body
}

// compositeSweepSpans is the sum of compositeSweep's three spans about the
// world origin.
func compositeSweepSpans(t *testing.T) massMoments {
	t.Helper()
	rise := exactBoxMoments(ratVec(-1, -1, 0), ratVec(1, 1, 4))
	run := exactBoxMoments(ratVec(5, -1, 8), ratVec(15, 1, 10))

	// The arc span about its center c = (5, 0, 4): a section point at radius
	// ρ ∈ [4, 6] from the axis, height y ∈ [-1, 1] along it, swept through
	// θ ∈ [0, π/2] sits at (−ρ·cos θ, y, ρ·sin θ), volume element ρ dρ dy dθ.
	pi := revolvePi(t)
	radial := func(k int64) *big.Rat { // ∫ ρ^k·ρ dρ over [4, 6]
		six, four := big.NewRat(1, 1), big.NewRat(1, 1)
		for range k + 2 {
			six.Mul(six, big.NewRat(6, 1))
			four.Mul(four, big.NewRat(4, 1))
		}
		return new(big.Rat).Quo(new(big.Rat).Sub(six, four), big.NewRat(k+2, 1))
	}
	height := [3]*big.Rat{big.NewRat(2, 1), new(big.Rat), big.NewRat(2, 3)} // ∫ y^k dy
	halfPi := new(big.Rat).Quo(pi, big.NewRat(2, 1))
	quarterPi := new(big.Rat).Quo(pi, big.NewRat(4, 1))
	one, half := big.NewRat(1, 1), big.NewRat(1, 2)
	minus := func(x *big.Rat) *big.Rat { return new(big.Rat).Neg(x) }
	arc := massMoments{volume: ratProduct(halfPi, radial(0), height[0])}
	// ∫cos θ = ∫sin θ = 1, ∫cos² θ = ∫sin² θ = π/4, ∫sin θ cos θ = 1/2.
	arc.first = [3]*big.Rat{
		ratProduct(minus(one), radial(1), height[0]),
		ratProduct(halfPi, radial(0), height[1]),
		ratProduct(one, radial(1), height[0]),
	}
	xx := ratProduct(quarterPi, radial(2), height[0])
	xy := ratProduct(minus(one), radial(1), height[1])
	xz := ratProduct(minus(half), radial(2), height[0])
	yy := ratProduct(halfPi, radial(0), height[2])
	yz := ratProduct(one, radial(1), height[1])
	zz := ratProduct(quarterPi, radial(2), height[0])
	arc.second = [3][3]*big.Rat{{xx, xy, xz}, {xy, yy, yz}, {xz, yz, zz}}
	arc = arc.shifted(ratVec(5, 0, 4))
	return rise.plus(arc, 1).plus(run, 1)
}

// TestMassPropertiesCompositeSweepSumsSpans checks a three-span composite
// sweep against the sum of its spans' exact moments.
func TestMassPropertiesCompositeSweepSumsSpans(t *testing.T) {
	doc := decad.New()
	body := compositeSweep(t, doc)
	density := units.KilogramsPerCubicMillimeter(1.0 / 1024)
	got, err := body.MassProperties(t.Context(), density)
	require.NoError(t, err)
	rho := new(big.Rat).SetFloat64(density.Base())
	requireMomentsReadings(t, got, compositeSweepSpans(t), rho, r3.Identity(), nil)
	// The arc couples X and Z, so XZ carries a nonzero product of inertia
	// that a per-span centroidal sum without the shared anchor would miss.
	require.Greater(t, -got.Inertia.XZ.Value.Base(), 0.5)

	again, err := body.MassProperties(t.Context(), density)
	require.NoError(t, err)
	require.Equal(t, got, again)
}

// TestMassPropertiesCompositeSweepPlaced places the composite sweep by a
// composed rotation, whose held basis is not exactly orthonormal, and checks
// the readings against the exact image under that basis.
func TestMassPropertiesCompositeSweepPlaced(t *testing.T) {
	doc := decad.New()
	body := compositeSweep(t, doc)
	// The composite replay certifies its span joins under this turn about X
	// through the origin; it refuses several other turns of this path.
	pose := composedRotation(t, r3.Vec{}, r3.NewVec(1, 0, 0), 30, 8)
	placed, err := body.PlacedCopy(t.Context(), pose)
	require.NoError(t, err)
	density := units.KilogramsPerCubicMillimeter(1.0 / 1024)
	got, err := placed.MassProperties(t.Context(), density)
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	q := heldRotation(pose, frame)
	rho := new(big.Rat).SetFloat64(density.Base())
	requireMomentsReadings(t, got, compositeSweepSpans(t), rho, pose, &q)
}

// TestMassPropertiesCompositeSweepTurnedFrame sweeps an 8×2 section whose
// sketch axes are turned in plane by atan(4/3), so the held span frames are
// orthonormal only to rounding, along two collinear straight spans. Each
// span's expected moments are its frame-local box's exact image under the
// held frame the span records, |det F|·F·P and |det F|·F·Q·Fᵀ, carried to the
// span's own origin and summed before the centroidal tensor is formed.
func TestMassPropertiesCompositeSweepTurnedFrame(t *testing.T) {
	turned, err := r3.NewFrame(r3.Vec{}, r3.NewVec(0.6, 0.8, 0), r3.NewVec(-0.8, 0.6, 0))
	require.NoError(t, err)
	w := sketch.NewWorld()
	plane, err := w.CreatePlaneFromFrame(turned)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	rect := s.CreateRectangle(-4, -1, 4, 1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	held, err := s.Plane().Frame()
	require.NoError(t, err)
	path, err := decad.NewPath(r3.Vec{},
		decad.LineTo{End: r3.NewVec(0, 0, 4)},
		decad.LineTo{End: r3.NewVec(0, 0, 10)},
	)
	require.NoError(t, err)
	body, err := decad.New().Sweep(t.Context(), s, s.Profiles()[0], path)
	require.NoError(t, err)
	density := units.KilogramsPerCubicMillimeter(1.0 / 1024)
	got, err := body.MassProperties(t.Context(), density)
	require.NoError(t, err)

	// The span records the plane's axes normalized once more, as Sweep does.
	recorded, err := r3.NewFrame(held.Origin(), held.U(), held.V())
	require.NoError(t, err)
	wide := heldRotation(r3.Identity(), recorded)
	var q [3][3]*big.Rat
	for i := range q {
		for j := range q[i] {
			q[i][j], _ = wide[i][j].Rat(nil)
		}
	}
	lower := exactBoxMoments(ratVec(-4, -1, 0), ratVec(4, 1, 4)).image(q)
	upper := exactBoxMoments(ratVec(-4, -1, 0), ratVec(4, 1, 6)).image(q).shifted(ratVec(0, 0, 4))
	rho := new(big.Rat).SetFloat64(density.Base())
	requireMomentsReadings(t, got, lower.plus(upper, 1), rho, r3.Identity(), nil)
	// The turned section couples X and Y.
	require.Less(t, got.Inertia.XY.Value.Base(), -0.1)
}

// TestMassPropertiesOneSpanSweeps checks that a one-span sweep reads as the
// analytic reduction it is: a straight span as its prism, an arc span as its
// partial revolve.
func TestMassPropertiesOneSpanSweeps(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(-1, -1, 1, 1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	density := units.KilogramsPerCubicMillimeter(1.0 / 1024)
	rho := new(big.Rat).SetFloat64(density.Base())

	straight, err := decad.NewPath(r3.NewVec(0, 0, 0), decad.LineTo{End: r3.NewVec(0, 0, 4)})
	require.NoError(t, err)
	body, err := decad.New().Sweep(t.Context(), s, s.Profiles()[0], straight)
	require.NoError(t, err)
	got, err := body.MassProperties(t.Context(), density)
	require.NoError(t, err)
	requireMomentsReadings(t, got, exactBoxMoments(ratVec(-1, -1, 0), ratVec(1, 1, 4)), rho, r3.Identity(), nil)

	arc, err := decad.NewPath(r3.NewVec(0, 0, 0),
		decad.ArcThrough{Through: r3.NewVec(2, 0, 4), End: r3.NewVec(5, 0, 5)})
	require.NoError(t, err)
	body, err = decad.New().Sweep(t.Context(), s, s.Profiles()[0], arc)
	require.NoError(t, err)
	got, err = body.MassProperties(t.Context(), density)
	require.NoError(t, err)
	// The same quarter turn as compositeSweep's arc span, centered at
	// (5, 0, 0): the composite's moments less its two boxes, moved down by 4.
	spans := compositeSweepSpans(t).
		plus(exactBoxMoments(ratVec(-1, -1, 0), ratVec(1, 1, 4)), -1).
		plus(exactBoxMoments(ratVec(5, -1, 8), ratVec(15, 1, 10)), -1).
		shifted(ratVec(0, 0, -4))
	requireMomentsReadings(t, got, spans, rho, r3.Identity(), nil)
}
