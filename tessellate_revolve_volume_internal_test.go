package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// revolveAxisU is the sketch plane's own u axis, the axis every body below
// revolves about.
var revolveAxisU = SketchLine{Start: Point2{U: 0, V: 0}, End: Point2{U: 1, V: 0}}

// internalCylinderBody is the 10 x 8 rectangle clear of the axis: a full turn
// sweeps the cylinder of radius 8 and height 10, whose generator is STRAIGHT,
// so its Mmeridian term is exactly zero.
func internalCylinderBody(t *testing.T) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 10, 8)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := New().Revolve(s, s.Profiles()[0], revolveAxisU, FullRevolution{})
	require.NoError(t, err)
	return body
}

// internalSphereBody is the half disk of radius 5 about (5, 0): a full turn
// sweeps the sphere of radius 5, whose generator is CIRCULAR, so Mmeridian
// carries a real charge.
func internalSphereBody(t *testing.T) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a := s.CreatePoint(0, 0)
	s.Fix(a)
	b := s.CreatePoint(10, 0)
	c := s.CreatePoint(5, 0)
	s.CreateArc(c, a, b)
	s.CreateLine(b, a)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := New().Revolve(s, s.Profiles()[0], revolveAxisU, FullRevolution{})
	require.NoError(t, err)
	return body
}

// internalMeshVolume is the mesh's own enclosed volume by the divergence sum
// over its facets.
func internalMeshVolume(m *Mesh) float64 {
	total := 0.0
	for _, tri := range m.triangles {
		a, b, c := m.vertices[tri[0]], m.vertices[tri[1]], m.vertices[tri[2]]
		total += a.Dot(b.Cross(c)) / 6
	}
	return math.Abs(total)
}

// Every reference below is a DENSE NUMERIC evaluation of the very integral
// docs/tessellation-design.md §11 states, built from that paragraph's own H
// rather than from the closed form the implementation derives from it. It is a
// falsifier and nothing else: a published bound BELOW it disproves the bound,
// and a bound above it proves nothing on its own. The upper assertions beside
// each one pin the reading's tightness, so a bound that grew wildly loose would
// also fail rather than pass silently.

// homotopyPoint evaluates docs/tessellation-design.md §11's H(λ, t, u) in the
// axis basis (w, e0, e1) taken as the world axes, with the meridian chord
// running from (z0, ρ0) to (z1, ρ1) and the angular interval [0, dφ].
func homotopyPoint(z0, rho0, z1, rho1, dphi, lam, t, u float64) r3.Vec {
	z := z0 + t*(z1-z0)
	rho := rho0 + t*(rho1-rho0)
	arc := r3.Vec{Y: math.Cos(u * dphi), Z: math.Sin(u * dphi)}
	chord := r3.Vec{Y: (1 - u) + u*math.Cos(dphi), Z: u * math.Sin(dphi)}
	dir := arc.Scale(1 - lam).Add(chord.Scale(lam))
	return r3.Vec{X: z}.Add(dir.Scale(rho))
}

// tripleIntegralReference is a midpoint-rule reading of
// ∫|∂H/∂λ · (∂H/∂t × ∂H/∂u)| over the unit cube, with every partial taken by a
// central difference of homotopyPoint itself.
func tripleIntegralReference(z0, rho0, z1, rho1, dphi float64, n int) float64 {
	const h = 1e-6
	total := 0.0
	for i := range n {
		for j := range n {
			for k := range n {
				lam := (float64(i) + 0.5) / float64(n)
				t := (float64(j) + 0.5) / float64(n)
				u := (float64(k) + 0.5) / float64(n)
				dLam := homotopyPoint(z0, rho0, z1, rho1, dphi, lam+h, t, u).
					Sub(homotopyPoint(z0, rho0, z1, rho1, dphi, lam-h, t, u)).Scale(1 / (2 * h))
				dT := homotopyPoint(z0, rho0, z1, rho1, dphi, lam, t+h, u).
					Sub(homotopyPoint(z0, rho0, z1, rho1, dphi, lam, t-h, u)).Scale(1 / (2 * h))
				dU := homotopyPoint(z0, rho0, z1, rho1, dphi, lam, t, u+h).
					Sub(homotopyPoint(z0, rho0, z1, rho1, dphi, lam, t, u-h)).Scale(1 / (2 * h))
				total += math.Abs(dLam.Dot(dT.Cross(dU)))
			}
		}
	}
	return total / float64(n*n*n)
}

func TestRevolveAngularHomotopyFactorEnclosesTheAngularIntegral(t *testing.T) {
	t.Parallel()
	// The angular factor alone is the triple integral of a cell whose meridian
	// chord has |z'| = 1 and ρ ≡ 1, since ∫ρ² dt is then 1.
	for _, dphi := range []float64{2 * math.Pi / 3, 2 * math.Pi / 8, 2 * math.Pi / 40, 1.0, 3.0, 5.5} {
		g, err := revolveAngularHomotopyFactor(pointInterval(floatRat(dphi)))
		require.NoError(t, err)
		got := ratFloatUp(g)
		want := tripleIntegralReference(0, 1, 1, 1, dphi, 60)
		require.Positive(t, want)
		require.GreaterOrEqual(t, got, want, `dφ=%v: the published factor sits below the integral it must bound`, dphi)
		require.LessOrEqual(t, got, want*1.1, `dφ=%v: the published factor is far looser than the integral`, dphi)
	}
}

func TestRevolveAngularHomotopyFactorFollowsTheCubeOfTheStep(t *testing.T) {
	t.Parallel()
	// A small angular step's factor is dφ³/12 to leading order — which is what
	// makes Σ Icell reproduce a chorded cylinder's own volume deficit — so
	// halving the step must divide the factor by very nearly eight.
	coarse, err := revolveAngularHomotopyFactor(pointInterval(floatRat(2 * math.Pi / 40)))
	require.NoError(t, err)
	fine, err := revolveAngularHomotopyFactor(pointInterval(floatRat(2 * math.Pi / 80)))
	require.NoError(t, err)
	ratio := ratFloatUp(coarse) / ratFloatUp(fine)
	require.InDelta(t, 8.0, ratio, 0.05)

	step := 2 * math.Pi / 64
	closed := step * step * step / 12
	got := ratFloatUp(mustAngularFactor(t, step))
	require.GreaterOrEqual(t, got, closed)
	require.LessOrEqual(t, got, closed*1.05)
}

func mustAngularFactor(t *testing.T, dphi float64) *big.Rat {
	t.Helper()
	g, err := revolveAngularHomotopyFactor(pointInterval(floatRat(dphi)))
	require.NoError(t, err)
	return g
}

func TestRevolveAngularHomotopyFactorRefusesAnUnenclosableStep(t *testing.T) {
	t.Parallel()
	// A reversed enclosure states no angle at all, and §11's integral may not
	// be answered from one: the mesh refuses rather than publishing a figure it
	// cannot stand behind.
	_, err := revolveAngularHomotopyFactor(interval(big.NewRat(1, 1), new(big.Rat)))
	require.ErrorIs(t, err, ErrUnsupported)
}

func TestRevolveCellSweptVolumeBoundsTheCellIntegral(t *testing.T) {
	t.Parallel()
	cases := []struct {
		name               string
		z0, rho0, z1, rho1 float64
		dphi               float64
		tightness          float64
	}{
		{name: "cylinder wall", z0: 0, rho0: 8, z1: 10, rho1: 8, dphi: 2 * math.Pi / 40, tightness: 1.1},
		{name: "cone wall", z0: 0, rho0: 2, z1: 6, rho1: 9, dphi: 2 * math.Pi / 24, tightness: 1.1},
		{name: "pole fan", z0: 0, rho0: 0, z1: 4, rho1: 7, dphi: 2 * math.Pi / 16, tightness: 1.1},
		{name: "coarsest full turn", z0: 0, rho0: 5, z1: 5, rho1: 5, dphi: 2 * math.Pi / 3, tightness: 1.1},
		{name: "reversed meridian", z0: 9, rho0: 6, z1: 1, rho1: 3, dphi: 2 * math.Pi / 12, tightness: 1.1},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			g := mustAngularFactor(t, tc.dphi)
			lo := revMeridian{zIv: pointInterval(floatRat(tc.z0)), rhoIv: pointInterval(floatRat(tc.rho0))}
			hi := revMeridian{zIv: pointInterval(floatRat(tc.z1)), rhoIv: pointInterval(floatRat(tc.rho1))}
			got := ratFloatUp(revolveCellSweptVolume(lo, hi, g))
			want := tripleIntegralReference(tc.z0, tc.rho0, tc.z1, tc.rho1, tc.dphi, 60)
			require.Positive(t, want)
			require.GreaterOrEqual(t, got, want, `Icell sits below the swept volume it must bound`)
			require.LessOrEqual(t, got, want*tc.tightness, `Icell is far looser than the volume it bounds`)
		})
	}
}

func TestRevolveCellSweptVolumeIsZeroForAnAxisLevelCell(t *testing.T) {
	t.Parallel()
	// A planar cap's own meridian cell runs at a fixed z, so its angular
	// homotopy moves nothing along the axis and sweeps no volume at all: |z'|
	// is the factor that vanishes.
	g := mustAngularFactor(t, 2*math.Pi/32)
	lo := revMeridian{zIv: pointInterval(floatRat(3)), rhoIv: pointInterval(new(big.Rat))}
	hi := revMeridian{zIv: pointInterval(floatRat(3)), rhoIv: pointInterval(floatRat(8))}
	require.Zero(t, revolveCellSweptVolume(lo, hi, g).Sign())
}

func TestRevolveMeshSymDiffBoundsTheCylinderVolumeDeficit(t *testing.T) {
	t.Parallel()
	// The falsifier for the whole §11 composition: a chorded cylinder is
	// inscribed in the one it stands for, so the exact symmetric difference IS
	// the analytic volume minus the mesh's own. volSymDiff must cover it.
	body := internalCylinderBody(t)

	mesh, err := tessellateContext(t.Context(), body, units.Millimeters(0.05))
	require.NoError(t, err)
	require.True(t, mesh.symDiffOK)
	require.Positive(t, mesh.volSymDiff)
	require.False(t, isNonFinite(mesh.volSymDiff))

	vol, err := body.Volume()
	require.NoError(t, err)
	analytic, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	held := internalMeshVolume(mesh)
	deficit := analytic - held
	require.Positive(t, deficit)
	require.GreaterOrEqual(t, mesh.volSymDiff, deficit,
		`volSymDiff must cover the volume the chorded mesh actually loses`)
	require.LessOrEqual(t, mesh.volSymDiff, deficit*1.1,
		`volSymDiff is far looser than the deficit it bounds`)
}

func TestRevolveMeshSymDiffFallsWithTolerance(t *testing.T) {
	t.Parallel()
	body := internalCylinderBody(t)

	coarse, err := tessellateContext(t.Context(), body, units.Millimeters(0.2))
	require.NoError(t, err)
	fine, err := tessellateContext(t.Context(), body, units.Millimeters(0.01))
	require.NoError(t, err)
	require.True(t, coarse.symDiffOK)
	require.True(t, fine.symDiffOK)
	require.Less(t, fine.volSymDiff, coarse.volSymDiff)
}

func TestRevolveMeridianMomentChargesACurvedGenerator(t *testing.T) {
	t.Parallel()
	// A sphere's generator is one circular walk, so B0 and BM genuinely differ
	// and Mmeridian is the term that says by how much. A cylinder's is straight,
	// so its slivers are empty and the term is exactly zero.
	sphere := internalSphereBody(t)
	plan, err := planRevolve(t.Context(), sphere, sphere.payload.(revolvePayload), 0.05)
	require.NoError(t, err)
	require.Positive(t, revolveMeridianMoment(plan))

	cyl := internalCylinderBody(t)
	cylPlan, err := planRevolve(t.Context(), cyl, cyl.payload.(revolvePayload), 0.05)
	require.NoError(t, err)
	require.Zero(t, revolveMeridianMoment(cylPlan))
}

func TestRevolveSweepUpperNeverUnderstatesAFullTurn(t *testing.T) {
	t.Parallel()
	body := internalCylinderBody(t)
	plan, err := planRevolve(t.Context(), body, body.payload.(revolvePayload), 0.05)
	require.NoError(t, err)
	// math.Pi is the nearest float to π and sits BELOW it, which is the wrong
	// side for a bound, so the published figure must beat it strictly.
	require.Greater(t, revolveSweepUpper(plan), 2*math.Pi)
}

func TestPairChordToleranceClearsARevolveCoordinateReservation(t *testing.T) {
	t.Parallel()
	// A revolve reserves both coordinate stages out of its tolerance before it
	// chords anything (docs/tessellation-design.md §8), so a small part modelled
	// at a large coordinate needs more tolerance than the pair's own diameter
	// would give it. The pair raises past the reservation instead of handing the
	// operand a budget it has already spent.
	doc := New()
	body := internalCylinderBody(t)
	far, err := r3.Translation(r3.Vec{X: 1e12, Y: 1e12, Z: 1e12})
	require.NoError(t, err)
	placed, err := body.Placed(far)
	require.NoError(t, err)

	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 4, 4)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	near, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(4), Dir: Along})
	require.NoError(t, err)
	box, err := near.Placed(far)
	require.NoError(t, err)

	reserve := coordDisplacementOf(t.Context(), placed)
	require.Positive(t, reserve)
	require.Zero(t, coordDisplacementOf(t.Context(), box))

	tol, dPair, err := pairChordTolerance(t.Context(), placed, box)
	require.NoError(t, err)
	require.Greater(t, tol, reserve,
		`a tolerance under the revolve's own coordinate reservation leaves it no chord budget`)
	require.Greater(t, tol, dPair*boolChordFactor,
		`the reservation, not the pair diameter, is what set this tolerance`)

	// The raised tolerance is one the revolve can actually be meshed at, which
	// is the whole point: without it planRevolve refuses before it chords.
	mesh, err := tessellateContext(t.Context(), placed, units.Millimeters(tol))
	require.NoError(t, err)
	require.True(t, mesh.symDiffOK)

	_, err = tessellateContext(t.Context(), placed, units.Millimeters(dPair*boolChordFactor))
	require.ErrorIs(t, err, ErrUnsupported)
}

func TestAngularHomotopyBulgesBoundTheSecondDerivatives(t *testing.T) {
	t.Parallel()
	// The per-piece allowance rests on max|f''| for each of P and Q. Both bounds
	// are elementary rather than derived, so the falsifier is a dense numeric
	// second difference of the very functions revolveAngularHomotopyFactor
	// encloses: a claimed bound below the observed curvature disproves it.
	p := func(u, d float64) float64 {
		return d * ((1-u)*(1-math.Cos(u*d)) + u*(1-math.Cos((1-u)*d)))
	}
	q := func(u, d float64) float64 {
		return math.Sin(u*d)*(1-math.Cos(d)) - math.Sin(d)*(1-math.Cos(u*d))
	}
	n := int64(revolveAngularIntegralSteps)
	scale := float64(8 * n * n)
	for _, d := range []float64{2 * math.Pi / 3, 2 * math.Pi / 40, 1.0, 3.0, 5.5} {
		bp, bq := angularHomotopyBulges(pointInterval(floatRat(d)), n)
		require.Positive(t, ratFloatUp(bp))
		require.Positive(t, ratFloatUp(bq))
		const h = 1e-4
		maxP, maxQ := 0.0, 0.0
		for i := range 201 {
			u := float64(i) / 200
			if u < h || u > 1-h {
				continue
			}
			maxP = math.Max(maxP, math.Abs((p(u+h, d)-2*p(u, d)+p(u-h, d))/(h*h)))
			maxQ = math.Max(maxQ, math.Abs((q(u+h, d)-2*q(u, d)+q(u-h, d))/(h*h)))
		}
		require.GreaterOrEqual(t, ratFloatUp(bp)*scale, maxP, `dφ=%v: the P allowance understates its own curvature`, d)
		require.GreaterOrEqual(t, ratFloatUp(bq)*scale, maxQ, `dφ=%v: the Q allowance understates its own curvature`, d)
	}
}

func TestRevolveSymDiffChargesEveryStageSeparately(t *testing.T) {
	t.Parallel()
	// docs/tessellation-design.md §11's four stages are ABSOLUTE swept volumes
	// that may never cancel one another, so each of them must move the published
	// figure on its own. Feeding the composition one stage at a time is the
	// falsifier: a leg that has stopped being charged leaves the answer where it
	// was.
	build := func(t *testing.T, body *Body) (*Mesh, *revolvePlan) {
		t.Helper()
		mesh, err := tessellateContext(t.Context(), body, units.Millimeters(0.05))
		require.NoError(t, err)
		plan, err := planRevolve(t.Context(), body, body.payload.(revolvePayload), 0.05)
		require.NoError(t, err)
		return mesh, plan
	}
	sum := func(t *testing.T, m *Mesh, p *revolvePlan, angular *big.Rat, dC, dR float64) float64 {
		t.Helper()
		got, err := revolveSymDiff(m, p, angular, dC, dR)
		require.NoError(t, err)
		return got
	}

	cylMesh, cylPlan := build(t, internalCylinderBody(t))
	zero := new(big.Rat)
	// A straight generator's slivers are empty, so with no angular term and no
	// coordinate rounding there is nothing left to charge.
	require.Zero(t, sum(t, cylMesh, cylPlan, zero, 0, 0))

	angular := big.NewRat(1, 7)
	require.Greater(t, sum(t, cylMesh, cylPlan, angular, 0, 0), 0.14,
		`the angular homotopy sum must reach the published figure`)
	require.Greater(t, sum(t, cylMesh, cylPlan, zero, 1e-3, 0), 0.0,
		`construction rounding sweeps volume of its own`)
	require.Greater(t, sum(t, cylMesh, cylPlan, zero, 0, 1e-3), 0.0,
		`placement rounding sweeps volume of its own`)
	require.Greater(t, sum(t, cylMesh, cylPlan, angular, 1e-3, 1e-3), sum(t, cylMesh, cylPlan, angular, 0, 0),
		`the coordinate stages add to the angular one rather than replacing it`)

	// A circular generator's slivers are not empty, and Mmeridian is the only
	// term still standing once every other stage is set to nothing.
	sphMesh, sphPlan := build(t, internalSphereBody(t))
	require.Positive(t, sum(t, sphMesh, sphPlan, zero, 0, 0))
}

func TestRevolveSymDiffRefusesAnAbsentAngularTerm(t *testing.T) {
	t.Parallel()
	mesh, err := tessellateContext(t.Context(), internalCylinderBody(t), units.Millimeters(0.05))
	require.NoError(t, err)
	body := internalCylinderBody(t)
	plan, err := planRevolve(t.Context(), body, body.payload.(revolvePayload), 0.05)
	require.NoError(t, err)
	_, err = revolveSymDiff(mesh, plan, nil, 0, 0)
	require.ErrorIs(t, err, ErrUnsupported)
	_, err = revolveSymDiff(mesh, plan, big.NewRat(-1, 1), 0, 0)
	require.ErrorIs(t, err, ErrUnsupported)
}
