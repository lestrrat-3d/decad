package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// arcCellFixture builds the meridian model of one circular chord from plain
// floats, the way revolveArcChordCell builds it from an axis walk.
func arcCellFixture(cV, radius, th0, dth float64) revArcCell {
	return revArcCell{
		cV:     big.NewRat(0, 1).SetFloat64(cV),
		radius: big.NewRat(0, 1).SetFloat64(radius),
		th0:    big.NewRat(0, 1).SetFloat64(th0),
		dth:    big.NewRat(0, 1).SetFloat64(dth),
	}
}

// arcCellReference is a dense numeric reading of ∫₀¹|scale·ρ(t) − held|·w(t) dt
// for one half domain — a reference the certified bound must sit ABOVE, never a
// substitute for it.
func arcCellReference(cell revArcCell, scale, held float64, weight int) float64 {
	cV, _ := cell.cV.Float64()
	radius, _ := cell.radius.Float64()
	th0, _ := cell.th0.Float64()
	dth, _ := cell.dth.Float64()
	const n = 200000
	total := 0.0
	for i := range n {
		t := (float64(i) + 0.5) / n
		w := 1.0
		switch weight {
		case revolveWeightT:
			w = t
		case revolveWeightOneMinusT:
			w = 1 - t
		}
		rho := cV + radius*math.Sin(th0+t*dth)
		total += math.Abs(scale*rho-held) * w / n
	}
	return total
}

func TestRevolveArcCellSlackBoundsASignChangingJacobianGap(t *testing.T) {
	// docs/tessellation-design.md §14's inner-torus cell: the meridian runs
	// through the tube's innermost point, so ρ(t) dips and the Jacobian error
	// against the flat facet's constant density changes sign TWICE inside one
	// cell. The held density is set at the true density's own mean, so the
	// SIGNED error cancels to nothing and only a non-cancelling reading says
	// anything at all — the whole reason §10.2 puts the absolute value inside
	// the integral.
	const cV, radius = 10.0, 3.0
	th0, dth := 3*math.Pi/2-0.4, 0.8
	const nPhi = 16
	dPhi := 2 * math.Pi / nPhi
	scale := radius * dth * dPhi

	const m = 200000
	mean := 0.0
	for i := range m {
		tt := (float64(i) + 0.5) / m
		mean += (cV + radius*math.Sin(th0+tt*dth)) / m
	}
	held := scale * mean

	cell := arcCellFixture(cV, radius, th0, dth)
	step := intervalScale(twoPiInterval(), big.NewRat(1, nPhi))
	area := pointInterval(big.NewRat(0, 1).SetFloat64(held))
	got, err := revolveArcCellSlack(cell, step, [2]ratInterval{area, area}, 0)
	require.NoError(t, err)

	want := arcCellReference(cell, scale, held, revolveWeightT) +
		arcCellReference(cell, scale, held, revolveWeightOneMinusT)
	require.Positive(t, want, `the fixture's own error must be nonzero for the bound to bound anything`)
	require.GreaterOrEqual(t, got, want, `Ecell must bound the absolute local density error`)
	require.LessOrEqual(t, got, 1.25*want, `certified subdivision must stay a bound, not a blow-up`)

	// The SIGNED integral of the same difference is zero by construction, so a
	// whole-cell reading that let it cancel would report no error at all where
	// the certified one reports the sum of both lobes.
	signed := 0.0
	for i := range m {
		tt := (float64(i) + 0.5) / m
		signed += (scale*(cV+radius*math.Sin(th0+tt*dth)) - held) / m
	}
	require.Less(t, math.Abs(signed), 1e-6*want, `this fixture's signed error cancels to nothing`)
	require.Greater(t, got, 0.9*want)
}

func TestRevolveArcCellSlackIsExactOnAFixedSignCell(t *testing.T) {
	// A cell whose difference never changes sign: the certified subdivision
	// must land close to the true integral, which is what says the fixed
	// budget buys real tightness rather than a safe but useless figure.
	const cV, radius = 10.0, 3.0
	th0, dth := 0.0, 0.3
	const nPhi = 24
	dPhi := 2 * math.Pi / nPhi
	scale := radius * dth * dPhi
	held := 0.0 // Jheld = 0 keeps scale·ρ strictly positive throughout.

	cell := arcCellFixture(cV, radius, th0, dth)
	step := intervalScale(twoPiInterval(), big.NewRat(1, nPhi))
	area := pointInterval(new(big.Rat))
	got, err := revolveArcCellSlack(cell, step, [2]ratInterval{area, area}, 0)
	require.NoError(t, err)
	want := arcCellReference(cell, scale, held, revolveWeightT) +
		arcCellReference(cell, scale, held, revolveWeightOneMinusT)
	require.GreaterOrEqual(t, got, want)
	require.InDelta(t, want, got, 0.01*want)
}

func TestRevolveArcFanSlackBoundsAPoleCell(t *testing.T) {
	// A sphere's polar cell: Jheld is linear in t through the pole, Jtrue keeps
	// its sinusoidal ρ, and the fan integrates over the whole unit square.
	const cV, radius = 0.0, 5.0
	th0, dth := 0.0, 0.2
	const nPhi = 20
	dPhi := 2 * math.Pi / nPhi
	scale := radius * dth * dPhi
	twoArea := 0.9 * scale * radius * math.Sin(dth)

	cell := arcCellFixture(cV, radius, th0, dth)
	step := intervalScale(twoPiInterval(), big.NewRat(1, nPhi))
	area := pointInterval(big.NewRat(0, 1).SetFloat64(twoArea))
	for _, poleFirst := range []bool{true, false} {
		got, err := revolveArcFanSlack(cell, poleFirst, step, area, 0)
		require.NoError(t, err)
		const n = 200000
		want := 0.0
		for i := range n {
			tt := (float64(i) + 0.5) / n
			held := twoArea * tt
			if !poleFirst {
				held = twoArea * (1 - tt)
			}
			want += math.Abs(scale*(cV+radius*math.Sin(th0+tt*dth))-held) / n
		}
		require.GreaterOrEqual(t, got, want, `poleFirst=%v`, poleFirst)
		require.LessOrEqual(t, got, 1.15*want, `poleFirst=%v`, poleFirst)
	}
}

func TestRevolveArcCellSlackWidensWithTheModelSlack(t *testing.T) {
	// The meridian model is the payload's own axis-coordinate circle, and the
	// caller hands it the composed coordinate displacement to widen ρ by. A
	// larger displacement must produce a larger allowance, never the same one.
	cell := arcCellFixture(10, 3, 0.2, 0.3)
	step := intervalScale(twoPiInterval(), big.NewRat(1, 24))
	area := pointInterval(big.NewRat(0, 1).SetFloat64(0.5))
	tight, err := revolveArcCellSlack(cell, step, [2]ratInterval{area, area}, 0)
	require.NoError(t, err)
	loose, err := revolveArcCellSlack(cell, step, [2]ratInterval{area, area}, 1e-3)
	require.NoError(t, err)
	require.Greater(t, loose, tight)

	_, err = revolveArcCellSlack(cell, step, [2]ratInterval{area, area}, math.Inf(1))
	require.ErrorIs(t, err, ErrUnsupported)
}

func TestChordSegmentAreaBoundsTheTrueCircularSegments(t *testing.T) {
	// The proven r²θ³/(12n²) must sit above the true Σ (r²/2)(φ − sin φ) and
	// stay close to it, and it must never call a trig function to say so.
	for _, tc := range []struct {
		radius, sweep float64
		n             int
	}{
		{3, 2 * math.Pi, 12},
		{5, math.Pi, 8},
		{0.5, math.Pi / 2, 3},
		{10, 2 * math.Pi, 64},
	} {
		phi := tc.sweep / float64(tc.n)
		want := float64(tc.n) * tc.radius * tc.radius / 2 * (phi - math.Sin(phi))
		got := chordSegmentArea(tc.radius, tc.sweep, tc.n)
		require.GreaterOrEqual(t, got, want, `r=%v sweep=%v n=%d`, tc.radius, tc.sweep, tc.n)
		require.LessOrEqual(t, got, 1.3*want, `r=%v sweep=%v n=%d`, tc.radius, tc.sweep, tc.n)
	}
	require.Equal(t, 0.0, chordSegmentArea(0, math.Pi, 4))
	require.True(t, math.IsInf(chordSegmentArea(1, math.Pi, 0), 1))
	require.True(t, math.IsInf(chordSegmentArea(1, math.Inf(1), 4), 1))
}

func TestRevolveArcStationEnclosesTheRecordedPoint(t *testing.T) {
	// The axis frame is the plane's own u axis, so (z, ρ) is (u, v) exactly and
	// the station's stored pair can be checked against the recorded circle by
	// hand.
	ax := axisFrame{dU: 1, snapTol: 1e-9}
	seg := CircleSeg{Center: Point2{U: 0, V: 10}, Radius: units.Millimeters(3), CCW: true, TStart: 0, TEnd: 1}

	station, gap, err := revolveArcStation(ax, seg, 1, 4)
	require.NoError(t, err)
	require.InDelta(t, 0.0, station.z, 1e-12)
	require.InDelta(t, 13.0, station.rho, 1e-12)
	require.LessOrEqual(t, gap, 8*math.Nextafter(13, math.Inf(1))-8*13)

	// Every quarter turn of a recorded circle is exact, so a station on one
	// commits no rounding at all.
	require.Equal(t, 0.0, gap)

	// A station outside the walk's own interior states no bound.
	_, _, err = revolveArcStation(ax, seg, 0, 4)
	require.ErrorIs(t, err, ErrUnsupported)
	_, _, err = revolveArcStation(ax, seg, 4, 4)
	require.ErrorIs(t, err, ErrUnsupported)

	// A circle centred ON the axis has a station at ρ = 0 only where the
	// generator crosses it, which sweeps no manifold solid.
	onAxis := CircleSeg{Center: Point2{U: 0, V: 0}, Radius: units.Millimeters(3), CCW: true, TStart: 0, TEnd: 1}
	_, _, err = revolveArcStation(ax, onAxis, 2, 4)
	require.ErrorIs(t, err, ErrDegenerate)
}

func TestChordCountHonoursTheWalkMinimum(t *testing.T) {
	whole := segmentWalk{radius: 1, th0: 0, th1: 2 * math.Pi, closed: true}

	t.Run("a budget at or above 2r takes the minimum with no inverse", func(t *testing.T) {
		// docs/tessellation-design.md §14: for radius 1, budgets equal to and
		// above 2r — 5 mm among them — must choose the minimum count without
		// evaluating an out-of-domain inverse.
		for _, tol := range []float64{2, 5, 100} {
			n, s, err := chordCount(whole, tol, chordWalkMin(whole))
			require.NoError(t, err)
			require.Equal(t, 3, n, `tol=%v`, tol)
			require.LessOrEqual(t, s, tol)
		}
	})

	t.Run("the radius-1 full-turn threshold sits at n = 122", func(t *testing.T) {
		at122 := chordSagitta(1, 2*math.Pi, 122)
		n, s, err := chordCount(whole, at122, chordWalkMin(whole))
		require.NoError(t, err)
		require.Equal(t, 122, n)
		require.Equal(t, at122, s)

		n, _, err = chordCount(whole, math.Nextafter(at122, 0), chordWalkMin(whole))
		require.NoError(t, err)
		require.Equal(t, 123, n, `one ulp below the threshold buys one more chord`)
	})

	t.Run("an axis-to-axis meridian takes at least two chords", func(t *testing.T) {
		// A circular generator with both ends on the axis cannot chord to a
		// single on-axis segment (docs/tessellation-design.md §9).
		meridian := segmentWalk{radius: 5, th0: 0, th1: math.Pi, kind: walkCircular, startV: 0, endV: 0}
		require.Equal(t, 2, revolveMeridianMin(meridian))
		n, _, err := chordCount(meridian, 1000, revolveMeridianMin(meridian))
		require.NoError(t, err)
		require.Equal(t, 2, n)

		offAxis := segmentWalk{radius: 5, th0: 0, th1: math.Pi, kind: walkCircular, startV: 1, endV: 2}
		require.Equal(t, 1, revolveMeridianMin(offAxis))
		closed := segmentWalk{radius: 5, th0: 0, th1: 2 * math.Pi, kind: walkCircular, closed: true}
		require.Equal(t, 3, revolveMeridianMin(closed))
	})

	t.Run("a minimum past the per-walk cap refuses", func(t *testing.T) {
		_, _, err := chordCount(whole, 1, maxChordsPerWalk+1)
		require.ErrorIs(t, err, ErrUnsupported)
	})
}

func TestRequireWalkClearanceGatesTheSagittaTubes(t *testing.T) {
	// A flattened rectangle whose two long sides are 0.3 apart: with a 0.1
	// tube on each the pair clears, and with a 0.2 tube on each it does not.
	pts := []Point2{{U: 0, V: 0}, {U: 10, V: 0}, {U: 10, V: 0.3}, {U: 0, V: 0.3}}
	loops := [][]int{{0, 1, 2, 3}}

	require.NoError(t, requireWalkClearance(t.Context(), pts, loops, [][]float64{{0.1, 0, 0.1, 0}}))

	err := requireWalkClearance(t.Context(), pts, loops, [][]float64{{0.2, 0, 0.2, 0}})
	require.ErrorIs(t, err, ErrDegenerate)
	require.ErrorContains(t, err, "clearance gate")
	require.ErrorContains(t, err, "finer chord tolerance")

	// Adjacent chords are REQUIRED to meet at the sample they share, so they
	// are never gated: a straight-walled loop with no tube at all passes.
	require.NoError(t, requireWalkClearance(t.Context(), pts, loops, [][]float64{{0, 0, 0, 0}}))
}

func TestRevolveCircularMeshAreaSlackCoversTheHeldAreaGap(t *testing.T) {
	// docs/tessellation-design.md §14's "check areaSlack against high-precision
	// local area differences", for the two circular generator classes: the
	// published slack must cover the gap between the body's own analytic area
	// and the area its facets actually hold, without swamping the surface it
	// speaks for.
	for _, tc := range []struct {
		name  string
		build func(*testing.T) (*sketch.Sketch, *sketch.Profile)
		tol   float64
	}{
		{"sphere with a pole at each end", func(t *testing.T) (*sketch.Sketch, *sketch.Profile) {
			w := sketch.NewWorld()
			s, err := w.CreateSketch(w.XY())
			require.NoError(t, err)
			o := s.CreatePoint(0, 0)
			s.Fix(o)
			end := s.CreatePoint(10, 0)
			c := s.CreatePoint(5, 0)
			s.CreateLine(o, end)
			s.CreateArc(c, end, o)
			_, err = s.Solve(t.Context())
			require.NoError(t, err)
			return s, s.Profiles()[0]
		}, 0.1},
		{"ring torus clear of the axis", func(t *testing.T) (*sketch.Sketch, *sketch.Profile) {
			w := sketch.NewWorld()
			s, err := w.CreateSketch(w.XY())
			require.NoError(t, err)
			c := s.CreatePoint(0, 10)
			s.Fix(c)
			s.CreateCircle(c, 3)
			_, err = s.Solve(t.Context())
			require.NoError(t, err)
			return s, s.Profiles()[0]
		}, 0.2},
	} {
		t.Run(tc.name, func(t *testing.T) {
			s, p := tc.build(t)
			axis := SketchLine{Start: Point2{U: 0, V: 0}, End: Point2{U: 1, V: 0}}
			body, err := New().Revolve(s, p, axis, FullRevolution{})
			require.NoError(t, err)
			mesh, err := body.Tessellate(units.Millimeters(tc.tol))
			require.NoError(t, err)

			held := 0.0
			for _, tri := range mesh.triangles {
				a, b, c := mesh.vertices[tri[0]], mesh.vertices[tri[1]], mesh.vertices[tri[2]]
				held += b.Sub(a).Cross(c.Sub(a)).Len() / 2
			}
			area, err := body.Area()
			require.NoError(t, err)
			analytic, err := area.Value.In(units.SquareMillimeter)
			require.NoError(t, err)

			gap := math.Abs(analytic - held)
			require.Positive(t, gap, `a chorded revolve holds less area than the surface it stands for`)
			require.Positive(t, mesh.areaSlack)
			require.GreaterOrEqual(t, mesh.areaSlack, gap, `areaSlack must cover the gap it exists to bound`)
			require.Less(t, mesh.areaSlack, 0.5*analytic)
		})
	}
}
