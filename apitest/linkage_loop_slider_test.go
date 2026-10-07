package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file holds the prismatic loop tests of docs/linkage-check-design.md
// §15.10: scene 8 (the slider-crank against an end stop), its pin leg, and a
// slider-crank driven at its slide. The closed forms are the slider-crank's
// own: with crank r = 30 and rod l = 80 on a rail through the crank's pivot,
// the slider pin stands at x(θ) = 30·cos θ + √(6400 − 900·sin² θ) and the
// rod's angle from +X is −asin(30·sin θ/80).

// sliderCrankOpen attaches scene 8's crank, rod and slider block to l, the
// slider on a prismatic joint along dir, and returns the rod and the slider,
// not yet closed.
func sliderCrankOpen(t *testing.T, doc *decad.Document, l *decad.Linkage, dir r3.Vec) (*decad.Link, *decad.Link) {
	t.Helper()
	z := r3.NewVec(0, 0, 1)
	crank, err := l.Ground().Revolute(r3.Vec{}, z, []*decad.Body{boxBodyAtZ(t, doc, 0, -3, 30, 3, 0, 8)})
	require.NoError(t, err)
	rod, err := crank.Revolute(r3.NewVec(30, 0, 0), z, []*decad.Body{boxBodyAtZ(t, doc, 30, -3, 110, 3, 10, 8)})
	require.NoError(t, err)
	slider, err := l.Ground().Prismatic(dir, []*decad.Body{boxBodyAtZ(t, doc, 105, -5, 115, 5, 20, 8)})
	require.NoError(t, err)
	return rod, slider
}

// sliderCrank is scene 8's linkage.
type sliderCrank struct {
	doc                 *decad.Document
	linkage             *decad.Linkage
	crank, rod, slider  *decad.Link
	sliderBody, rodBody *decad.Body
	loop                *decad.LinkageLoop
}

func buildSliderCrank(t *testing.T) sliderCrank {
	t.Helper()
	sc := sliderCrank{doc: decad.New(), linkage: decad.NewLinkage()}
	sc.rod, sc.slider = sliderCrankOpen(t, sc.doc, sc.linkage, r3.NewVec(1, 0, 0))
	sc.crank = sc.rod.Parent()
	sc.rodBody, sc.sliderBody = sc.rod.Bodies()[0], sc.slider.Bodies()[0]
	var err error
	sc.loop, err = sc.linkage.Close(sc.rod, sc.slider, r3.NewVec(110, 0, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	return sc
}

func (sc sliderCrank) drive(to units.Value) decad.Drive {
	return decad.Drive{{Link: sc.crank, From: units.Degrees(0), To: to}}
}

// sliderPin is the slider pin's x at crank angle th.
func sliderPin(th float64) float64 {
	return 30*math.Cos(th) + math.Sqrt(6400-900*math.Sin(th)*math.Sin(th))
}

// TestVerifyLinkageLoopSliderCrank is scene 8 of docs/linkage-check-design.md
// §15.10. A stop x ∈ [60, 70], y ∈ [−20, 20], z ∈ [19, 29] sits in the
// slider's layer; the block's left face x − 5 reaches it when x = 75, at
// cos θ = 1/36.
//
// Legs seen to fail when deleted: the rail (the slider pin is then free in
// the plane, and the zero pose cannot be enclosed); reading the slide in
// millimetres (the slider's value then carries the wrong unit).
func TestVerifyLinkageLoopSliderCrank(t *testing.T) {
	t.Parallel()
	t.Run("the bars are the crank's and the rod's", func(t *testing.T) {
		t.Parallel()
		sc := buildSliderCrank(t)
		bars := sc.loop.Bars()
		require.Len(t, bars, 2, `a slide's rail is no bar`)
		for n, want := range []struct {
			link   *decad.Link
			length float64
		}{{sc.crank, 30}, {sc.rod, 80}} {
			require.Same(t, want.link, bars[n].Link)
			require.Equal(t, want.length, bars[n].Length.Value.Mag())
			require.Equal(t, decad.Exact, bars[n].Length.Exactness)
		}
		require.Equal(t, []*decad.Link{sc.crank, sc.rod, sc.slider}, sc.loop.Links())
	})
	t.Run("the slider reaches the stop", func(t *testing.T) {
		t.Parallel()
		sc := buildSliderCrank(t)
		stop := boxBodyAtZ(t, sc.doc, 60, -20, 70, 20, 19, 10)
		report := verifyLinkage(t, sc.doc, sc.linkage, sc.drive(units.Degrees(90)), decad.WithResolution(units.Scalar(1.0/256)))
		thStar := math.Acos(1.0 / 36)
		sStar := thStar / (math.Pi / 2)
		require.InDelta(t, 88.408246, thStar*180/math.Pi, 1e-6)
		require.InDelta(t, 0.982314, sStar, 1e-6)

		require.Equal(t, decad.Interfering, report.Status)
		first := report.Collisions[0]
		require.Same(t, sc.sliderBody, first.A)
		require.Same(t, stop, first.B)
		require.Equal(t, units.Scalar(252.0/256), first.At, `the first grid point above s*`)
		x := sliderPin(252.0 / 256 * math.Pi / 2)
		require.InDelta(t, 74.901876, x, 1e-6)
		require.InDelta(t, 80*(75-x), first.Volume.Value.Mag(), 1e-6)
		require.InDelta(t, 7.849912, first.Volume.Value.Mag(), 1e-6)
		require.Less(t, first.Volume.Bound.Mag(), first.Volume.Value.Mag())
		for _, c := range report.Collisions {
			require.Greater(t, c.At.Mag(), sStar)
		}
		for _, iv := range report.Intervals {
			if iv.Outcome == decad.IntervalClear {
				require.LessOrEqual(t, iv.To.Mag(), sStar)
			}
		}

		for _, p := range report.Poses {
			th := p.Pose.At.Mag() * math.Pi / 2
			require.Equal(t, units.Millimeter, p.Pose.Values[2].Unit())
			require.InDelta(t, sliderPin(th)-110, p.Pose.Values[2].Mag(), 1e-9)
			require.GreaterOrEqual(t, p.Pose.Bounds[2].Mag(), 0.0)
			require.Less(t, p.Pose.Bounds[2].Mag(), 1e-9)
			require.InDelta(t, -th-math.Asin(30*math.Sin(th)/80), p.Pose.Values[1].Mag(), 1e-9)
		}
	})
	t.Run("the crank turned the other way", func(t *testing.T) {
		t.Parallel()
		sc := buildSliderCrank(t)
		report := verifyLinkage(t, sc.doc, sc.linkage, sc.drive(units.Degrees(-90)), decad.WithResolution(units.Scalar(1)))
		end := report.Poses[len(report.Poses)-1].Pose
		th := -math.Pi / 2
		require.InDelta(t, sliderPin(th)-110, end.Values[2].Mag(), 1e-9)
		require.InDelta(t, -th-math.Asin(30*math.Sin(th)/80), end.Values[1].Mag(), 1e-9)
	})
}

// TestVerifyLinkageLoopSliderPin is scene 8's pin leg for a prismatic
// dependent: the stop replaced by a 2 × 2 × 2 mm pin at x ∈ [85, 87] in the
// slider's layer, the crank driven 0° → 180°. The block's left face passes
// x = 86 at θ = 59.380079°, s = 0.329889, between the grid points 1/4 and 1/2
// of WithResolution(1/4), across which the slider moves 24.19 mm; contact
// begins earlier, when the face reaches the pin's far face x = 87.
//
// Leg seen to fail when deleted: the prismatic dependent's term in τ — the
// slider is a child of the ground, so its travel is its own slide's alone, and
// the interval [1/4, 1/2] then certifies from its ends.
func TestVerifyLinkageLoopSliderPin(t *testing.T) {
	t.Parallel()
	thPass := math.Acos(2781.0 / 5460)
	require.InDelta(t, 59.380079, thPass*180/math.Pi, 1e-6)
	require.InDelta(t, 86, sliderPin(thPass)-5, 1e-9)
	sPass := thPass / math.Pi
	require.InDelta(t, 0.329889, sPass, 1e-6)
	require.InDelta(t, 24.19, sliderPin(math.Pi/4)-sliderPin(math.Pi/2), 5e-3)
	build := func(t *testing.T) (sliderCrank, *decad.Body) {
		sc := buildSliderCrank(t)
		return sc, boxBodyAtZ(t, sc.doc, 85, -1, 87, 1, 23, 2)
	}
	t.Run("coarse", func(t *testing.T) {
		t.Parallel()
		sc, _ := build(t)
		report := verifyLinkage(t, sc.doc, sc.linkage, sc.drive(units.Degrees(180)), decad.WithResolution(units.Scalar(1.0/4)))
		require.Equal(t, decad.Suspect, report.Status)
		require.Empty(t, report.Collisions)
		var holding *decad.MotionInterval
		for n, iv := range report.Intervals {
			if iv.From.Mag() == 1.0/4 && iv.To.Mag() == 1.0/2 {
				holding = &report.Intervals[n]
			}
		}
		require.NotNil(t, holding)
		require.Equal(t, decad.IntervalUndecided, holding.Outcome)
	})
	t.Run("fine", func(t *testing.T) {
		t.Parallel()
		sc, pin := build(t)
		report := verifyLinkage(t, sc.doc, sc.linkage, sc.drive(units.Degrees(180)), decad.WithResolution(units.Scalar(1.0/1024)))
		require.Equal(t, decad.Interfering, report.Status)
		first := report.Collisions[0]
		require.Same(t, sc.sliderBody, first.A)
		require.Same(t, pin, first.B)
		// Contact begins when the left face reaches the pin's far face x = 87:
		// x(θ) = 92, cos θ = 2964/5520.
		sOn := math.Acos(2964.0/5520) / math.Pi
		require.InDelta(t, 87, sliderPin(sOn*math.Pi)-5, 1e-9)
		require.Greater(t, first.At.Mag(), sOn)
		require.LessOrEqual(t, first.At.Mag(), sOn+2.0/1024)
	})
}

// buildSlideDriven is the slider-crank driven at its slide: the crank along
// +Y at the zero pose, A = (0, 30), so the slider pin P = (√5500, 0) sits off
// the dead centre. It returns the document, the linkage and the slider.
func buildSlideDriven(t *testing.T) (*decad.Document, *decad.Linkage, *decad.Link) {
	t.Helper()
	px := math.Sqrt(5500)
	doc := decad.New()
	l := decad.NewLinkage()
	z := r3.NewVec(0, 0, 1)
	crank, err := l.Ground().Revolute(r3.Vec{}, z, []*decad.Body{boxBodyAtZ(t, doc, -3, 0, 3, 30, 0, 8)})
	require.NoError(t, err)
	rod, err := crank.Revolute(r3.NewVec(0, 30, 0), z, []*decad.Body{barBody(t, doc, [2]float64{0, 30}, [2]float64{px, 0}, 10)})
	require.NoError(t, err)
	slider, err := l.Ground().Prismatic(r3.NewVec(1, 0, 0), []*decad.Body{boxBodyAtZ(t, doc, px-5, -5, px+5, 5, 20, 8)})
	require.NoError(t, err)
	_, err = l.Close(rod, slider, r3.NewVec(px, 0, 0), z)
	require.NoError(t, err)
	return doc, l, slider
}

// TestVerifyLinkageLoopSlideDriven drives a slider-crank at its slide. The
// crank stands along +Y at the zero pose, A = (0, 30), so the slider pin
// P = (√5500, 0) is off the dead centre and the rod's length √6400 = 80 is
// reached from an irrational pin; the crank's angle φ from +X then follows
// 30·cos φ + √(6400 − 900·sin² φ) = x, on the branch through φ = 90°. A slide
// of +5 mm reads on the scene's own side, and one of −5 mm on the half-turned
// side, where the slide's scene value is its negation.
//
// Leg seen to fail when deleted: the half-turned side — the backward slide is
// then asked as a forward one, and the crank reads the turn a +5 mm slide
// makes.
func TestVerifyLinkageLoopSlideDriven(t *testing.T) {
	t.Parallel()
	px := math.Sqrt(5500)
	crankAngle := func(x float64) float64 {
		return bisectRoot(func(phi float64) float64 {
			return 30*math.Cos(phi) + math.Sqrt(6400-900*math.Sin(phi)*math.Sin(phi)) - x
		}, math.Pi/3, 2*math.Pi/3)
	}
	rodAngle := func(phi, x float64) float64 { return math.Atan2(-30*math.Sin(phi), x-30*math.Cos(phi)) }
	for _, slide := range []float64{5, -5} {
		t.Run(units.Millimeters(slide).String(), func(t *testing.T) {
			t.Parallel()
			_, l, slider := buildSlideDriven(t)
			bars := l.Loops()[0].Bars()
			require.Len(t, bars, 2)
			require.Equal(t, decad.Approximate, bars[1].Length.Exactness, `the rod's length is reached from an irrational pin`)
			require.InDelta(t, 80, bars[1].Length.Value.Mag(), 1e-12)

			drive := decad.Drive{{Link: slider, From: units.Millimeters(0), To: units.Millimeters(slide)}}
			sched, err := l.Schedule(t.Context(), drive)
			require.NoError(t, err)
			for _, at := range []float64{0, 0.5, 1} {
				pose, err := sched.PoseAt(t.Context(), units.Scalar(at))
				require.NoError(t, err)
				x := px + slide*at
				phi := crankAngle(x)
				require.Equal(t, units.Millimeters(slide*at), pose.Values[2])
				require.InDelta(t, phi-math.Pi/2, pose.Values[0].Mag(), 1e-9)
				require.InDelta(t, rodAngle(phi, x)-phi-(rodAngle(math.Pi/2, px)-math.Pi/2), pose.Values[1].Mag(), 1e-9)
				require.Zero(t, pose.Bounds[2].Mag(), `the stated slide carries no half-width`)
			}
		})
	}
}
