package apitest_test

import (
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// meridianArc is a counter-clockwise arc from the polygon point before it to
// End about Center, in (radius, height).
type meridianArc struct{ Center, End [2]float64 }

// revolveSectionMeridian revolves a closed meridian drawn on world XZ, in (x, z),
// a full turn about world Z. A step is a [2]float64 vertex reached by a line,
// or a meridianArc.
func revolveSectionMeridian(t *testing.T, start [2]float64, steps ...any) (*decad.Document, *decad.Body) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XZ())
	require.NoError(t, err)
	first := s.CreatePoint(start[0], start[1])
	s.Fix(first)
	prev := first
	for i, step := range steps {
		var next *sketch.Point
		end := func(p [2]float64) *sketch.Point {
			if i == len(steps)-1 && p == start {
				return first
			}
			return s.CreatePoint(p[0], p[1])
		}
		switch step := step.(type) {
		case [2]float64:
			next = end(step)
			s.CreateLine(prev, next)
		case meridianArc:
			next = end(step.End)
			s.CreateArc(s.CreatePoint(step.Center[0], step.Center[1]), prev, next)
		default:
			t.Fatalf("unknown meridian step %T", step)
		}
		prev = next
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	doc := decad.New()
	body, err := decad.Revolve(t.Context(), doc, s, s.Profiles()[0], decad.SketchLine{
		Start: decad.Point2{}, End: decad.Point2{V: 1},
	}, decad.FullRevolution{})
	require.NoError(t, err)
	return doc, body
}

// steppedShaft is r=10 for z 0..20 and r=6 for z 20..40 about world Z.
func steppedShaft(t *testing.T) (*decad.Document, *decad.Body) {
	t.Helper()
	return revolveSectionMeridian(t, [2]float64{0, 0},
		[2]float64{10, 0}, [2]float64{10, 20}, [2]float64{6, 20}, [2]float64{6, 40},
		[2]float64{0, 40}, [2]float64{0, 0})
}

// meridianTube is a tube r 4..10, z 0..20 about world Z.
func meridianTube(t *testing.T) (*decad.Document, *decad.Body) {
	t.Helper()
	return revolveSectionMeridian(t, [2]float64{4, 0},
		[2]float64{10, 0}, [2]float64{10, 20}, [2]float64{4, 20}, [2]float64{4, 0})
}

// meridianCapsule is a cylinder r=5, z 0..10, under a hemisphere r=5 centred
// at z=10.
func meridianCapsule(t *testing.T) (*decad.Document, *decad.Body) {
	t.Helper()
	return revolveSectionMeridian(t, [2]float64{0, 0},
		[2]float64{5, 0}, [2]float64{5, 10},
		meridianArc{Center: [2]float64{0, 10}, End: [2]float64{0, 15}}, [2]float64{0, 0})
}

func meridianFrame(t *testing.T, origin, u, v r3.Vec) r3.Frame {
	t.Helper()
	frame, err := r3.NewFrame(origin, u, v)
	require.NoError(t, err)
	return frame
}

// requireExactArea measures one returned profile and requires its area to be
// want within a bound below 1e-9 mm².
func requireExactArea(t *testing.T, section *sketch.Sketch, profile *sketch.Profile, want float64) {
	t.Helper()
	measured, err := decad.MeasureProfile(section, profile)
	require.NoError(t, err)
	area, err := measured.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-want), area.Bound.Base()+expectedRounding(want))
	require.Less(t, area.Bound.Base(), 1e-9)
}

// expectedRounding covers the rounding of an expected value the test itself
// computes in binary64, such as 51*math.Pi: a few units in its last place.
func expectedRounding(want float64) float64 {
	return 4 * (math.Nextafter(math.Abs(want), math.Inf(1)) - math.Abs(want))
}

// requireVerifiedExtrude extrudes one returned profile by depth and requires
// the exact volume and a verified mesh.
func requireVerifiedExtrude(t *testing.T, section *sketch.Sketch, profile *sketch.Profile, depth, wantVolume float64) {
	t.Helper()
	child, err := decad.Extrude(t.Context(), decad.New(), section, profile, decad.Distance{
		D: units.Millimeters(depth), Dir: decad.Along,
	})
	require.NoError(t, err)
	volume, err := child.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-wantVolume), volume.Bound.Base()+expectedRounding(wantVolume))
	require.Less(t, volume.Bound.Base(), 1e-9)
	mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

// sectionPoints lists the section Sketch's points as (u, v) pairs, sorted.
func sectionPoints(section *sketch.Sketch) [][2]float64 {
	var out [][2]float64
	for _, p := range section.Points() {
		out = append(out, [2]float64{p.X(), p.Y()})
	}
	slices.SortFunc(out, func(a, b [2]float64) int {
		if a[0] != b[0] {
			return cmpFloat(a[0], b[0])
		}
		return cmpFloat(a[1], b[1])
	})
	return out
}

func cmpFloat(a, b float64) int {
	switch {
	case a < b:
		return -1
	case a > b:
		return 1
	}
	return 0
}

func TestSectionRevolveMeridianAxialStepped(t *testing.T) {
	doc, shaft := steppedShaft(t)
	// The meridian and its mirror share the axis run, so the section is one
	// region of area 2*(10*20 + 6*20).
	for _, tc := range []struct {
		name string
		u    r3.Vec
	}{
		{name: "meridian plane", u: r3.Vec{X: 1}},
		{name: "quarter turn", u: r3.Vec{Y: 1}},
		{name: "reflected", u: r3.Vec{X: -1}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			frame := meridianFrame(t, r3.Vec{}, tc.u, r3.Vec{Z: 1})
			section, profiles, err := shaft.Section(t.Context(), frame)
			require.NoError(t, err)
			require.Len(t, profiles, 1)
			require.True(t, profiles[0].Valid)
			require.Empty(t, profiles[0].Holes)
			require.Equal(t, [][2]float64{
				{-10, 0}, {-10, 20}, {-6, 20}, {-6, 40}, {0, 0}, {0, 40},
				{6, 20}, {6, 40}, {10, 0}, {10, 20},
			}, sectionPoints(section))
			requireExactArea(t, section, profiles[0], 640)
			requireVerifiedExtrude(t, section, profiles[0], 1, 640)
		})
	}
	require.Equal(t, []*decad.Body{shaft}, doc.Bodies())
}

func TestSectionRevolveMeridianAxialTubeTwoRegions(t *testing.T) {
	_, tube := meridianTube(t)
	section, profiles, err := tube.Section(t.Context(), meridianFrame(t, r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Z: 1}))
	require.NoError(t, err)
	require.Len(t, profiles, 2)
	require.NotSame(t, profiles[0], profiles[1])
	require.Equal(t, [][2]float64{
		{-10, 0}, {-10, 20}, {-4, 0}, {-4, 20}, {4, 0}, {4, 20}, {10, 0}, {10, 20},
	}, sectionPoints(section))
	for _, profile := range profiles {
		requireExactArea(t, section, profile, 6*20)
	}
	requireVerifiedExtrude(t, section, profiles[1], 2, 2*6*20)
}

func TestSectionRevolveMeridianAxialArcEnd(t *testing.T) {
	_, capsule := meridianCapsule(t)
	section, profiles, err := capsule.Section(t.Context(), meridianFrame(t, r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Z: 1}))
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	// A 10x10 rectangle under a half disk of radius 5.
	want := 100 + 12.5*math.Pi
	requireExactArea(t, section, profiles[0], want)
	requireVerifiedExtrude(t, section, profiles[0], 1, want)
}

func TestSectionRevolveMeridianNormalLevels(t *testing.T) {
	_, shaft := steppedShaft(t)
	_, tube := meridianTube(t)
	_, capsule := meridianCapsule(t)
	_, notch := revolveSectionMeridian(t, [2]float64{0, 5}, [2]float64{5, 0}, [2]float64{5, 10}, [2]float64{0, 5})
	for _, tc := range []struct {
		name         string
		body         *decad.Body
		z            float64
		outer, inner float64
	}{
		{name: "lower step", body: shaft, z: 10, outer: 10},
		{name: "upper step", body: shaft, z: 30, outer: 6},
		{name: "tube", body: tube, z: 10, outer: 10, inner: 4},
		// The hemisphere at heights 3 and 4 above its centre.
		{name: "dome 3-4-5", body: capsule, z: 13, outer: 4},
		{name: "dome 4-3-5", body: capsule, z: 14, outer: 3},
		// The notch's inner cone line crosses z=2 at r=3.
		{name: "inner cone", body: notch, z: 2, outer: 5, inner: 3},
	} {
		t.Run(tc.name, func(t *testing.T) {
			frame := meridianFrame(t, r3.Vec{Z: tc.z}, r3.Vec{X: 1}, r3.Vec{Y: 1})
			section, profiles, err := tc.body.Section(t.Context(), frame)
			require.NoError(t, err)
			require.Len(t, profiles, 1)
			if tc.inner > 0 {
				require.Len(t, profiles[0].Holes, 1)
			} else {
				require.Empty(t, profiles[0].Holes)
			}
			area := math.Pi * (tc.outer*tc.outer - tc.inner*tc.inner)
			requireExactArea(t, section, profiles[0], area)
			requireVerifiedExtrude(t, section, profiles[0], 2, 2*area)
		})
	}
}

func TestSectionRevolveMeridianPlacement(t *testing.T) {
	_, shaft := steppedShaft(t)
	motion, err := r3.FromBasis(r3.Basis{
		EX: r3.Vec{Y: 1}, EY: r3.Vec{Z: 1}, EZ: r3.Vec{X: 1},
	}, r3.NewVec(20, 30, 40))
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), shaft, motion)
	require.NoError(t, err)
	// The axis now runs along +X through (20, 30, 40).
	axial := meridianFrame(t, r3.NewVec(20, 30, 40), r3.Vec{X: 1}, r3.Vec{Y: 1})
	section, profiles, err := placed.Section(t.Context(), axial)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	requireExactArea(t, section, profiles[0], 640)
	requireVerifiedExtrude(t, section, profiles[0], 1, 640)

	normal := meridianFrame(t, r3.NewVec(30, 0, 0), r3.Vec{Y: 1}, r3.Vec{Z: 1})
	section, profiles, err = placed.Section(t.Context(), normal)
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	requireExactArea(t, section, profiles[0], 100*math.Pi)
	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0], decad.Distance{
		D: units.Millimeters(2), Dir: decad.Along,
	})
	require.NoError(t, err)
	box, err := child.Bounds()
	require.NoError(t, err)
	require.Equal(t, r3.NewVec(30, 20, 30), box.Min)
	require.Equal(t, r3.NewVec(32, 40, 50), box.Max)
}

func TestSectionRevolveMeridianRefusesUnprovedCuts(t *testing.T) {
	doc, shaft := steppedShaft(t)
	_, capsule := meridianCapsule(t)
	_, notch := revolveSectionMeridian(t, [2]float64{0, 5}, [2]float64{5, 0}, [2]float64{5, 10}, [2]float64{0, 5})
	_, slanted := revolveSectionMeridian(t, [2]float64{0, 5}, [2]float64{3, 0}, [2]float64{3, 10}, [2]float64{0, 5})
	level := func(z float64) r3.Frame { return meridianFrame(t, r3.Vec{Z: z}, r3.Vec{X: 1}, r3.Vec{Y: 1}) }
	for _, tc := range []struct {
		name  string
		body  *decad.Body
		plane r3.Frame
	}{
		{name: "shoulder", body: shaft, plane: level(20)},
		{name: "bottom cap", body: shaft, plane: level(0)},
		{name: "top cap level", body: shaft, plane: level(40)},
		{name: "exterior level", body: shaft, plane: level(50)},
		{name: "offset axial", body: shaft, plane: meridianFrame(t, r3.Vec{Y: 1}, r3.Vec{X: 1}, r3.Vec{Z: 1})},
		{name: "parallel to axis", body: shaft, plane: meridianFrame(t, r3.Vec{X: 3}, r3.Vec{Y: 1}, r3.Vec{Z: 1})},
		{name: "oblique", body: shaft, plane: meridianFrame(t, r3.Vec{Z: 10}, r3.Vec{X: 1}, r3.Vec{Y: 1, Z: 1})},
		// The notch meets the axis at one vertex, so its mirror pinches.
		{name: "pinched axial", body: notch, plane: meridianFrame(t, r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Z: 1})},
		{name: "pinch level", body: notch, plane: level(5)},
		// r = 3*(5-1)/5 = 2.4 is not a binary64 value.
		{name: "inexact crossing", body: slanted, plane: level(1)},
		// sqrt(25 - 4) is not a binary64 value.
		{name: "inexact dome radius", body: capsule, plane: level(12)},
		{name: "dome apex", body: capsule, plane: level(15)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			section, profiles, err := tc.body.Section(t.Context(), tc.plane)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.ErrorContains(t, err, "exact meridian cut")
			require.Nil(t, section)
			require.Nil(t, profiles)
		})
	}
	require.Equal(t, []*decad.Body{shaft}, doc.Bodies())
}

// revolveCircleMeridian revolves a full turn about world Z a meridian drawn
// on world XZ: one whole circle, or a 10x20 rectangle on the axis with that
// circle as a hole.
func revolveCircleMeridian(t *testing.T, center [2]float64, radius float64, holed bool) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XZ())
	require.NoError(t, err)
	c := s.CreatePoint(center[0], center[1])
	s.Fix(c)
	s.CreateCircle(c, radius)
	if holed {
		a, b := s.CreatePoint(0, 0), s.CreatePoint(10, 0)
		cc, d := s.CreatePoint(10, 20), s.CreatePoint(0, 20)
		s.CreateLine(a, b)
		s.CreateLine(b, cc)
		s.CreateLine(cc, d)
		s.CreateLine(d, a)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var region *sketch.Profile
	for _, profile := range s.Profiles() {
		if profile.Valid && len(profile.Holes) == map[bool]int{false: 0, true: 1}[holed] {
			region = profile
		}
	}
	require.NotNil(t, region)
	body, err := decad.Revolve(t.Context(), decad.New(), s, region, decad.SketchLine{
		Start: decad.Point2{}, End: decad.Point2{V: 1},
	}, decad.FullRevolution{})
	require.NoError(t, err)
	return body
}

func TestSectionRevolveMeridianCircles(t *testing.T) {
	torus := revolveCircleMeridian(t, [2]float64{10, 0}, 5, false)
	// Two disks of radius 5 either side of the axis.
	section, profiles, err := torus.Section(t.Context(), meridianFrame(t, r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Z: 1}))
	require.NoError(t, err)
	require.Len(t, profiles, 2)
	for _, profile := range profiles {
		requireExactArea(t, section, profile, 25*math.Pi)
	}
	requireVerifiedExtrude(t, section, profiles[0], 2, 50*math.Pi)
	// At z=3 the tube's circle spans r = 10 ± 4.
	section, profiles, err = torus.Section(t.Context(), meridianFrame(t, r3.Vec{Z: 3}, r3.Vec{X: 1}, r3.Vec{Y: 1}))
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	require.Len(t, profiles[0].Holes, 1)
	requireExactArea(t, section, profiles[0], (14*14-6*6)*math.Pi)
	requireVerifiedExtrude(t, section, profiles[0], 2, 2*(14*14-6*6)*math.Pi)

	// A shaft r=10, z 0..20 with a toroidal void whose meridian circle has
	// centre (5, 10) and radius 2.
	void := revolveCircleMeridian(t, [2]float64{5, 10}, 2, true)
	section, profiles, err = void.Section(t.Context(), meridianFrame(t, r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Z: 1}))
	require.NoError(t, err)
	require.Len(t, profiles, 1)
	require.Len(t, profiles[0].Holes, 2)
	requireExactArea(t, section, profiles[0], 400-8*math.Pi)
	requireVerifiedExtrude(t, section, profiles[0], 1, 400-8*math.Pi)
	// Through the void's centre the cut is a disk r=3 inside an annulus 7..10.
	section, profiles, err = void.Section(t.Context(), meridianFrame(t, r3.Vec{Z: 10}, r3.Vec{X: 1}, r3.Vec{Y: 1}))
	require.NoError(t, err)
	require.Len(t, profiles, 2)
	requireExactArea(t, section, profiles[0], 9*math.Pi)
	requireExactArea(t, section, profiles[1], 51*math.Pi)
	requireVerifiedExtrude(t, section, profiles[1], 2, 102*math.Pi)
}
