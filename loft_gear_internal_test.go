package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

// This file holds the helical spur gear fixture docs/loft-gear-bounds-design.md
// measures: module 2, 20° pressure angle, 10 mm face, 20° helix twist, and
// one fit spline through fitPoints involute points per flank.

// loftGear is one spur gear's tooth geometry.
type loftGear struct {
	module, teeth, pressure float64
	fitPoints               int
}

// loftGearZ is the design's gear with z teeth and five fit points per flank.
func loftGearZ(z float64) loftGear {
	return loftGear{module: 2, teeth: z, pressure: 20 * math.Pi / 180, fitPoints: 5}
}

// radii returns the base, root and tip radii.
func (g loftGear) radii() (float64, float64, float64) {
	r := g.module * g.teeth / 2
	return r * math.Cos(g.pressure), r - 1.25*g.module, r + g.module
}

// flank returns one involute flank as (radius, angle) fit points from the
// base circle to the tip circle.
func (g loftGear) flank() [][2]float64 {
	rb, _, ra := g.radii()
	thetaB := math.Pi/(2*g.teeth) + (math.Tan(g.pressure) - g.pressure)
	tmax := math.Sqrt((ra/rb)*(ra/rb) - 1)
	out := make([][2]float64, g.fitPoints)
	for k := range out {
		tt := tmax * float64(k) / float64(g.fitPoints-1)
		out[k] = [2]float64{rb * math.Sqrt(1+tt*tt), -(thetaB - (tt - math.Atan(tt)))}
	}
	return out
}

func loftGearPolar(s *sketch.Sketch, r, ang, rot float64) *sketch.Point {
	p := s.CreatePoint(r*math.Cos(ang+rot), r*math.Sin(ang+rot))
	s.Fix(p)
	return p
}

// sketchOn draws count consecutive teeth on plane: per tooth a root line, a
// fit-spline flank, a tip arc, the mirrored flank and a root line, joined by
// root arcs. One tooth closes with a line across its root.
func (g loftGear) sketchOn(t *testing.T, w *sketch.World, plane *sketch.Plane, count int) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	_, rf, _ := g.radii()
	fl := g.flank()
	thetaB := -fl[0][1]
	origin := s.CreatePoint(0, 0)
	s.Fix(origin)
	pitch := 2 * math.Pi / g.teeth
	var firstRoot, prevRoot *sketch.Point
	for k := range count {
		rot := float64(k) * pitch
		rootR := loftGearPolar(s, rf, -thetaB, rot)
		right := make([]*sketch.Point, len(fl))
		for i, p := range fl {
			right[i] = loftGearPolar(s, p[0], p[1], rot)
		}
		left := make([]*sketch.Point, len(fl))
		for i, p := range fl {
			left[len(fl)-1-i] = loftGearPolar(s, p[0], -p[1], rot)
		}
		rootL := loftGearPolar(s, rf, thetaB, rot)
		s.CreateLine(rootR, right[0])
		_, err = s.CreateFitSpline(right...)
		require.NoError(t, err)
		s.CreateArc(origin, right[len(right)-1], left[0])
		_, err = s.CreateFitSpline(left...)
		require.NoError(t, err)
		s.CreateLine(left[len(left)-1], rootL)
		if prevRoot != nil {
			s.CreateArc(origin, prevRoot, rootR)
		} else {
			firstRoot = rootR
		}
		prevRoot = rootL
	}
	if count == 1 {
		s.CreateLine(prevRoot, firstRoot)
	} else {
		s.CreateArc(origin, prevRoot, firstRoot)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	return s, profiles[0]
}

// loftGearSketches draws count teeth on z = 0 and again on z = 10, the second
// turned by the 20° helix's twist over the face width.
func loftGearSketches(t *testing.T, g loftGear, count int) (*sketch.Sketch, *sketch.Profile, *sketch.Sketch, *sketch.Profile) {
	t.Helper()
	const height = 10.0
	twist := height * math.Tan(20*math.Pi/180) / (g.module * g.teeth / 2)
	w := sketch.NewWorld()
	f0, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	f1, err := r3.NewFrame(r3.NewVec(0, 0, height), r3.NewVec(math.Cos(twist), math.Sin(twist), 0), r3.NewVec(-math.Sin(twist), math.Cos(twist), 0))
	require.NoError(t, err)
	pl0, err := w.CreatePlaneFromFrame(f0)
	require.NoError(t, err)
	pl1, err := w.CreatePlaneFromFrame(f1)
	require.NoError(t, err)
	s0, p0 := g.sketchOn(t, w, pl0, count)
	s1, p1 := g.sketchOn(t, w, pl1, count)
	return s0, p0, s1, p1
}

// loftGearAssembly runs evalLoft's prefix up to assembleLoft over count teeth
// of g and returns the assembled triangle set with its structure.
func loftGearAssembly(t *testing.T, g loftGear, count int) loftAssembly {
	t.Helper()
	s0, p0, s1, p1 := loftGearSketches(t, g, count)
	profile0, plane0, _, err := recordProfile(s0, p0)
	require.NoError(t, err)
	profile1, plane1, _, err := recordProfile(s1, p1)
	require.NoError(t, err)
	work0, work1 := freeform.NewFreeformWork(), freeform.NewFreeformWork()
	offsets, walks0, walks1, target, err := validateLoftRecords(profile0, profile1, plane0, plane1, nil, loftRecordAreas(t, profile0, profile1), work0, work1)
	require.NoError(t, err)
	pairs, _, _, stationRound, err := loftPairings(profile0, profile1, offsets, walks0, walks1, target, work0, work1)
	require.NoError(t, err)
	frame0, err := r3.NewFrame(plane0.Origin, plane0.U, plane0.V)
	require.NoError(t, err)
	frame1, err := r3.NewFrame(plane1.Origin, plane1.U, plane1.V)
	require.NoError(t, err)
	a, err := assembleLoft(t.Context(), pairs, frame0, frame1, plane0, r3.Identity(), stationRound)
	require.NoError(t, err)
	return a
}
