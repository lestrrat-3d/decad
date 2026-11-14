package apitest_test

import (
	"math"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The pairwise disjointness proof of agent-docs/mirror-pattern-design.md §6.3:
// a pattern whose single arrangement scene would exceed the arranger cap is
// proven apart one box-overlapping pair at a time.

// roundedPort extrudes the siren's stator port: a trapezoid of two vertical
// chords at x = rIn·cos(half) and x = rOut·cos(half) between two lines through
// the origin at ±half, every corner rounded by round. It returns the body and
// its exact plan area.
func (sc *mirrorScene) roundedPort(t *testing.T, plane *sketch.Plane, rIn, rOut, half, round float64,
	e decad.Extent) (*decad.Body, float64) {
	t.Helper()
	s, err := sc.w.CreateSketch(plane)
	require.NoError(t, err)
	cos, sin := math.Cos(half), math.Sin(half)
	p1, p2 := s.CreatePoint(rIn*cos, -rIn*sin), s.CreatePoint(rIn*cos, rIn*sin)
	p3, p4 := s.CreatePoint(rOut*cos, -rOut*sin), s.CreatePoint(rOut*cos, rOut*sin)
	inner, outer := s.CreateLine(p1, p2), s.CreateLine(p3, p4)
	right, left := s.CreateLine(p1, p3), s.CreateLine(p2, p4)
	x1 := s.CreatePoint(10, 0)
	ref := s.CreateLine(s.Origin(), x1)
	ref.SetConstruction(true)
	deg := half * 180 / math.Pi
	s.AddConstraint(
		sketch.NewHorizontal(ref), sketch.NewDistance(s.Origin(), x1, 10),
		sketch.NewVertical(inner), sketch.NewDistancePointLine(s.Origin(), inner, rIn*cos),
		sketch.NewVertical(outer), sketch.NewDistancePointLine(s.Origin(), outer, rOut*cos),
		sketch.NewPointOnLine(s.Origin(), right), sketch.NewPointOnLine(s.Origin(), left),
		sketch.NewAngle(ref, right, -deg), sketch.NewAngle(ref, left, deg),
	)
	for _, c := range []struct {
		a, b *sketch.Line
		at   *sketch.Point
	}{{inner, right, p1}, {left, inner, p2}, {right, outer, p3}, {outer, left, p4}} {
		_, err := sketch.CreateFillet(s, c.a, c.b, c.at, round)
		require.NoError(t, err)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var profile *sketch.Profile
	for _, p := range s.Profiles() {
		if p.Valid && len(p.Holes) == 0 {
			require.Nil(t, profile)
			profile = p
		}
	}
	require.NotNil(t, profile)
	body, err := decad.Extrude(t.Context(), sc.doc, s, profile, e)
	require.NoError(t, err)

	// A round of radius r at an interior angle θ removes r²(cot(θ/2) − (π−θ)/2).
	corner := func(theta float64) float64 {
		return round * round * (1/math.Tan(theta/2) - (math.Pi-theta)/2)
	}
	area := math.Tan(half)*(rOut*rOut-rIn*rIn)*cos*cos -
		2*(corner(math.Pi/2+half)+corner(math.Pi/2-half))
	return body, area
}

// TestPatternedPairwiseRing is the siren stator's 20 rounded ports: one
// arrangement scene of every port charges 20560 arranger segments against a
// cap of 4096, so only the pairwise proof admits the ring. The group holds 20
// lumps and the chamfered disc it cuts keeps the volume of the closed form.
//
// Shown red: without the pairwise proof Patterned refuses with the "analytic
// disjointness scene charges at least 4112" message.
func TestPatternedPairwiseRing(t *testing.T) {
	t.Parallel()
	const (
		count  = 20
		height = 5.0
	)
	sc := newMirrorScene()
	plane, err := sc.w.CreateOffsetPlane(sc.w.XY(), -1)
	require.NoError(t, err)
	port, area := sc.roundedPort(t, plane, 30, 42, math.Pi/(2*count), 1, mirrorAlong(height))

	start := time.Now()
	ring, err := decad.Patterned(t.Context(), port, decad.CircularPattern{Axis: patternZ, Count: count})
	require.NoError(t, err)
	require.Less(t, time.Since(start), 5*time.Second)
	require.False(t, tookMeshPath(ring))
	require.Len(t, ring.Lumps(), count)
	v, err := ring.Volume()
	require.NoError(t, err)
	want := count * area * height
	require.InDelta(t, want, volumeMM(t, v), 1e-6*want+bound(t, v))
	require.Equal(t, []*decad.Body{ring}, sc.doc.Bodies())

	disc := sc.cylinder(t, sc.w.XY(), 0, 0, 58, mirrorAlong(3))
	loops := decad.Edges(decad.OuterLoopOf(decad.CapStart(disc))).Or(decad.OuterLoopOf(decad.CapEnd(disc)))
	disc, err = decad.Chamfer(t.Context(), disc, loops, units.Millimeters(0.5))
	require.NoError(t, err)
	cut, err := decad.Cut(t.Context(), disc, ring)
	require.NoError(t, err)
	dv, err := disc.Volume()
	require.NoError(t, err)
	cv, err := cut.Volume()
	require.NoError(t, err)
	// The ports pass through the 3 mm disc.
	wantCut := volumeMM(t, dv) - count*area*3
	require.InDelta(t, wantCut, volumeMM(t, cv), 1e-6*wantCut+bound(t, dv)+bound(t, cv))
}

// bound is the measurement's own error bound in cubic millimetres.
func bound(t *testing.T, m decad.Measurement) float64 {
	t.Helper()
	b, err := m.Bound.In(units.CubicMillimeter)
	require.NoError(t, err)
	return b
}

// TestPatternedPairwiseRingOverlap is the same over-cap ring with ports so wide
// that neighbours overlap. The pairwise proof must not admit it as a group of
// disjoint lumps: Patterned takes §6.1's rule 3 or refuses.
func TestPatternedPairwiseRingOverlap(t *testing.T) {
	t.Parallel()
	const count = 20
	sc := newMirrorScene()
	plane, err := sc.w.CreateOffsetPlane(sc.w.XY(), -1)
	require.NoError(t, err)
	port, _ := sc.roundedPort(t, plane, 30, 42, 2.5*math.Pi/(2*count), 1, mirrorAlong(5))
	ring, err := decad.Patterned(t.Context(), port, decad.CircularPattern{Axis: patternZ, Count: count})
	if err != nil {
		require.ErrorIs(t, err, decad.ErrUnsupported)
		return
	}
	require.NotEqual(t, count, len(ring.Lumps()), "overlapping ports are not %d disjoint lumps", count)
}
