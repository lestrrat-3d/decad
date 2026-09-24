package decad_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

// This file owns the axis-snap charges: what a revolve publishes about a
// profile or chain whose on-axis endpoint the evaluator moved onto the axis
// itself (revolve_axis.go's axisFrame.walk). The snap is a displacement decad
// COMMITS, so every published measurement must enclose the geometry decad
// actually built rather than the geometry it was handed, and each fixture below
// reads the published interval back against that geometry's exact truth in
// rational arithmetic.
//
// Two charges are under test and they land in different places. The wall face
// area's charge is on the wall's own LENGTH: a snapped endpoint moves the wall's
// two ends apart as well as inward, and the recorded length is the unsnapped
// one. The volume's charge is on the region MOMENTS: the faces follow the
// snapped boundary while Pappus integrates the recorded region. A chain
// publishes no volume, so the first charge is provable on both paths and the
// second only on the profile-fed one.

// requireEnclosesPiMultiple asserts that a published measurement's own proven
// interval [value − bound, value + bound] contains num/den · π exactly. π
// arrives as the reference bracket piRefLo/piRefHi (revolve_bounds_test.go) and
// every comparison runs over the rationals, so the truth is never rounded onto
// the held float first — at these scales the miss being pinned is a few hundred
// ulps and rounding would hide it.
//
// The failure message names the interval, the truth and the shortfall, because
// a bare ordering failure on two rationals says nothing about which measurement
// missed what.
func requireEnclosesPiMultiple(t *testing.T, what string, value, bound float64, num, den int64) {
	t.Helper()
	toRat := func(x float64) *big.Rat {
		r := new(big.Rat)
		require.NotNilf(t, r.SetFloat64(x), "%s: %v must be finite to convert exactly", what, x)
		return r
	}
	lo := new(big.Rat).Sub(toRat(value), toRat(bound))
	hi := new(big.Rat).Add(toRat(value), toRat(bound))
	factor := big.NewRat(num, den)
	truthLo := new(big.Rat).Mul(factor, piRefLo)
	truthHi := new(big.Rat).Mul(factor, piRefHi)
	require.LessOrEqualf(t, lo.Cmp(truthLo), 0,
		"%s: the published interval [%s, %s] starts ABOVE the exact %d/%d*pi = %s, missing it by %s; value %v with bound %v",
		what, lo.FloatString(20), hi.FloatString(20), num, den, truthLo.FloatString(20),
		new(big.Rat).Sub(lo, truthLo).FloatString(20), value, bound)
	require.GreaterOrEqualf(t, hi.Cmp(truthHi), 0,
		"%s: the published interval [%s, %s] ends BELOW the exact %d/%d*pi = %s, missing it by %s; value %v with bound %v",
		what, lo.FloatString(20), hi.FloatString(20), num, den, truthHi.FloatString(20),
		new(big.Rat).Sub(truthHi, hi).FloatString(20), value, bound)
}

// TestRevolveChainSnappedPoleEnclosesRadialWallArea is the chain path's own
// proof that the snap's effect on the WALL LENGTH is charged.
//
// The meridian runs from (z = 1e6, r = 9e-4) to (z = 1e6, r = 3). The snap band
// scales with the section, so at z = 1e6 it is 1e-3 and the near end snaps onto
// the axis: the wall decad builds is the full disk of radius 3, whose swept area
// is exactly 9π mm². The recorded wall is 9e-4 mm SHORTER than that disk's
// radius, and it is a radial wall rather than a steep one, so the whole
// discarded radius lands on the length — which is what makes this the regime the
// mean-radius charge alone cannot cover.
func TestRevolveChainSnappedPoleEnclosesRadialWallArea(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	near := s.CreatePoint(1e6, 9e-4)
	s.Fix(near)
	s.CreateLine(near, s.CreatePoint(1e6, 3))
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Chains(), 1)

	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}}
	body, err := decad.New().RevolveChain(s, s.Chains()[0], axis, decad.FullRevolution{})
	require.NoError(t, err)
	require.Len(t, body.Faces(), 1)
	_, err = decad.Edges(decad.Free()).Exactly(1).SelectEdges(body)
	require.NoError(t, err)

	area, err := body.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, area.Exactness)
	requireEnclosesPiMultiple(t, "the snapped chain disk's Area", area.Value.Base(), area.Bound.Base(), 9, 1)
}

// TestRevolveSnappedRimEnclosesAreaAndVolume is the profile-fed path's own
// proof, for both charges at once.
//
// The rectangle's two near corners sit inside the snap band, so both snap onto
// the axis and the near edge becomes an on-axis line sweeping no face: the solid
// decad builds is a closed cylinder of radius 3 and length 5, with Area exactly
// 48π mm² and Volume exactly 45π mm³. Neither reading can reach those figures
// from the recorded rectangle alone — the two disk walls are each short by the
// discarded radius, and Pappus integrates a tube with a bore rather than a solid
// cylinder — so both the wall-length charge and the region-moment charge are
// load-bearing here.
//
// The first row is the smallest snap the geometry admits at that offset and the
// second is a larger one further down the axis, so a charge that happened to
// cover one scale does not pass for the other.
func TestRevolveSnappedRimEnclosesAreaAndVolume(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name        string
		z0, z1      float64
		innerRadius float64
	}{
		{name: "snap band at z=1000", z0: 1000, z1: 1005, innerRadius: 1e-6},
		{name: "wider band at z=1e6", z0: 1e6, z1: 1e6 + 5, innerRadius: 9e-4},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			w := sketch.NewWorld()
			s, err := w.CreateSketch(w.XY())
			require.NoError(t, err)
			p0 := s.CreatePoint(tc.z0, tc.innerRadius)
			p1 := s.CreatePoint(tc.z0, 3)
			p2 := s.CreatePoint(tc.z1, 3)
			p3 := s.CreatePoint(tc.z1, tc.innerRadius)
			s.Fix(p0)
			s.CreateLine(p0, p1)
			s.CreateLine(p1, p2)
			s.CreateLine(p2, p3)
			s.CreateLine(p3, p0)
			_, err = s.Solve(t.Context())
			require.NoError(t, err)
			require.Len(t, s.Profiles(), 1)

			axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}}
			body, err := decad.New().Revolve(s, s.Profiles()[0], axis, decad.FullRevolution{})
			require.NoError(t, err)
			// The on-axis near edge sweeps nothing: two disks and one cylinder.
			require.Len(t, body.Faces(), 3)

			area, err := body.Area()
			require.NoError(t, err)
			requireEnclosesPiMultiple(t, "the snapped cylinder's Area", area.Value.Base(), area.Bound.Base(), 48, 1)

			volume, err := body.Volume()
			require.NoError(t, err)
			require.Equal(t, decad.Approximate, volume.Exactness)
			requireEnclosesPiMultiple(t, "the snapped cylinder's Volume", volume.Value.Base(), volume.Bound.Base(), 45, 1)
		})
	}
}
