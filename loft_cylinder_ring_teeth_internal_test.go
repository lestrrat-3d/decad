package decad

import (
	"context"
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// ringTeethPolygon returns the edges of a closed polygon at height z whose
// vertex radii are radius plus up to jitter, so one ring can cross the blank
// radius and reach the inner, seam and outer edge branches.
func ringTeethPolygon(rng *rand.Rand, sides int, radius, jitter, z float64) [][2]proofarith.Xpt {
	points := make([]proofarith.Xpt, sides)
	for i := range points {
		angle := 2 * math.Pi * float64(i) / float64(sides)
		r := radius + jitter*(2*rng.Float64()-1)
		points[i] = proofarith.XptOf(r3.Vec{X: r * math.Cos(angle), Y: r * math.Sin(angle), Z: z})
	}
	edges := make([][2]proofarith.Xpt, sides)
	for i := range edges {
		edges[i] = [2]proofarith.Xpt{points[i], points[(i+1)%sides]}
	}
	return edges
}

// ringTeethFinger returns a circle of radius 2 at height z with a thin radial
// finger through the blank radius 3 at angle ang, half-width d in radians. Its
// two radial edges are seam edges and its tip edge is an outer edge.
func ringTeethFinger(ang, d, z float64) [][2]proofarith.Xpt {
	polar := func(r, a float64) proofarith.Xpt {
		return proofarith.XptOf(r3.Vec{X: r * math.Cos(a), Y: r * math.Sin(a), Z: z})
	}
	var points []proofarith.Xpt
	for i := range 16 {
		a := 2 * math.Pi * float64(i) / 16
		if i == 1 {
			points = append(points, polar(2.9, ang+d), polar(3.1, ang+d), polar(3.1, ang-d), polar(2.9, ang-d))
		}
		if i == 0 || a > 0.5 {
			points = append(points, polar(2, a))
		}
	}
	edges := make([][2]proofarith.Xpt, len(points))
	for i := range edges {
		edges[i] = [2]proofarith.Xpt{points[i], points[(i+1)%len(points)]}
	}
	return edges
}

func requireRingResultEqual(t *testing.T, want, got loftcylinder.RingOccupancy, wantErr, gotErr error) {
	t.Helper()
	if wantErr != nil {
		require.EqualError(t, gotErr, wantErr.Error())
	} else {
		require.NoError(t, gotErr)
	}
	require.Equal(t, want.Inner, got.Inner)
	require.Equal(t, want.Outer, got.Outer)
	require.Equal(t, want.Seam, got.Seam)
	require.Equal(t, want.Candidates, got.Candidates)
	require.Equal(t, want.MinimumClearance2 == nil, got.MinimumClearance2 == nil)
	if want.MinimumClearance2 != nil {
		require.Zero(t, want.MinimumClearance2.Cmp(got.MinimumClearance2))
	}
}

func requireRingToothEqual(t *testing.T, want, got loftCylinderRingTooth) {
	t.Helper()
	for _, pair := range [][2]*big.Rat{
		{want.circle.Z0, got.circle.Z0}, {want.circle.Z1, got.circle.Z1},
		{want.circle.SafeBottom, got.circle.SafeBottom}, {want.circle.SafeTop, got.circle.SafeTop},
		{want.circle.Radius, got.circle.Radius}, {want.bound2, got.bound2},
	} {
		require.Zero(t, pair[0].Cmp(pair[1]))
	}
	require.Equal(t, len(want.exterior), len(got.exterior))
	require.Equal(t, len(want.fitted), len(got.fitted))
	for i := range want.exterior {
		require.Equal(t, want.exterior[i].Box, got.exterior[i].Box)
	}
	for i := range want.fitted {
		require.Equal(t, want.fitted[i].Box, got.fitted[i].Box)
	}
}

// TestLoftCylinderRingTeethReuseMatchesFresh compares ring checks that share
// one tooth memo with checks that build every record fresh, across rings that
// reach the inner, seam and outer branches, a second radius, a reflected level
// shift and a source whose last tooth cannot build.
func TestLoftCylinderRingTeethReuseMatchesFresh(t *testing.T) {
	t.Parallel()
	_, joined := loftCylinderTabJoin(t)
	fp, ok := joined.payload.(facetedPayload)
	require.True(t, ok)
	source, ok := loftCylinderChamferSourceOf(fp)
	require.True(t, ok)
	require.NoError(t, loftCylinderChamferBridgeSources(&source))
	require.NotEmpty(t, source.teeth)
	level := (source.z0 + source.z1) / 2

	other := source
	other.blankRadius = 2.5
	shifted := source
	shifted.ringLevelShift = make([]*big.Rat, len(source.teeth))
	for i := range shifted.ringLevelShift {
		shifted.ringLevelShift[i] = big.NewRat(1, 8)
	}
	broken := source
	broken.teeth = append([]*Body(nil), source.teeth...)
	broken.teeth[len(broken.teeth)-1] = nil

	rng := rand.New(rand.NewPCG(31, 37))
	rings := [][][2]proofarith.Xpt{
		ringTeethPolygon(rng, 16, 2, 0, level),
		ringTeethPolygon(rng, 16, 3, 0.4, level),
		ringTeethPolygon(rng, 12, 3.02, 0, level),
		ringTeethPolygon(rng, 16, 4, 0.1, level),
		ringTeethPolygon(rng, 8, 2.9, 0.3, level),
		ringTeethFinger(0.05, 0.03, level),
		ringTeethFinger(-0.05, 0.01, level),
		ringTeethFinger(0.2, 0.03, level),
	}
	sources := []loftCylinderChamferSource{source, other, shifted, broken}

	teeth := make(loftCylinderRingTeeth)
	sawOK, sawErr := false, false
	for range 2 {
		for _, src := range sources {
			for _, ring := range rings {
				want, wantErr := loftCylinderSourceRingInsideInputs(t.Context(), src, ring, nil, nil)
				got, gotErr := loftCylinderSourceRingInsideInputs(t.Context(), src, ring, teeth, nil)
				requireRingResultEqual(t, want, got, wantErr, gotErr)
				sawOK = sawOK || wantErr == nil
				sawErr = sawErr || wantErr != nil
			}
		}
	}
	require.True(t, sawOK)
	require.True(t, sawErr)

	// Records are kept per (body, radius), only on success. The tab's source
	// circle builds at its blank radius alone, so the other radius refuses and
	// the nil tooth refuses: neither may leave a record, and neither may be
	// answered from the blank radius record.
	require.Len(t, teeth, len(source.teeth))
	for _, body := range source.teeth {
		want, err := loftCylinderRingToothOf(body, source.blankRadius)
		require.NoError(t, err)
		got, cached := teeth[loftCylinderRingToothKey{body: body, radius: source.blankRadius}]
		require.True(t, cached)
		requireRingToothEqual(t, want, got)
		_, cached = teeth[loftCylinderRingToothKey{body: body, radius: other.blankRadius}]
		require.False(t, cached)
	}
	_, err := loftCylinderRingToothOf(source.teeth[0], other.blankRadius)
	require.Error(t, err)

	// A cancelled context returns the context error and leaves a fresh memo empty.
	cancelled, cancel := context.WithCancel(t.Context())
	cancel()
	empty := make(loftCylinderRingTeeth)
	_, err = loftCylinderSourceRingInsideInputs(cancelled, source, rings[0], empty, nil)
	require.ErrorIs(t, err, context.Canceled)
	require.Empty(t, empty)

	// A ring that crosses the blank radius inside the tooth sector succeeds
	// through the facet entry point with seam edges, so the seam records feed
	// a real proof; a second call on the shared memo equals a fresh call.
	finger := ringTeethFinger(0.05, 0.03, level)
	fresh, err := loftCylinderSourceRingInside(t.Context(), fp, finger, nil, nil)
	require.NoError(t, err)
	require.Positive(t, fresh.Seam)
	require.Positive(t, fresh.Outer)
	seamTeeth := make(loftCylinderRingTeeth)
	for range 2 {
		got, err := loftCylinderSourceRingInside(t.Context(), fp, finger, seamTeeth, nil)
		require.NoError(t, err)
		requireRingResultEqual(t, fresh, got, nil, nil)
	}
	require.Len(t, seamTeeth, len(source.teeth))

	// The facet entry point reads the same records.
	want, wantErr := loftCylinderSourceRingInsideInputs(t.Context(), source, rings[0], nil, nil)
	got, gotErr := loftCylinderSourceRingInside(t.Context(), fp, rings[0], teeth, nil)
	require.Equal(t, wantErr == nil, gotErr == nil)
	if wantErr == nil {
		requireRingResultEqual(t, want, got, wantErr, gotErr)
	}
}
