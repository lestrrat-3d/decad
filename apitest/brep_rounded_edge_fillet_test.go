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

func roundedPlateFrontTopEdge() decad.EdgeSelector {
	return decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0)),
		decad.EndpointAt(r3.NewVec(3, 0, 20))).Exactly(1)
}

// TestBrepRoundedEdgeFilletPublicP8 builds the actual P8 Sketch→Fillet→Cut
// source before rounding its front top edge. The radius-3 corner cylinders
// are the two curved third faces that route E refuses as SB7.
func TestBrepRoundedEdgeFilletPublicP8(t *testing.T) {
	t.Parallel()
	plate := roundedDrilledPlate(t)
	got, err := plate.Fillet(t.Context(), roundedPlateFrontTopEdge(), units.Millimeters(1))
	require.NoError(t, err)
	require.Len(t, got.Faces(), 12)
	requireEveryEdgeOnTwoFaces(t, got)

	volume, err := got.Volume()
	require.NoError(t, err)
	require.Less(t, volume.Bound.Base(), 1.2)
	// The source has volume 15280. The straight 34 mm run loses
	// 34(1−π/4). Each rounded end loses the integral of the circular
	// segment at the radius-3 wall, from x=3−√5 to x=3. Monotone
	// left/right sums at 10,000 steps give the conservative enclosure
	// [15272.2670, 15272.2672] mm³, including float evaluation slack.
	value := new(big.Rat).SetFloat64(volume.Value.Base())
	bound := new(big.Rat).SetFloat64(volume.Bound.Base())
	lo, _ := new(big.Rat).SetString("15272.2670")
	hi, _ := new(big.Rat).SetString("15272.2672")
	require.LessOrEqual(t, new(big.Rat).Sub(value, bound).Cmp(lo), 0)
	require.GreaterOrEqual(t, new(big.Rat).Add(value, bound).Cmp(hi), 0)
	centroid, err := got.Centroid()
	require.NoError(t, err)
	// The two rounded ends and centered bore make x=20 an exact symmetry
	// plane even after the front edge is rounded.
	cx := new(big.Rat).SetFloat64(centroid.Value.X)
	cb := new(big.Rat).SetFloat64(centroid.Bound.Base())
	require.LessOrEqual(t, new(big.Rat).Abs(new(big.Rat).Sub(cx, big.NewRat(20, 1))).Cmp(cb), 0)
	area, err := got.Area()
	require.NoError(t, err)
	require.Positive(t, area.Value.Base())
	require.Positive(t, area.Bound.Base())

	mesh, err := got.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	report, err := got.Document().Verify(t.Context())
	require.NoError(t, err)
	reading, err := report.ForBody(got)
	require.NoError(t, err)
	require.Equal(t, decad.Sound, reading.Status)
	require.Equal(t, decad.ToleranceSatisfied, reading.Area.Tolerance.State)
	require.Equal(t, decad.ToleranceSatisfied, reading.Bounds.Tolerance.State)
}

func TestBrepRoundedEdgeFilletPublicAdmission(t *testing.T) {
	t.Parallel()
	plate := roundedDrilledPlate(t)
	_, err := plate.Fillet(t.Context(), roundedPlateFrontTopEdge(), units.Millimeters(0.5))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	// A bore whose top reaches the cutter's z=19 boundary is outside the
	// admitted source. The producer still uses the public Sketch and Cut.
	near := roundedDrilledPlateWithBoreZ(t, 16)
	_, err = near.Fillet(t.Context(), roundedPlateFrontTopEdge(), units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	// Refusals leave the exact source live for the admitted request.
	_, err = plate.Fillet(t.Context(), roundedPlateFrontTopEdge(), units.Millimeters(1))
	require.NoError(t, err)
}

func roundedDrilledPlateWithBoreZ(t *testing.T, boreZ float64) *decad.Body {
	t.Helper()
	doc := decad.New()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 40, 20)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	box, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	require.NoError(t, err)
	rounded, err := box.Fillet(t.Context(), decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1))).Exactly(4),
		units.Millimeters(3))
	require.NoError(t, err)
	plane, err := w.CreateOffsetPlane(w.XZ(), -10)
	require.NoError(t, err)
	ds, err := w.CreateSketch(plane)
	require.NoError(t, err)
	c := ds.CreatePoint(20, boreZ)
	ds.Fix(c)
	ds.CreateCircle(c, 3)
	_, err = ds.Solve(t.Context())
	require.NoError(t, err)
	drill, err := doc.Extrude(ds, ds.Profiles()[0], decad.Symmetric{D: units.Millimeters(11)})
	require.NoError(t, err)
	out, err := decad.Cut(t.Context(), rounded, drill)
	require.NoError(t, err)
	return out
}
