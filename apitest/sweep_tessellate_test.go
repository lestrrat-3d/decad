package apitest_test

import (
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// sweepHexagonCorners is docs/multibody-dynamics-design.md §2's parts-bin
// hexagon: 20 mm across flats on dyadic corners, so every mesh coordinate is
// exact.
var sweepHexagonCorners = [][2]float64{{-11.5, 0}, {-5.75, -10}, {5.75, -10}, {11.5, 0}, {5.75, 10}, {-5.75, 10}}

// TestSweepStraightTessellatesAsItsPrism is docs/sweep-design.md Table D row
// D2's one exception: §2's straight 14 mm hexagon sweep meshes as the prism it
// reduced to. The same profile's Extrude is the reference, so the comparison
// is against the real prism tessellator's own output rather than hand-written
// coordinates.
//
// Leg shown to fail: with tessellateBodyContext's sweep arm reading the walls
// through prismWallRole instead of sweepWallRole (the span prefix deleted),
// Tessellate refuses with ErrDegenerate `the body carries no face for role
// "side(0,0)"`, and this test goes red at its first NoError.
func TestSweepStraightTessellatesAsItsPrism(t *testing.T) {
	t.Parallel()

	tol := units.Millimeters(0.1)

	w := sketch.NewWorld()
	s, profile := meshPolygonSketch(t, w, w.XY(), sweepHexagonCorners)
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.NewVec(0, 0, 14)})
	require.NoError(t, err)
	sweepDoc := decad.New()
	swept, err := sweepDoc.Sweep(t.Context(), s, profile, path)
	require.NoError(t, err)
	got, err := swept.Tessellate(t.Context(), tol)
	require.NoError(t, err)

	ew := sketch.NewWorld()
	es, eprofile := meshPolygonSketch(t, ew, ew.XY(), sweepHexagonCorners)
	extruded, err := decad.New().Extrude(es, eprofile, decad.Distance{D: units.Millimeters(14), Dir: decad.Along})
	require.NoError(t, err)
	want, err := extruded.Tessellate(t.Context(), tol)
	require.NoError(t, err)

	// Six walls of two triangles each, and two caps of four each.
	require.Len(t, want.Triangles(), 20)
	require.Equal(t, want.Vertices(), got.Vertices())
	require.Equal(t, want.Triangles(), got.Triangles())
	require.Zero(t, want.Bound().Base())
	require.Equal(t, want.Bound(), got.Bound())
	require.True(t, want.BoundaryVerified())
	require.True(t, got.BoundaryVerified())
	require.True(t, want.VolumeVerified())
	require.True(t, got.VolumeVerified())

	// Every triangle names a live face of the sweep itself, under the role the
	// extrude's triangle at the same index carries with the span prefix added.
	live := map[*decad.Face]struct{}{}
	for _, f := range swept.Faces() {
		live[f] = struct{}{}
	}
	wantSources := want.SourceFaces()
	gotSources := got.SourceFaces()
	require.Len(t, gotSources, len(got.Triangles()))
	walls := 0
	for i, f := range gotSources {
		require.Contains(t, live, f, "triangle %d names a face the sweep does not carry", i)
		gotOrigins := f.Origins()
		wantOrigins := wantSources[i].Origins()
		require.Len(t, gotOrigins, 1)
		require.Len(t, wantOrigins, 1)
		role := wantOrigins[0].Role
		if suffix, ok := strings.CutPrefix(role, "side("); ok {
			role = "side(0," + suffix
			walls++
		}
		require.Equal(t, role, gotOrigins[0].Role, "triangle %d", i)
	}
	require.Equal(t, 12, walls)
}

// TestSweepStraightIsABooleanOperand is Table D row D3 for the same
// reduction: its mesh carries the prism's occupied-volume proof, so a mesh
// Union takes it and reads the volume the same profile's Extrude reads in its
// place. The box is placed off every hexagon plane so the mesh path classifies
// no coplanar facet pair.
//
// Leg shown to fail: with the sweep arm refusing every sweep, Union refuses
// the sweep operand and this test goes red at its NoError.
func TestSweepStraightIsABooleanOperand(t *testing.T) {
	t.Parallel()

	move, err := r3.Translation(r3.NewVec(0.25, 0.5, 3))
	require.NoError(t, err)
	union := func(t *testing.T, build func(doc *decad.Document, s *sketch.Sketch, p *sketch.Profile) (*decad.Body, error)) decad.Measurement {
		t.Helper()
		w := sketch.NewWorld()
		s, p := meshPolygonSketch(t, w, w.XY(), sweepHexagonCorners)
		doc := decad.New()
		hexagon, err := build(doc, s, p)
		require.NoError(t, err)
		bw := sketch.NewWorld()
		bs, bp := meshPolygonSketch(t, bw, bw.XY(), [][2]float64{{0, 0}, {20, 0}, {20, 20}, {0, 20}})
		box, err := doc.Extrude(bs, bp, decad.Distance{D: units.Millimeters(4), Dir: decad.Along})
		require.NoError(t, err)
		box, err = box.Placed(t.Context(), move)
		require.NoError(t, err)
		out, err := decad.Union(t.Context(), hexagon, box)
		require.NoError(t, err)
		volume, err := out.Volume()
		require.NoError(t, err)
		return volume
	}
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.NewVec(0, 0, 14)})
	require.NoError(t, err)
	got := union(t, func(doc *decad.Document, s *sketch.Sketch, p *sketch.Profile) (*decad.Body, error) {
		return doc.Sweep(t.Context(), s, p, path)
	})
	want := union(t, func(doc *decad.Document, s *sketch.Sketch, p *sketch.Profile) (*decad.Body, error) {
		return doc.Extrude(s, p, decad.Distance{D: units.Millimeters(14), Dir: decad.Along})
	})
	require.Equal(t, want, got)
	// The hexagon holds 345 mm² × 14 mm and the box 20 × 20 × 4 mm; the
	// interval must enclose their sum less a positive overlap.
	require.Less(t, got.Value.Base()-got.Bound.Base(), 4830.0+1600.0)
	require.Greater(t, got.Value.Base()+got.Bound.Base(), 4830.0)
}

func TestSweepArcTessellatesAsItsRevolve(t *testing.T) {
	t.Parallel()
	s, profile := plateSketch(t)
	swept, err := decad.New().Sweep(t.Context(), s, profile, sweepArcPath(t))
	require.NoError(t, err)
	got, err := swept.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)

	revolved, err := decad.New().Revolve(s, profile,
		decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 100, V: 0}},
		decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
	require.NoError(t, err)
	want, err := revolved.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.Equal(t, want.Vertices(), got.Vertices())
	require.Equal(t, want.Triangles(), got.Triangles())
	require.Equal(t, want.Bound(), got.Bound())
	require.True(t, got.BoundaryVerified())
	require.True(t, got.VolumeVerified())
	unverified, err := swept.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyNone))
	require.NoError(t, err)
	require.Equal(t, got.Vertices(), unverified.Vertices())
	require.Equal(t, got.Triangles(), unverified.Triangles())
	require.False(t, unverified.BoundaryVerified())
	require.False(t, unverified.VolumeVerified())
	live := map[*decad.Face]struct{}{}
	for _, face := range swept.Faces() {
		live[face] = struct{}{}
	}
	for _, face := range got.SourceFaces() {
		require.Contains(t, live, face)
	}
	motion, err := r3.Translation(r3.NewVec(7, -3, 11))
	require.NoError(t, err)
	placedSweep, err := swept.Placed(t.Context(), motion)
	require.NoError(t, err)
	placedRevolve, err := revolved.Placed(t.Context(), motion)
	require.NoError(t, err)
	gotPlaced, err := placedSweep.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	wantPlaced, err := placedRevolve.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.Equal(t, wantPlaced.Vertices(), gotPlaced.Vertices())
	require.Equal(t, wantPlaced.Triangles(), gotPlaced.Triangles())
	require.Equal(t, wantPlaced.Bound(), gotPlaced.Bound())
	require.True(t, gotPlaced.VolumeVerified())
}

func TestSweepArcTessellationMapsReversedCaps(t *testing.T) {
	t.Parallel()
	s, profile := plateSketch(t)
	path, err := decad.NewPath(r3.NewVec(20, 110, 0), decad.ArcThrough{
		Through: r3.NewVec(20, 106, 8), End: r3.NewVec(20, 100, 10),
	})
	require.NoError(t, err)
	body, err := decad.New().Sweep(t.Context(), s, profile, path)
	require.NoError(t, err)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Contains(t, mesh.SourceFaces(), faceByRole(t, body, "capStart"))
	require.Contains(t, mesh.SourceFaces(), faceByRole(t, body, "capEnd"))
}

func TestSweepArcIsABooleanOperand(t *testing.T) {
	t.Parallel()
	s, profile := plateSketch(t)
	doc := decad.New()
	swept, err := doc.Sweep(t.Context(), s, profile, sweepArcPath(t))
	require.NoError(t, err)
	bw := sketch.NewWorld()
	bs, bp := meshPolygonSketch(t, bw, bw.XY(), [][2]float64{
		{20, 45}, {40, 45}, {40, 65}, {20, 65},
	})
	box, err := doc.Extrude(bs, bp, decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	require.NoError(t, err)
	move, err := r3.Translation(r3.NewVec(0, 0, 20))
	require.NoError(t, err)
	box, err = box.Placed(t.Context(), move)
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), swept, box)
	require.NoError(t, err)
	volume, err := joined.Volume()
	require.NoError(t, err)
	sweepVolume, err := swept.Volume()
	require.NoError(t, err)
	boxVolume, err := box.Volume()
	require.NoError(t, err)
	require.Greater(t, volume.Value.Base()+volume.Bound.Base(), sweepVolume.Value.Base())
	require.Less(t, volume.Value.Base()-volume.Bound.Base(),
		sweepVolume.Value.Base()+boxVolume.Value.Base())
}

// TestSweepCompositeTessellationRefused keeps D2 staged for a composite path.
// Deleting the span-count gate meshes its unused prism field instead.
func TestSweepCompositeTessellationRefused(t *testing.T) {
	t.Parallel()
	s, profile := plateSketch(t)
	path, err := decad.NewPath(
		r3.Vec{},
		decad.LineTo{End: r3.NewVec(0, 0, 5)},
		decad.LineTo{End: r3.NewVec(0, 0, 10)},
	)
	require.NoError(t, err)
	body, err := decad.New().Sweep(t.Context(), s, profile, path)
	require.NoError(t, err)
	_, err = body.Tessellate(t.Context(), units.Millimeters(0.1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
}
