package apitest_test

import (
	"math"
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

// TestSweepCompositeTessellatesSharedGrid covers a real arc-line-arc solid.
// Each join point must use one mesh vertex across both adjacent spans.
func TestSweepCompositeTessellatesSharedGrid(t *testing.T) {
	t.Parallel()
	s, profile, path, _, joins := orthogonalSweepFixture(t)
	body, err := decad.New().Sweep(t.Context(), s, profile, path)
	require.NoError(t, err)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Greater(t, meshVolume(mesh), 0.0)
	unverified, err := body.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyNone))
	require.NoError(t, err)
	require.Equal(t, mesh.Vertices(), unverified.Vertices())
	require.Equal(t, mesh.Triangles(), unverified.Triangles())
	require.False(t, unverified.BoundaryVerified())
	require.False(t, unverified.VolumeVerified())
	require.Len(t, mesh.SourceFaces(), len(mesh.Triangles()))
	live := map[*decad.Face]struct{}{}
	for _, face := range body.Faces() {
		live[face] = struct{}{}
	}
	for _, face := range mesh.SourceFaces() {
		require.Contains(t, live, face)
	}
	for _, join := range joins {
		for _, point := range join {
			matches := 0
			for _, vertex := range mesh.Vertices() {
				if vertex.Sub(point).Len() < 1e-9 {
					matches++
				}
			}
			require.Equal(t, 1, matches, "join point %v must have one shared mesh vertex", point)
		}
	}
}

func TestSweepCompositePolygonHoleTessellates(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	outer := s.CreateRectangle(-1, -1, 1, 1)
	s.Fix(outer.A)
	s.CreateRectangle(-0.5, -0.5, 0.5, 0.5)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var profile *sketch.Profile
	for _, candidate := range s.Profiles() {
		if candidate.Valid && len(candidate.Holes) == 1 {
			profile = candidate
			break
		}
	}
	require.NotNil(t, profile)
	body, err := decad.New().Sweep(t.Context(), s, profile, orthogonalSweepPath(t))
	require.NoError(t, err)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.05))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())

	outerSketch, outerProfile, outerPath, _, _ := orthogonalSweepFixture(t)
	full, err := decad.New().Sweep(t.Context(), outerSketch, outerProfile, outerPath)
	require.NoError(t, err)
	holeVolume, err := body.Volume()
	require.NoError(t, err)
	fullVolume, err := full.Volume()
	require.NoError(t, err)
	require.InDelta(t, fullVolume.Value.Base()*0.75, holeVolume.Value.Base(), 1e-8)
	require.InDelta(t, holeVolume.Value.Base(), meshVolume(mesh), holeVolume.Value.Base()*0.01)
}

func TestSweepCompositePlacedMesh(t *testing.T) {
	t.Parallel()
	s, profile, path, _, _ := orthogonalSweepFixture(t)
	base, err := decad.New().Sweep(t.Context(), s, profile, path)
	require.NoError(t, err)
	baseMesh, err := base.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	move, err := r3.Translation(r3.NewVec(0.1, 0.2, 0.3))
	require.NoError(t, err)
	placed, err := base.PlacedCopy(t.Context(), move)
	require.NoError(t, err)
	mesh, err := placed.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Len(t, mesh.Triangles(), len(baseMesh.Triangles()))
	require.InDelta(t, meshVolume(baseMesh), meshVolume(mesh), 1e-8)
}

func TestSweepCompositeSheetMesh(t *testing.T) {
	t.Parallel()
	s, profile, path, _, _ := orthogonalSweepFixture(t)
	sheet, err := decad.New().Sweep(t.Context(), s, profile, path, decad.WithSurfaceResult())
	require.NoError(t, err)
	require.Equal(t, decad.BodySheet, sheet.Kind())
	mesh, err := sheet.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.False(t, mesh.VolumeVerified())
	require.Len(t, mesh.SourceFaces(), len(mesh.Triangles()))
	live := map[*decad.Face]struct{}{}
	for _, face := range sheet.Faces() {
		live[face] = struct{}{}
	}
	for _, face := range mesh.SourceFaces() {
		require.Contains(t, live, face)
	}
}

func TestSweepCompositeMeshBoolean(t *testing.T) {
	t.Parallel()
	s, profile, path, _, _ := orthogonalSweepFixture(t)
	doc := decad.New()
	swept, err := doc.Sweep(t.Context(), s, profile, path)
	require.NoError(t, err)
	w := sketch.NewWorld()
	bs, bp := meshPolygonSketch(t, w, w.XY(), [][2]float64{{0, 0}, {2, 0}, {2, 2}, {0, 2}})
	box, err := doc.Extrude(bs, bp, decad.Distance{D: units.Millimeters(2), Dir: decad.Along})
	require.NoError(t, err)
	move, err := r3.Translation(r3.NewVec(10.25, 0.25, 4.5))
	require.NoError(t, err)
	box, err = box.Placed(t.Context(), move)
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), swept, box)
	require.NoError(t, err)
	volume, err := joined.Volume()
	require.NoError(t, err)
	// The line span has a 2 × 0.75 × 1.5 mm³ overlap with the box.
	want := 40 + 20*math.Pi + 8 - 2.25
	require.LessOrEqual(t, volume.Value.Base()-volume.Bound.Base(), want)
	require.GreaterOrEqual(t, volume.Value.Base()+volume.Bound.Base(), want)
}

func TestSweepCompositeCircularHoleTessellates(t *testing.T) {
	t.Parallel()
	s, profile := orthogonalSweepProfileWithHole(t)
	path := orthogonalSweepPath(t)
	body, err := decad.New().Sweep(t.Context(), s, profile, path)
	require.NoError(t, err)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	volume, err := body.Volume()
	require.NoError(t, err)
	require.InDelta(t, volume.Value.Base(), meshVolume(mesh), volume.Value.Base()*0.02)
	sheet, err := decad.New().Sweep(t.Context(), s, profile, path, decad.WithSurfaceResult())
	require.NoError(t, err)
	sheetMesh, err := sheet.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.True(t, sheetMesh.BoundaryVerified())
	require.False(t, sheetMesh.VolumeVerified())
}

func TestSweepCompositeStraightCircularHoleMesh(t *testing.T) {
	t.Parallel()
	s, profile := orthogonalSweepProfileWithHole(t)
	path, err := decad.NewPath(r3.Vec{},
		decad.LineTo{End: r3.NewVec(0, 0, 5)},
		decad.LineTo{End: r3.NewVec(0, 0, 10)})
	require.NoError(t, err)
	body, err := decad.New().Sweep(t.Context(), s, profile, path)
	require.NoError(t, err)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	volume, err := body.Volume()
	require.NoError(t, err)
	require.InDelta(t, (4-math.Pi/4)*10, volume.Value.Base(), 1e-8)
	require.Greater(t, meshVolume(mesh), volume.Value.Base())
	require.Less(t, meshVolume(mesh), 40.0)
}
