package apitest_test

import (
	"fmt"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestBrepChamferAllStraightOuterEdges(t *testing.T) {
	t.Parallel()
	_, bar := crossDrilledBar(t)
	baseMesh, err := bar.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	selected := decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0))).
		Or(decad.ParallelTo(r3.NewVec(0, 1, 0))).
		Or(decad.ParallelTo(r3.NewVec(0, 0, 1))).Exactly(12)
	got, err := bar.Chamfer(t.Context(), selected, units.Millimeters(1))
	require.NoError(t, err)
	requireEveryEdgeOnTwoFaces(t, got)
	require.Len(t, got.Faces(), 19)
	volume, err := got.Volume()
	require.NoError(t, err)
	requireAnalyticPiSqrt2Interval(t, volume.Value.Base(), volume.Bound.Base(), 15846, -180, 0)
	area, err := got.Area()
	require.NoError(t, err)
	requireAnalyticPiSqrt2Interval(t, area.Value.Base(), area.Bound.Base(), 3384, 102, 302)
	centroid, err := got.Centroid()
	require.NoError(t, err)
	require.Equal(t, r3.NewVec(20, 10, 10), centroid.Value)
	mesh, err := got.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.InDelta(t, 154.0, meshVolume(baseMesh)-meshVolume(mesh), 1e-8)
	require.Len(t, mesh.SourceFaces(), len(mesh.Triangles()))
	used := map[*decad.Face]bool{}
	for _, face := range mesh.SourceFaces() {
		used[face] = true
	}
	for _, face := range got.Faces() {
		require.True(t, used[face])
	}
}

// requireAnalyticPiSqrt2Interval compares the published measurement interval
// with rational enclosures of a+bπ+c√2, independently of its float formula.
func requireAnalyticPiSqrt2Interval(t *testing.T, held, bound float64, a, b, c int64) {
	t.Helper()
	floatRat := func(v float64) *big.Rat { return new(big.Rat).SetFloat64(v) }
	pi := [2]float64{math.Nextafter(math.Pi, 0), math.Nextafter(math.Pi, math.Inf(1))}
	sqrt2 := [2]float64{math.Nextafter(math.Sqrt2, 0), math.Nextafter(math.Sqrt2, math.Inf(1))}
	intervalEnd := func(high bool) *big.Rat {
		result := big.NewRat(a, 1)
		for _, term := range []struct {
			coefficient int64
			bounds      [2]float64
		}{{b, pi}, {c, sqrt2}} {
			index := 0
			if high != (term.coefficient < 0) {
				index = 1
			}
			result.Add(result, new(big.Rat).Mul(big.NewRat(term.coefficient, 1), floatRat(term.bounds[index])))
		}
		return result
	}
	require.LessOrEqual(t, new(big.Rat).Sub(floatRat(held), floatRat(bound)).Cmp(intervalEnd(false)), 0)
	require.GreaterOrEqual(t, new(big.Rat).Add(floatRat(held), floatRat(bound)).Cmp(intervalEnd(true)), 0)
}

func TestBrepChamferAllStraightOuterEdgesSetbackAdmission(t *testing.T) {
	t.Parallel()
	_, bar := crossDrilledBar(t)
	selected := decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0))).
		Or(decad.ParallelTo(r3.NewVec(0, 1, 0))).
		Or(decad.ParallelTo(r3.NewVec(0, 0, 1))).Exactly(12)
	_, err := bar.Chamfer(t.Context(), selected, units.Millimeters(0.1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	got, err := bar.Chamfer(t.Context(), selected, units.Millimeters(0.5))
	require.NoError(t, err)
	mesh, err := got.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestBrepChamferAllStraightOuterEdgesPlacement(t *testing.T) {
	quarterTurn, err := r3.FromBasis(r3.Basis{
		EX: r3.NewVec(0, 0, 1), EY: r3.NewVec(0, 1, 0), EZ: r3.NewVec(-1, 0, 0),
	}, r3.NewVec(20, 0, 0))
	require.NoError(t, err)
	reflection, err := r3.FromBasis(r3.Basis{
		EX: r3.NewVec(-1, 0, 0), EY: r3.NewVec(0, 1, 0), EZ: r3.NewVec(0, 0, 1),
	}, r3.NewVec(40, 0, 0))
	require.NoError(t, err)
	boreReverse, err := r3.FromBasis(r3.Basis{
		EX: r3.NewVec(1, 0, 0), EY: r3.NewVec(0, -1, 0), EZ: r3.NewVec(0, 0, 1),
	}, r3.NewVec(0, 20, 0))
	require.NoError(t, err)
	move, err := r3.Translation(r3.NewVec(5, 0, 0))
	require.NoError(t, err)
	for _, tc := range []struct {
		name     string
		pose     r3.Transform
		min, max r3.Vec
		center   r3.Vec
		first    string
	}{
		{"translation", move, r3.NewVec(5, 0, 0), r3.NewVec(45, 20, 20), r3.NewVec(25, 10, 10), "face(0)"},
		{"quarter turn", quarterTurn, r3.NewVec(0, 0, 0), r3.NewVec(20, 20, 40), r3.NewVec(10, 10, 20), "face(5)"},
		{"reflection", reflection, r3.NewVec(0, 0, 0), r3.NewVec(40, 20, 20), r3.NewVec(20, 10, 10), "face(1)"},
		{"bore reverse", boreReverse, r3.NewVec(0, 0, 0), r3.NewVec(40, 20, 20), r3.NewVec(20, 10, 10), "face(0)"},
	} {
		for _, copyBody := range []bool{false, true} {
			t.Run(fmt.Sprintf("%s/copy=%t", tc.name, copyBody), func(t *testing.T) {
				t.Parallel()
				_, bar := crossDrilledBar(t)
				selected := decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0))).
					Or(decad.ParallelTo(r3.NewVec(0, 1, 0))).
					Or(decad.ParallelTo(r3.NewVec(0, 0, 1))).Exactly(12)
				got, err := bar.Chamfer(t.Context(), selected, units.Millimeters(1))
				require.NoError(t, err)
				originalRoles := map[string]bool{}
				for _, face := range got.Faces() {
					originalRoles[face.Origins()[0].Role] = true
				}
				if copyBody {
					got, err = got.PlacedCopy(t.Context(), tc.pose)
				} else {
					got, err = got.Placed(t.Context(), tc.pose)
				}
				require.NoError(t, err)
				requireEveryEdgeOnTwoFaces(t, got)
				require.Len(t, got.Faces(), 19)
				require.Equal(t, tc.first, got.Faces()[0].Origins()[0].Role)
				placedRoles := map[string]bool{}
				for _, face := range got.Faces() {
					placedRoles[face.Origins()[0].Role] = true
				}
				require.Equal(t, originalRoles, placedRoles)
				bounds, err := got.Bounds()
				require.NoError(t, err)
				require.Equal(t, tc.min, bounds.Min)
				require.Equal(t, tc.max, bounds.Max)
				volume, err := got.Volume()
				require.NoError(t, err)
				requireAnalyticPiSqrt2Interval(t, volume.Value.Base(), volume.Bound.Base(), 15846, -180, 0)
				area, err := got.Area()
				require.NoError(t, err)
				requireAnalyticPiSqrt2Interval(t, area.Value.Base(), area.Bound.Base(), 3384, 102, 302)
				centroid, err := got.Centroid()
				require.NoError(t, err)
				require.Equal(t, tc.center, centroid.Value)
				mesh, err := got.Tessellate(t.Context(), units.Millimeters(0.1),
					decad.WithVerification(decad.VerifyAll))
				require.NoError(t, err)
				require.True(t, mesh.BoundaryVerified())
				require.True(t, mesh.VolumeVerified())
				used := map[*decad.Face]bool{}
				for _, face := range mesh.SourceFaces() {
					used[face] = true
				}
				for _, face := range got.Faces() {
					require.True(t, used[face])
				}
			})
		}
	}
}

func TestBrepChamferAllStraightOuterEdgesPlacementComposition(t *testing.T) {
	t.Parallel()
	_, bar := crossDrilledBar(t)
	selected := decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0))).
		Or(decad.ParallelTo(r3.NewVec(0, 1, 0))).
		Or(decad.ParallelTo(r3.NewVec(0, 0, 1))).Exactly(12)
	got, err := bar.Chamfer(t.Context(), selected, units.Millimeters(1))
	require.NoError(t, err)
	move, err := r3.Translation(r3.NewVec(5, 0, 0))
	require.NoError(t, err)
	first, err := got.PlacedCopy(t.Context(), move)
	require.NoError(t, err)
	second, err := first.PlacedCopy(t.Context(), move)
	require.NoError(t, err)
	centroid, err := second.Centroid()
	require.NoError(t, err)
	require.Equal(t, r3.NewVec(30, 10, 10), centroid.Value)
	mesh, err := second.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestBrepChamferAllStraightOuterEdgesPlacementRefusal(t *testing.T) {
	t.Parallel()
	oblique, err := r3.RotationAround(r3.Vec{}, r3.NewVec(0, 0, 1), units.Degrees(37))
	require.NoError(t, err)
	rounded, err := r3.Translation(r3.NewVec(0.1, 0, 0))
	require.NoError(t, err)
	for _, tc := range []struct {
		name string
		pose r3.Transform
	}{{"oblique", oblique}, {"rounded coordinate", rounded}} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			_, bar := crossDrilledBar(t)
			selected := decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0))).
				Or(decad.ParallelTo(r3.NewVec(0, 1, 0))).
				Or(decad.ParallelTo(r3.NewVec(0, 0, 1))).Exactly(12)
			got, err := bar.Chamfer(t.Context(), selected, units.Millimeters(1))
			require.NoError(t, err)
			_, err = got.PlacedCopy(t.Context(), tc.pose)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			_, err = got.Placed(t.Context(), tc.pose)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			_, err = got.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
		})
	}
}

// TestBrepPartialChamferCrossDrilledBar cuts a through bore in a Sketch-built
// bar, then chamfers the two outer edges meeting at its front upper corner.
// Their triangular prisms overlap in a tetrahedron of volume d³/3.
func TestBrepPartialChamferCrossDrilledBar(t *testing.T) {
	t.Parallel()
	_, bar := crossDrilledBar(t)
	baseMesh, err := bar.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)

	corner := r3.NewVec(40, 0, 20)
	selected := decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0)), decad.EndpointAt(corner)).
		Or(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.EndpointAt(corner)).Exactly(2)
	got, err := bar.Chamfer(t.Context(), selected, units.Millimeters(1))
	require.NoError(t, err)
	requireEveryEdgeOnTwoFaces(t, got)
	require.Len(t, got.Faces(), 9)
	roles := map[string]bool{}
	for _, face := range got.Faces() {
		for _, origin := range face.Origins() {
			roles[origin.Role] = true
		}
	}
	patches := chamferLoopPatches(got)
	require.Len(t, patches, 2)
	wantPatchDirections := []r3.Vec{r3.NewVec(1, -1, 0), r3.NewVec(0, -1, 1)}
	for i, patch := range patches {
		_, planar := patch.Surface().(decad.Plane)
		require.True(t, planar)
		require.Equal(t, fmt.Sprintf("chamferLoop(5,0,%d)", i), patch.Origins()[0].Role)
		at := patch.Loops()[0].Edges()[0].Start().Position().Value
		normal, normalErr := patch.NormalAt(at)
		require.NoError(t, normalErr)
		require.Greater(t, normal.Value.Dot(wantPatchDirections[i]), 1.4)
	}

	baseVolume := 16000 - 180*math.Pi
	removedVolume := 20.0 + 10.0 - 1.0/3
	volume, err := got.Volume()
	require.NoError(t, err)
	require.InDelta(t, baseVolume-removedVolume, volume.Value.Base(), volume.Bound.Base()+1e-10)
	centroid, err := got.Centroid()
	require.NoError(t, err)
	removedMoment := r3.NewVec(
		400+1190.0/3-317.0/24,
		20.0/3+10.0/3-1.0/12,
		1180.0/3+100-157.0/24,
	)
	wantCentroid := r3.NewVec(
		(20*baseVolume-removedMoment.X)/(baseVolume-removedVolume),
		(10*baseVolume-removedMoment.Y)/(baseVolume-removedVolume),
		(10*baseVolume-removedMoment.Z)/(baseVolume-removedVolume),
	)
	require.InDelta(t, wantCentroid.X, centroid.Value.X, centroid.Bound.Base()+1e-10)
	require.InDelta(t, wantCentroid.Y, centroid.Value.Y, centroid.Bound.Base()+1e-10)
	require.InDelta(t, wantCentroid.Z, centroid.Value.Z, centroid.Bound.Base()+1e-10)

	mesh, err := got.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	usedRoles := map[string]bool{}
	require.Len(t, mesh.SourceFaces(), len(mesh.Triangles()))
	for _, source := range mesh.SourceFaces() {
		require.NotNil(t, source)
		require.NotEmpty(t, source.Origins())
		role := source.Origins()[0].Role
		require.True(t, roles[role], "mesh source %q belongs to the result", role)
		usedRoles[role] = true
	}
	for _, patch := range patches {
		require.True(t, usedRoles[patch.Origins()[0].Role], "the mesh retains each chamfer source face")
	}
	// Both meshes share the bore approximation, so their difference isolates
	// the two analytic chamfer patches and their corner overlap.
	require.InDelta(t, removedVolume, meshVolume(baseMesh)-meshVolume(mesh), 1e-8)
}

// TestBrepPartialChamferBlindPocket checks the same route on a separate Cut
// topology, with two horizontal selected edges and a half-millimetre reach.
func TestBrepPartialChamferBlindPocket(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 30, 18, 12)
	tool := boxBodyAtZ(t, doc, 11, 6, 19, 11, 9, 4)
	pocket, err := decad.Cut(t.Context(), plate, tool)
	require.NoError(t, err)
	baseMesh, err := pocket.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)

	corner := r3.NewVec(30, 18, 12)
	selected := decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0)), decad.EndpointAt(corner)).
		Or(decad.ParallelTo(r3.NewVec(0, 1, 0)), decad.EndpointAt(corner)).Exactly(2)
	got, err := pocket.Chamfer(t.Context(), selected, units.Millimeters(0.5))
	require.NoError(t, err)
	requireEveryEdgeOnTwoFaces(t, got)
	require.Len(t, chamferLoopPatches(got), 2)
	volume, err := got.Volume()
	require.NoError(t, err)
	// The pocket removes 8×5×3. The chamfer removes two triangular prisms
	// of total volume 6, less their common tetrahedron of volume 1/24.
	removedVolume := 143.0 / 24
	require.InDelta(t, 30*18*12-8*5*3-removedVolume, volume.Value.Base(), volume.Bound.Base()+1e-10)
	mesh, err := got.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.InDelta(t, removedVolume, meshVolume(baseMesh)-meshVolume(mesh), 1e-8)
}

func TestBrepPartialChamferMirroredCrossDrilledBar(t *testing.T) {
	t.Parallel()
	_, bar := crossDrilledBar(t)
	baseMesh, err := bar.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	corner := r3.NewVec(0, 20, 20)
	selected := decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0)), decad.EndpointAt(corner)).
		Or(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.EndpointAt(corner)).Exactly(2)
	got, err := bar.Chamfer(t.Context(), selected, units.Millimeters(0.5))
	require.NoError(t, err)
	requireEveryEdgeOnTwoFaces(t, got)
	require.Len(t, chamferLoopPatches(got), 2)
	wantPatchDirections := []r3.Vec{r3.NewVec(0, 1, 1), r3.NewVec(-1, 1, 0)}
	for i, patch := range chamferLoopPatches(got) {
		at := patch.Loops()[0].Edges()[0].Start().Position().Value
		normal, normalErr := patch.NormalAt(at)
		require.NoError(t, normalErr)
		require.Greater(t, normal.Value.Dot(wantPatchDirections[i]), 1.4)
	}
	volume, err := got.Volume()
	require.NoError(t, err)
	baseVolume := 16000 - 180*math.Pi
	removedVolume := 179.0 / 24
	require.InDelta(t, baseVolume-removedVolume, volume.Value.Base(), volume.Bound.Base()+1e-10)
	centroid, err := got.Centroid()
	require.NoError(t, err)
	d := 0.5
	overlap := d * d * d / 3
	removedMoment := r3.NewVec(
		5*20+2.5*(d/3)-overlap*(3*d/8),
		7.5*(20-d/3)-overlap*(20-d/4),
		5*(20-d/3)+2.5*10-overlap*(20-3*d/8),
	)
	wantCentroid := r3.NewVec(
		(20*baseVolume-removedMoment.X)/(baseVolume-removedVolume),
		(10*baseVolume-removedMoment.Y)/(baseVolume-removedVolume),
		(10*baseVolume-removedMoment.Z)/(baseVolume-removedVolume),
	)
	require.InDelta(t, wantCentroid.X, centroid.Value.X, centroid.Bound.Base()+1e-10)
	require.InDelta(t, wantCentroid.Y, centroid.Value.Y, centroid.Bound.Base()+1e-10)
	require.InDelta(t, wantCentroid.Z, centroid.Value.Z, centroid.Bound.Base()+1e-10)
	mesh, err := got.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.InDelta(t, removedVolume, meshVolume(baseMesh)-meshVolume(mesh), 1e-8)
}
