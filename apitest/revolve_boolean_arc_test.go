package apitest_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// arcBooleanOperands revolves a radius-10 half-disk (a sphere about the u
// axis) and a coaxial rectangle over z in [lo, 20] with radius r.
func arcBooleanOperands(t *testing.T, doc *decad.Document, lo, r float64) (*decad.Body, *decad.Body) {
	t.Helper()
	s, p := circleSketch(t, 0, 0, 10)
	sphere, err := decad.Revolve(t.Context(), doc, s, p, uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	tool := revolveMeridian(t, doc, [][2]float64{{lo, 0}, {20, 0}, {20, r}, {lo, r}}, decad.FullRevolution{})
	return sphere, tool
}

func requireArcResultMesh(t *testing.T, b *decad.Body) {
	t.Helper()
	mesh, err := b.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func requireFaceKind(t *testing.T, b *decad.Body, kind decad.SurfaceKind) {
	t.Helper()
	for _, f := range b.Faces() {
		if f.Surface().Kind() == kind {
			return
		}
	}
	require.Fail(t, "no face of the expected surface kind", "want %v", kind)
}

// A sphere is a half-disk meridian with an arc. Common-axis Booleans arrange
// the arc and the tool's lines in one private scene, so the result stays an
// exact revolve instead of a faceted mesh.
//
// Shown to fail with the ArcSeg arm of PrepareCommonAxis removed: each row
// falls to the mesh path, which refuses at require.NoError with "this chord
// tolerance asks for more than 65536 facets in one revolve mesh".
func TestCommonAxisRevolveArcMeridianBooleans(t *testing.T) {
	t.Parallel()
	const capVolume = 625 * math.Pi / 3
	for _, tc := range []struct {
		name string
		want float64
		run  func(context.Context, *decad.Body, *decad.Body) (*decad.Body, error)
	}{
		{name: "sphere cut", want: 4000*math.Pi/3 - capVolume, run: pairCut},
		{name: "sphere intersect", want: capVolume, run: pairIntersect},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New(decad.ExactOnly())
			sphere, tool := arcBooleanOperands(t, doc, 5, 15)
			result, err := tc.run(t.Context(), sphere, tool)
			require.NoError(t, err)
			requireAnalyticVolume(t, result, tc.want)
			requireFaceKind(t, result, decad.KindSphere)
			requireArcResultMesh(t, result)
		})
	}
	t.Run("union with shaft", func(t *testing.T) {
		t.Parallel()
		doc := decad.New(decad.ExactOnly())
		sphere, shaft := arcBooleanOperands(t, doc, 5, 3)
		result, err := decad.Union(t.Context(), sphere, shaft)
		require.NoError(t, err)
		// Outside the sphere the shaft is the solid of revolution of 20 - sqrt(100 - r^2) over r in [0, 3].
		shaftOutside := 180*math.Pi - 2*math.Pi/3*(1000-math.Pow(91, 1.5))
		requireAnalyticVolume(t, result, 4000*math.Pi/3+shaftOutside)
		requireArcResultMesh(t, result)
	})
}

// Two equal spheres whose centres are 5 apart meet in a lens, pi(4R+d)(2R-d)^2/12
// = 843.75 pi. Both operands are arcs, and the crossing circle is off the axis.
func TestCommonAxisRevolveArcMeridianLens(t *testing.T) {
	t.Parallel()
	doc := decad.New(decad.ExactOnly())
	s, p := circleSketch(t, 0, 0, 10)
	a, err := decad.Revolve(t.Context(), doc, s, p, uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	s2, p2 := circleSketch(t, 5, 0, 10)
	b, err := decad.Revolve(t.Context(), doc, s2, p2, uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	result, err := decad.Intersect(t.Context(), a, b)
	require.NoError(t, err)
	requireAnalyticVolume(t, result, 843.75*math.Pi)
	requireArcResultMesh(t, result)
}

// A disk touching the sphere only at its pole is a point contact, not a
// solid join, and two equal spheres share their whole arc span. Neither is
// joined by this route, and both continue to the mesh path's refusal.
func TestCommonAxisRevolveArcMeridianMisses(t *testing.T) {
	t.Parallel()
	t.Run("tangent at pole", func(t *testing.T) {
		t.Parallel()
		doc := decad.New(decad.ExactOnly())
		sphere, tool := arcBooleanOperands(t, doc, 10, 15)
		before := doc.Bodies()
		_, err := decad.Union(t.Context(), sphere, tool)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "more than 65536 facets")
		require.Equal(t, before, doc.Bodies())
	})
	t.Run("shared arc span", func(t *testing.T) {
		t.Parallel()
		doc := decad.New(decad.ExactOnly())
		s, p := circleSketch(t, 0, 0, 10)
		a, err := decad.Revolve(t.Context(), doc, s, p, uAxis, decad.FullRevolution{})
		require.NoError(t, err)
		s2, p2 := circleSketch(t, 0, 0, 10)
		b, err := decad.Revolve(t.Context(), doc, s2, p2, uAxis, decad.FullRevolution{})
		require.NoError(t, err)
		before := doc.Bodies()
		_, err = decad.Union(t.Context(), a, b)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "more than 65536 facets")
		require.Equal(t, before, doc.Bodies())
	})
}

// A circle meridian off the axis is a torus. A coaxial block over z in [0, 10]
// cuts it through the circle's centre line, leaving half the torus: the full
// volume is 2 pi^2 R r^2 = 108 pi^2 for R = 6, r = 3.
//
// Shown to fail with the CircleSeg arm of PrepareCommonAxis removed: both rows
// fall to the mesh path, which refuses at require.NoError with the same facet
// limit as the sphere rows.
func TestCommonAxisRevolveCircleMeridianTorus(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name string
		run  func(context.Context, *decad.Body, *decad.Body) (*decad.Body, error)
	}{
		{name: "torus cut", run: pairCut},
		{name: "torus intersect", run: pairIntersect},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New(decad.ExactOnly())
			w := sketch.NewWorld()
			s, err := w.CreateSketch(w.XY())
			require.NoError(t, err)
			s.CreateCircle(s.CreatePoint(0, 6), 3)
			_, err = s.Solve(t.Context())
			require.NoError(t, err)
			require.Len(t, s.Profiles(), 1)
			torus, err := decad.Revolve(t.Context(), doc, s, s.Profiles()[0], uAxis, decad.FullRevolution{})
			require.NoError(t, err)
			tool := revolveMeridian(t, doc, [][2]float64{{0, 0}, {10, 0}, {10, 10}, {0, 10}}, decad.FullRevolution{})
			result, err := tc.run(t.Context(), torus, tool)
			require.NoError(t, err)
			requireAnalyticVolume(t, result, 54*math.Pi*math.Pi)
			requireFaceKind(t, result, decad.KindTorus)
			requireArcResultMesh(t, result)
		})
	}
}
