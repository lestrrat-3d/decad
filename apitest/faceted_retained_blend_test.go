package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The fixtures of docs/retained-edge-blend-design.md §9 for increment B1: a
// 40x20x20 plate bored by a Ø8 tool along z that a Transform placed, which
// keeps the pair on the mesh path.

// boredPlate is the plate and its through bore along z at (20, 10).
func boredPlate(t *testing.T) *decad.Body {
	t.Helper()
	sc := newRetentionScene()
	s, err := sc.w.CreateSketch(sc.w.XY())
	require.NoError(t, err)
	r := s.CreateRectangle(0, 0, 40, 20)
	s.Fix(r.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	plate, err := decad.Extrude(t.Context(), sc.doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	require.NoError(t, err)
	ts, err := sc.w.CreateSketch(sc.w.XY())
	require.NoError(t, err)
	c := ts.CreatePoint(20, 10)
	ts.Fix(c)
	ts.CreateCircle(c, 4)
	_, err = ts.Solve(t.Context())
	require.NoError(t, err)
	pin, err := decad.Extrude(t.Context(), sc.doc, ts, ts.Profiles()[0], decad.Symmetric{D: units.Millimeters(60)})
	require.NoError(t, err)
	move, err := r3.Translation(r3.Vec{Z: 0.001})
	require.NoError(t, err)
	tool, err := decad.Transform(t.Context(), pin, move)
	require.NoError(t, err)
	bored, err := decad.Cut(t.Context(), plate, tool)
	require.NoError(t, err)
	return bored
}

// rimAt selects the bore's rim at height z.
func rimAt(t *testing.T, b *decad.Body, z float64) decad.EdgeSelector {
	t.Helper()
	for _, e := range b.Edges() {
		c, ok := e.Curve().(decad.Circle3)
		if !ok || math.Abs(c.Center.Z-z) > 1e-6 {
			continue
		}
		return decad.Edges(decad.EndpointAt(e.Start().Position().Value)).Exactly(1)
	}
	require.Fail(t, "no rim at that height")
	return nil
}

func TestFacetedRetainedBlend(t *testing.T) {
	t.Parallel()

	t.Run("hole mouth fillet", func(t *testing.T) {
		t.Parallel()
		bored := boredPlate(t)
		before, err := bored.Volume()
		require.NoError(t, err)
		filleted, err := decad.Fillet(t.Context(), bored, rimAt(t, bored, 20), units.Millimeters(1))
		require.NoError(t, err)
		after, err := filleted.Volume()
		require.NoError(t, err)
		const r, rim = 1.0, 4.0
		removed := math.Pi*r*(2*rim*r+r*r) - 2*math.Pi*((rim+r)-4*r/(3*math.Pi))*(math.Pi*r*r/4)
		got := before.Value.Mag() - after.Value.Mag()
		bound := before.Bound.Mag() + after.Bound.Mag()
		t.Logf("removed %.9f got %.9f bound %.3g (before %.3g after %.3g)", removed, got, bound,
			before.Bound.Mag(), after.Bound.Mag())
		require.InDelta(t, removed, got, bound+1e-6)

		var torus *decad.Face
		for _, f := range filleted.Faces() {
			if _, ok := f.Surface().(decad.Torus); ok {
				require.Nil(t, torus, "one blend face")
				torus = f
			}
		}
		require.NotNil(t, torus)
		tor := torus.Surface().(decad.Torus)
		require.InDelta(t, rim+r, tor.Major.Base(), 1e-12)
		require.InDelta(t, r, tor.Minor.Base(), 1e-12)
		require.InDelta(t, 19, tor.Center.Z, 1e-9)
		area, err := torus.Area()
		require.NoError(t, err)
		wantArea := math.Pi*math.Pi*r*(rim+r) - 2*math.Pi*r*r
		t.Logf("torus area %.9f want %.9f bound %.3g", area.Value.Mag(), wantArea, area.Bound.Mag())
		require.InDelta(t, wantArea, area.Value.Mag(), area.Bound.Mag()+1e-9)
		n, err := torus.NormalAt(firstVertex(t, torus))
		require.NoError(t, err)
		require.Less(t, n.Bound.Base(), 0.1)
		mesh, err := filleted.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
		require.NoError(t, err)
		require.True(t, mesh.BoundaryVerified())
		require.True(t, mesh.VolumeVerified())
	})
}
