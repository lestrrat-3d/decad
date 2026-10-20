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

// boredPlate is the plate and its through bore along z at (20, 10), with the
// scene that built them.
func boredPlate(t *testing.T) (*retentionScene, *decad.Body) {
	t.Helper()
	sc := newRetentionScene()
	plate := blendPlate(t, sc)
	bored, err := decad.Cut(t.Context(), plate, zCylinder(t, sc, -30, 30))
	require.NoError(t, err)
	return sc, bored
}

// zCylinder is a Ø8 cylinder along z at (20, 10) from z0 to z1,
// placed by a translation so a Boolean takes the mesh path.
func zCylinder(t *testing.T, sc *retentionScene, z0, z1 float64) *decad.Body {
	t.Helper()
	ts, err := sc.w.CreateSketch(sc.w.XY())
	require.NoError(t, err)
	c := ts.CreatePoint(20, 10)
	ts.Fix(c)
	ts.CreateCircle(c, 4)
	_, err = ts.Solve(t.Context())
	require.NoError(t, err)
	pin, err := decad.Extrude(t.Context(), sc.doc, ts, ts.Profiles()[0],
		decad.Distance{D: units.Millimeters(z1 - z0), Dir: decad.Along})
	require.NoError(t, err)
	move, err := r3.Translation(r3.Vec{Z: z0 + 0.001})
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), pin, move)
	require.NoError(t, err)
	return placed
}

func blendPlate(t *testing.T, sc *retentionScene) *decad.Body {
	t.Helper()
	s, err := sc.w.CreateSketch(sc.w.XY())
	require.NoError(t, err)
	r := s.CreateRectangle(0, 0, 40, 20)
	s.Fix(r.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	plate, err := decad.Extrude(t.Context(), sc.doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	require.NoError(t, err)
	return plate
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

// requireVolumeChange checks that blending moved the volume by want within
// the growth of the published bound: outside the blend region the held and
// true solids are untouched, so the rewrite's own error is all that can
// separate the measured change from the exact one (design §6.3, Claim K).
func requireVolumeChange(t *testing.T, before, after decad.Measurement, want float64) {
	t.Helper()
	got := before.Value.Mag() - after.Value.Mag()
	growth := after.Bound.Mag() - before.Bound.Mag()
	t.Logf("volume change %.9f, want %.9f, bound growth %.3g", got, want, growth)
	require.Positive(t, growth)
	require.InDelta(t, want, got, growth)
}

func TestFacetedRetainedBlend(t *testing.T) {
	t.Parallel()

	t.Run("hole mouth fillet", func(t *testing.T) {
		t.Parallel()
		_, bored := boredPlate(t)
		before, err := bored.Volume()
		require.NoError(t, err)
		filleted, err := decad.Fillet(t.Context(), bored, rimAt(t, bored, 20), units.Millimeters(1))
		require.NoError(t, err)
		after, err := filleted.Volume()
		require.NoError(t, err)
		const r, rim = 1.0, 4.0
		removed := math.Pi*r*(2*rim*r+r*r) - 2*math.Pi*((rim+r)-4*r/(3*math.Pi))*(math.Pi*r*r/4)
		requireVolumeChange(t, before, after, removed)

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

	t.Run("hole mouth chamfer and both mouths", func(t *testing.T) {
		t.Parallel()
		_, bored := boredPlate(t)
		before, err := bored.Volume()
		require.NoError(t, err)
		chamfered, err := decad.Chamfer(t.Context(), bored, rimAt(t, bored, 20), units.Millimeters(1))
		require.NoError(t, err)
		after, err := chamfered.Volume()
		require.NoError(t, err)
		// The bevel removes a right triangle of legs 1 revolved about the axis;
		// its centroid sits a third of the leg out from the rim.
		removed := 2 * math.Pi * (4 + 1.0/3) * 0.5
		requireVolumeChange(t, before, after, removed)
		var cone *decad.Face
		for _, f := range chamfered.Faces() {
			if _, ok := f.Surface().(decad.Cone); ok {
				cone = f
			}
		}
		require.NotNil(t, cone)
		c := cone.Surface().(decad.Cone)
		require.InDelta(t, math.Pi/4, c.HalfAngle.Base(), 1e-12)
	})

	t.Run("both mouths", func(t *testing.T) {
		t.Parallel()
		_, bored := boredPlate(t)
		before, err := bored.Volume()
		require.NoError(t, err)
		both, err := decad.Fillet(t.Context(), bored, decad.Edges(decad.Circular()).Exactly(2), units.Millimeters(1))
		require.NoError(t, err)
		after, err := both.Volume()
		require.NoError(t, err)
		tori := 0
		for _, f := range both.Faces() {
			if _, ok := f.Surface().(decad.Torus); ok {
				tori++
			}
		}
		require.Equal(t, 2, tori)
		mouth := math.Pi*(2*4+1) - 2*math.Pi*(5-4/(3*math.Pi))*(math.Pi/4)
		requireVolumeChange(t, before, after, 2*mouth)
	})

	t.Run("bottom mouth fillet", func(t *testing.T) {
		t.Parallel()
		_, bored := boredPlate(t)
		before, err := bored.Volume()
		require.NoError(t, err)
		filleted, err := decad.Fillet(t.Context(), bored, rimAt(t, bored, 0), units.Millimeters(1))
		require.NoError(t, err)
		after, err := filleted.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, math.Pi*(2*4+1)-2*math.Pi*(5-4/(3*math.Pi))*(math.Pi/4))
	})

	t.Run("boss root fillet fills the corner", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		joined, err := decad.Union(t.Context(), blendPlate(t, sc), zCylinder(t, sc, 15, 30))
		require.NoError(t, err)
		before, err := joined.Volume()
		require.NoError(t, err)
		filleted, err := decad.Fillet(t.Context(), joined, rimAt(t, joined, 20), units.Millimeters(1))
		require.NoError(t, err)
		after, err := filleted.Volume()
		require.NoError(t, err)
		added := math.Pi*(2*4+1) - 2*math.Pi*(5-4/(3*math.Pi))*(math.Pi/4)
		requireVolumeChange(t, before, after, -added)
		for _, f := range filleted.Faces() {
			if tor, ok := f.Surface().(decad.Torus); ok {
				require.InDelta(t, 5, tor.Major.Base(), 1e-12)
				require.InDelta(t, 21, tor.Center.Z, 1e-9, "the ball sits above the plate in the void")
				// The blend's outward normal at its first vertex points into
				// the void, toward the ball's centre.
				p := firstVertex(t, f)
				n, err := f.NormalAt(p)
				require.NoError(t, err)
				toCentre := r3.NewVec(20, 10, 21).Sub(p)
				require.Positive(t, n.Value.Dot(toCentre))
			}
		}
	})

	t.Run("shaft end fillet", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		cross, err := sc.w.CreateSketch(sc.w.XZ())
		require.NoError(t, err)
		c := cross.CreatePoint(20, 10)
		cross.Fix(c)
		cross.CreateCircle(c, 1)
		_, err = cross.Solve(t.Context())
		require.NoError(t, err)
		pin, err := decad.Extrude(t.Context(), sc.doc, cross, cross.Profiles()[0], decad.Symmetric{D: units.Millimeters(30)})
		require.NoError(t, err)
		move, err := r3.Translation(r3.Vec{Y: 0.001})
		require.NoError(t, err)
		hole, err := decad.Transform(t.Context(), pin, move)
		require.NoError(t, err)
		shaft := zCylinder(t, sc, 0, 20)
		_ = shaft
		drilled, err := decad.Cut(t.Context(), shaft, hole)
		require.NoError(t, err)
		before, err := drilled.Volume()
		require.NoError(t, err)
		filleted, err := decad.Fillet(t.Context(), drilled, rimAt(t, drilled, 20.001), units.Millimeters(1))
		require.NoError(t, err)
		after, err := filleted.Volume()
		require.NoError(t, err)
		const r, big = 1.0, 4.0
		removed := math.Pi*r*(2*big*r-r*r) - 2*math.Pi*((big-r)+4*r/(3*math.Pi))*(math.Pi*r*r/4)
		requireVolumeChange(t, before, after, removed)
	})

	t.Run("pocket floor fillet fills the corner", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		pocketed, err := decad.Cut(t.Context(), blendPlate(t, sc), zCylinder(t, sc, 10, 30))
		require.NoError(t, err)
		before, err := pocketed.Volume()
		require.NoError(t, err)
		filleted, err := decad.Fillet(t.Context(), pocketed, rimAt(t, pocketed, 10.001), units.Millimeters(1))
		require.NoError(t, err)
		after, err := filleted.Volume()
		require.NoError(t, err)
		const r, big = 1.0, 4.0
		added := math.Pi*r*(2*big*r-r*r) - 2*math.Pi*((big-r)+4*r/(3*math.Pi))*(math.Pi*r*r/4)
		requireVolumeChange(t, before, after, -added)
	})

	t.Run("the result is an ordinary retained body", func(t *testing.T) {
		t.Parallel()
		sc, bored := boredPlate(t)
		top, err := decad.Fillet(t.Context(), bored, rimAt(t, bored, 20), units.Millimeters(1))
		require.NoError(t, err)
		// The blended face's boundary vertices lie within their bounds of the
		// exact torus.
		for _, f := range top.Faces() {
			tor, ok := f.Surface().(decad.Torus)
			if !ok {
				continue
			}
			checked := 0
			for _, loop := range f.Loops() {
				for _, ce := range loop.CoEdges() {
					p := ce.Start().Position()
					q := p.Value.Sub(tor.Center)
					rho := math.Hypot(q.X, q.Y)
					off := math.Abs(math.Hypot(rho-tor.Major.Base(), q.Z) - tor.Minor.Base())
					require.LessOrEqual(t, off, p.Bound.Mag()+1e-9, "vertex %v", p.Value)
					checked++
				}
			}
			require.Positive(t, checked)
		}
		// A second blend on the other mouth, a Boolean and a placement all take it.
		both, err := decad.Fillet(t.Context(), top, rimAt(t, top, 0), units.Millimeters(1))
		require.NoError(t, err)
		cut, err := decad.Cut(t.Context(), both, sc.tool(t, 33))
		require.NoError(t, err)
		move, err := r3.Translation(r3.Vec{X: 5, Y: -2, Z: 7})
		require.NoError(t, err)
		moved, err := decad.Transform(t.Context(), cut, move)
		require.NoError(t, err)
		tori := 0
		for _, f := range moved.Faces() {
			if _, ok := f.Surface().(decad.Torus); ok {
				tori++
				n, err := f.NormalAt(firstVertex(t, f))
				require.NoError(t, err)
				require.Less(t, n.Bound.Base(), 1.0)
			}
		}
		require.Equal(t, 2, tori)
		mesh, err := moved.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
		require.NoError(t, err)
		require.True(t, mesh.BoundaryVerified())
		require.True(t, mesh.VolumeVerified())
	})

	t.Run("refusals", func(t *testing.T) {
		t.Parallel()
		// The face between the bore and the nearest plate edge is 6 mm wide.
		_, bored := boredPlate(t)
		_, err := decad.Fillet(t.Context(), bored, rimAt(t, bored, 20), units.Millimeters(6.5))
		require.ErrorIs(t, err, decad.ErrDegenerate, "the blend consumes the face")
		_, bored = boredPlate(t)
		_, err = decad.Fillet(t.Context(), bored, rimAt(t, bored, 20), units.Millimeters(6))
		require.ErrorIs(t, err, decad.ErrUnsupported, "the held bounds cannot show it fits")
		_, bored = boredPlate(t)
		_, err = decad.Fillet(t.Context(), bored, decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1))).Exactly(4),
			units.Millimeters(1))
		require.ErrorIs(t, err, decad.ErrUnsupported, "a plate edge is not a rim")
		_, bored = boredPlate(t)
		_, err = decad.Fillet(t.Context(), bored, rimAt(t, bored, 20), units.Millimeters(-1))
		require.Error(t, err)
	})
}
