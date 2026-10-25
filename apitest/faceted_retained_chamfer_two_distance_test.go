package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The fixtures of agent-docs/retained-modify-reach-design.md §12 for increment
// M1: a two-distance chamfer of an edge of a mesh Boolean result with retained
// tags.

// crossDrilled is the rounded box cut by the placed Ø8 tool along y at x = 20.
func crossDrilled(t *testing.T) *decad.Body {
	t.Helper()
	sc := newRetentionScene()
	out, err := decad.Cut(t.Context(), sc.roundedBox(t), sc.tool(t, 20))
	require.NoError(t, err)
	return out
}

// mouthAtY selects the tool's rim on the plane y.
func mouthAtY(t *testing.T, b *decad.Body, y float64) decad.EdgeSelector {
	t.Helper()
	for _, e := range b.Edges() {
		c, ok := e.Curve().(decad.Circle3)
		if !ok || math.Abs(c.Center.Y-y) > 1e-6 {
			continue
		}
		return decad.Edges(decad.EndpointAt(e.Start().Position().Value)).Exactly(1)
	}
	require.Fail(t, "no rim at that y")
	return nil
}

func TestFacetedRetainedChamferTwoDistance(t *testing.T) {
	t.Parallel()

	t.Run("hole mouth, reference the plane", func(t *testing.T) {
		t.Parallel()
		drilled := crossDrilled(t)
		before, err := drilled.Volume()
		require.NoError(t, err)
		// The front face (outward normal -y) takes 2 mm, the bore wall 1 mm.
		const dA, dB, rad = 2.0, 1.0, 4.0
		chamfered, err := decad.Chamfer(t.Context(), drilled, mouthAtY(t, drilled, 0), units.Millimeters(dA),
			decad.WithAsymmetricChamfer(decad.Faces(decad.Facing(r3.NewVec(0, -1, 0))), units.Millimeters(dB)))
		require.NoError(t, err)
		after, err := chamfered.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, math.Pi*dA*dB*(rad+dA/3))
		requireCone(t, chamfered, math.Atan2(dA, dB), dB, rad)
	})

	t.Run("plate edges, one reference face each", func(t *testing.T) {
		t.Parallel()
		_, bored := boredPlate(t)
		before, err := bored.Volume()
		require.NoError(t, err)
		z := r3.NewVec(0, 0, 1)
		edges := decad.Edges(decad.EndpointAt(r3.NewVec(0, 0, 0)), decad.ParallelTo(z)).
			Or(decad.EndpointAt(r3.NewVec(40, 0, 0)), decad.ParallelTo(z)).Exactly(2)
		// The faces x = 0 and x = 40 take 3 mm; the front face y = 0 takes 1 mm.
		ref := decad.Faces(decad.Facing(r3.NewVec(-1, 0, 0))).Or(decad.Facing(r3.NewVec(1, 0, 0)))
		chamfered, err := decad.Chamfer(t.Context(), bored, edges, units.Millimeters(3),
			decad.WithAsymmetricChamfer(ref, units.Millimeters(1)))
		require.NoError(t, err)
		after, err := chamfered.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, 2*20*3*1/2.0)
		// Each bevel plane runs through its setback on the reference face and on
		// the front face.
		requireBevel(t, chamfered, r3.NewVec(0, 3, 5), r3.NewVec(1, 0, 5))
		requireBevel(t, chamfered, r3.NewVec(40, 3, 5), r3.NewVec(39, 0, 5))
	})

	t.Run("groove edge, reference the wall", func(t *testing.T) {
		t.Parallel()
		const rad = 4.0
		k := [2]float64{10 - math.Sqrt(rad*rad-4), 20}
		o := [2]float64{10, 22}
		// The wall takes the positional distance (dB) and the top face the other (dA).
		for _, c := range []struct{ dA, dB float64 }{{2, 1}, {0.5, 1}} {
			grooved := groovedPlate(t)
			before, err := grooved.Volume()
			require.NoError(t, err)
			chamfered, err := decad.Chamfer(t.Context(), grooved,
				straightEdge(t, grooved, r3.NewVec(0, k[0], 20), r3.NewVec(1, 0, 0)), units.Millimeters(c.dB),
				decad.WithAsymmetricChamfer(decad.Faces(decad.Cylindrical()), units.Millimeters(c.dA)))
			require.NoError(t, err)
			after, err := chamfered.Volume()
			require.NoError(t, err)
			// The bevel runs c.dA along the top face and c.dB of arc down the wall.
			a := math.Atan2(k[1]-o[1], k[0]-o[0]) + c.dB/rad
			region := [][2]float64{k, {k[0] - c.dA, 20}, {o[0] + rad*math.Cos(a), o[1] + rad*math.Sin(a)}}
			a0 := math.Atan2(region[2][1]-o[1], region[2][0]-o[0])
			a1 := math.Atan2(k[1]-o[1], k[0]-o[0])
			for i := 1; i < 2000; i++ {
				b := a0 + (a1-a0)*float64(i)/2000
				region = append(region, [2]float64{o[0] + rad*math.Cos(b), o[1] + rad*math.Sin(b)})
			}
			requireVolumeChange(t, before, after, 40*shoelaceArea(region))
		}
	})

	t.Run("arc that breaks out of the side, reference the plane", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		broken, err := decad.Cut(t.Context(), blendPlate(t, sc), zCylinderAt(t, sc, 2, 10, 4, -30, 30))
		require.NoError(t, err)
		before, err := broken.Volume()
		require.NoError(t, err)
		var sel decad.EdgeSelector
		for _, e := range broken.Edges() {
			if arc, ok := e.Curve().(decad.Arc3); ok && math.Abs(arc.Center.Z-20) < 1e-6 {
				sel = decad.Edges(decad.Circular(), decad.EndpointAt(e.Start().Position().Value)).Exactly(1)
			}
		}
		require.NotNil(t, sel)
		const dA, dB = 1.0, 0.5
		chamfered, err := decad.Chamfer(t.Context(), broken, sel, units.Millimeters(dA),
			decad.WithAsymmetricChamfer(decad.Faces(decad.Facing(r3.NewVec(0, 0, 1))), units.Millimeters(dB)))
		require.NoError(t, err)
		after, err := chamfered.Volume()
		require.NoError(t, err)
		const n = 1500
		want := 0.0
		for i := range n {
			u := dA * (float64(i) + 0.5) / n
			rho := 4 + u
			span := 2*math.Pi - 2*math.Acos(2/rho)
			for j := range n {
				h := dB * (float64(j) + 0.5) / n
				if h <= dB*(1-u/dA) {
					want += span * rho * (dA / n) * (dB / n)
				}
			}
		}
		requireVolumeChange(t, before, after, want)
		cones := 0
		for _, f := range chamfered.Faces() {
			if cone, ok := f.Surface().(decad.Cone); ok {
				cones++
				require.InDelta(t, math.Atan2(dA, dB), cone.HalfAngle.Base(), 1e-12)
			}
		}
		require.Equal(t, 1, cones)
	})
}

// requireCone checks that the chamfer's one Cone face leans by half from its
// axis, starts depth from the front face y = 0 and has the bore's radius there.
func requireCone(t *testing.T, b *decad.Body, half, depth, radius float64) {
	t.Helper()
	cones := 0
	for _, f := range b.Faces() {
		cone, ok := f.Surface().(decad.Cone)
		if !ok {
			continue
		}
		cones++
		require.InDelta(t, half, cone.HalfAngle.Base(), 1e-12)
		require.InDelta(t, depth, math.Abs(cone.Origin.Y), 1e-9)
		require.InDelta(t, radius, cone.Radius.Base(), 1e-9)
	}
	require.Equal(t, 1, cones)
}

// requireBevel checks that exactly one planar face of b that is neither
// axis-aligned nor a retained original contains both points.
func requireBevel(t *testing.T, b *decad.Body, p, q r3.Vec) {
	t.Helper()
	hits := 0
	for _, f := range b.Faces() {
		pl, ok := f.Surface().(decad.Plane)
		if !ok {
			continue
		}
		n := pl.Frame.N()
		if math.Abs(n.X)+math.Abs(n.Y) < 0.1 || math.Abs(n.X) < 1e-6 || math.Abs(n.Y) < 1e-6 {
			continue
		}
		if math.Abs(n.Dot(p.Sub(pl.Frame.Origin()))) < 1e-9 && math.Abs(n.Dot(q.Sub(pl.Frame.Origin()))) < 1e-9 {
			hits++
		}
	}
	require.Equal(t, 1, hits, "one bevel through %v and %v", p, q)
}
