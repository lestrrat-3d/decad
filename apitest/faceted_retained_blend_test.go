package apitest_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The fixtures of agent-docs/retained-edge-blend-design.md §9 for increment B1: a
// 40x20x20 plate bored by a Ø8 tool along z that a Transform placed, which
// keeps the pair on the mesh path.

// boredPlate is the plate and its through bore along z at (20, 10), with the
// scene that built them.
func boredPlate(t *testing.T) (*retentionScene, *decad.Body) {
	t.Helper()
	sc := newRetentionScene()
	plate := blendPlate(t, sc)
	bored, err := decad.Cut(t.Context(), plate, zCylinder(t, sc, -30))
	require.NoError(t, err)
	return sc, bored
}

// zCylinder is a Ø8 cylinder along z at (20, 10) from z0 to 30, placed by a
// translation so a Boolean takes the mesh path.
func zCylinder(t *testing.T, sc *retentionScene, z0 float64) *decad.Body {
	t.Helper()
	return zCylinderAt(t, sc, 20, 10, 4, z0, 30)
}

// zCylinderAt is a cylinder of radius r along z at (cx, cy) from z0 to z1,
// both lifted by 0.001. It is extruded from z = 0 and moved by z0 + 0.001, so
// a cap level whose float sum rounds states a departure
// (agent-docs/faceted-source-retention-design.md §3.5).
func zCylinderAt(t *testing.T, sc *retentionScene, cx, cy, r, z0, z1 float64) *decad.Body {
	t.Helper()
	ts, err := sc.w.CreateSketch(sc.w.XY())
	require.NoError(t, err)
	c := ts.CreatePoint(cx, cy)
	ts.Fix(c)
	ts.CreateCircle(c, r)
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

// requireAreaChange is requireVolumeChange for the surface area: the held area
// moves from the exact area by at most what the published bound grew.
func requireAreaChange(t *testing.T, before, after decad.Measurement, want float64) {
	t.Helper()
	got := after.Value.Mag() - before.Value.Mag()
	growth := after.Bound.Mag() - before.Bound.Mag()
	t.Logf("area change %.9f, want %.9f, bound growth %.3g", got, want, growth)
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

	// A Ø6 pin sketched at z = −30 and moved by (0, 0, 0.1) holds its axis
	// origin at the rounded sum −30 + 0.1. The rounding runs along the axis,
	// which leaves the cylinder the exact image of its record, so the wall
	// states no departure (agent-docs/faceted-source-retention-design.md §3.5)
	// and both mouths stay circles that a fillet and a chamfer read.
	t.Run("a pin moved along its axis by a rounding sum keeps its mouths", func(t *testing.T) {
		t.Parallel()
		exact := new(big.Rat).Add(big.NewRat(-30, 1), new(big.Rat).SetFloat64(0.1))
		require.NotZero(t, exact.Cmp(new(big.Rat).SetFloat64(-30+0.1)), "the moved origin rounds")
		pinned := func(t *testing.T) *decad.Body {
			t.Helper()
			sc := newRetentionScene()
			plane, err := sc.w.CreateOffsetPlane(sc.w.XY(), -30)
			require.NoError(t, err)
			s, err := sc.w.CreateSketch(plane)
			require.NoError(t, err)
			c := s.CreatePoint(20, 10)
			s.Fix(c)
			s.CreateCircle(c, 3)
			_, err = s.Solve(t.Context())
			require.NoError(t, err)
			pin, err := decad.Extrude(t.Context(), sc.doc, s, s.Profiles()[0],
				decad.Distance{D: units.Millimeters(60), Dir: decad.Along})
			require.NoError(t, err)
			bored, err := decad.Cut(t.Context(), blendPlate(t, sc), translated(t, pin, 0, 0, 0.1))
			require.NoError(t, err)
			return bored
		}
		bored := pinned(t)
		circles := 0
		for _, e := range bored.Edges() {
			if c, ok := e.Curve().(decad.Circle3); ok {
				circles++
				require.Equal(t, r3.NewVec(20, 10, c.Center.Z), c.Center)
				require.Equal(t, 3.0, c.Radius.Base())
			}
		}
		require.Equal(t, 2, circles, "both mouths are circles")

		const r, rim = 1.0, 3.0
		before, err := bored.Volume()
		require.NoError(t, err)
		filleted, err := decad.Fillet(t.Context(), bored, rimAt(t, bored, 20), units.Millimeters(r))
		require.NoError(t, err)
		after, err := filleted.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after,
			math.Pi*r*(2*rim*r+r*r)-2*math.Pi*((rim+r)-4*r/(3*math.Pi))*(math.Pi*r*r/4))

		bored = pinned(t)
		before, err = bored.Volume()
		require.NoError(t, err)
		const d = 0.5
		chamfered, err := decad.Chamfer(t.Context(), bored, rimAt(t, bored, 20), units.Millimeters(d))
		require.NoError(t, err)
		after, err = chamfered.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, 2*math.Pi*(rim+d/3)*(d*d/2))
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
		joined, err := decad.Union(t.Context(), blendPlate(t, sc), zCylinder(t, sc, 15))
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
		// The shaft runs down from its top cap and is lifted by 20.001, so the
		// cap's level is the exact float 0 + 20.001. Lifted from its foot
		// instead, the top would be the rounded sum 20 + 0.001, a tag off its
		// true plane (agent-docs/faceted-source-retention-design.md §3.5).
		ss, err := sc.w.CreateSketch(sc.w.XY())
		require.NoError(t, err)
		sc0 := ss.CreatePoint(20, 10)
		ss.Fix(sc0)
		ss.CreateCircle(sc0, 4)
		_, err = ss.Solve(t.Context())
		require.NoError(t, err)
		shaft, err := decad.Extrude(t.Context(), sc.doc, ss, ss.Profiles()[0],
			decad.Distance{D: units.Millimeters(20), Dir: decad.Against})
		require.NoError(t, err)
		shaft = translated(t, shaft, 0, 0, 20.001)
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
		pocketed, err := decad.Cut(t.Context(), blendPlate(t, sc), zCylinder(t, sc, 10))
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
		// Three plate edges meet at one corner, which a sphere octant rounds: 39 + 19 + 19 mm of strip beyond the corner
		// cube, which loses r³ less the octant's πr³/6.
		before, err := bored.Volume()
		require.NoError(t, err)
		corner, err := decad.Fillet(t.Context(), bored, decad.Edges(decad.EndpointAt(r3.NewVec(0, 0, 0))).Exactly(3),
			units.Millimeters(1))
		require.NoError(t, err)
		after, err := corner.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, 77*(1-math.Pi/4)+1-math.Pi/6)
		_, bored = boredPlate(t)
		_, err = decad.Fillet(t.Context(), bored, rimAt(t, bored, 20), units.Millimeters(-1))
		require.Error(t, err)
	})
}

// cornerCentroid is the offset of the centroid of a quarter-turn fillet's
// corner region from the corner along either face, for radius r.
func cornerCentroid(r float64) float64 {
	return (r*r*r/2 - (r-4*r/(3*math.Pi))*math.Pi*r*r/4) / (r * r * (1 - math.Pi/4))
}

// wedgedPlate cuts the blend plate with a tool extruded symmetrically by
// depth from the polygon corners on the sketch plane, then moved by the exact
// translation move so the Cut takes the mesh path.
func wedgedPlate(t *testing.T, sketchPlane func(*retentionScene) *sketch.Plane, corners [][2]float64, depth float64,
	move r3.Vec) *decad.Body {
	t.Helper()
	sc := newRetentionScene()
	plate := blendPlate(t, sc)
	s, err := sc.w.CreateSketch(sketchPlane(sc))
	require.NoError(t, err)
	pts := make([]*sketch.Point, len(corners))
	for i, c := range corners {
		pts[i] = s.CreatePoint(c[0], c[1])
		s.Fix(pts[i])
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	tool, err := decad.Extrude(t.Context(), sc.doc, s, s.Profiles()[0], decad.Symmetric{D: units.Millimeters(depth)})
	require.NoError(t, err)
	cut, err := decad.Cut(t.Context(), plate, translated(t, tool, move.X, move.Y, move.Z))
	require.NoError(t, err)
	return cut
}

func rotation(t *testing.T, about, axis r3.Vec, angle float64) r3.Transform {
	t.Helper()
	turn, err := r3.RotationAround(about, axis, units.Radians(angle))
	require.NoError(t, err)
	return turn
}

// straightEdge selects the one straight edge of b parallel to dir that has an
// end within a micrometre of p.
func straightEdge(t *testing.T, b *decad.Body, p, dir r3.Vec) decad.EdgeSelector {
	t.Helper()
	for _, e := range b.Edges() {
		if _, ok := e.Curve().(decad.Line3); !ok {
			continue
		}
		for _, v := range []*decad.Vertex{e.Start(), e.End()} {
			if v.Position().Value.Sub(p).Len() < 5e-3 {
				run := e.End().Position().Value.Sub(e.Start().Position().Value)
				if run.Cross(dir).Len() < 1e-6*run.Len() {
					return decad.Edges(decad.EndpointAt(v.Position().Value), decad.ParallelTo(dir)).Exactly(1)
				}
			}
		}
	}
	require.Fail(t, "no such edge", "near %v along %v", p, dir)
	return nil
}

func TestFacetedRetainedBlendStraight(t *testing.T) {
	t.Parallel()

	t.Run("plate edge fillet and chamfer", func(t *testing.T) {
		t.Parallel()
		_, bored := boredPlate(t)
		before, err := bored.Volume()
		require.NoError(t, err)
		filleted, err := decad.Fillet(t.Context(), bored, straightEdge(t, bored, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1)),
			units.Millimeters(2))
		require.NoError(t, err)
		after, err := filleted.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, 20*4*(1-math.Pi/4))
		cyl := 0
		for _, f := range filleted.Faces() {
			if c, ok := f.Surface().(decad.Cylinder); ok && math.Abs(c.Radius.Base()-2) < 1e-12 {
				cyl++
				require.InDelta(t, 2, c.Origin.X, 1e-9)
				require.InDelta(t, 2, c.Origin.Y, 1e-9)
				require.InDelta(t, 1, math.Abs(c.Axis.Z), 1e-12)
			}
		}
		require.Equal(t, 1, cyl)
		mesh, err := filleted.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
		require.NoError(t, err)
		require.True(t, mesh.BoundaryVerified())
		require.True(t, mesh.VolumeVerified())

		_, bored = boredPlate(t)
		chamfered, err := decad.Chamfer(t.Context(), bored, straightEdge(t, bored, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1)),
			units.Millimeters(2))
		require.NoError(t, err)
		afterChamfer, err := chamfered.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, afterChamfer, 20*2*2/2.0)
	})

	t.Run("top edge ends on the plate's side faces", func(t *testing.T) {
		t.Parallel()
		_, bored := boredPlate(t)
		before, err := bored.Volume()
		require.NoError(t, err)
		filleted, err := decad.Fillet(t.Context(), bored, straightEdge(t, bored, r3.NewVec(0, 0, 20), r3.NewVec(1, 0, 0)),
			units.Millimeters(2))
		require.NoError(t, err)
		after, err := filleted.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, 40*4*(1-math.Pi/4))
	})

	// A box tool sketched at z = 10 over x 30..50 and moved by (0, 0.1, 0)
	// steps the plate down at x = 30. Its walls' and floor's frame origins
	// round only within their own planes, so the step faces are exactly
	// x = 30 and z = 10 and state no departure
	// (agent-docs/faceted-source-retention-design.md §3.5): the step's concave
	// and convex edges both take a fillet.
	t.Run("a step cut by a tool moved within its faces' planes", func(t *testing.T) {
		t.Parallel()
		stepped := func(t *testing.T) *decad.Body {
			t.Helper()
			sc := newRetentionScene()
			plane, err := sc.w.CreateOffsetPlane(sc.w.XY(), 10)
			require.NoError(t, err)
			s, err := sc.w.CreateSketch(plane)
			require.NoError(t, err)
			rect := s.CreateRectangle(30, -5, 50, 25)
			s.Fix(rect.A)
			s.Fix(rect.C)
			_, err = s.Solve(t.Context())
			require.NoError(t, err)
			tool, err := decad.Extrude(t.Context(), sc.doc, s, s.Profiles()[0],
				decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
			require.NoError(t, err)
			cut, err := decad.Cut(t.Context(), blendPlate(t, sc), translated(t, tool, 0, 0.1, 0))
			require.NoError(t, err)
			return cut
		}
		const r = 1.0
		corner := 20 * r * r * (1 - math.Pi/4)
		for _, tc := range []struct {
			name string
			z    float64
			want float64
		}{
			{"concave", 10, -corner},
			{"convex", 20, corner},
		} {
			t.Run(tc.name, func(t *testing.T) {
				t.Parallel()
				cut := stepped(t)
				before, err := cut.Volume()
				require.NoError(t, err)
				filleted, err := decad.Fillet(t.Context(), cut,
					straightEdge(t, cut, r3.NewVec(30, 0, tc.z), r3.NewVec(0, 1, 0)), units.Millimeters(r))
				require.NoError(t, err)
				after, err := filleted.Volume()
				require.NoError(t, err)
				requireVolumeChange(t, before, after, tc.want)
			})
		}
	})

	t.Run("a 45 degree edge", func(t *testing.T) {
		t.Parallel()
		// A bevel from (y=0, z=18) to (y=2, z=20) leaves a 45 degree edge
		// along the top at y=2. The tool's slanted wall is sketched at 45° in
		// the YZ plane, so its tag is exactly the wall it denotes.
		beveled := wedgedPlate(t, func(sc *retentionScene) *sketch.Plane { return sc.w.YZ() },
			[][2]float64{{-10, 8}, {12, 30}, {-10, 30}}, 100, r3.Vec{X: 0.5})
		before, err := beveled.Volume()
		require.NoError(t, err)
		const phi, r = math.Pi / 4, 3.0
		filleted, err := decad.Fillet(t.Context(), beveled, straightEdge(t, beveled, r3.NewVec(0, 2, 20), r3.NewVec(1, 0, 0)),
			units.Millimeters(r))
		require.NoError(t, err)
		after, err := filleted.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, 40*r*r*(math.Tan(phi/2)-phi/2))
		cylinders := 0
		for _, f := range filleted.Faces() {
			if c, ok := f.Surface().(decad.Cylinder); ok {
				cylinders++
				require.InDelta(t, r, c.Radius.Base(), 1e-12)
			}
		}
		require.Equal(t, 1, cylinders)
	})

	// The tool's face, through p0 with normal n, cuts the plate's top front
	// edge at an angle, so the edge ends on a slanted face. Each slant is
	// sketched at 45° on an axis plane, so the face's tag is exactly the plane:
	// a rotated tool would state a departure
	// (agent-docs/faceted-source-retention-design.md §3.5). A slant leaning
	// both about z and about y has no exact construction, so the two tilts run
	// separately.
	for _, tc := range []struct {
		name    string
		plane   func(*retentionScene) *sketch.Plane
		corners [][2]float64
		move    r3.Vec
		n, p0   r3.Vec
	}{
		// x − y = 5, leaning 45° about z.
		{"an end face oblique to the edge", func(sc *retentionScene) *sketch.Plane { return sc.w.XY() },
			[][2]float64{{-60, -30}, {-25, -30}, {35, 30}, {-60, 30}}, r3.Vec{Z: 0.5},
			r3.NewVec(1, -1, 0), r3.NewVec(5, 0, 10)},
		// x − z = −5, leaning 45° about y over the edge.
		{"an end face leaning over the edge", func(sc *retentionScene) *sketch.Plane { return sc.w.XZ() },
			[][2]float64{{-30, -25}, {30, 35}, {-30, 35}}, r3.Vec{Y: 0.5},
			r3.NewVec(1, 0, -1), r3.NewVec(0, 0, 5)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			const r = 2.0
			cut := wedgedPlate(t, tc.plane, tc.corners, 80, tc.move)
			before, err := cut.Volume()
			require.NoError(t, err)
			filleted, err := decad.Fillet(t.Context(), cut,
				straightEdge(t, cut, r3.NewVec(40, 0, 20), r3.NewVec(1, 0, 0)), units.Millimeters(r))
			require.NoError(t, err)
			after, err := filleted.Volume()
			require.NoError(t, err)
			// The generator through the section centroid runs from the tool's
			// face to x = 40.
			c := cornerCentroid(r)
			y, z := c, 20-c
			x0 := tc.p0.X - (tc.n.Y*(y-tc.p0.Y)+tc.n.Z*(z-tc.p0.Z))/tc.n.X
			removed := r * r * (1 - math.Pi/4) * (40 - x0)
			requireVolumeChange(t, before, after, removed)
			ellipses := 0
			for _, e := range filleted.Edges() {
				if el, ok := e.Curve().(decad.Ellipse3); ok && math.Abs(el.SemiMinor.Base()-r) < 1e-9 {
					ellipses++
				}
			}
			t.Logf("%d edges with the blend's ellipse", ellipses)
		})
	}

	t.Run("a concave edge in an L section", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		// An L: a 40 mm plate 10 mm thick with a 10 mm wall rising from its
		// left end, 20 mm deep in y and bored through the plate.
		s, err := sc.w.CreateSketch(sc.w.XZ())
		require.NoError(t, err)
		corners := [][2]float64{{0, 0}, {40, 0}, {40, 10}, {10, 10}, {10, 30}, {0, 30}}
		pts := make([]*sketch.Point, len(corners))
		for i, c := range corners {
			pts[i] = s.CreatePoint(c[0], c[1])
			s.Fix(pts[i])
		}
		for i := range pts {
			s.CreateLine(pts[i], pts[(i+1)%len(pts)])
		}
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		prism, err := decad.Extrude(t.Context(), sc.doc, s, s.Profiles()[0], decad.Symmetric{D: units.Millimeters(10)})
		require.NoError(t, err)
		shift, err := r3.Translation(r3.Vec{Y: 10})
		require.NoError(t, err)
		prism, err = decad.Transform(t.Context(), prism, shift)
		require.NoError(t, err)
		bored, err := decad.Cut(t.Context(), prism, zCylinder(t, sc, -30))
		require.NoError(t, err)
		before, err := bored.Volume()
		require.NoError(t, err)
		const r = 2.0
		filleted, err := decad.Fillet(t.Context(), bored, straightEdge(t, bored, r3.NewVec(10, 0, 10), r3.NewVec(0, 1, 0)),
			units.Millimeters(r))
		require.NoError(t, err)
		after, err := filleted.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, -20*r*r*(1-math.Pi/4))
		cylinders := 0
		for _, f := range filleted.Faces() {
			if c, ok := f.Surface().(decad.Cylinder); ok && math.Abs(c.Radius.Base()-r) < 1e-12 {
				cylinders++
				// The blend fills a concave corner: its normal points into the
				// void, toward the axis.
				p := firstVertex(t, f)
				n, err := f.NormalAt(p)
				require.NoError(t, err)
				require.Positive(t, n.Value.Dot(c.Origin.Sub(p)))
			}
		}
		require.Equal(t, 1, cylinders)
		mesh, err := filleted.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
		require.NoError(t, err)
		require.True(t, mesh.BoundaryVerified())
		require.True(t, mesh.VolumeVerified())
	})

	t.Run("concave edges at a wall's foot", func(t *testing.T) {
		t.Parallel()
		// The plate's top face wraps the wall's footprint, so it is reflex at
		// each foot corner; the blend's end caps extend the wall's end faces.
		build := func(t *testing.T) *decad.Body {
			t.Helper()
			sc := newRetentionScene()
			plate := blendPlate(t, sc)
			ws, err := sc.w.CreateSketch(sc.w.XY())
			require.NoError(t, err)
			wr := ws.CreateRectangle(10, 5, 30, 15)
			ws.Fix(wr.A)
			_, err = ws.Solve(t.Context())
			require.NoError(t, err)
			wall, err := decad.Extrude(t.Context(), sc.doc, ws, ws.Profiles()[0],
				decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
			require.NoError(t, err)
			lift, err := r3.Translation(r3.Vec{Z: 15})
			require.NoError(t, err)
			wall, err = decad.Transform(t.Context(), wall, lift)
			require.NoError(t, err)
			joined, err := decad.Union(t.Context(), plate, wall)
			require.NoError(t, err)
			return joined
		}
		const r = 1.0
		fill := r * r * (1 - math.Pi/4)

		joined := build(t)
		before, err := joined.Volume()
		require.NoError(t, err)
		filleted, err := decad.Fillet(t.Context(), joined, straightEdge(t, joined, r3.NewVec(10, 5, 20), r3.NewVec(1, 0, 0)),
			units.Millimeters(r))
		require.NoError(t, err)
		after, err := filleted.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, -20*fill)
		cylinders := 0
		for _, f := range filleted.Faces() {
			if c, ok := f.Surface().(decad.Cylinder); ok && math.Abs(c.Radius.Base()-r) < 1e-12 {
				cylinders++
				// The fill is concave: the blend's normal points into the void,
				// toward the axis.
				p := firstVertex(t, f)
				n, err := f.NormalAt(p)
				require.NoError(t, err)
				require.Positive(t, n.Value.Dot(c.Origin.Sub(p)))
			}
		}
		require.Equal(t, 1, cylinders)
		mesh, err := filleted.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
		require.NoError(t, err)
		require.True(t, mesh.BoundaryVerified())
		require.True(t, mesh.VolumeVerified())

		// The wall's two end faces grew by the end caps.
		endFaces := 0
		for _, f := range filleted.Faces() {
			pl, ok := f.Surface().(decad.Plane)
			if !ok || math.Abs(math.Abs(pl.Frame.N().X)-1) > 1e-9 {
				continue
			}
			if x := pl.Frame.Origin().X; math.Abs(x-10) < 1e-9 || math.Abs(x-30) < 1e-9 {
				endFaces++
			}
		}
		require.Equal(t, 2, endFaces)

		// The two long foot edges together, and a chamfer, build as well.
		joined = build(t)
		before, err = joined.Volume()
		require.NoError(t, err)
		both, err := decad.Fillet(t.Context(), joined,
			decad.Edges(decad.Concave(), decad.ParallelTo(r3.NewVec(1, 0, 0))).Exactly(2), units.Millimeters(r))
		require.NoError(t, err)
		afterBoth, err := both.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, afterBoth, -2*20*fill)

		joined = build(t)
		before, err = joined.Volume()
		require.NoError(t, err)
		chamfered, err := decad.Chamfer(t.Context(), joined, straightEdge(t, joined, r3.NewVec(10, 5, 20), r3.NewVec(1, 0, 0)),
			units.Millimeters(r))
		require.NoError(t, err)
		afterChamfer, err := chamfered.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, afterChamfer, -20*r*r/2)
	})

	t.Run("refusals and the result as an ordinary body", func(t *testing.T) {
		t.Parallel()
		// The left face is 20 mm wide.
		_, bored := boredPlate(t)
		_, err := decad.Fillet(t.Context(), bored, straightEdge(t, bored, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1)),
			units.Millimeters(21))
		require.ErrorIs(t, err, decad.ErrDegenerate, "the blend consumes the face")
		_, bored = boredPlate(t)
		_, err = decad.Fillet(t.Context(), bored, straightEdge(t, bored, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1)),
			units.Millimeters(20))
		require.ErrorIs(t, err, decad.ErrUnsupported, "the held bounds cannot show it fits")

		// A second blend, a Boolean and a placement all take the result.
		sc, bored := boredPlate(t)
		first, err := decad.Fillet(t.Context(), bored, straightEdge(t, bored, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1)),
			units.Millimeters(2))
		require.NoError(t, err)
		second, err := decad.Fillet(t.Context(), first, straightEdge(t, first, r3.NewVec(40, 20, 0), r3.NewVec(0, 0, 1)),
			units.Millimeters(2))
		require.NoError(t, err)
		cut, err := decad.Cut(t.Context(), second, sc.tool(t, 33))
		require.NoError(t, err)
		move, err := r3.Translation(r3.Vec{X: 5, Y: -2, Z: 7})
		require.NoError(t, err)
		moved, err := decad.Transform(t.Context(), cut, move)
		require.NoError(t, err)
		cylinders := 0
		for _, f := range moved.Faces() {
			if c, ok := f.Surface().(decad.Cylinder); ok && math.Abs(c.Radius.Base()-2) < 1e-12 {
				cylinders++
				n, err := f.NormalAt(firstVertex(t, f))
				require.NoError(t, err)
				require.Less(t, n.Bound.Base(), 1.0)
			}
		}
		require.Equal(t, 2, cylinders)
	})
}

// groovedPlate is the plate with a groove along x cut by a Ø8 tool whose axis
// sits at (y, z) = (10, 22): the groove meets the top face in two lines at
// y = 10 ± √12, z = 20.
func groovedPlate(t *testing.T) *decad.Body {
	t.Helper()
	sc := newRetentionScene()
	plate := blendPlate(t, sc)
	s, err := sc.w.CreateSketch(sc.w.YZ())
	require.NoError(t, err)
	c := s.CreatePoint(10, 22)
	s.Fix(c)
	s.CreateCircle(c, 4)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	tool, err := decad.Extrude(t.Context(), sc.doc, s, s.Profiles()[0], decad.Symmetric{D: units.Millimeters(30)})
	require.NoError(t, err)
	shift, err := r3.Translation(r3.Vec{X: 20})
	require.NoError(t, err)
	tool, err = decad.Transform(t.Context(), tool, shift)
	require.NoError(t, err)
	grooved, err := decad.Cut(t.Context(), plate, tool)
	require.NoError(t, err)
	return grooved
}

// shoelaceArea is the area of the polygon through pts, which wind either way.
func shoelaceArea(pts [][2]float64) float64 {
	sum := 0.0
	for i, p := range pts {
		q := pts[(i+1)%len(pts)]
		sum += p[0]*q[1] - q[0]*p[1]
	}
	return math.Abs(sum) / 2
}

func TestFacetedRetainedBlendCylinderLine(t *testing.T) {
	t.Parallel()
	// The groove's edge on the near side of the top face, in (y, z).
	const rad = 4.0
	k := [2]float64{10 - math.Sqrt(rad*rad-4), 20}
	o := [2]float64{10, 22}
	arcPoints := func(from, to [2]float64, n int) [][2]float64 {
		a0 := math.Atan2(from[1]-o[1], from[0]-o[0])
		a1 := math.Atan2(to[1]-o[1], to[0]-o[0])
		sweep := a1 - a0
		for sweep > math.Pi {
			sweep -= 2 * math.Pi
		}
		for sweep < -math.Pi {
			sweep += 2 * math.Pi
		}
		out := make([][2]float64, 0, n+1)
		for i := 0; i <= n; i++ {
			a := a0 + sweep*float64(i)/float64(n)
			out = append(out, [2]float64{o[0] + rad*math.Cos(a), o[1] + rad*math.Sin(a)})
		}
		return out
	}

	t.Run("fillet of the groove's edge", func(t *testing.T) {
		t.Parallel()
		grooved := groovedPlate(t)
		before, err := grooved.Volume()
		require.NoError(t, err)
		beforeArea, err := grooved.Area()
		require.NoError(t, err)
		const r = 1.0
		filleted, err := decad.Fillet(t.Context(), grooved, straightEdge(t, grooved, r3.NewVec(0, k[0], 20), r3.NewVec(1, 0, 0)),
			units.Millimeters(r))
		require.NoError(t, err)
		after, err := filleted.Volume()
		require.NoError(t, err)
		afterArea, err := filleted.Area()
		require.NoError(t, err)
		// The ball rolls inside the material, 1 mm under the top face and 5 mm
		// from the groove's axis: centre (6, 19). It touches the top face at
		// (6, 20) and the groove at (6.8, 19.6).
		centre := [2]float64{6, 19}
		ta, tb := [2]float64{6, 20}, [2]float64{6.8, 19.6}
		region := [][2]float64{k, ta}
		a0 := math.Atan2(ta[1]-centre[1], ta[0]-centre[0])
		a1 := math.Atan2(tb[1]-centre[1], tb[0]-centre[0])
		for i := 1; i < 2000; i++ {
			a := a0 + (a1-a0)*float64(i)/2000
			region = append(region, [2]float64{centre[0] + r*math.Cos(a), centre[1] + r*math.Sin(a)})
		}
		region = append(region, tb)
		region = append(region, arcPoints(tb, k, 2000)...)
		requireVolumeChange(t, before, after, 40*shoelaceArea(region))
		// The surface loses the top face's strip, the groove wall's band and a
		// corner of each end face, and gains the blend's cylinder.
		kq := shoelaceArea(region)
		q := math.Atan2(tb[1]-o[1], tb[0]-o[0]) - math.Atan2(k[1]-o[1], k[0]-o[0])
		phi := math.Acos(0.6)
		requireAreaChange(t, beforeArea, afterArea, 40*(r*phi-(k[0]-ta[0])-rad*q)-2*kq)
		// The growth carries the rewrite's error, the shortfall of the rebuilt
		// faces' geometric area terms and the new faces' own terms
		// (agent-docs/retained-edge-blend-design.md §6.3), about 0.42 here: a
		// loose ceiling, not the bound itself.
		require.Less(t, afterArea.Bound.Mag()-beforeArea.Bound.Mag(), 2.0,
			"the area bound grows by the rewrite's error, the terms' shortfall and the new terms only")
		cylinders := 0
		for _, f := range filleted.Faces() {
			if c, ok := f.Surface().(decad.Cylinder); ok && math.Abs(c.Radius.Base()-r) < 1e-12 {
				cylinders++
				require.InDelta(t, 6, c.Origin.Y, 1e-9)
				require.InDelta(t, 19, c.Origin.Z, 1e-9)
			}
		}
		require.Equal(t, 1, cylinders)
		mesh, err := filleted.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
		require.NoError(t, err)
		require.True(t, mesh.BoundaryVerified())
		require.True(t, mesh.VolumeVerified())
	})

	t.Run("chamfer of the groove's edge", func(t *testing.T) {
		t.Parallel()
		grooved := groovedPlate(t)
		before, err := grooved.Volume()
		require.NoError(t, err)
		const d = 1.0
		chamfered, err := decad.Chamfer(t.Context(), grooved, straightEdge(t, grooved, r3.NewVec(0, k[0], 20), r3.NewVec(1, 0, 0)),
			units.Millimeters(d))
		require.NoError(t, err)
		after, err := chamfered.Volume()
		require.NoError(t, err)
		// The bevel runs from 1 mm along the top face to 1 mm of arc down the wall.
		ta := [2]float64{k[0] - d, 20}
		a := math.Atan2(k[1]-o[1], k[0]-o[0]) + d/rad
		tb := [2]float64{o[0] + rad*math.Cos(a), o[1] + rad*math.Sin(a)}
		region := [][2]float64{k, ta, tb}
		region = append(region, arcPoints(tb, k, 2000)...)
		requireVolumeChange(t, before, after, 40*shoelaceArea(region))
	})

	t.Run("fillet at the foot of a boss lying on the plate", func(t *testing.T) {
		t.Parallel()
		// A Ø8 rod along x from 10 to 30 whose axis sits 2 mm above the plate's
		// top face: it rises out of the plate along two lines. The plate's top
		// face wraps the rod's footprint, so it is reflex at each end.
		sc := newRetentionScene()
		plate := blendPlate(t, sc)
		s, err := sc.w.CreateSketch(sc.w.YZ())
		require.NoError(t, err)
		c := s.CreatePoint(10, 22)
		s.Fix(c)
		s.CreateCircle(c, rad)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		rod, err := decad.Extrude(t.Context(), sc.doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
		require.NoError(t, err)
		shift, err := r3.Translation(r3.Vec{X: 10})
		require.NoError(t, err)
		rod, err = decad.Transform(t.Context(), rod, shift)
		require.NoError(t, err)
		joined, err := decad.Union(t.Context(), plate, rod)
		require.NoError(t, err)
		before, err := joined.Volume()
		require.NoError(t, err)
		const r = 1.0
		filleted, err := decad.Fillet(t.Context(), joined, straightEdge(t, joined, r3.NewVec(10, k[0], 20), r3.NewVec(1, 0, 0)),
			units.Millimeters(r))
		require.NoError(t, err)
		after, err := filleted.Volume()
		require.NoError(t, err)
		// The ball sits in the void, 1 mm above the plate and 5 mm from the
		// rod's axis: centre (10 - sqrt(24), 21).
		centre := [2]float64{10 - math.Sqrt(24), 21}
		ta := [2]float64{centre[0], 20}
		dx, dz := centre[0]-o[0], centre[1]-o[1]
		scale := rad / math.Hypot(dx, dz)
		tb := [2]float64{o[0] + dx*scale, o[1] + dz*scale}
		region := [][2]float64{k, ta}
		a0 := math.Atan2(ta[1]-centre[1], ta[0]-centre[0])
		a1 := math.Atan2(tb[1]-centre[1], tb[0]-centre[0])
		sweep := a1 - a0
		for sweep > math.Pi {
			sweep -= 2 * math.Pi
		}
		for sweep < -math.Pi {
			sweep += 2 * math.Pi
		}
		for i := 1; i < 2000; i++ {
			a := a0 + sweep*float64(i)/2000
			region = append(region, [2]float64{centre[0] + r*math.Cos(a), centre[1] + r*math.Sin(a)})
		}
		region = append(region, tb)
		region = append(region, arcPoints(tb, k, 2000)...)
		requireVolumeChange(t, before, after, -20*shoelaceArea(region))
		mesh, err := filleted.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
		require.NoError(t, err)
		require.True(t, mesh.BoundaryVerified())
		require.True(t, mesh.VolumeVerified())
	})
}

func TestFacetedRetainedBlendArc(t *testing.T) {
	t.Parallel()
	// A Ø8 bore at x = 2 breaks out through the plate's x = 0 face: its rims are
	// arcs, from one end on that face to the other.
	broken := func(t *testing.T) *decad.Body {
		t.Helper()
		sc := newRetentionScene()
		bored, err := decad.Cut(t.Context(), blendPlate(t, sc), zCylinderAt(t, sc, 2, 10, 4, -30, 30))
		require.NoError(t, err)
		return bored
	}
	topRim := func(t *testing.T, b *decad.Body) decad.EdgeSelector {
		t.Helper()
		for _, e := range b.Edges() {
			if arc, ok := e.Curve().(decad.Arc3); ok && math.Abs(arc.Center.Z-20) < 1e-6 {
				return decad.Edges(decad.Circular(), decad.EndpointAt(e.Start().Position().Value)).Exactly(1)
			}
		}
		require.Fail(t, "no arc at the top")
		return nil
	}
	// mouth is the volume a fillet of radius r takes from the arc's rim: the
	// corner region swept over the angles where the plate has material.
	mouth := func(r float64) float64 {
		const n = 1500
		sum := 0.0
		for i := range n {
			rho := 4 + r*(float64(i)+0.5)/n
			span := 2*math.Pi - 2*math.Acos(2/rho)
			for j := range n {
				z := -r + r*(float64(j)+0.5)/n
				dr, dz := rho-(4+r), z+r
				if dr*dr+dz*dz > r*r {
					sum += span * rho * (r / n) * (r / n)
				}
			}
		}
		return sum
	}

	t.Run("fillet of an arc that breaks out of the side", func(t *testing.T) {
		t.Parallel()
		bored := broken(t)
		before, err := bored.Volume()
		require.NoError(t, err)
		filleted, err := decad.Fillet(t.Context(), bored, topRim(t, bored), units.Millimeters(1))
		require.NoError(t, err)
		after, err := filleted.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, mouth(1))
		tori := 0
		for _, f := range filleted.Faces() {
			if _, ok := f.Surface().(decad.Torus); ok {
				tori++
			}
		}
		require.Equal(t, 1, tori)
		mesh, err := filleted.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
		require.NoError(t, err)
		require.True(t, mesh.BoundaryVerified())
		require.True(t, mesh.VolumeVerified())
	})

	t.Run("chamfer of an arc that breaks out of the side", func(t *testing.T) {
		t.Parallel()
		bored := broken(t)
		before, err := bored.Volume()
		require.NoError(t, err)
		const d = 1.0
		chamfered, err := decad.Chamfer(t.Context(), bored, topRim(t, bored), units.Millimeters(d))
		require.NoError(t, err)
		after, err := chamfered.Volume()
		require.NoError(t, err)
		const n = 1500
		want := 0.0
		for i := range n {
			rho := 4 + d*(float64(i)+0.5)/n
			span := 2*math.Pi - 2*math.Acos(2/rho)
			for j := range n {
				z := -d + d*(float64(j)+0.5)/n
				if z >= rho-4-d {
					want += span * rho * (d / n) * (d / n)
				}
			}
		}
		requireVolumeChange(t, before, after, want)
		cones := 0
		for _, f := range chamfered.Faces() {
			if _, ok := f.Surface().(decad.Cone); ok {
				cones++
			}
		}
		require.Equal(t, 1, cones)
	})

	t.Run("refuses an arc whose ends meet one face", func(t *testing.T) {
		t.Parallel()
		// A Ø12 rod along z whose side beyond x = 25 is cut flat: its top rim is
		// an arc from one end on the flat to the other, with the material inside.
		sc := newRetentionScene()
		rs, err := sc.w.CreateSketch(sc.w.XY())
		require.NoError(t, err)
		rc := rs.CreatePoint(20, 10)
		rs.Fix(rc)
		rs.CreateCircle(rc, 6)
		_, err = rs.Solve(t.Context())
		require.NoError(t, err)
		rod, err := decad.Extrude(t.Context(), sc.doc, rs, rs.Profiles()[0],
			decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
		require.NoError(t, err)
		fs, err := sc.w.CreateSketch(sc.w.XY())
		require.NoError(t, err)
		fr := fs.CreateRectangle(25, -20, 60, 40)
		fs.Fix(fr.A)
		_, err = fs.Solve(t.Context())
		require.NoError(t, err)
		slab, err := decad.Extrude(t.Context(), sc.doc, fs, fs.Profiles()[0], decad.Symmetric{D: units.Millimeters(30)})
		require.NoError(t, err)
		lift, err := r3.Translation(r3.Vec{Z: 0.001})
		require.NoError(t, err)
		slab, err = decad.Transform(t.Context(), slab, lift)
		require.NoError(t, err)
		flat, err := decad.Cut(t.Context(), rod, slab)
		require.NoError(t, err)
		var sel decad.EdgeSelector
		for _, e := range flat.Edges() {
			if arc, ok := e.Curve().(decad.Arc3); ok && math.Abs(arc.Center.Z-20) < 1e-3 {
				sel = decad.Edges(decad.Circular(), decad.EndpointAt(e.Start().Position().Value)).Exactly(1)
			}
		}
		require.NotNil(t, sel)
		_, err = decad.Fillet(t.Context(), flat, sel, units.Millimeters(1))
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})
}
