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
// M3: Fillet and Chamfer of a closed rounded outline of a mesh Boolean result.
// The cross-drilled rounded box has two such outlines, the top and the bottom
// face's, each four lines and four r = 3 arcs.

const (
	chainBoxR  = 3.0
	chainLines = 2*34 + 2*14 // the straight run of one outline
)

// chainFilletRemoved is the volume one closed outline's fillet of radius r
// removes: the corner area times the path length at the corner's centroid.
func chainFilletRemoved(r float64) float64 {
	af := r * r * (1 - math.Pi/4)
	xbar := r * (10 - 3*math.Pi) / (3 * (4 - math.Pi))
	return af * (chainLines + 2*math.Pi*(chainBoxR-xbar))
}

// chainChamferRemoved is the same for a chamfer of setback tp on the base
// and tw on the walls.
func chainChamferRemoved(tp, tw float64) float64 {
	return tp * tw / 2 * (chainLines + 2*math.Pi*(chainBoxR-tp/3))
}

func xParallel(n int) decad.EdgeSelector {
	return decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0))).Exactly(n)
}

// topFront selects the top-front edge from (3,0,20) to (37,0,20).
func topFront() decad.EdgeSelector {
	return decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0)), decad.EndpointAt(r3.NewVec(3, 0, 20))).Exactly(1)
}

func requireVerifiedMesh(t *testing.T, b *decad.Body) {
	t.Helper()
	mesh, err := b.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

// blendKinds counts the cylinders of the given radius and the tori of the
// given tube radius.
func blendKinds(b *decad.Body, radius float64) (cylinders, tori int, majors []float64) {
	for _, f := range b.Faces() {
		switch s := f.Surface().(type) {
		case decad.Cylinder:
			if math.Abs(s.Radius.Base()-radius) < 1e-12 {
				cylinders++
			}
		case decad.Torus:
			if math.Abs(s.Minor.Base()-radius) < 1e-12 {
				tori++
				majors = append(majors, s.Major.Base())
			}
		}
	}
	return cylinders, tori, majors
}

func TestFacetedRetainedBlendChain(t *testing.T) {
	t.Parallel()

	t.Run("seed expands to the closed outline and fillets it", func(t *testing.T) {
		t.Parallel()
		drilled := crossDrilled(t)
		before, err := drilled.Volume()
		require.NoError(t, err)
		const r = 1.0
		out, err := decad.Fillet(t.Context(), drilled, topFront(), units.Millimeters(r), decad.WithTangentChain())
		require.NoError(t, err)
		after, err := out.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, chainFilletRemoved(r))
		cylinders, tori, majors := blendKinds(out, r)
		require.Equal(t, 4, cylinders)
		require.Equal(t, 4, tori)
		for _, m := range majors {
			// The ball lies inside the corner cylinder: the tube centre is R - r.
			require.InDelta(t, chainBoxR-r, m, 1e-12)
		}
		requireVerifiedMesh(t, out)
	})

	t.Run("junctions of the blends meet on one shared section", func(t *testing.T) {
		t.Parallel()
		drilled := crossDrilled(t)
		const r = 1.0
		out, err := decad.Fillet(t.Context(), drilled, topFront(), units.Millimeters(r), decad.WithTangentChain())
		require.NoError(t, err)
		junctions := 0
		for _, e := range out.Edges() {
			cyl, tor := false, false
			for _, f := range e.Faces() {
				switch s := f.Surface().(type) {
				case decad.Cylinder:
					cyl = cyl || math.Abs(s.Radius.Base()-r) < 1e-12
				case decad.Torus:
					tor = true
				}
			}
			if !cyl || !tor {
				continue
			}
			junctions++
			arc, isArc := e.Curve().(decad.Arc3)
			require.True(t, isArc, "a junction between a cylinder and a torus blend is an Arc3")
			require.InDelta(t, r, arc.Radius.Base(), 1e-12)
			// A quarter circle of radius r: its ends are r*sqrt(2) apart.
			chord := e.Start().Position().Value.Sub(e.End().Position().Value).Len()
			require.InDelta(t, r*math.Sqrt2, chord, 1e-9)
		}
		require.Equal(t, 8, junctions)
	})

	t.Run("four x-parallel seeds give two closed chains", func(t *testing.T) {
		t.Parallel()
		drilled := crossDrilled(t)
		before, err := drilled.Volume()
		require.NoError(t, err)
		const r = 1.0
		out, err := decad.Fillet(t.Context(), drilled, xParallel(4), units.Millimeters(r), decad.WithTangentChain())
		require.NoError(t, err)
		after, err := out.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, 2*chainFilletRemoved(r))
		cylinders, tori, _ := blendKinds(out, r)
		require.Equal(t, 8, cylinders)
		require.Equal(t, 8, tori)
		requireVerifiedMesh(t, out)
	})

	t.Run("all eight edges named build without the option", func(t *testing.T) {
		t.Parallel()
		drilled := crossDrilled(t)
		before, err := drilled.Volume()
		require.NoError(t, err)
		var outline *decad.EdgeQuery
		named := 0
		for _, e := range drilled.Edges() {
			p, q := e.Start().Position().Value, e.End().Position().Value
			if math.Abs(p.Z-20) > 1e-9 || math.Abs(q.Z-20) > 1e-9 {
				continue
			}
			named++
			if outline == nil {
				outline = decad.Edges(decad.EndpointAt(p), decad.EndpointAt(q))
				continue
			}
			outline = outline.Or(decad.EndpointAt(p), decad.EndpointAt(q))
		}
		require.Equal(t, 8, named)
		out, err := decad.Fillet(t.Context(), drilled, outline.Exactly(8), units.Millimeters(1))
		require.NoError(t, err)
		after, err := out.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, chainFilletRemoved(1))
	})

	t.Run("a chamfer of the closed outline, equal setbacks", func(t *testing.T) {
		t.Parallel()
		drilled := crossDrilled(t)
		before, err := drilled.Volume()
		require.NoError(t, err)
		out, err := decad.Chamfer(t.Context(), drilled, topFront(), units.Millimeters(1), decad.WithTangentChain())
		require.NoError(t, err)
		after, err := out.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, chainChamferRemoved(1, 1))
		requireVerifiedMesh(t, out)
	})

	t.Run("a chamfer with the base as reference takes both distances", func(t *testing.T) {
		t.Parallel()
		drilled := crossDrilled(t)
		before, err := drilled.Volume()
		require.NoError(t, err)
		out, err := decad.Chamfer(t.Context(), drilled, topFront(), units.Millimeters(2), decad.WithTangentChain(),
			decad.WithAsymmetricChamfer(decad.Faces(decad.Facing(r3.NewVec(0, 0, 1))), units.Millimeters(1)))
		require.NoError(t, err)
		after, err := out.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, chainChamferRemoved(2, 1))
	})

	t.Run("a chamfer of both outlines takes each outline's own base as reference", func(t *testing.T) {
		t.Parallel()
		const tp, tw = 2.0, 1.0
		drilled := crossDrilled(t)
		before, err := drilled.Volume()
		require.NoError(t, err)
		bases := decad.Faces(decad.Facing(r3.NewVec(0, 0, 1))).Or(decad.Facing(r3.NewVec(0, 0, -1)))
		out, err := decad.Chamfer(t.Context(), drilled, xParallel(4), units.Millimeters(tp), decad.WithTangentChain(),
			decad.WithAsymmetricChamfer(bases, units.Millimeters(tw)))
		require.NoError(t, err)
		after, err := out.Volume()
		require.NoError(t, err)
		requireVolumeChange(t, before, after, 2*chainChamferRemoved(tp, tw))
		requireVerifiedMesh(t, out)

		// The corner blends are cones about the corner axes: radius R - tp on
		// the base, widening by tp/tw per millimetre down the wall.
		cones, planes := 0, 0
		for _, f := range out.Faces() {
			switch s := f.Surface().(type) {
			case decad.Cone:
				cones++
				require.InDelta(t, chainBoxR-tp, s.Radius.Base(), 1e-9)
				require.InDelta(t, math.Atan2(tp, tw), s.HalfAngle.Base(), 1e-9)
				require.Contains(t, []float64{0, 20}, s.Origin.Z)
			case decad.Plane:
				if n := s.Frame.N(); math.Abs(n.Z) > 1e-9 && math.Abs(math.Abs(n.Z)-1) > 1e-9 {
					planes++
				}
			}
		}
		require.Equal(t, 8, cones)
		require.Equal(t, 8, planes)

		// Each straight bevel plane passes through the setback generators: tp
		// in along the base and tw along the wall, on the top and the bottom.
		for _, z := range []struct{ base, wall float64 }{{20, 19}, {0, 1}} {
			for _, g := range [][2]r3.Vec{
				{r3.NewVec(10, 2, z.base), r3.NewVec(10, 0, z.wall)},
				{r3.NewVec(10, 18, z.base), r3.NewVec(10, 20, z.wall)},
				{r3.NewVec(2, 10, z.base), r3.NewVec(0, 10, z.wall)},
				{r3.NewVec(38, 10, z.base), r3.NewVec(40, 10, z.wall)},
			} {
				found := false
				for _, f := range out.Faces() {
					pl, ok := f.Surface().(decad.Plane)
					if !ok {
						continue
					}
					n, o := pl.Frame.N(), pl.Frame.Origin()
					if math.Abs(n.Dot(g[0].Sub(o))) < 1e-9 && math.Abs(n.Dot(g[1].Sub(o))) < 1e-9 &&
						math.Abs(math.Abs(n.Z)-1) > 1e-9 {
						found = true
					}
				}
				require.True(t, found, "no bevel plane through %v and %v", g[0], g[1])
			}
		}
	})

	t.Run("a cavity crossing the removed corner is refused", func(t *testing.T) {
		t.Parallel()
		// The cavity's upper front edge reaches into the corner the fillet
		// removes along the top-front edge: its corner (y, z) = (0.25, 19.75)
		// lies between the sharp edge and the fillet's arc, and the rest of the
		// cavity lies in the material below it.
		sc := newRetentionScene()
		drilled, err := decad.Cut(t.Context(), sc.roundedBox(t), sc.tool(t, 20))
		require.NoError(t, err)
		cavity, err := decad.Cut(t.Context(), drilled, placed(t, rectBox(t, sc.w, sc.doc, 15, 0.25, 17, 1.25, 1), 18.75))
		require.NoError(t, err)
		_, err = decad.Fillet(t.Context(), cavity, topFront(), units.Millimeters(1), decad.WithTangentChain())
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "the blend meets another facet of the body")
		requireContactX(t, err, 15, 17)
	})

	t.Run("one straight edge without the option names the option", func(t *testing.T) {
		t.Parallel()
		_, err := decad.Fillet(t.Context(), crossDrilled(t), topFront(), units.Millimeters(1))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "WithTangentChain")
		require.Regexp(t, `(3|37),0,20`, err.Error())
	})
}
