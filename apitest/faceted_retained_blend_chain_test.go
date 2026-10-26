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

	t.Run("one straight edge without the option names the option", func(t *testing.T) {
		t.Parallel()
		_, err := decad.Fillet(t.Context(), crossDrilled(t), topFront(), units.Millimeters(1))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "WithTangentChain")
		require.Regexp(t, `(3|37),0,20`, err.Error())
	})

	t.Run("an open chain is refused", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		s, err := sc.w.CreateSketch(sc.w.XY())
		require.NoError(t, err)
		r := s.CreateRectangle(0, 0, 40, 20)
		s.Fix(r.A)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		box, err := decad.Extrude(t.Context(), sc.doc, s, s.Profiles()[0],
			decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
		require.NoError(t, err)
		corner, err := decad.Fillet(t.Context(), box,
			decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.EndpointAt(r3.Vec{})).Exactly(1),
			units.Millimeters(3))
		require.NoError(t, err)
		drilled, err := decad.Cut(t.Context(), corner, sc.tool(t, 20))
		require.NoError(t, err)
		var seed decad.EdgeSelector
		for _, e := range drilled.Edges() {
			if arc, ok := e.Curve().(decad.Arc3); ok && math.Abs(arc.Center.Z-20) < 1e-9 {
				seed = decad.Edges(decad.Circular(), decad.EndpointAt(e.Start().Position().Value)).Exactly(1)
			}
		}
		require.NotNil(t, seed)
		_, err = decad.Fillet(t.Context(), drilled, seed, units.Millimeters(1), decad.WithTangentChain())
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "open tangent chain")
	})
}
