package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The fixtures of docs/faceted-source-retention-design.md §1 and §6: a 40x20x20
// box with four lateral edges filleted at r=3, then drilled through y by a Ø8
// tool that a Transform placed, which keeps the pair on the mesh path.

type retentionScene struct {
	w   *sketch.World
	doc *decad.Document
}

func newRetentionScene() *retentionScene {
	return &retentionScene{w: sketch.NewWorld(), doc: decad.New()}
}

func (sc *retentionScene) roundedBox(t *testing.T) *decad.Body {
	t.Helper()
	s, err := sc.w.CreateSketch(sc.w.XY())
	require.NoError(t, err)
	r := s.CreateRectangle(0, 0, 40, 20)
	s.Fix(r.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	box, err := decad.Extrude(t.Context(), sc.doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	require.NoError(t, err)
	rounded, err := decad.Fillet(t.Context(), box,
		decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1))).Exactly(4), units.Millimeters(3))
	require.NoError(t, err)
	return rounded
}

// tool builds a Ø8 cylinder along y through (cx, ·, 10), placed by a pure
// translation so the Cut is not a same-placement class-B pair.
func (sc *retentionScene) tool(t *testing.T, cx float64) *decad.Body {
	t.Helper()
	s, err := sc.w.CreateSketch(sc.w.XZ())
	require.NoError(t, err)
	c := s.CreatePoint(cx, 10)
	s.Fix(c)
	s.CreateCircle(c, 4)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	cyl, err := decad.Extrude(t.Context(), sc.doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(60), Dir: decad.Along})
	require.NoError(t, err)
	move, err := r3.Translation(r3.Vec{Y: 30})
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), cyl, move)
	require.NoError(t, err)
	return placed
}

func retentionKinds(b *decad.Body) map[string]int {
	kinds := map[string]int{}
	for _, f := range b.Faces() {
		switch f.Surface().(type) {
		case decad.Plane:
			kinds["plane"]++
		case decad.Cylinder:
			kinds["cylinder"]++
		case decad.Faceted:
			kinds["faceted"]++
		default:
			kinds["other"]++
		}
	}
	return kinds
}

func firstVertex(t *testing.T, f *decad.Face) r3.Vec {
	t.Helper()
	loops := f.Loops()
	require.NotEmpty(t, loops)
	edges := loops[0].CoEdges()
	require.NotEmpty(t, edges)
	return edges[0].Start().Position().Value
}

func TestFacetedSourceRetention(t *testing.T) {
	t.Parallel()

	t.Run("mesh Cut keeps every operand surface", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		drilled, err := decad.Cut(t.Context(), sc.roundedBox(t), sc.tool(t, 20))
		require.NoError(t, err)
		kinds := retentionKinds(drilled)
		require.Zero(t, kinds["faceted"], "kinds %v", kinds)
		require.Zero(t, kinds["other"], "kinds %v", kinds)
		// Four blends and the hole wall are cylinders; the six box faces stay
		// planes.
		require.Equal(t, 5, kinds["cylinder"], "kinds %v", kinds)
		require.Equal(t, 6, kinds["plane"], "kinds %v", kinds)
	})

	t.Run("normals point out of the material", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		drilled, err := decad.Cut(t.Context(), sc.roundedBox(t), sc.tool(t, 20))
		require.NoError(t, err)
		center := r3.NewVec(20, 10, 10)
		for _, f := range drilled.Faces() {
			p := firstVertex(t, f)
			n, err := f.NormalAt(p)
			require.NoError(t, err)
			require.Less(t, n.Bound.Base(), 1.0, "a retained normal states a usable direction")
			switch f.Surface().(type) {
			case decad.Plane:
				// The six box faces: the normal points away from the centre.
				require.Positive(t, n.Value.Dot(p.Sub(center)), "plane normal at %v", p)
			case decad.Cylinder:
				toAxis := r3.NewVec(20-p.X, 0, 10-p.Z)
				if math.Abs(toAxis.Len()-4) < 0.01 {
					// The hole wall: material is outside the tool, so the
					// outward normal points toward the tool axis.
					require.Positive(t, n.Value.Dot(toAxis), "hole wall normal at %v", p)
					continue
				}
				// A blend: material is inside, so the normal points away
				// from the body centre.
				require.Positive(t, n.Value.Dot(p.Sub(center)), "blend normal at %v", p)
			}
		}
	})

	t.Run("plane normal is exact and cylinder normal carries its displacement", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		drilled, err := decad.Cut(t.Context(), sc.roundedBox(t), sc.tool(t, 20))
		require.NoError(t, err)
		for _, f := range drilled.Faces() {
			n, err := f.NormalAt(firstVertex(t, f))
			require.NoError(t, err)
			if _, ok := f.Surface().(decad.Plane); ok {
				require.Less(t, n.Bound.Base(), 1e-9)
			}
		}
	})

	t.Run("a second Boolean keeps the surfaces", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		once, err := decad.Cut(t.Context(), sc.roundedBox(t), sc.tool(t, 12))
		require.NoError(t, err)
		twice, err := decad.Cut(t.Context(), once, sc.tool(t, 28))
		require.NoError(t, err)
		kinds := retentionKinds(twice)
		require.Zero(t, kinds["faceted"], "kinds %v", kinds)
		require.Equal(t, 6, kinds["cylinder"], "kinds %v", kinds)
	})

	t.Run("Union keeps the orientation of both operands", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		plate := sc.roundedBox(t)
		// A pin standing in the plate's middle, wider than the plate is thick
		// at its ends, so the walls stay outside the part.
		s, err := sc.w.CreateSketch(sc.w.XZ())
		require.NoError(t, err)
		c := s.CreatePoint(20, 10)
		s.Fix(c)
		s.CreateCircle(c, 4)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		pin, err := decad.Extrude(t.Context(), sc.doc, s, s.Profiles()[0],
			decad.Distance{D: units.Millimeters(60), Dir: decad.Along})
		require.NoError(t, err)
		move, err := r3.Translation(r3.Vec{Y: 30})
		require.NoError(t, err)
		placed, err := decad.Transform(t.Context(), pin, move)
		require.NoError(t, err)
		joined, err := decad.Union(t.Context(), plate, placed)
		require.NoError(t, err)
		center := r3.NewVec(20, 10, 10)
		for _, f := range joined.Faces() {
			if _, ok := f.Surface().(decad.Cylinder); !ok {
				continue
			}
			p := firstVertex(t, f)
			n, err := f.NormalAt(p)
			require.NoError(t, err)
			// Every cylinder of a Union (blend or pin wall) bounds material
			// on its inside.
			radial := r3.NewVec(p.X-20, 0, p.Z-10)
			if radial.Len() < 4.5 {
				require.Positive(t, n.Value.Dot(radial), "pin wall normal at %v", p)
				continue
			}
			require.Positive(t, n.Value.Dot(p.Sub(center)), "blend normal at %v", p)
		}
	})

	t.Run("ScaledAt of a retained result still builds", func(t *testing.T) {
		t.Parallel()
		sc := newRetentionScene()
		drilled, err := decad.Cut(t.Context(), sc.roundedBox(t), sc.tool(t, 20))
		require.NoError(t, err)
		scaled, err := decad.ScaledAt(t.Context(), drilled, r3.Vec{}, units.Scalar(2))
		require.NoError(t, err)
		kinds := retentionKinds(scaled)
		require.Zero(t, kinds["other"], "kinds %v", kinds)
		// A scaled plane keeps its tag; a scaled cylinder reads Faceted.
		require.Equal(t, 6, kinds["plane"], "kinds %v", kinds)
		require.Equal(t, 5, kinds["faceted"], "kinds %v", kinds)
	})
}
