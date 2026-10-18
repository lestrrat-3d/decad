package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The root handle type names a failed owner mapping reports.
const (
	ownerFace   = "Face"
	ownerEdge   = "Edge"
	ownerVertex = "Vertex"
)

// ownerPanic is the panic value a root handle mapping raises on a node whose
// Owner is not its handle: a failed type assertion naming the handle type.
func ownerPanic(handle string) string {
	return "interface conversion: interface {} is struct {}, not *decad." + handle
}

func ownerRectSketch(t *testing.T) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 10, 6)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	return s, s.Profiles()[0]
}

func ownerPatch(t *testing.T) *Body {
	t.Helper()
	s, p := ownerRectSketch(t)
	patch, err := PatchProfile(t.Context(), New(), s, p)
	require.NoError(t, err)
	return patch
}

func ownerBox(t *testing.T) *Body {
	t.Helper()
	s, p := ownerRectSketch(t)
	box, err := Extrude(t.Context(), New(), s, p, Distance{D: units.Millimeters(4), Dir: Along})
	require.NoError(t, err)
	return box
}

func ownerFirstCoEdge(f *topo.Face) topo.CoEdge { return f.Loops[0].CoEdges[0] }

func ownerMove(t *testing.T) r3.Transform {
	t.Helper()
	move, err := r3.Translation(r3.NewVec(1, 2, 3))
	require.NoError(t, err)
	return move
}

// TestStitchKeepsOwnerAssertions pins the panic Stitch, Unstitch and the
// placement of their payloads raise on a face, edge or vertex node whose
// Owner is not its root handle: the handle mappings the root builders made on
// those nodes before the builders moved into internal/stitchrecord.
//
// Shown to fail first, each with one entry assertion removed: Stitch's face
// walk (face), assertStitchSourceOwners (vertex, free edge, interior edge),
// Unstitch's assertUnstitchFaceOwners (edge, vertex), and placePayload's
// stitch and unstitch arms (the four placed cases). Unstitch's face case is
// the unchanged Body.Faces call.
func TestStitchKeepsOwnerAssertions(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name   string
		handle string
		run    func(t *testing.T)
	}{
		{"stitch face", ownerFace, func(t *testing.T) {
			patch := ownerPatch(t)
			topobuild.Faces(&patch.n)[0].Owner = struct{}{}
			_, _ = Stitch(t.Context(), patch)
		}},
		{"stitch vertex", ownerVertex, func(t *testing.T) {
			patch := ownerPatch(t)
			topobuild.CoEdgeStart(ownerFirstCoEdge(topobuild.Faces(&patch.n)[0])).Owner = struct{}{}
			_, _ = Stitch(t.Context(), patch)
		}},
		{"stitch free edge", ownerEdge, func(t *testing.T) {
			patch := ownerPatch(t)
			ownerFirstCoEdge(topobuild.Faces(&patch.n)[0]).Edge.Owner = struct{}{}
			_, _ = Stitch(t.Context(), patch)
		}},
		{"stitch interior edge", ownerEdge, func(t *testing.T) {
			s, p := ownerRectSketch(t)
			walls, err := Extrude(t.Context(), New(), s, p, Distance{D: units.Millimeters(4), Dir: Along}, WithSurfaceResult())
			require.NoError(t, err)
			var interior *topo.Edge
			for _, e := range topobuild.Edges(&walls.n) {
				if len(e.Faces) == 2 {
					interior = e
					break
				}
			}
			require.NotNil(t, interior)
			interior.Owner = struct{}{}
			_, _ = Stitch(t.Context(), walls)
		}},
		{"unstitch face", ownerFace, func(t *testing.T) {
			box := ownerBox(t)
			topobuild.Faces(&box.n)[0].Owner = struct{}{}
			_, _ = Unstitch(t.Context(), box)
		}},
		{"unstitch edge", ownerEdge, func(t *testing.T) {
			box := ownerBox(t)
			ownerFirstCoEdge(topobuild.Faces(&box.n)[0]).Edge.Owner = struct{}{}
			_, _ = Unstitch(t.Context(), box)
		}},
		{"unstitch vertex", ownerVertex, func(t *testing.T) {
			box := ownerBox(t)
			ownerFirstCoEdge(topobuild.Faces(&box.n)[0]).Edge.Start.Owner = struct{}{}
			_, _ = Unstitch(t.Context(), box)
		}},
		{"placed stitch edge", ownerEdge, func(t *testing.T) {
			box := internalStitchedBox(t, New())
			sp, ok := box.n.Payload.(stitchPayload)
			require.True(t, ok)
			ownerFirstCoEdge(sp.Faces[0]).Edge.Owner = struct{}{}
			_, _ = Transform(t.Context(), box, ownerMove(t))
		}},
		{"placed stitch vertex", ownerVertex, func(t *testing.T) {
			box := internalStitchedBox(t, New())
			sp, ok := box.n.Payload.(stitchPayload)
			require.True(t, ok)
			ownerFirstCoEdge(sp.Faces[0]).Edge.Start.Owner = struct{}{}
			_, _ = Transform(t.Context(), box, ownerMove(t))
		}},
		{"placed unstitch edge", ownerEdge, func(t *testing.T) {
			pieces, err := Unstitch(t.Context(), ownerBox(t))
			require.NoError(t, err)
			up, ok := pieces[0].n.Payload.(unstitchPayload)
			require.True(t, ok)
			ownerFirstCoEdge(up.Face).Edge.Owner = struct{}{}
			_, _ = Transform(t.Context(), pieces[0], ownerMove(t))
		}},
		{"placed unstitch vertex", ownerVertex, func(t *testing.T) {
			pieces, err := Unstitch(t.Context(), ownerBox(t))
			require.NoError(t, err)
			up, ok := pieces[0].n.Payload.(unstitchPayload)
			require.True(t, ok)
			ownerFirstCoEdge(up.Face).Edge.Start.Owner = struct{}{}
			_, _ = Transform(t.Context(), pieces[0], ownerMove(t))
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			require.PanicsWithError(t, ownerPanic(tc.handle), func() { tc.run(t) })
		})
	}
}
