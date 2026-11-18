package meshbool

import (
	"errors"
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/r3"
)

// TestStitchManifoldContact pins that a Union of boxes touching only along an
// edge, or only at a vertex, stitches to the refusal the switch-off pass gives,
// not to an invariant failure or a pinched mesh.
//
// Leg shown to fail: stitching without the audit returns ErrBooleanFailed for
// the edge contact and a mesh for the vertex contact.
func TestStitchManifoldContact(t *testing.T) {
	for _, tc := range []struct {
		name string
		loB  r3.Vec
	}{
		{"edge only", r3.Vec{X: 10, Y: 0.5, Z: 10}},
		{"vertex only", r3.Vec{X: 10, Y: 10, Z: 10}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			a := exactBoxMesh(t, r3.Vec{}, r3.Vec{X: 10, Y: 10, Z: 10})
			b := exactBoxMesh(t, tc.loB, r3.Vec{X: tc.loB.X + 10, Y: tc.loB.Y + 10, Z: tc.loB.Z + 10})
			kept, stats, err := MeshBooleanStats(coplanarCtx(t), OpUnion, a, b, NewContactMemo(a, b))
			require.NoError(t, err)
			require.NotNil(t, stats.Coplanar)

			_, err = StitchFacetsWith(t.Context(), kept, StitchOptions{Coplanar: stats.Coplanar})
			got, ok := errors.AsType[*TouchContactError](err)
			require.True(t, ok, "want the legacy refusal, got %v", err)

			_, off := pairCuts(t.Context(), a, b, NewContactMemo(a, b), newKeyMemo())
			want, ok := errors.AsType[*TouchContactError](off)
			require.True(t, ok, "%v", off)
			require.Equal(t, want.What, got.What)
			require.Equal(t, want.Contacts, got.Contacts)

			if tc.name == "edge only" {
				_, err = StitchFacetsContext(t.Context(), kept)
				require.ErrorIs(t, err, decaderr.ErrBooleanFailed, "without the audit the edge is an invariant failure")
			}
		})
	}
}

// TestStitchManifoldAuditPassesAClosedResult pins that the audit leaves a
// manifold result alone.
func TestStitchManifoldAuditPassesAClosedResult(t *testing.T) {
	a := exactBoxMesh(t, r3.Vec{}, r3.Vec{X: 10, Y: 10, Z: 10})
	b := exactBoxMesh(t, r3.Vec{X: 0.5, Z: 10}, r3.Vec{X: 10.5, Y: 10, Z: 20})
	kept, stats, err := MeshBooleanStats(coplanarCtx(t), OpUnion, a, b, NewContactMemo(a, b))
	require.NoError(t, err)
	require.NotNil(t, stats.Coplanar)
	with, err := StitchFacetsWith(t.Context(), kept, StitchOptions{Coplanar: stats.Coplanar})
	require.NoError(t, err)
	plain, err := StitchFacetsContext(t.Context(), kept)
	require.NoError(t, err)
	require.Equal(t, plain.Tris, with.Tris)
}

// TestStitchFacetsWithZeroOptionsUnchanged pins that zero options are the
// plain stitch, on a crossing fixture.
func TestStitchFacetsWithZeroOptionsUnchanged(t *testing.T) {
	a := boxMesh(t, r3.Vec{X: 1, Y: 1, Z: 1}, func(v r3.Vec) r3.Vec { return v }, r3.Vec{})
	b := boxMesh(t, r3.Vec{X: 1, Y: 1, Z: 1}, func(v r3.Vec) r3.Vec { return v }, r3.Vec{X: 0.5, Y: 0.4, Z: 0.3})
	kept, _, err := MeshBooleanStats(t.Context(), OpUnion, a, b, NewContactMemo(a, b))
	require.NoError(t, err)
	want, err := StitchFacetsContext(t.Context(), kept)
	require.NoError(t, err)
	got, err := StitchFacetsWith(t.Context(), kept, StitchOptions{})
	require.NoError(t, err)
	require.Equal(t, want.Verts, got.Verts)
	require.Equal(t, want.Tris, got.Tris)
	require.Equal(t, want.Src, got.Src)
}

func TestManifoldDefects(t *testing.T) {
	t.Parallel()
	tetra := [][3]int{{0, 2, 1}, {0, 1, 3}, {1, 2, 3}, {0, 3, 2}}
	require.Empty(t, manifoldDefects(tetra))
	require.NotEmpty(t, manifoldDefects(tetra[:3]), "an open edge")
	// Two tetrahedra sharing vertex 0 only: every edge pairs, vertex 0 pinches.
	second := [][3]int{{0, 5, 4}, {0, 4, 6}, {4, 5, 6}, {0, 6, 5}}
	pinched := append(append([][3]int{}, tetra...), second...)
	bad := manifoldDefects(pinched)
	require.Len(t, bad, 1)
	_, ok := bad[0]
	require.True(t, ok)
	// A pinch is found even when an edge elsewhere fails too.
	open := append(append([][3]int{}, pinched...), [3]int{10, 11, 12})
	_, ok = manifoldDefects(open)[0]
	require.True(t, ok, "the pinched vertex is reported beside an open edge")
}
