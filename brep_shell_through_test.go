package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/throughshell"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestShellCutGates shells the box with one through bore by removing the
// bore's wall and breaks one fact of the route at a time. A broken fact makes
// the route refuse with the gate that owns it; removing another face takes the
// general cavity route.
func TestShellCutGates(t *testing.T) {
	t.Parallel()

	run := func(t *testing.T, edit func(*topo.Body, *brepPayload, *throughshell.Call)) (int, error) {
		t.Helper()
		body, bp, _ := allEdgeChamferFixture(t)
		bp.Faces = append([]breprecord.Face(nil), bp.Faces...)
		var wall *topo.Face
		for _, f := range topobuild.Faces(body) {
			if _, ok := f.Surface.(surfacegeom.Cylinder); ok {
				wall = f
			}
		}
		require.NotNil(t, wall)
		call := throughshell.Call{Removed: []*topo.Face{wall}, Inward: true, T: units.Millimeters(1), Tmm: 1, Tol: shellTol}
		edit(body, &bp, &call)
		out, err := throughshell.ShellCut(t.Context(), envOf(body.Doc, body.Doc.NextProducer), body, bp, call, nil)
		if err != nil {
			return 0, err
		}
		return len(topobuild.Faces(out)), nil
	}

	t.Run("round port", func(t *testing.T) {
		t.Parallel()
		faces, err := run(t, func(*topo.Body, *brepPayload, *throughshell.Call) {})
		require.NoError(t, err)
		require.Equal(t, 14, faces)
	})
	t.Run("a cap removed takes the cavity route", func(t *testing.T) {
		t.Parallel()
		faces, err := run(t, func(b *topo.Body, _ *brepPayload, c *throughshell.Call) {
			c.Removed = []*topo.Face{topobuild.Faces(b)[0]}
		})
		require.NoError(t, err)
		require.Equal(t, 13, faces)
	})

	refusals := []struct {
		name string
		want string
		edit func(*topo.Body, *brepPayload, *throughshell.Call)
	}{
		{"outward wall", "modify-general SG1", func(_ *topo.Body, _ *brepPayload, c *throughshell.Call) { c.Inward = false }},
		{"held thickness", "needs an exactly converted thickness", func(_ *topo.Body, _ *brepPayload, c *throughshell.Call) { c.TDelta = 1e-9 }},
		{"thickness fills the box", "has too little wall thickness or port clearance", func(_ *topo.Body, _ *brepPayload, c *throughshell.Call) {
			c.T, c.Tmm = units.Millimeters(9), 9
		}},
		{"thickness not exact", "needs exactly represented offset coordinates", func(_ *topo.Body, _ *brepPayload, c *throughshell.Call) {
			c.T, c.Tmm = units.Millimeters(0.1), 0.1
		}},
		{"section displacement", "needs exact source faces and levels", func(_ *topo.Body, bp *brepPayload, _ *throughshell.Call) {
			bp.Faces[0].Delta = 1
		}},
		{"bottom displacement", "needs exact source faces and levels", func(_ *topo.Body, bp *brepPayload, _ *throughshell.Call) {
			bp.Faces[0].Z0Delta = 1
		}},
		{"top displacement", "needs exact source faces and levels", func(_ *topo.Body, bp *brepPayload, _ *throughshell.Call) {
			bp.Faces[0].Z1Delta = 1
		}},
		{"wall removed with another face", "modify-general SG4", func(b *topo.Body, _ *brepPayload, c *throughshell.Call) {
			c.Removed = append(c.Removed, topobuild.Faces(b)[0])
		}},
		{"six faces", "modify-general SG3", func(_ *topo.Body, bp *brepPayload, _ *throughshell.Call) {
			bp.Faces = bp.Faces[:6]
		}},
	}
	for _, tc := range refusals {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			_, err := run(t, tc.edit)
			require.ErrorIs(t, err, ErrUnsupported)
			require.ErrorContains(t, err, tc.want)
		})
	}
}
