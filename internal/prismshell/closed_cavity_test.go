package prismshell

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/stretchr/testify/require"
)

// TestRequireOneCavity reads the closed shell's lump check directly: one lump
// of one closed outer shell around one closed void passes, and a cavity that
// splits into two voids, a cavity read as a second lump, an open shell or a
// void flag on the wrong shell refuses.
func TestRequireOneCavity(t *testing.T) {
	t.Parallel()
	outer := func() *topo.FaceShell { return &topo.FaceShell{} }
	void := func() *topo.FaceShell { return &topo.FaceShell{Void: true} }
	lump := func(shells ...*topo.FaceShell) *topo.Lump { return &topo.Lump{Shells: shells} }

	require.NoError(t, requireOneCavity([]*topo.Lump{lump(outer(), void())}))
	tests := []struct {
		name  string
		lumps []*topo.Lump
		want  string
	}{
		{"the cavity splits in two", []*topo.Lump{lump(outer(), void(), void())}, "1 lump(s) of 3 shell(s)"},
		{"the cavity reads as a second lump", []*topo.Lump{lump(outer()), lump(outer())}, "2 lump(s) of 2 shell(s)"},
		{"no cavity", []*topo.Lump{lump(outer())}, "1 lump(s) of 1 shell(s)"},
		{"an open cavity", []*topo.Lump{lump(outer(), &topo.FaceShell{Void: true, Open: true})}, "do not close"},
		{"an open outer shell", []*topo.Lump{lump(&topo.FaceShell{Open: true}, void())}, "do not close"},
		{"the outer shell flagged void", []*topo.Lump{lump(void(), void())}, "do not close"},
		{"the cavity not flagged void", []*topo.Lump{lump(outer(), outer())}, "do not close"},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			err := requireOneCavity(tc.lumps)
			require.ErrorIs(t, err, decaderr.ErrUnsupported)
			require.ErrorContains(t, err, tc.want)
		})
	}
}
