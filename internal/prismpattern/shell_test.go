package prismpattern_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/prismpattern"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/stretchr/testify/require"
)

// blankOf is a body whose faces carry one origin role each; a nil entry is a
// face with no origin at all.
func blankOf(roles ...*string) *topo.Body {
	body := &topo.Body{}
	shell := &topo.FaceShell{}
	for _, role := range roles {
		face := &topo.Face{Body: body}
		if role != nil {
			face.Origins = []provenance.Ref{{Producer: 1, Role: *role}}
		}
		shell.Faces = append(shell.Faces, face)
	}
	body.Lumps = []*topo.Lump{{Shells: []*topo.FaceShell{shell}}}
	return body
}

func TestShellRefusesAChangedBlank(t *testing.T) {
	t.Parallel()

	wall, start, end := prismrecord.OuterFirstSideRole, provenance.RoleCapStart, provenance.RoleCapEnd
	other := "shellCap"
	tests := []struct {
		name  string
		roles []*string
		want  string
	}{
		{"a face without an origin", []*string{&wall, nil, &end}, "circular blank lost a source role"},
		{"a face of another role", []*string{&wall, &start, &other}, "circular blank changed its three source faces"},
		{"a repeated wall", []*string{&wall, &wall, &end}, "circular blank changed its three source faces"},
		{"a repeated start cap", []*string{&wall, &start, &start}, "circular blank changed its three source faces"},
		{"a repeated end cap", []*string{&wall, &end, &end}, "circular blank changed its three source faces"},
		{"no end cap", []*string{&wall, &start}, "circular blank lost a source face"},
		{"no wall", []*string{&start, &end}, "circular blank lost a source face"},
		{"no start cap", []*string{&wall, &end}, "circular blank lost a source face"},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			mesh, _, _, _, _, err := prismpattern.Shell(t.Context(), blankOf(tc.roles...),
				&facetedrecord.PrismPatternSource{}, nil)
			require.Nil(t, mesh)
			require.ErrorIs(t, err, decaderr.ErrUnsupported)
			require.ErrorContains(t, err, tc.want)
		})
	}
}
