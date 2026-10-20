package revolvecontact_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/revolvecontact"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/stretchr/testify/require"
)

func TestUniqueResultSourceFace(t *testing.T) {
	t.Parallel()

	faces := []*topo.Face{{}, {}, {}}
	body := &topo.Body{Lumps: []*topo.Lump{{Shells: []*topo.FaceShell{{Faces: faces}}}}}

	body.Payload = facetedrecord.Payload{Src: []int{4, 7, 7}, FaceOf: []int{0, 1, 2}}
	require.Same(t, faces[0], revolvecontact.UniqueResultSourceFace(body, 4), "one face names the group")
	require.Nil(t, revolvecontact.UniqueResultSourceFace(body, 7), "a split group names no single face")
	require.Nil(t, revolvecontact.UniqueResultSourceFace(body, 9), "an absent group names no face")

	body.Payload = nil
	require.Nil(t, revolvecontact.UniqueResultSourceFace(body, 4), "a body with no faceted payload names no face")
}
