package apitest_test

import (
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/stretchr/testify/require"
)

func checkDraftCircleModifyRoles(t *testing.T, body *decad.Body,
	wantSurfaces map[decad.SurfaceKind]int, rolePrefix string) {
	t.Helper()
	for _, cap := range []decad.FeatureRef{decad.CapStart(body), decad.CapEnd(body)} {
		faces, err := decad.Faces(decad.FaceCreatedBy(cap)).Exactly(1).SelectFaces(body)
		require.NoError(t, err)
		require.Equal(t, decad.KindPlane, faces[0].Surface().Kind())
		edges, err := decad.Edges(decad.OuterLoopOf(cap), decad.Circular()).Exactly(1).SelectEdges(body)
		require.NoError(t, err)
		require.Len(t, edges[0].Faces(), 2)
	}
	gotSurfaces := make(map[decad.SurfaceKind]int, len(wantSurfaces))
	var featureFaces int
	for _, face := range body.Faces() {
		kind := face.Surface().Kind()
		if _, ok := wantSurfaces[kind]; ok {
			gotSurfaces[kind]++
		}
		for _, origin := range face.Origins() {
			if strings.HasPrefix(origin.Role, rolePrefix) {
				featureFaces++
			}
		}
	}
	require.Equal(t, wantSurfaces, gotSurfaces)
	require.Equal(t, 1, featureFaces)
	requireEveryEdgeOnTwoFaces(t, body)
}
