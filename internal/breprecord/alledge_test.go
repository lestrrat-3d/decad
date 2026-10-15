package breprecord_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// crossDrilled is P1's record: a 40x20x20 box with a radius-3 bore through y
// centred at (x, z) = (20, 10), every edge set back by 1 mm.
func crossDrilled() breprecord.AllEdgeChamferPayload {
	return breprecord.AllEdgeChamferPayload{
		Xform:  r3.Identity(),
		Box:    [3][2]float64{{0, 40}, {0, 20}, {0, 20}},
		Center: [2]float64{20, 10},
		Radius: 3,
		D:      1,
	}
}

func buildBody(t *testing.T, p breprecord.AllEdgeChamferPayload) *topo.Body {
	t.Helper()
	body, err := breprecord.BuildAllEdgeChamfer(t.Context(), topo.Env{Doc: &topo.Document{}, Ref: 3, Alloc: topotest.Alloc{}}, p)
	require.NoError(t, err)
	return body
}

func TestBuildAllEdgeChamfer(t *testing.T) {
	t.Parallel()
	p := crossDrilled()
	body := buildBody(t, p)
	faces := topobuild.Faces(body)
	require.Len(t, faces, 19)
	require.Equal(t, "face(0)", faces[0].Origins[0].Role)
	require.Equal(t, "chamfer(0,0,0)", faces[6].Origins[0].Role)
	require.Equal(t, "wall(6)", faces[18].Origins[0].Role)
	for _, e := range topobuild.Edges(body) {
		require.Len(t, e.Faces, 2)
	}
	// 40*20*20 - pi*9*20 - 2*(40+20+20) + 6 = 15846 - 180 pi.
	want := 15846 - 180*math.Pi
	require.InDelta(t, want, body.Volume.Value.Base(), body.Volume.Bound.Base()+1e-9)
	require.Equal(t, r3.NewVec(20, 10, 10), body.Centroid.Value)
	require.Equal(t, p, body.Payload)
}

func TestPlacedKeepsFaceRoles(t *testing.T) {
	t.Parallel()
	quarterTurn, err := r3.FromBasis(r3.Basis{
		EX: r3.NewVec(0, 0, 1), EY: r3.NewVec(0, 1, 0), EZ: r3.NewVec(-1, 0, 0),
	}, r3.NewVec(20, 0, 0))
	require.NoError(t, err)
	env := topo.Env{Doc: &topo.Document{}, Ref: 3, Alloc: topotest.Alloc{}}
	placed, err := crossDrilled().Placed(t.Context(), env, quarterTurn)
	require.NoError(t, err)
	faces := topobuild.Faces(placed)
	require.Len(t, faces, 19)
	require.Equal(t, "face(5)", faces[0].Origins[0].Role)
	require.Equal(t, r3.NewVec(0, 0, 0), placed.Bounds.Min)
	require.Equal(t, r3.NewVec(20, 20, 40), placed.Bounds.Max)
	require.Equal(t, quarterTurn, placed.Payload.Transform())
}

func TestPlacedRefusesObliqueMotion(t *testing.T) {
	t.Parallel()
	oblique, err := r3.Rotation(r3.NewVec(0, 0, 1), units.Degrees(37))
	require.NoError(t, err)
	env := topo.Env{Doc: &topo.Document{}, Alloc: topotest.Alloc{}}
	_, err = crossDrilled().Placed(t.Context(), env, oblique)
	require.Error(t, err)
}
