package apitest_test

import (
	"encoding/json"
	"fmt"
	"math"
	"os"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// ulpChainSolid builds the solid of internal/facetproof's
// placement_ulp_chain.json fixture in a new document made with opts: 50
// facets whose ridge holds seven vertices one to 23 ulps apart along X near
// x = -2.6, the chain an earlier embedding repair leaves when it spreads
// coincident points over neighbouring floats. It returns the solid and its
// vertices.
func ulpChainSolid(t *testing.T, opts ...decad.DocumentOption) (*decad.Body, []r3.Vec) {
	t.Helper()
	raw, err := os.ReadFile("../internal/facetproof/testdata/placement_ulp_chain.json")
	require.NoError(t, err)
	var d struct {
		V [][3]float64 `json:"V"`
		T [][3]int     `json:"T"`
	}
	require.NoError(t, json.Unmarshal(raw, &d))
	verts := make([]r3.Vec, len(d.V))
	for i, v := range d.V {
		verts[i] = r3.Vec{X: v[0], Y: v[1], Z: v[2]}
	}
	mesh, err := decad.NewMeshBody(t.Context(), verts, d.T)
	require.NoError(t, err)
	body, err := decad.SolidFromMesh(t.Context(), decad.New(opts...), mesh)
	require.NoError(t, err)
	return body, verts
}

// TestTransformKeepsUlpChainApart moves a solid whose held mesh has a chain
// of vertices a few ulps apart to coordinates where the ulp is coarser:
// translations of 50, 100, 200 and 1000 mm along each axis, alone and after a
// quarter turn or an oblique turn. Nearest rounding merges the chain at 100 mm
// along X. Every placement must build, keep the volume within its bounds, and
// move the bounds with the motion.
func TestTransformKeepsUlpChainApart(t *testing.T) {
	t.Parallel()
	quarter, err := r3.Rotation(r3.Vec{Z: 1}, units.Degrees(90))
	require.NoError(t, err)
	tilt, err := r3.Rotation(r3.Vec{X: 1, Y: 2, Z: 3}, units.Degrees(37))
	require.NoError(t, err)
	turns := []struct {
		name string
		t    r3.Transform
	}{{"", r3.Identity()}, {"quarter turn then ", quarter}, {"oblique turn then ", tilt}}
	for _, dist := range []float64{50, 100, 200, 1000} {
		for _, axis := range []struct {
			name string
			dir  r3.Vec
		}{{"X", r3.Vec{X: 1}}, {"Y", r3.Vec{Y: 1}}, {"Z", r3.Vec{Z: 1}}} {
			for _, turn := range turns {
				shift, err := r3.Translation(axis.dir.Scale(dist))
				require.NoError(t, err)
				motion, err := turn.t.Then(shift)
				require.NoError(t, err)
				t.Run(fmt.Sprintf("%s%v along %s", turn.name, dist, axis.name), func(t *testing.T) {
					t.Parallel()
					src, verts := ulpChainSolid(t)
					srcVol, err := src.Volume()
					require.NoError(t, err)
					require.Positive(t, srcVol.Value.Base())
					placed, err := decad.Transform(src, motion).Do(t.Context())
					require.NoError(t, err)
					vol, err := placed.Volume()
					require.NoError(t, err)
					require.InDelta(t, srcVol.Value.Base(), vol.Value.Base(),
						vol.Bound.Base()+srcVol.Bound.Base()+1e-12*srcVol.Value.Base())
					box, err := placed.Bounds()
					require.NoError(t, err)
					bound, err := box.Bound.In(units.Millimeter)
					require.NoError(t, err)
					lo, hi := placedVertexBox(motion, verts)
					for _, pair := range [][2]float64{
						{lo.X, box.Min.X}, {lo.Y, box.Min.Y}, {lo.Z, box.Min.Z},
						{hi.X, box.Max.X}, {hi.Y, box.Max.Y}, {hi.Z, box.Max.Z},
					} {
						require.InDelta(t, pair[0], pair[1], bound+1e-12)
					}
				})
			}
		}
	}

	t.Run("exact-only document refuses the merged chain", func(t *testing.T) {
		t.Parallel()
		src, _ := ulpChainSolid(t, decad.ExactOnly())
		shift, err := r3.Translation(r3.Vec{X: 100})
		require.NoError(t, err)
		_, err = decad.Transform(src, shift).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})
}

// placedVertexBox is the box of verts placed by t.
func placedVertexBox(t r3.Transform, verts []r3.Vec) (r3.Vec, r3.Vec) {
	lo := r3.Vec{X: math.Inf(1), Y: math.Inf(1), Z: math.Inf(1)}
	hi := r3.Vec{X: math.Inf(-1), Y: math.Inf(-1), Z: math.Inf(-1)}
	for _, v := range verts {
		q := t.Apply(v)
		lo = r3.Vec{X: min(lo.X, q.X), Y: min(lo.Y, q.Y), Z: min(lo.Z, q.Z)}
		hi = r3.Vec{X: max(hi.X, q.X), Y: max(hi.Y, q.Y), Z: max(hi.Z, q.Z)}
	}
	return lo, hi
}
