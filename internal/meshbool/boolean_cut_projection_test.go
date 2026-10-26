package meshbool

import (
	"math"
	"math/big"
	"strconv"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// cutRegionSnapshot copies scalar values, so later scratch reuse cannot change
// the reference and a failure reports only one bounded coordinate or flag.
func cutRegionSnapshot(regions []CutRegion) []string {
	var out []string
	point := func(p proof.Xpt) {
		for _, v := range [4]*big.Int{p.X, p.Y, p.Z, p.W} {
			out = append(out, v.Text(16))
		}
	}
	for _, r := range regions {
		out = append(out, strconv.FormatBool(r.HasAnchor), strconv.Itoa(len(r.Tris)))
		for _, p := range r.Partner {
			for _, v := range [3]float64{p.X, p.Y, p.Z} {
				out = append(out, strconv.FormatUint(math.Float64bits(v), 16))
			}
		}
		point(r.Probe)
		for _, tri := range r.Tris {
			for _, p := range tri {
				point(p)
			}
		}
	}
	return out
}

func TestCutProjectionScratchDoesNotEscapeIntoRegions(t *testing.T) {
	var scratch cutProjectionScratch
	var retained [][]CutRegion
	var snapshots [][]string
	for _, tc := range []struct {
		verts [3]r3.Vec
		half  r3.Vec
		at    r3.Vec
	}{
		{[3]r3.Vec{{}, {X: 10}, {Y: 10}}, r3.Vec{X: 0.8, Y: 0.6, Z: 1.7}, r3.Vec{X: 2.5, Y: 2.5, Z: 0.125}},
		{[3]r3.Vec{{}, {Y: 10}, {X: 10}}, r3.Vec{X: 0.6, Y: 0.8, Z: 1.7}, r3.Vec{X: 2.5, Y: 2.5, Z: -0.125}},
		{[3]r3.Vec{{}, {Y: 10}, {Z: 10}}, r3.Vec{X: 1.7, Y: 0.8, Z: 0.6}, r3.Vec{X: 0.125, Y: 2.5, Z: 2.5}},
	} {
		m, err := PrepContext(t.Context(), tc.verts[:], [][3]int{{0, 1, 2}}, []int{0})
		require.NoError(t, err)
		b := boxMesh(t, tc.half, func(v r3.Vec) r3.Vec { return v }, tc.at)
		var segs []Xseg
		for j := range b.Tris {
			contact, err := TriTriClassify(TriCorners(m, 0), TriCorners(b, j),
				XtriCorners(m, 0), XtriCorners(b, j), m.Norms[0], b.Norms[j])
			require.NoError(t, err)
			if contact.Kind != ContactSegment {
				continue
			}
			require.False(t, contact.P0OnA || contact.P1OnA, "the box cuts a closed interior loop")
			segs = append(segs, Xseg{A: contact.P0, B: contact.P1, Partner: TriCorners(b, j)})
		}
		require.GreaterOrEqual(t, len(segs), 4)
		want, err := CutTriangle(t.Context(), XtriCorners(m, 0), m.Norms[0], segs)
		require.NoError(t, err)
		snapshots = append(snapshots, cutRegionSnapshot(want))
		got, err := cutTriangle(t.Context(), XtriCorners(m, 0), m.Norms[0], segs, &scratch)
		require.NoError(t, err)
		retained = append(retained, got)
		for i, r := range scratch.rats {
			r.SetFrac64(int64(i+17), int64(i+19))
		}
		for i, regions := range retained {
			actual := cutRegionSnapshot(regions)
			require.Len(t, actual, len(snapshots[i]))
			for field, value := range actual {
				require.Equal(t, snapshots[i][field], value, "retained result %d field %d", i, field)
			}
		}
	}
}
