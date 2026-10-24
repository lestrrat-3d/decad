package meshbool

import (
	"fmt"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestConformScanTreeFindsEveryExactEdgeInteriorVertex(t *testing.T) {
	for _, offset := range []float64{0, 1e12} {
		t.Run(fmt.Sprintf("offset_%g", offset), func(t *testing.T) {
			point := func(x, y, z float64) proof.Xpt {
				return proof.XptOf(r3.Vec{X: offset + x, Y: offset + y, Z: offset + z})
			}
			verts := []proof.Xpt{
				point(0, 0, 0),
				point(10, 10, 10),
				point(0, 1, 0),
				point(5, 5, 5),
				point(5, 5, 6),
				point(8, 2, 2),
				point(9, 9, 9),
			}
			scan, err := NewConformScan(proofbound.NewWorkBudget(t.Context()), verts)
			require.NoError(t, err)

			tri := [3]int{0, 1, 2}
			got, err := scan.EdgeInteriorHits(proofbound.NewWorkBudget(t.Context()), 0, 1, tri)
			require.NoError(t, err)
			var want []int
			for i := range verts {
				if i == tri[0] || i == tri[1] || i == tri[2] {
					continue
				}
				if OnSegmentInterior3(verts[0], verts[1], verts[i]) {
					want = append(want, i)
				}
			}
			require.Equal(t, want, got)
		})
	}
}
