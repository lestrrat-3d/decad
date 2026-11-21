package facetedrecord

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestRawMeshPlaneCoversDistantCoplanarTriangles(t *testing.T) {
	a := r3.Vec{X: 3, Y: 2, Z: 1}
	u, v := r3.Vec{X: 1, Y: 2, Z: 3}, r3.Vec{X: 4, Y: -1, Z: 2}
	// Both small triangles share the same exact plane, while the last
	// triangle is much further from the first one than either edge length.
	far := a.Add(u.Scale(17)).Add(v.Scale(23))
	slab := rawTriangleSlab(a, a.Add(u), a.Add(v), 256)
	require.NotNil(t, slab)
	require.Positive(t, slab.Distance)
	frame := slab.Frame
	n := proof.DvCross(proof.DyVec(frame.U()), proof.DyVec(frame.V()))
	d := proof.DyOfFinite(slab.Distance)
	limit := proof.DyMul(proof.DyMul(d, d), proof.DvDot(n, n))
	for _, point := range []r3.Vec{a, a.Add(u), a.Add(v), far, far.Add(u), far.Add(v)} {
		height := proof.DvDot(n, proof.DvSub(proof.DyVec(point), proof.DyVec(frame.Origin())))
		require.LessOrEqual(t, proof.DyCmp(proof.DyMul(height, height), limit), 0)
	}
	require.Nil(t, rawTriangleSlab(a, a, a, 256))
	require.Nil(t, rawTriangleSlab(a, a.Add(u), a.Add(v), math.Inf(1)))
}
