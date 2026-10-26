package meshparse_test

import (
	"bytes"
	"context"
	"encoding/binary"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshparse"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

var limits = meshparse.Limits{Triangles: 100, Bytes: 1 << 20}

func TestOBJWeldsRepeatedCornersAcrossFaces(t *testing.T) {
	t.Parallel()
	src := "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 0\nv 1 0 0\nv 0 0 1\nf 1 2 3\nf 4 5 6\n"
	got, err := meshparse.OBJ(t.Context(), strings.NewReader(src), limits)
	require.NoError(t, err)
	require.Equal(t, []r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)}, got.Vertices)
	require.Equal(t, [][3]int{{0, 1, 2}, {0, 1, 3}}, got.Triangles, "the repeated corners share vertex indices")
}

func TestOBJNegativeIndexCountsBackFromTheLatestVertex(t *testing.T) {
	t.Parallel()
	src := "v 0 0 0\nv 1 0 0\nv 0 1 0\nf -3 -2 -1\n"
	got, err := meshparse.OBJ(t.Context(), strings.NewReader(src), limits)
	require.NoError(t, err)
	require.Equal(t, [][3]int{{0, 1, 2}}, got.Triangles)
}

func TestOBJQuadFansFromItsFirstVertex(t *testing.T) {
	t.Parallel()
	src := "v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nf 1 2 3 4\n"
	got, err := meshparse.OBJ(t.Context(), strings.NewReader(src), limits)
	require.NoError(t, err)
	require.Equal(t, [][3]int{{0, 1, 2}, {0, 2, 3}}, got.Triangles)
}

func TestSTLBinaryCoordinatesAreExactFloat32Values(t *testing.T) {
	t.Parallel()
	var buf bytes.Buffer
	buf.Write(make([]byte, 80))
	_ = binary.Write(&buf, binary.LittleEndian, uint32(1))
	_ = binary.Write(&buf, binary.LittleEndian, [3]float32{})
	for _, v := range [][3]float32{{0.1, 0, 0}, {1, 0, 0}, {0, 1, 0}} {
		_ = binary.Write(&buf, binary.LittleEndian, v)
	}
	buf.Write([]byte{0, 0})
	got, err := meshparse.STL(t.Context(), &buf, limits)
	require.NoError(t, err)
	require.Equal(t, float64(float32(0.1)), got.Vertices[0].X, "0.1 reads as the float32 nearest to it, widened exactly")
}

func TestSTLASCIIKeywordsAreCaseInsensitiveAndLeadingSpaceIsFine(t *testing.T) {
	t.Parallel()
	src := "  \n solid x\n FACET NORMAL 0 0 1\n OUTER LOOP\n VERTEX 0 0 0\n VERTEX 1 0 0\n VERTEX 0 1 0\n ENDLOOP\n ENDFACET\nENDSOLID x\n"
	got, err := meshparse.STL(t.Context(), strings.NewReader(src), limits)
	require.NoError(t, err)
	require.Equal(t, [][3]int{{0, 1, 2}}, got.Triangles)
}

func TestSTLASCIIRefusesMoreTrianglesThanTheLimit(t *testing.T) {
	t.Parallel()
	var b strings.Builder
	b.WriteString("solid x\n")
	for i := range 3 {
		off := float64(i)
		b.WriteString("facet normal 0 0 1\nouter loop\n")
		for _, v := range []string{"0 0", "1 0", "0 1"} {
			b.WriteString("vertex " + v + " " + string(rune('0'+int(off))) + "\n")
		}
		b.WriteString("endloop\nendfacet\n")
	}
	b.WriteString("endsolid x\n")
	_, err := meshparse.STL(t.Context(), strings.NewReader(b.String()), meshparse.Limits{Triangles: 2, Bytes: 1 << 20})
	require.ErrorIs(t, err, meshparse.ErrTooLarge)
}

func TestSTLRefusesGarbageAndCanceledContext(t *testing.T) {
	t.Parallel()
	_, err := meshparse.STL(t.Context(), strings.NewReader("nothing like a mesh"), limits)
	require.ErrorIs(t, err, meshparse.ErrMalformed)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	var buf bytes.Buffer
	buf.Write(make([]byte, 80))
	_ = binary.Write(&buf, binary.LittleEndian, uint32(1))
	buf.Write(make([]byte, 50))
	_, err = meshparse.STL(ctx, &buf, limits)
	require.ErrorIs(t, err, context.Canceled)
}
