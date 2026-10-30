package revolveboretess

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestHeldCapPlaneRequiresExactIncidence(t *testing.T) {
	frame, err := r3.NewFrame(r3.Vec{Y: 2}, r3.Vec{X: 1}, r3.Vec{Z: -1})
	require.NoError(t, err)
	tool := prismrecord.Payload{Frame: frame, Xform: r3.Identity()}
	points := []r3.Vec{{X: 2, Y: 5}, {Y: 5, Z: 4}}
	require.True(t, onHeldCap(tool, 3, points))
	points[1].Y = math.Nextafter(5, 6)
	require.False(t, onHeldCap(tool, 3, points))
}
