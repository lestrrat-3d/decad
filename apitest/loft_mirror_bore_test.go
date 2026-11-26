package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad"
)

// roundedMirrorJoin lofts a five-tooth gear outline from the XY plane to the outline
// turned by twist half above it, rounds its root corners, and joins it to its
// mirror image across the upper section, the construction of a herringbone
// gear.
func roundedMirrorJoin(t *testing.T, half, twist float64) *decad.Body {
	t.Helper()
	g := newTwistedGear(5, 20*math.Pi/180, 0.05)
	body, bottom, top := g.loft(t, half, twist, 0)
	rounded, err := decad.Fillet(t.Context(), body, rungQuery(bottom, top, half).Exactly(len(bottom)),
		units.Millimeters(0.25))
	require.NoError(t, err)
	end, err := r3.NewFrame(r3.Vec{Z: half}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	mirrored, err := decad.MirroredCopy(t.Context(), rounded, decad.MirrorFrame{Frame: end})
	require.NoError(t, err)
	joined, err := decad.Union(t.Context(), rounded, mirrored)
	require.NoError(t, err)
	return joined
}

// TestLoftMirrorJoinThroughBore cuts a centered through-bore from a rounded
// twisted gear loft joined to its mirror image. The Cut rebuilds the join from
// its bored lower half and that half's reflection, so the result is one solid
// that removes exactly a cylinder of volume πr²·2h within the two volume
// bounds, and every mesh vertex near the axis lies on the bore.
func TestLoftMirrorJoinThroughBore(t *testing.T) {
	const half, r = 1.5, 0.5
	twist := 14.5 * math.Pi / 180
	joined := roundedMirrorJoin(t, half, twist)
	before, err := joined.Volume()
	require.NoError(t, err)
	bored, err := decad.Cut(t.Context(), joined, circleBodyAtZ(t, joined.Document(), 0, r, -1, 2*half+2))
	require.NoError(t, err)
	after, err := bored.Volume()
	require.NoError(t, err)
	full := math.Pi * r * r * 2 * half
	require.InDelta(t, full, before.Value.Base()-after.Value.Base(), before.Bound.Base()+after.Bound.Base())
	require.True(t, bored.IsSolid())
	mesh, err := bored.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Equal(t, 0, loftMirrorChamferEuler(mesh), "the bored shell has one hole")
	near := 0
	for _, p := range mesh.Vertices() {
		if radius := math.Hypot(p.X, p.Y); radius < 2*r {
			require.InDelta(t, r, radius, 1e-9, "a vertex near the axis is on the bore")
			near++
		}
	}
	require.Positive(t, near)
}

// TestLoftMirrorJoinShortToolKeepsTheTop cuts a tool that stops short of the
// mirrored join's top cap. Boring the lower half and reflecting it would bore
// the whole join, so the route declines, and no result may remove the full
// cylinder.
func TestLoftMirrorJoinShortToolKeepsTheTop(t *testing.T) {
	const half, r = 1.5, 0.5
	joined := roundedMirrorJoin(t, half, 14.5*math.Pi/180)
	before, err := joined.Volume()
	require.NoError(t, err)
	short, err := decad.Cut(t.Context(), joined, circleBodyAtZ(t, joined.Document(), 0, r, -1, half+2))
	if err != nil {
		return
	}
	got, err := short.Volume()
	require.NoError(t, err)
	require.Less(t, before.Value.Base()-got.Value.Base(),
		math.Pi*r*r*2*half-before.Bound.Base()-got.Bound.Base())
}
