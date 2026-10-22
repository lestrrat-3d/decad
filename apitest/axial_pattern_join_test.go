package apitest_test

import (
	"math"
	"strconv"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad"
)

// joinedByPattern patterns one lobe about the disc's axis and joins the copies.
func joinedByPattern(t *testing.T, r, height float64, n int, center r3.Vec, axis r3.Vec) (*decad.Body, error) {
	t.Helper()
	doc := decad.New()
	disc := circleBodyAtZ(t, doc, 0, r, 0, height)
	lobe := lobeLoft(t, doc, r, 0, 0.4*math.Pi/float64(n), height)
	copies, err := decad.PatternCopies(t.Context(), lobe, decad.CircularPattern{Center: center, Axis: axis, Count: n})
	require.NoError(t, err)
	return decad.Union(t.Context(), disc, append([]*decad.Body{lobe}, copies...)...)
}

// drawnAtAngles joins the same lobes each drawn at its own angle.
func drawnAtAngles(t *testing.T, n int) *decad.Body {
	t.Helper()
	const r, height = 3.0, 2.0
	doc := decad.New()
	disc := circleBodyAtZ(t, doc, 0, r, 0, height)
	lobes := make([]*decad.Body, n)
	for k := range lobes {
		lobes[k] = lobeLoft(t, doc, r, 2*math.Pi*float64(k)/float64(n), 0.4*math.Pi/float64(n), height)
	}
	joined, err := decad.Union(t.Context(), disc, lobes...)
	require.NoError(t, err)
	return joined
}

// TestPatternedLobesJoinADisc joins one lobe and its pattern copies to a disc and
// checks the result against the same lobes each drawn at its angle: the two volumes
// agree within their bounds, and the join's bound carries the rotation's rounding.
func TestPatternedLobesJoinADisc(t *testing.T) {
	t.Parallel()
	for _, n := range []int{8, 12, 17, 24} {
		t.Run(strconv.Itoa(n)+" copies", func(t *testing.T) {
			t.Parallel()
			joined, err := joinedByPattern(t, 3.0, 2.0, n, r3.Vec{}, r3.NewVec(0, 0, 1))
			require.NoError(t, err)
			require.True(t, joined.IsSolid())
			drawn := drawnAtAngles(t, n)
			got, err := joined.Volume()
			require.NoError(t, err)
			want, err := drawn.Volume()
			require.NoError(t, err)
			require.InDelta(t, want.Value.Base(), got.Value.Base(), want.Bound.Base()+got.Bound.Base())
			require.Greater(t, got.Bound.Base(), want.Bound.Base(),
				"placed operands are re-recorded at float precision, and the join says so")
			mesh, err := joined.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			require.True(t, mesh.VolumeVerified())
		})
	}
}

// TestPatternedLobesAboutATiltedAxisAreRefused turns the copies about an axis that
// tilts out of the lobes' plane: no in-plane map exists, so the join takes no
// axial-section route and refuses rather than rounding the tilt away.
func TestPatternedLobesAboutATiltedAxisAreRefused(t *testing.T) {
	t.Parallel()
	_, err := joinedByPattern(t, 2.5, 1.5, 8, r3.Vec{}, r3.NewVec(0.05, 0, 1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
}
