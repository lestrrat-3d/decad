package decad_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// A finite but extreme direction must not overflow or underflow the parallel
// test that NormalTo and edge ParallelTo share. holePlateBody is a 100×60×8
// plate whose every planar face normal and straight edge is axis-aligned, so a
// 45° direction is normal to no face and parallel to no edge — the huge case
// used to read cross <= eps*la*lb as finite <= +Inf and match everything, the
// subnormal case used to underflow the reference length to 0 and match nothing.
func TestParallelDirsExtremeDirections(t *testing.T) {
	body := holePlateBody(t)

	huge := math.MaxFloat64
	tiny := math.SmallestNonzeroFloat64

	t.Run("NormalToHuge45MatchesNothing", func(t *testing.T) {
		_, err := decad.Faces(decad.NormalTo(r3.NewVec(huge, huge, 0))).SelectFaces(body)
		require.ErrorIs(t, err, decad.ErrNoMatch,
			`a huge 45° direction is normal to no face of the plate`)
	})

	t.Run("NormalToSubnormal45MatchesNothing", func(t *testing.T) {
		_, err := decad.Faces(decad.NormalTo(r3.NewVec(tiny, tiny, 0))).SelectFaces(body)
		require.ErrorIs(t, err, decad.ErrNoMatch,
			`a subnormal 45° direction is normal to no face`)
	})

	t.Run("EdgeParallelToHuge45MatchesNothing", func(t *testing.T) {
		_, err := decad.Edges(decad.ParallelTo(r3.NewVec(huge, huge, 0))).SelectEdges(body)
		require.ErrorIs(t, err, decad.ErrNoMatch,
			`a huge 45° direction is parallel to no straight edge`)
	})

	// A huge direction that REALLY is an axis normal still matches: normalizing
	// must preserve the true parallel, not just suppress the false one. +z is
	// the two caps' normal.
	t.Run("NormalToHugeZStillMatchesBothCaps", func(t *testing.T) {
		faces, err := decad.Faces(decad.NormalTo(r3.NewVec(0, 0, huge))).SelectFaces(body)
		require.NoError(t, err)
		require.Len(t, faces, 2, `NormalTo(+z) matches both caps regardless of magnitude`)
	})

	// A subnormal +z direction likewise still names the caps once the length no
	// longer underflows to zero.
	t.Run("NormalToSubnormalZStillMatchesBothCaps", func(t *testing.T) {
		faces, err := decad.Faces(decad.NormalTo(r3.NewVec(0, 0, tiny))).SelectFaces(body)
		require.NoError(t, err)
		require.Len(t, faces, 2)
	})

	// The threat model requires a genuinely parallel direction to match at ANY
	// magnitude — the edge ParallelTo false-miss path, where a subnormal used to
	// underflow the length to 0 and match nothing. +x is the direction of the
	// plate's four axis-aligned edges (two per cap). Compare against the unit
	// direction so the expected set is the geometry's own, not a hard-coded count.
	unitX, err := decad.Edges(decad.ParallelTo(r3.NewVec(1, 0, 0))).SelectEdges(body)
	require.NoError(t, err)
	require.NotEmpty(t, unitX, `sanity: the plate has edges parallel to +x`)

	t.Run("EdgeParallelToSubnormalXStillMatches", func(t *testing.T) {
		edges, err := decad.Edges(decad.ParallelTo(r3.NewVec(tiny, 0, 0))).SelectEdges(body)
		require.NoError(t, err)
		require.Len(t, edges, len(unitX),
			`a subnormal +x direction is parallel to the same edges as unit +x`)
	})

	t.Run("EdgeParallelToHugeXStillMatches", func(t *testing.T) {
		edges, err := decad.Edges(decad.ParallelTo(r3.NewVec(huge, 0, 0))).SelectEdges(body)
		require.NoError(t, err)
		require.Len(t, edges, len(unitX),
			`a huge +x direction is parallel to the same edges as unit +x`)
	})
}
