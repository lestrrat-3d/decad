package planar

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

// TestClassifyPlanarStopsAtStepLimit holds ClassifyPlanar to its work limit:
// a limit below the pair's polls refuses with ErrWorkLimit and no result,
// and the limit at the pair's own polls leaves the relation unchanged.
//
// Leg shown to fail: counting steps without comparing them to the limit never
// refuses.
func TestClassifyPlanarStopsAtStepLimit(t *testing.T) {
	floor := fineBox([3]proof.Dyadic{proof.DyZero(), proof.DyZero(), proof.DyInt(-2)},
		[3]proof.Dyadic{proof.DyInt(4), proof.DyInt(4), proof.DyInt(2)}, 4, proof.DyZero())
	guest := fineBox([3]proof.Dyadic{proof.DyInt(1), proof.DyInt(1), proof.DyInt(1)},
		[3]proof.Dyadic{proof.DyInt(1), proof.DyInt(1), proof.DyInt(1)}, 2, proof.DyZero())
	for _, s := range []*PlanarSolid{&floor, &guest} {
		ok, err := CheckPlanarSolid(s, noPollInternal)
		require.NoError(t, err)
		require.True(t, ok, "premise: an audited solid")
	}
	var polls int
	want, err := ClassifyPlanar(&floor, &guest, countingPoll(&polls))
	require.NoError(t, err)
	require.Greater(t, polls, 100, "premise: the pair charges polls")

	old := stepLimit
	t.Cleanup(func() { stepLimit = old })
	stepLimit = polls
	got, err := ClassifyPlanar(&floor, &guest, noPollInternal)
	require.NoError(t, err)
	require.Equal(t, want.Relation, got.Relation)
	stepLimit = polls - 1
	got, err = ClassifyPlanar(&floor, &guest, noPollInternal)
	require.ErrorIs(t, err, ErrWorkLimit)
	require.Equal(t, PlanarResult{}, got)
}
