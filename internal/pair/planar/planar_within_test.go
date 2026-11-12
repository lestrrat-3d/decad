package planar

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/pair"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

func noPollWithin() error { return nil }

func TestClassifyPlanarWithinMatchesFullRelation(t *testing.T) {
	tests := []struct {
		name   string
		a, b   PlanarSolid
		margin proof.Dyadic
		want   pair.Relation
	}{
		{"separated", cubeSolid(proof.DyInt(0), proof.DyInt(2), 0), cubeSolid(proof.DyInt(6), proof.DyInt(8), 0), proof.DyInt(1), pair.Separated},
		{"near within displacement", cubeSolid(proof.DyInt(0), proof.DyInt(2), 0), cubeSolid(proof.DyInt(3), proof.DyInt(5), 0), proof.DyInt(2), 0},
		{"touching at zero displacement", cubeSolid(proof.DyInt(0), proof.DyInt(2), 0), cubeSolid(proof.DyInt(2), proof.DyInt(4), 0), proof.DyInt(0), pair.Touching},
		{"overlapping", cubeSolid(proof.DyInt(0), proof.DyInt(3), 0), cubeSolid(proof.DyInt(2), proof.DyInt(5), 0), proof.DyInt(1), pair.Overlapping},
		{"far beyond displacement", cubeSolid(proof.DyInt(0), proof.DyInt(2), 0), cubeSolid(proof.DyInt(6), proof.DyInt(8), 0), proof.DyInt(3), pair.Separated},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			full, err := ClassifyPlanar(&tt.a, &tt.b, noPollWithin)
			require.NoError(t, err)
			got, err := ClassifyPlanarWithin(&tt.a, &tt.b, tt.margin, noPollWithin)
			require.NoError(t, err)
			if tt.name == "near within displacement" {
				require.Equal(t, pair.Separated, full.Relation)
				require.Equal(t, pair.NoGapProof, got.Reason)
				require.Zero(t, got.Relation)
				return
			}
			require.Equal(t, tt.want, full.Relation)
			require.Equal(t, tt.want, got.Relation)
			if tt.want == pair.Touching {
				require.Equal(t, full.Contacts, got.Contacts)
			}
		})
	}
}

func TestClassifyPlanarWithinStopsNearScanEarly(t *testing.T) {
	a := cubeSolid(proof.DyInt(0), proof.DyInt(2), 0)
	b := cubeSolid(proof.DyInt(3), proof.DyInt(5), 0)
	count := func() *int {
		calls := 0
		return &calls
	}
	fullCalls, cappedCalls := count(), count()
	_, err := ClassifyPlanar(&a, &b, func() error { *fullCalls++; return nil })
	require.NoError(t, err)
	result, err := ClassifyPlanarWithin(&a, &b, proof.DyInt(2), func() error { *cappedCalls++; return nil })
	require.NoError(t, err)
	require.Equal(t, pair.NoGapProof, result.Reason)
	require.Less(t, *cappedCalls, *fullCalls, "the positive displacement cap should stop the distance scan early")
	t.Logf("polls: full=%d, capped=%d", *fullCalls, *cappedCalls)
}
