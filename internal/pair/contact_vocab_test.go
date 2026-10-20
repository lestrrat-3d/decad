package pair_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/pair"
	"github.com/lestrrat-3d/decad/report"
	"github.com/stretchr/testify/require"
)

func TestContactRelationPairRelation(t *testing.T) {
	t.Parallel()

	for relation, want := range map[report.ContactRelation]pair.Relation{
		report.ContactSeparated:   pair.Separated,
		report.ContactTouching:    pair.Touching,
		report.ContactOverlapping: pair.Overlapping,
		report.ContactUndecided:   pair.Undecided,
		report.ContactBand:        pair.Undecided,
	} {
		require.Equal(t, want, pair.RelationOfContact(relation), relation)
	}
}

func TestContactReasonOf(t *testing.T) {
	t.Parallel()
	for reason, want := range map[pair.Reason]report.ContactReason{
		pair.NoReason:           report.ContactNoReason,
		pair.NoGapProof:         report.ContactNoGapProof,
		pair.AmbiguousFeature:   report.ContactAmbiguousFeature,
		pair.PointTooCoarse:     report.ContactPointTooCoarse,
		pair.PayloadUnsupported: report.ContactPayloadUnsupported,
		pair.NoNormalProof:      report.ContactNoNormalProof,
		pair.NoNormalProof + 1:  report.ContactNoReason,
	} {
		require.Equal(t, want, pair.ContactReasonOf(reason), "pair reason %d", reason)
	}
}
