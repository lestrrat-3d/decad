package reportvocab_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/pair"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/stretchr/testify/require"
)

func TestContactReasonOf(t *testing.T) {
	t.Parallel()
	for reason, want := range map[pair.Reason]reportvocab.ContactReason{
		pair.NoReason:           reportvocab.ContactNoReason,
		pair.NoGapProof:         reportvocab.ContactNoGapProof,
		pair.AmbiguousFeature:   reportvocab.ContactAmbiguousFeature,
		pair.PointTooCoarse:     reportvocab.ContactPointTooCoarse,
		pair.PayloadUnsupported: reportvocab.ContactPayloadUnsupported,
		pair.NoNormalProof:      reportvocab.ContactNoNormalProof,
		pair.NoNormalProof + 1:  reportvocab.ContactNoReason,
	} {
		require.Equal(t, want, reportvocab.ContactReasonOf(reason), "pair reason %d", reason)
	}
}
