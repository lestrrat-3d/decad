package reportvocab_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/chainrecord"
	"github.com/lestrrat-3d/decad/internal/coilshell"
	"github.com/lestrrat-3d/decad/internal/cupwall"
	"github.com/lestrrat-3d/decad/internal/draftrecord"
	"github.com/lestrrat-3d/decad/internal/patchchain"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/sweepmitre"
	"github.com/lestrrat-3d/decad/internal/sweeprecord"
	"github.com/lestrrat-3d/decad/internal/twistsweep"
	"github.com/stretchr/testify/require"
)

// TestStagedSurveyNamesRootPayloadClass pins the class name a staged-survey
// message prints for each payload that moved out of the root package. The
// message names the old root type, not the moved type's own Go name.
func TestStagedSurveyNamesRootPayloadClass(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		class   string
		payload any
	}{
		{"prismPayload", prismrecord.Payload{}},
		{"allEdgeChamferPayload", breprecord.AllEdgeChamferPayload{}},
		{"brepPayload", breprecord.Payload{}},
		{"stackedPrismPayload", stackedrecord.Payload{}},
		{"chainPayload", chainrecord.Payload{}},
		{"cupPayload", cupwall.Payload{}},
		{"patchPayload", patchchain.Payload{}},
		{"bodyPatchPayload", patchchain.BodyPayload{}},
		{"revolvePayload", revolveplan.Payload{}},
		{"chainRevolvePayload", revolveplan.ChainPayload{}},
		{"mitredSweepPayload", sweepmitre.Payload{}},
		{"coilPayload", coilshell.Payload{}},
		{"twistedSweepPayload", twistsweep.Payload{}},
		{"compositeTwistedSweepPayload", twistsweep.CompositePayload{}},
		{"draftPayload", draftrecord.Payload{}},
		{"twoSidedDraftPayload", draftrecord.TwoSidedPayload{}},
		{"sweepPayload", sweeprecord.Payload{}},
		{"chainSweepPayload", sweeprecord.ChainPayload{}},
	} {
		t.Run(tc.class, func(t *testing.T) {
			t.Parallel()
			diags := reportvocab.UndercutDiagnostics[int, int, struct{}](1, tc.payload,
				reportvocab.UndercutSurvey[int]{Reason: reportvocab.SurveyPayloadStaged})
			require.Len(t, diags, 1)
			require.Equal(t, reportvocab.DiagUnsupportedSurveyPayload, diags[0].Code)
			require.Equal(t, tc.class+" pull survey support is not implemented; use an analytic body or wait for wider "+
				tc.class+" survey support", diags[0].Message)
		})
	}
}
