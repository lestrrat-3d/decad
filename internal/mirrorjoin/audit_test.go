package mirrorjoin_test

import (
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/mirrorjoin"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

func unitSquare(counterClockwise bool) momentinput.Profile {
	corners := []sectionrecord.Point2{{}, {U: 1}, {U: 1, V: 1}, {V: 1}}
	if !counterClockwise {
		slices.Reverse(corners)
	}
	segs := make([]sectionrecord.CurveSegment, len(corners))
	for i := range corners {
		segs[i] = sectionrecord.LineSeg{Start: corners[i], End: corners[(i+1)%len(corners)], TStart: 0, TEnd: 1}
	}
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segs}}
}

func TestAuditJoinedSection(t *testing.T) {
	t.Run("counter-clockwise square", func(t *testing.T) {
		err := mirrorjoin.AuditJoinedSection(proofbound.NewWorkBudget(t.Context()),
			unitSquare(true), freeform.NewFreeformWork())
		require.NoError(t, err)
	})
	t.Run("clockwise square", func(t *testing.T) {
		err := mirrorjoin.AuditJoinedSection(proofbound.NewWorkBudget(t.Context()),
			unitSquare(false), freeform.NewFreeformWork())
		require.ErrorIs(t, err, decaderr.ErrDegenerate)
		require.ErrorContains(t, err, "the join's spliced loop 0 winds the wrong way")
	})
}
