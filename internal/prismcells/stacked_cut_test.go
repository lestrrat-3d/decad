package prismcells_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

func TestCanonicalStackedCutHoleCount(t *testing.T) {
	candidate := momentinput.Profile{Holes: []sectionrecord.LoopRecord{{}}}
	_, err := prismcells.CanonicalStackedCut(proofbound.NewWorkBudget(t.Context()),
		momentinput.Profile{}, &sketch.Profile{}, nil, candidate)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "the cut profile has an unexpected hole count")
}
