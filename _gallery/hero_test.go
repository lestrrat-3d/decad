package main

import (
	"testing"

	"github.com/stretchr/testify/require"
)

func TestHeroAIsOnePart(t *testing.T) {
	ch, err := heroAnimationScript().Channels(0)
	require.NoError(t, err)
	take, err := wordmarkTake(t.Context(), ch)
	require.NoError(t, err)
	require.Len(t, take.Style.Parts, 8) // plate, five letters, peg, dome
}
