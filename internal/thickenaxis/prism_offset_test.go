package thickenaxis

import (
	"context"
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/stretchr/testify/require"
)

func TestOffsetSectionRefusalLandsOnUnsupported(t *testing.T) {
	t.Run("nil and unsupported pass through", func(t *testing.T) {
		require.NoError(t, offsetSectionRefusal(nil))
		require.Same(t, decaderr.ErrUnsupported, offsetSectionRefusal(decaderr.ErrUnsupported))
	})
	t.Run("cancellation passes through unchanged", func(t *testing.T) {
		require.Same(t, context.Canceled, offsetSectionRefusal(context.Canceled))
		require.Equal(t, context.DeadlineExceeded, offsetSectionRefusal(context.DeadlineExceeded))
	})
	t.Run("degenerate becomes unsupported", func(t *testing.T) {
		cause := errors.Join(decaderr.ErrDegenerate, errors.New("flipped offset"))
		got := offsetSectionRefusal(cause)
		require.ErrorIs(t, got, decaderr.ErrUnsupported)
		require.NotErrorIs(t, got, decaderr.ErrDegenerate)
		require.ErrorContains(t, got, "the offset section audit refused")
		require.ErrorContains(t, got, decaderr.ErrDegenerate.Error())
		require.ErrorContains(t, got, "flipped offset")
	})
}
