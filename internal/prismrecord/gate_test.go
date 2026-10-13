package prismrecord_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/stretchr/testify/require"
)

func TestRequireExactSection(t *testing.T) {
	require.NoError(t, prismrecord.Payload{}.RequireExactSection("fillets"))

	err := prismrecord.Payload{SectionDelta: 0.25}.RequireExactSection("shells")
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "this evaluator shells a receiver")
	require.ErrorContains(t, err, "a proven displacement of 0.25 mm")
}
