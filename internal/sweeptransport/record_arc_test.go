package sweeptransport_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/sweepinput"
	"github.com/lestrrat-3d/decad/internal/sweeptransport"
	"github.com/stretchr/testify/require"
)

func TestRecordArcWithoutCarrier(t *testing.T) {
	_, err := sweeptransport.RecordArc(sweeptransport.Frame{}, sweepinput.PathRecord{})
	require.ErrorIs(t, err, decaderr.ErrDegenerate)
	require.ErrorContains(t, err, "an arc span holds no circular carrier")
}
