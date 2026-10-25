package compositesweep_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/compositesweep"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

func TestCompositeSweepWorkLimitDetails(t *testing.T) {
	err := compositesweep.Preflight(momentinput.Profile{}, 257)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	var detail *proofbound.WorkLimitError
	require.ErrorAs(t, err, &detail)
	require.Equal(t, "sweep_spans", detail.Resource)
	require.Equal(t, uint64(257), detail.Observed)
	require.Equal(t, uint64(256), detail.Maximum)
	profile := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: make([]sectionrecord.CurveSegment, 1024)}}
	err = compositesweep.Preflight(profile, 256)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorAs(t, err, &detail)
	require.Equal(t, "sweep_faces", detail.Resource)
	require.Equal(t, uint64(262_146), detail.Observed)
	require.Equal(t, uint64(262_144), detail.Maximum)
	err = compositesweep.Audit(t.Context(), make([]compositesweep.AuditSpan, 4001))
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorAs(t, err, &detail)
	require.Equal(t, "sweep_span_pairs", detail.Resource)
	require.Equal(t, uint64(8_002_000), detail.Observed)
	require.Equal(t, uint64(8_000_000), detail.Maximum)
}
