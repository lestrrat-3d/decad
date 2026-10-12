package decad

import (
	"testing"

	"github.com/stretchr/testify/require"
)

func TestCurvedHeelTailStrictOutsideToApex(t *testing.T) {
	blank, tooth, trim := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, tooth)
	require.NoError(t, err)
	require.True(t, ok)
	tail, ok, err := certifyRevolveLoftFacesHeelTail(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	require.NotNil(t, tail)
	require.Same(t, proof.heelFace, tail.capFace)
	require.Equal(t, 0, tail.endX.Cmp(trim.heel.apex))
	require.Equal(t, 3, len(tail.bands))
	require.Equal(t, 0, tail.bands[0].lo.Cmp(tail.terminal.endTime))
	for index, band := range tail.bands {
		require.Equal(t, tail.terminal.upper.band+index, band.band)
		require.True(t, band.lo.Cmp(band.hi) < 0)
		require.Positive(t, band.margin[0].Sign())
		require.Positive(t, band.margin[1].Sign())
	}

	changed := *proof
	changed.section = proof.section
	changed.section.spans = append([]revolveSectionSpan(nil), proof.section.spans...)
	changed.section.spans[changed.section.axisSpan].end.U += 0.125
	rejected, ok, err := certifyRevolveLoftFacesHeelTail(t.Context(), &changed)
	require.NoError(t, err)
	require.False(t, ok)
	require.Nil(t, rejected)

	originalRevision := trim.source.source.revision
	trim.source.source.revision++
	defer func() { trim.source.source.revision = originalRevision }()
	rejected, ok, err = certifyRevolveLoftFacesHeelTail(t.Context(), proof)
	require.NoError(t, err)
	require.False(t, ok)
	require.Nil(t, rejected)
}
