package decad

import (
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
)

// The census installs one meridian memo; the manifest path installs none.
// Both must certify the same shell manifest, and a shared memo must return
// the same endpoint roots as a direct call.
func TestCurvedCensusMeridianMemoMatchesDirect(t *testing.T) {
	blank, trimmed, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)
	census, ok, err := revolveLoftFacesCensus(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	manifest, ok, err := revolveLoftFacesAllSideManifest(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	require.True(t, revolveLoftFacesSameShellManifest(census.manifest, manifest))

	trim := proof.trim
	memo := loftface.NewMeridianMemo()
	compared := 0
	for _, section := range proof.section.Spans {
		if section.Face == nil || section.Start.U == section.End.U {
			continue
		}
		for segment, source := range trim.Source.Profile.Outer.Segments {
			spans, ok := revolveloftfaces.RevolveLoftFacesStraightOrFitSpans(trim, segment, source)
			if !ok {
				continue
			}
			for _, span := range spans {
				for band := 0; band+1 < len(trim.Maps); band++ {
					for _, atEnd := range [...]bool{false, true} {
						want, wantOK := revolveloftfaces.RevolveLoftFacesEndpointFiniteRootsWith(nil,
							trim, section, span, band, atEnd)
						for range 2 {
							got, gotOK := revolveloftfaces.RevolveLoftFacesEndpointFiniteRootsWith(memo,
								trim, section, span, band, atEnd)
							require.Equal(t, wantOK, gotOK)
							require.Len(t, got, len(want))
							for i := range want {
								require.Equal(t, want[i].Direction, got[i].Direction)
								require.Equal(t, want[i].Finite, got[i].Finite)
								require.Zero(t, want[i].Time.Lo.Cmp(got[i].Time.Lo))
								require.Zero(t, want[i].Time.Hi.Cmp(got[i].Time.Hi))
								require.Zero(t, want[i].X.Lo.Cmp(got[i].X.Lo))
								require.Zero(t, want[i].X.Hi.Cmp(got[i].X.Hi))
							}
						}
						compared++
					}
				}
			}
		}
	}
	require.Positive(t, compared, "the fixture must have nonvertical fitted or straight spans")
}
