package decad

import (
	"errors"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestFixPatchOrientation pins fu153: fixPatchOrientation must not swallow
// its own Face.NormalAt error and leave face.reversed at whatever the caller
// constructed it with. Both cases build a fresh whole-turn Cone band patch
// (chamferedCircularBand, capblend_normal_internal_test.go), read the TRUE
// outward direction independently before touching face.reversed, then flip it
// so the function under test has real work to do.
func TestFixPatchOrientation(t *testing.T) {
	for _, tc := range []struct {
		name string
		// sample returns the point fixPatchOrientation is called with. The
		// "decides the sign" case reuses the build's own valid sample point;
		// "refuses an unreadable sample" uses the cone's own apex, where
		// NormalAt's radial direction is zero.
		sample func(band bandUnderTest, valid r3.Vec, cone Cone) r3.Vec
	}{
		{
			name:   "decides the sign",
			sample: func(_ bandUnderTest, valid r3.Vec, _ Cone) r3.Vec { return valid },
		},
		{
			name:   "refuses an unreadable sample",
			sample: func(_ bandUnderTest, _ r3.Vec, cone Cone) r3.Vec { return cone.Origin },
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			band := chamferedCircularBand(t, diskSection(0, 0, 30), 20, 5, nil)
			cone, ok := band.face.Surface().(Cone)
			require.True(t, ok, "a whole-turn band patch is a Cone")

			// The build's own sample point and reference direction, read back
			// independently: the plane-local basis is orthonormal
			// (docs/api-design.md's r3.Frame contract), so decomposing the
			// TRUE outward normal against it is a dot product, never a
			// matrix solve.
			valid := band.pl.point(band.geom.cU+band.geom.sideRadius, band.geom.cV, band.geom.sideZ)
			truth, err := band.face.NormalAt(valid)
			require.NoError(t, err, "the band built successfully, so its own sample point must read")
			eu, ev, en := band.pl.dir(1, 0, 0), band.pl.dir(0, 1, 0), band.pl.dir(0, 0, 1)
			refU, refV, refZ := truth.Value.Dot(eu), truth.Value.Dot(ev), truth.Value.Dot(en)

			before := band.face.reversed
			band.face.reversed = !before // give the function real work to do

			sample := tc.sample(band, valid, cone)
			err = fixPatchOrientation(band.face, band.pl, sample, refU, refV, refZ)

			if tc.name == "refuses an unreadable sample" {
				require.ErrorIs(t, err, ErrUnsupported)
				require.False(t, errors.Is(err, ErrDegenerate),
					"ErrDegenerate and ErrUnsupported are opposite existence claims; only one may ride along")
				require.Equal(t, !before, band.face.reversed,
					"a refusal must not half-decide before returning")
				return
			}
			require.NoError(t, err)
			require.Equal(t, before, band.face.reversed, "the function restores the correct sign")
			n, err := band.face.NormalAt(valid)
			require.NoError(t, err)
			ref := band.pl.dir(refU, refV, refZ)
			require.Positive(t, n.Value.Dot(ref))
		})
	}
}
