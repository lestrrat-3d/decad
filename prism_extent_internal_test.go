package decad

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestPrismPlacementCoeffAllowIdentityWorldAxes(t *testing.T) {
	frames := []r3.Frame{identityFrame(t)}
	rotated, err := r3.NewFrame(
		r3.NewVec(7.5, -3.25, 11),
		r3.NewVec(0.6, 0.8, 0),
		r3.NewVec(-0.8, 0.6, 0),
	)
	require.NoError(t, err)
	frames = append(frames, rotated)
	for _, frame := range frames {
		pp := prismPayload{frame: frame, xform: r3.Identity()}
		for _, g := range []r3.Vec{
			r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1),
		} {
			base := pp.xform.Apply(frame.Origin()).Dot(g)
			gu := pp.dir(1, 0, 0).Dot(g)
			gv := pp.dir(0, 1, 0).Dot(g)
			gz := pp.dir(0, 0, 1).Dot(g)
			require.Zero(t, exactIsometryDotRound(pp.xform, frame.Origin(), g, true, base))
			require.Zero(t, exactIsometryDotRound(pp.xform, frame.U(), g, false, gu))
			require.Zero(t, exactIsometryDotRound(pp.xform, frame.V(), g, false, gv))
			require.Zero(t, exactIsometryDotRound(pp.xform, frame.N(), g, false, gz))
			want := absSumUpper(0, 0, 0, 0, prismDecompositionRoundAllow(gu, gv, gz, base, 23, 9))
			require.Equal(t, want, prismPlacementCoeffAllow(pp, g, base, gu, gv, gz, 23, 9))
		}
	}
}

func TestPrismPlacementCoeffAllowKeepsGeneralProof(t *testing.T) {
	frame, err := r3.NewFrame(
		r3.NewVec(7.5, -3.25, 11),
		r3.NewVec(0.6, 0.8, 0),
		r3.NewVec(-0.8, 0.6, 0),
	)
	require.NoError(t, err)
	rotation, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
	require.NoError(t, err)
	for _, tc := range []struct {
		transform r3.Transform
		direction r3.Vec
	}{
		{r3.Identity(), r3.NewVec(0.6, 0.8, 0)},
		{rotation, r3.NewVec(1, 0, 0)},
	} {
		pp := prismPayload{frame: frame, xform: tc.transform}
		g := tc.direction
		base := pp.xform.Apply(frame.Origin()).Dot(g)
		gu := pp.dir(1, 0, 0).Dot(g)
		gv := pp.dir(0, 1, 0).Dot(g)
		gz := pp.dir(0, 0, 1).Dot(g)
		want := absSumUpper(
			exactIsometryDotRound(pp.xform, frame.Origin(), g, true, base),
			directionalPerturbationAllow(exactIsometryDotRound(pp.xform, frame.U(), g, false, gu), 23),
			directionalPerturbationAllow(exactIsometryDotRound(pp.xform, frame.V(), g, false, gv), 23),
			directionalPerturbationAllow(exactIsometryDotRound(pp.xform, frame.N(), g, false, gz), 9),
			prismDecompositionRoundAllow(gu, gv, gz, base, 23, 9),
		)
		require.Equal(t, want, prismPlacementCoeffAllow(pp, g, base, gu, gv, gz, 23, 9))
	}
}
