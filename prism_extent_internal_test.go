package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/prismextent"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"

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
		pp := prismrecord.Payload{Frame: frame, Xform: r3.Identity()}
		for _, g := range []r3.Vec{
			r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1),
		} {
			base := pp.Xform.Apply(frame.Origin()).Dot(g)
			gu := pp.Dir(1, 0, 0).Dot(g)
			gv := pp.Dir(0, 1, 0).Dot(g)
			gz := pp.Dir(0, 0, 1).Dot(g)
			require.Zero(t, proofbound.ExactIsometryDotRound(pp.Xform, frame.Origin(), g, true, base))
			require.Zero(t, proofbound.ExactIsometryDotRound(pp.Xform, frame.U(), g, false, gu))
			require.Zero(t, proofbound.ExactIsometryDotRound(pp.Xform, frame.V(), g, false, gv))
			require.Zero(t, proofbound.ExactIsometryDotRound(pp.Xform, frame.N(), g, false, gz))
			want := proofbound.AbsSumUpper(0, 0, 0, 0, prismDecompositionRoundAllow(gu, gv, gz, base, 23, 9))
			require.Equal(t, want, prismrecord.PlacementCoeffAllow(pp, g, base, gu, gv, gz, 23, 9))
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
		pp := prismrecord.Payload{Frame: frame, Xform: tc.transform}
		g := tc.direction
		base := pp.Xform.Apply(frame.Origin()).Dot(g)
		gu := pp.Dir(1, 0, 0).Dot(g)
		gv := pp.Dir(0, 1, 0).Dot(g)
		gz := pp.Dir(0, 0, 1).Dot(g)
		want := proofbound.AbsSumUpper(
			proofbound.ExactIsometryDotRound(pp.Xform, frame.Origin(), g, true, base),
			proofbound.DirectionalPerturbationAllow(proofbound.ExactIsometryDotRound(pp.Xform, frame.U(), g, false, gu), 23),
			proofbound.DirectionalPerturbationAllow(proofbound.ExactIsometryDotRound(pp.Xform, frame.V(), g, false, gv), 23),
			proofbound.DirectionalPerturbationAllow(proofbound.ExactIsometryDotRound(pp.Xform, frame.N(), g, false, gz), 9),
			prismDecompositionRoundAllow(gu, gv, gz, base, 23, 9),
		)
		require.Equal(t, want, prismrecord.PlacementCoeffAllow(pp, g, base, gu, gv, gz, 23, 9))
	}
}

func prismDecompositionRoundAllow(gu, gv, gz, base, coordUpper, zUpper float64) float64 {
	return prismextent.DecompositionRoundAllow(gu, gv, gz, base, coordUpper, zUpper)
}
