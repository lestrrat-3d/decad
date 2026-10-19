package prismextent_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/prismextent"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestPlacementCoeffAllowIdentityWorldAxes(t *testing.T) {
	frames := []r3.Frame{identityFrame(t)}
	rotated, err := r3.NewFrame(
		r3.NewVec(7.5, -3.25, 11),
		r3.NewVec(0.6, 0.8, 0),
		r3.NewVec(-0.8, 0.6, 0),
	)
	require.NoError(t, err)
	frames = append(frames, rotated)
	for _, frame := range frames {
		for _, direction := range []r3.Vec{
			r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1),
		} {
			base := frame.Origin().Dot(direction)
			gu := frame.U().Dot(direction)
			gv := frame.V().Dot(direction)
			gz := frame.N().Dot(direction)
			want := proofbound.AbsSumUpper(0, 0, 0, 0,
				prismextent.DecompositionRoundAllow(gu, gv, gz, base, 23, 9))
			got := prismextent.PlacementCoeffAllow(r3.Identity(), frame, direction,
				base, gu, gv, gz, 23, 9)
			require.Equal(t, want, got)
		}
	}
}

func TestPlacementCoeffAllowKeepsGeneralProof(t *testing.T) {
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
		base := tc.transform.Apply(frame.Origin()).Dot(tc.direction)
		gu := tc.transform.Apply(frame.U()).Dot(tc.direction)
		gv := tc.transform.Apply(frame.V()).Dot(tc.direction)
		gz := tc.transform.Apply(frame.N()).Dot(tc.direction)
		want := proofbound.AbsSumUpper(
			proofbound.ExactIsometryDotRound(tc.transform, frame.Origin(), tc.direction, true, base),
			proofbound.DirectionalPerturbationAllow(proofbound.ExactIsometryDotRound(tc.transform, frame.U(), tc.direction, false, gu), 23),
			proofbound.DirectionalPerturbationAllow(proofbound.ExactIsometryDotRound(tc.transform, frame.V(), tc.direction, false, gv), 23),
			proofbound.DirectionalPerturbationAllow(proofbound.ExactIsometryDotRound(tc.transform, frame.N(), tc.direction, false, gz), 9),
			prismextent.DecompositionRoundAllow(gu, gv, gz, base, 23, 9),
		)
		got := prismextent.PlacementCoeffAllow(tc.transform, frame, tc.direction,
			base, gu, gv, gz, 23, 9)
		require.Equal(t, want, got)
	}
}

func identityFrame(t *testing.T) r3.Frame {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	return frame
}
