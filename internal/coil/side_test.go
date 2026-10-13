package coil_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/coil"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

// rectangle is a counterclockwise line rectangle u in [u0, u1], v in [0, 0.5].
func rectangle(u0, u1 float64) momentinput.Profile {
	corners := [4]sectionrecord.Point2{{U: u0, V: 0}, {U: u1, V: 0}, {U: u1, V: 0.5}, {U: u0, V: 0.5}}
	segs := make([]sectionrecord.CurveSegment, 4)
	for i := range segs {
		segs[i] = sectionrecord.LineSeg{Start: corners[i], End: corners[(i+1)%4], TStart: 0, TEnd: 1}
	}
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segs}}
}

func TestProfileSide(t *testing.T) {
	line := revolveaxis.Line2{DV: 1}
	work := freeform.NewFreeformWork

	side, err := coil.ProfileSide(t.Context(), rectangle(1, 2), line, 1, 2, work())
	require.NoError(t, err)
	require.NotZero(t, side)

	_, err = coil.ProfileSide(t.Context(), rectangle(1, 2), line, 0.25, 1, work())
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "at or past the 0.25 mm pitch")

	// Under one turn the axial extent is not compared with the pitch.
	_, err = coil.ProfileSide(t.Context(), rectangle(1, 2), line, 0.25, 0.5, work())
	require.NoError(t, err)

	_, err = coil.ProfileSide(t.Context(), rectangle(0, 1), line, 1, 2, work())
	require.ErrorIs(t, err, decaderr.ErrDegenerate)
	require.ErrorContains(t, err, "touches its axis")
}
