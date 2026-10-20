package revolveplan_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

func rectBetween(v0, v1 float64) momentinput.Profile {
	corners := []sectionrecord.Point2{{U: 1, V: v0}, {U: 3, V: v0}, {U: 3, V: v1}, {U: 1, V: v1}}
	var segs []sectionrecord.CurveSegment
	for i, c := range corners {
		segs = append(segs, sectionrecord.LineSeg{Start: c, End: corners[(i+1)%4], TEnd: 1})
	}
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segs}}
}

func TestPayloadBlendAxis(t *testing.T) {
	t.Parallel()

	rp := revolveplan.Payload{Ax: revolveaxis.Frame{DU: 1}}
	work := freeform.NewFreeformWork()

	ax, err := rp.BlendAxis(t.Context(), rectBetween(1, 3), work)
	require.NoError(t, err)
	require.Equal(t, 1.0, ax.DU)
	require.Zero(t, ax.DV)

	_, err = rp.BlendAxis(t.Context(), rectBetween(-3, -1), work)
	require.ErrorIs(t, err, decaderr.ErrDegenerate)
	require.ErrorContains(t, err, "lies across the revolve axis")

	// A displaced-by-bound axis origin cannot decide the side.
	loose := revolveplan.Payload{Ax: revolveaxis.Frame{DU: 1, AVBound: 100}}
	_, err = loose.BlendAxis(t.Context(), rectBetween(1, 3), work)
	require.ErrorIs(t, err, decaderr.ErrDegenerate)
	require.ErrorContains(t, err, "which does not decide which side of the axis")

	// The receiver's axis is read field by field: a displaced axis origin
	// moves the side the same profile falls on.
	shifted := revolveplan.Payload{Ax: revolveaxis.Frame{DU: 1, AV: 2}}
	_, err = shifted.BlendAxis(t.Context(), rectBetween(1, 3), work)
	require.Error(t, err)
}
