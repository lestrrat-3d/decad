package prismrecord_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestSplinePrismBoundsReportApproximationOnlyForBracketedExtreme(t *testing.T) {
	t.Parallel()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	newPayload := func(points []sectionrecord.Point2) prismrecord.Payload {
		return prismrecord.Payload{
			Profile: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
				sectionrecord.FitSplineSeg{Fit: points, TEnd: 1},
			}}},
			Frame: frame, Z1: 5, Xform: r3.Identity(),
		}
	}

	curved := newPayload([]sectionrecord.Point2{{}, {U: 1, V: 2}, {U: 3, V: 2}, {U: 4}})
	box, err := prismrecord.Bounds(t.Context(), curved, freeform.NewFreeformWork(), nil)
	require.NoError(t, err)
	require.Equal(t, measurement.Approximate, box.Exactness)
	require.Positive(t, box.Bound.Base())

	monotone := newPayload([]sectionrecord.Point2{{}, {U: 1, V: 1}, {U: 2, V: 2}, {U: 3, V: 3}})
	box, err = prismrecord.Bounds(t.Context(), monotone, freeform.NewFreeformWork(), nil)
	require.NoError(t, err)
	require.Equal(t, measurement.Exact, box.Exactness)
	require.Zero(t, box.Bound.Base())
}
