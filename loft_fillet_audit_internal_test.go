package decad

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func loftMixedAuditProfile(midV float64) profileRecord {
	return profileRecord{Outer: loopRecord{Segments: []curveSegment{
		lineSeg{Start: Point2{U: 0, V: 0}, End: Point2{U: 1, V: 0}, TStart: 0, TEnd: 1},
		lineSeg{Start: Point2{U: 1, V: 0}, End: Point2{U: 1, V: 1}, TStart: 0, TEnd: 1},
		fitSplineSeg{Fit: point2ToRecordSlice([]Point2{{U: 1, V: 1}, {U: 0.5, V: midV}, {U: 0, V: 1}}), TStart: 0, TEnd: 1},
		lineSeg{Start: Point2{U: 0, V: 1}, End: Point2{U: 0, V: 0}, TStart: 0, TEnd: 1},
	}}}
}

func TestLoftFilletAuditRefusesPossibleSplineContact(t *testing.T) {
	for _, tc := range []struct {
		name   string
		midV   float64
		refuse bool
	}{
		{name: "separated", midV: 0.8},
		{name: "contact", midV: -0.2, refuse: true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			err := auditLoftFilletProfile(context.Background(), proofbound.NewWorkBudget(context.Background()),
				loftMixedAuditProfile(tc.midV), map[int]bool{0: true}, 0, freeform.NewFreeformWork())
			if tc.refuse {
				require.ErrorIs(t, err, ErrUnsupported)
				require.ErrorContains(t, err, "may contact segment")
			} else {
				require.NoError(t, err)
			}
		})
	}
}
