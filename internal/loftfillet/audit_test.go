package loftfillet_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftfillet"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

func loftMixedAuditProfile(midV float64) momentinput.Profile {
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 0, V: 0}, End: sectionrecord.Point2{U: 1, V: 0}, TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 1, V: 0}, End: sectionrecord.Point2{U: 1, V: 1}, TStart: 0, TEnd: 1},
		sectionrecord.FitSplineSeg{Fit: []sectionrecord.Point2{{U: 1, V: 1}, {U: 0.5, V: midV}, {U: 0, V: 1}}, TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 0, V: 1}, End: sectionrecord.Point2{U: 0, V: 0}, TStart: 0, TEnd: 1},
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
			err := loftfillet.AuditProfile(context.Background(), proofbound.NewWorkBudget(context.Background()),
				loftMixedAuditProfile(tc.midV), map[int]bool{0: true}, 0, freeform.NewFreeformWork())
			if tc.refuse {
				require.ErrorIs(t, err, decaderr.ErrUnsupported)
				require.ErrorContains(t, err, "may contact segment")
			} else {
				require.NoError(t, err)
			}
		})
	}
}
