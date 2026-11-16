package gatewitness_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/gatewitness"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

// A displaced cap-loop chamfer answers one witness prism per loop, each point
// within the largest LoopDelta of the denoted body in the plane and within the
// axial term along the sweep (agent-docs/carrier-record-design.md §8.8).
func TestPrismsOfDisplacedCapBlend(t *testing.T) {
	square := func(s float64) sectionrecord.LoopRecord {
		pts := []sectionrecord.Point2{{U: 0, V: 0}, {U: s, V: 0}, {U: s, V: s}, {U: 0, V: s}}
		segs := make([]sectionrecord.CurveSegment, len(pts))
		for i := range pts {
			segs[i] = sectionrecord.LineSeg{Start: pts[i], End: pts[(i+1)%len(pts)], TStart: 0, TEnd: 1}
		}
		return sectionrecord.LoopRecord{Segments: segs}
	}
	base := capblend.Payload{
		Profile:    momentinput.Profile{Outer: square(10), Holes: []sectionrecord.LoopRecord{square(2)}},
		Z1:         9,
		Z0Delta:    1e-9,
		Start:      capblend.Setback{Ds: 2},
		StartLoops: map[int]bool{1: true},
	}
	dp := capblend.DisplacedPayload{Base: base, LoopDelta: []float64{0.01, 0}}
	prisms, displacement, ok := gatewitness.Prisms(dp)
	require.True(t, ok)
	require.Len(t, prisms, 2)
	require.Equal(t, capblend.WitnessPrisms(base), prisms)
	require.GreaterOrEqual(t, displacement, 0.01+base.AxialDelta())
	for _, p := range prisms {
		require.Zero(t, p.SectionDelta)
	}
	_, _, ok = gatewitness.Prisms(capblend.DisplacedPayload{})
	require.True(t, ok, "the arm answers for any displaced payload")
}
