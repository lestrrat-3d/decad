package capblend_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

func squareLoop(side float64) sectionrecord.LoopRecord {
	pts := []sectionrecord.Point2{{U: 0, V: 0}, {U: side, V: 0}, {U: side, V: side}, {U: 0, V: side}}
	segs := make([]sectionrecord.CurveSegment, len(pts))
	for i := range pts {
		segs[i] = sectionrecord.LineSeg{Start: pts[i], End: pts[(i+1)%len(pts)], TStart: 0, TEnd: 1}
	}
	return sectionrecord.LoopRecord{Segments: segs}
}

func TestRequireNotReceiver(t *testing.T) {
	err := capblend.RequireNotReceiver(capblend.Payload{}, "chamfers")
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.Contains(t, err.Error(), "chamfers a receiver this evaluator built directly")
	require.NoError(t, capblend.RequireNotReceiver(prismrecord.Payload{}, "chamfers"))
	require.NoError(t, capblend.RequireNotReceiver(nil, "chamfers"))
}

func TestWitnessPrisms(t *testing.T) {
	pl := capblend.Payload{
		Profile:    momentinput.Profile{Outer: squareLoop(10), Holes: []sectionrecord.LoopRecord{squareLoop(2)}},
		Z0:         1,
		Z1:         9,
		Start:      capblend.Setback{Ds: 2},
		End:        capblend.Setback{Ds: 3},
		StartLoops: map[int]bool{0: true},
		EndLoops:   map[int]bool{1: true},
	}
	got := capblend.WitnessPrisms(pl)
	require.Len(t, got, 2)
	require.Equal(t, 3.0, got[0].Z0)
	require.Equal(t, 9.0, got[0].Z1)
	require.Equal(t, 1.0, got[1].Z0)
	require.Equal(t, 6.0, got[1].Z1)
	require.Equal(t, pl.Profile.Outer, got[0].Profile.Outer)
	require.Equal(t, pl.Profile.Holes[0], got[1].Profile.Outer)
	require.Empty(t, got[1].Profile.Holes)
}
