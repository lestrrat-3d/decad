package brepbandmesh

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestFullBandContourAtStationLimit(t *testing.T) {
	ctx := t.Context()
	circle := sectionrecord.CircleSeg{Radius: units.Millimeters(1), CCW: true, TEnd: 1}
	band := breprecord.LoopBand{Kind: breprecord.BandFillet, Orig: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{circle}}}
	body := breprecord.Payload{LoopBands: []breprecord.LoopBand{band}}
	view := capblend.Payload{Fillet: true, Z1: 1, StartLoops: map[int]bool{0: true}, Start: capblend.Setback{Dc: .1, Ds: .1}}
	chord := tessellation.ChordSagitta(1, 2*math.Pi, freeform.MaxChordsPerWalk-1)
	mesh, err := chordFullBand(ctx, proofbound.NewWorkBudget(ctx), freeform.NewFreeformWork(), body, band, view, chord)
	require.NoError(t, err)
	require.Len(t, mesh.SideSag, 1)
	require.Greater(t, mesh.SideSag[0], chord/2)
	require.LessOrEqual(t, mesh.SideSag[0], chord)
	_, err = chordFullBand(ctx, proofbound.NewWorkBudget(ctx), freeform.NewFreeformWork(), body, band, view, chord/2)
	require.ErrorIs(t, err, freeform.ErrTooManyChords)
}
