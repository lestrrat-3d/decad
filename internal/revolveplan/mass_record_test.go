package revolveplan_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestMassRecord(t *testing.T) {
	t.Parallel()

	frame, err := r3.NewFrame(r3.Vec{X: 1}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	move, err := r3.Translation(r3.Vec{Z: 4})
	require.NoError(t, err)
	profile := momentinput.Profile{Outer: sectionrecord.LoopRecord{
		Segments: []sectionrecord.CurveSegment{sectionrecord.LineSeg{TEnd: 1}}}}
	rp := revolveplan.Payload{
		Den:     revolveangle.Sweep{Phi0: revolveangle.Zero(), Phi1: revolveangle.Angle{Rad: big.NewRat(1, 2), Turn: new(big.Rat)}},
		Profile: profile, Frame: frame, Xform: move, Phi0: 0.25, Phi1: 1.5, SectionDelta: 0.125,
		Ax: revolveaxis.Frame{AU: 1, AV: 2, DU: 3, DV: 4, AUBound: 5, AVBound: 6, DUBound: 7, DVBound: 8,
			RadialAdmitAllow: 9},
	}

	got := rp.MassRecord()
	require.Equal(t, profile, got.Profile)
	require.Equal(t, frame, got.Frame)
	require.Equal(t, move, got.Transform)
	require.Equal(t, rp.Den, got.Den)
	require.NotZero(t, got.Den.Phi1)
	require.Equal(t, [2]float64{0.25, 1.5}, [2]float64{got.Phi0, got.Phi1})
	require.Equal(t, [4]float64{1, 2, 3, 4}, [4]float64{got.AU, got.AV, got.DU, got.DV})
	require.Equal(t, [4]float64{5, 6, 7, 8}, [4]float64{got.AUBound, got.AVBound, got.DUBound, got.DVBound})
	require.Equal(t, 0.125, got.SectionDelta)
	require.Equal(t, 9.0, got.RadialAdmitAllow)
	require.False(t, got.AxisSnap, "no snap charge")

	for name, snap := range map[string]revolveaxis.SnapAllow{
		"area": {Area: 1}, "first": {First: 1}, "mixed": {Mixed: 1}, "second": {Second: 1},
	} {
		rp.Ax.Snap = snap
		require.True(t, rp.MassRecord().AxisSnap, name)
	}
}
