package prismrecord_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestCapFrameOrientsOutward checks the cap frame sits at height z on the
// payload's axis and that its normal is the cap's outward normal: +z for the
// start cap (flip false) and -z for the end cap (flip true), whether or not
// the placement is a reflection.
//
// Shown to fail: with the reflection test replaced by `false && pp.Reflected()`
// the reflected subtest reads a flipped normal.
func TestCapFrameOrientsOutward(t *testing.T) {
	t.Parallel()
	xy, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	xz, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	mirror, err := r3.Reflection(xz)
	require.NoError(t, err)

	for name, xform := range map[string]r3.Transform{"identity": r3.Identity(), "reflected": mirror} {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			pp := prismrecord.Payload{Frame: xy, Xform: xform}
			require.Equal(t, name == "reflected", pp.Reflected())

			start, err := pp.CapFrame(3, false)
			require.NoError(t, err)
			require.Equal(t, r3.NewVec(0, 0, 3), start.Origin())
			require.Equal(t, r3.NewVec(0, 0, 1), start.N())

			end, err := pp.CapFrame(0, true)
			require.NoError(t, err)
			require.Equal(t, r3.NewVec(0, 0, 0), end.Origin())
			require.Equal(t, r3.NewVec(0, 0, -1), end.N())
		})
	}
}

// TestMassRecordCarriesTheRecord checks every field the mass integrator reads
// reaches the massmoment.PrismRecord under its own name.
//
// Shown to fail: swapping Z0Delta and Z1Delta in MassRecord.
func TestMassRecordCarriesTheRecord(t *testing.T) {
	t.Parallel()
	frame, err := r3.NewFrame(r3.NewVec(1, 2, 3), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	profile := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.LineSeg{
			Start: sectionrecord.Point2{U: 0, V: 0}, End: sectionrecord.Point2{U: 2, V: 0}, TStart: 0, TEnd: 1,
		},
	}}}
	pp := prismrecord.Payload{
		Profile: profile, Frame: frame, Xform: r3.Identity(),
		Z0: 0.5, Z1: 4, Z0Delta: 0.25, Z1Delta: 0.125, SectionDelta: 0.0625,
	}
	require.Equal(t, massmoment.PrismRecord{
		Profile: profile, Frame: frame, Transform: r3.Identity(),
		Z0: 0.5, Z1: 4, SectionDelta: 0.0625, Z0Delta: 0.25, Z1Delta: 0.125,
	}, pp.MassRecord())
}
