package decad

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file covers docs/modify-reach-design.md §9.3.1's displacement: a revolve
// shell publishes its wall's section displacement only where the offset's
// float cuts moved a coordinate, so a right-angle or exact-level shell keeps
// a zero sectionDelta — the payload every reading takes bit for bit as before
// — while a slanted cut publishes a positive whole-section one.
//
// Legs shown to fail before this fixture was accepted: publishing every shell
// with sectionWhole set regardless of its displacement sends the exact rows'
// whole flag red, and dropping the wall survey's displacement gate sends the
// cone rows' undecided wall red.

func shellMeridianBody(t *testing.T, pts [][2]float64, extent AngularExtent) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	ps := make([]*sketch.Point, len(pts))
	for i, p := range pts {
		ps[i] = s.CreatePoint(p[0], p[1])
		s.Fix(ps[i])
	}
	for i := range ps {
		s.CreateLine(ps[i], ps[(i+1)%len(ps)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := New().Revolve(s, s.Profiles()[0], revolveAxisU, extent)
	require.NoError(t, err)
	return body
}

func TestRevolveShellSectionDelta(t *testing.T) {
	t.Parallel()
	halfTurn := AngleExtent{A: units.Degrees(180), Dir: Along}
	caps := Faces(NormalTo(r3.NewVec(0, 0, 1))).Exactly(2)
	cylinder := [][2]float64{{0, 0}, {20, 0}, {20, 10}, {0, 10}}
	ring := [][2]float64{{0, 5}, {20, 5}, {20, 10}, {0, 10}}
	cone := [][2]float64{{0, 0}, {20, 0}, {0, 10}}
	for _, tc := range []struct {
		name      string
		pts       [][2]float64
		opts      []ShellOption
		displaced bool
	}{
		{"cylinder inward", cylinder, nil, false},
		{"cylinder outward", cylinder, []ShellOption{WithShellSense(Outward)}, false},
		{"ring inward", ring, nil, false},
		{"ring outward", ring, []ShellOption{WithShellSense(Outward)}, false},
		{"cone inward", cone, nil, true},
		{"cone outward", cone, []ShellOption{WithShellSense(Outward)}, true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			body := shellMeridianBody(t, tc.pts, halfTurn)
			shelled, err := body.Shell(t.Context(), caps, units.Millimeters(1), tc.opts...)
			require.NoError(t, err)
			rp := shelled.payload.(revolvePayload)
			if !tc.displaced {
				require.Zero(t, rp.sectionDelta, `an exact offset publishes no displacement`)
				require.False(t, rp.sectionWhole)
				return
			}
			require.Positive(t, rp.sectionDelta)
			require.True(t, rp.sectionWhole)
			// The wall survey reads the recorded meridian, so it stays
			// undecided over a displaced one, as a displaced prism's does.
			report, err := shelled.doc.Verify(t.Context(), WithMinWallThickness(units.Millimeters(0.5)))
			require.NoError(t, err)
			require.Equal(t, ScalarUndecided, report.Bodies[0].Wall.Outcome)
			require.Equal(t, Suspect, report.Status)
			// A displaced meridian has no proven rewrite, so a second shell
			// refuses rather than offsetting the recorded one.
			_, err = shelled.Shell(t.Context(), Faces(NormalTo(r3.NewVec(0, 0, 1))).Exactly(2), units.Millimeters(0.1))
			require.ErrorIs(t, err, ErrUnsupported)
			require.Contains(t, err.Error(), `section displacement`)
		})
	}
}

func TestRevolveShellDisplacedUndercutUndecided(t *testing.T) {
	t.Parallel()
	// The meridian (0, 0), (30, 0), (20, 10), (0, 10) shelled 0.3 mm inward
	// under WithNoOpenings: the cavity's base is recorded from a rounded miter,
	// (0.30000000000000071, 9.7) to (0.3, 0), tilted about 7e-16 off the
	// plane z = 0.3 it denotes. Read as exact, that tilt lists the planar
	// cavity base as an undercut against a pull along the sketch's v axis. The
	// undercut survey reads the recorded meridian's tangents, so over a
	// displaced one it is undecided, as the wall survey is. Leg shown to fail
	// before this fixture was accepted: dropping the survey's displacement
	// gate lists the cavity base.
	for _, tc := range []struct {
		name   string
		extent AngularExtent
		opts   []ShellOption
		faces  *FaceQuery
		pull   r3.Vec
	}{
		{"full turn", FullRevolution{}, []ShellOption{WithNoOpenings()}, nil, r3.NewVec(0, 1, 0)},
		{"half turn", AngleExtent{A: units.Degrees(180), Dir: Along}, nil, Faces(NormalTo(r3.NewVec(0, 0, 1))).Exactly(2), r3.NewVec(0, 0, 1)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			body := shellMeridianBody(t, [][2]float64{{0, 0}, {30, 0}, {20, 10}, {0, 10}}, tc.extent)
			shelled, err := body.Shell(t.Context(), tc.faces, units.Millimeters(0.3), tc.opts...)
			require.NoError(t, err)
			require.Positive(t, shelled.payload.(revolvePayload).sectionDelta)
			report, err := shelled.doc.Verify(t.Context(), WithPullDirection(tc.pull))
			require.NoError(t, err)
			require.Equal(t, CoverageUndecided, report.Bodies[0].Undercut.Coverage)
			require.Empty(t, report.Bodies[0].Undercut.Faces, `no face of a displaced meridian is listed as an undercut`)
			require.Equal(t, Suspect, report.Status)
		})
	}
}
