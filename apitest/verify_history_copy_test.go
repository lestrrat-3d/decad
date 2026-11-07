package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// discAt extrudes a disc of radius r about (cx, cy) on the XY plane offset to
// z0, by length. Unlike coaxialSection followed by placedAlongZ, the level
// lives in the sketch plane, so the body's placement is left to the caller:
// two discs moved by one lift share that lift bit for bit.
func discAt(t *testing.T, doc *decad.Document, cx, cy, z0, length, r float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), z0)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	c := s.CreatePoint(cx, cy)
	s.Fix(c)
	s.CreateCircle(c, r)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(length), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// TestVerifyHistoryCopyLiftTranslation seats the Ø2 stem of a hook, copied by
// a lift-local move, in the second of two Ø2 bores of a block. Block, bore
// tools, stem and ring all sit on one inexact lift (a rotation about a point
// off the origin), so the stem and the bore tool share that lift bit for bit.
//
// The master hook is the Union of a stem in the first bore and a ring high above the
// block, clear of the block's box even moved to the second bore. The master pair is decided by the history certificate: the stem lies
// in the tool that drilled the bore. A copy is a history leaf unless it was
// made by WithCopyLiftTranslation directly from the master: Verify then reads
// it as the master moved by the exact vector t = lift·(to − from), and the stem
// again lies exactly in a tool, now the second bore's.
//
// A float placement of the same move is rounded away from the exact seat and is
// not decided. Neither is an exact move that really leaves the bore (0.01 mm
// off, or by a lift that differs by 1e-9 rad from the one the bores were
// sketched on), a ring sunk into the block, or a copy of a copy, which records
// no translation. A Refine between master and copy changes nothing.
//
// Leg shown to fail: dropping the copy step in historyWalk.resolve leaves the
// exact move with a pair diagnostic, and flipping the sign of the shift
// reports the exact move as undecided too.
func TestVerifyHistoryCopyLiftTranslation(t *testing.T) {
	t.Parallel()
	liftAbout := func(t *testing.T, angle float64) r3.Transform {
		lift, err := r3.RotationAround(r3.Vec{X: 3, Y: -2, Z: 1}, r3.Vec{X: 1, Y: 2, Z: 3}, units.Radians(angle))
		require.NoError(t, err)
		return lift
	}
	// scene builds the block with two bores on lift, and the master hook with
	// its ring at local height ringZ; the master's stem is in the first bore.
	type scene struct {
		doc     *decad.Document
		lift    r3.Transform
		drilled *decad.Body
		master  *decad.Body
	}
	build := func(t *testing.T, ringZ float64) scene {
		lift := liftAbout(t, 0.4)
		doc := decad.New()
		on := func(b *decad.Body) *decad.Body {
			out, err := decad.Transform(t.Context(), b, lift)
			require.NoError(t, err)
			return out
		}
		drilled := on(slab(t, doc, 0, -5, 30, 5, 0, 10, r3.Identity()))
		for _, x := range []float64{10, 20} {
			var err error
			drilled, err = decad.Cut(t.Context(), drilled, on(discAt(t, doc, x, 0, -1, 60, 1)))
			require.NoError(t, err)
		}
		stem := on(discAt(t, doc, 10, 0, 2, 40, 1))
		master, err := decad.Union(t.Context(), stem, on(hookRing(t, doc, ringZ)))
		require.NoError(t, err)
		return scene{doc: doc, lift: lift, drilled: drilled, master: master}
	}
	copyOf := func(t *testing.T, body *decad.Body, opt decad.CopyOption) *decad.Body {
		c, err := decad.Copy(t.Context(), body, opt)
		require.NoError(t, err)
		return c
	}
	first, second := r3.Vec{X: 10}, r3.Vec{X: 20}
	for _, tc := range []struct {
		name string
		ring float64
		// copy makes the body checked against the drilled block.
		copy func(t *testing.T, sc scene) *decad.Body
		// separated says whether the (drilled, copy) pair is decided clear.
		separated bool
		// masterClear says whether the (drilled, master) pair is decided clear.
		masterClear bool
	}{
		{"exact lift move into the second bore", 40, func(t *testing.T, sc scene) *decad.Body {
			return copyOf(t, sc.master, decad.WithCopyLiftTranslation(sc.lift, first, second))
		}, true, true},
		{"float placement of the same move", 40, func(t *testing.T, sc scene) *decad.Body {
			move, err := r3.Translation(sc.lift.Basis().EX.Scale(10))
			require.NoError(t, err)
			return copyOf(t, sc.master, decad.WithCopyPlacement(move))
		}, false, true},
		{"exact move 0.01 off the bore", 40, func(t *testing.T, sc scene) *decad.Body {
			return copyOf(t, sc.master, decad.WithCopyLiftTranslation(sc.lift, first, r3.Vec{X: 20.01}))
		}, false, true},
		{"exact move under another lift", 40, func(t *testing.T, sc scene) *decad.Body {
			return copyOf(t, sc.master, decad.WithCopyLiftTranslation(liftAbout(t, 0.4+1e-9), first, second))
		}, false, true},
		{"exact move that sinks the ring into the block", 9.5, func(t *testing.T, sc scene) *decad.Body {
			return copyOf(t, sc.master, decad.WithCopyLiftTranslation(sc.lift, first, second))
		}, false, false},
		{"copy of a copy", 40, func(t *testing.T, sc scene) *decad.Body {
			away := copyOf(t, sc.master, decad.WithCopyLiftTranslation(sc.lift, first, r3.Vec{X: 20, Y: 40}))
			return copyOf(t, away, decad.WithCopyLiftTranslation(sc.lift, r3.Vec{X: 20, Y: 40}, second))
		}, false, true},
		{"exact move of a refined master", 40, func(t *testing.T, sc scene) *decad.Body {
			refined, err := decad.Refine(t.Context(), sc.master, units.Millimeters(0.05))
			require.NoError(t, err)
			return copyOf(t, refined, decad.WithCopyLiftTranslation(sc.lift, first, second))
		}, true, true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			sc := build(t, tc.ring)
			copied := tc.copy(t, sc)
			report, err := sc.doc.Verify(t.Context())
			require.NoError(t, err)
			interferences, diagnostics := pairFindings(report, sc.drilled, copied)
			if tc.separated {
				require.Zero(t, interferences)
				require.Zero(t, diagnostics, "the copy is its master moved into the second bore")
			} else {
				require.Positive(t, interferences+diagnostics, "the copy must be reported or left undecided")
			}
			masterInterferences, masterDiagnostics := pairFindings(report, sc.drilled, sc.master)
			if tc.masterClear {
				require.Zero(t, masterInterferences)
				require.Zero(t, masterDiagnostics, "the master is decided by its own history")
				return
			}
			require.Positive(t, masterInterferences+masterDiagnostics)
		})
	}
}
