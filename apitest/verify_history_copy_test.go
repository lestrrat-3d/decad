package apitest_test

import (
	"fmt"
	"slices"
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
		lift := liftAbout(t, 0.15)
		doc := decad.New()
		on := func(b *decad.Body) *decad.Body {
			out, err := decad.Transform(t.Context(), b, lift)
			require.NoError(t, err)
			return out
		}
		drilled := on(slab(t, doc, 0, -5, 30, 5, 0, 10, r3.Identity()))
		for _, x := range []float64{10, 20} {
			var err error
			drilled, err = decad.Cut(t.Context(), drilled, on(discAt(t, doc, x, 0, -1, 30, 1)))
			require.NoError(t, err)
		}
		stem := on(discAt(t, doc, 10, 0, 2, 20, 1))
		master, err := decad.Union(t.Context(), stem, on(hookRingSweep(t, doc, ringZ, 90)))
		require.NoError(t, err)
		require.True(t, tookMeshPath(master))
		return scene{doc: doc, lift: lift, drilled: drilled, master: master}
	}
	copyOf := func(t *testing.T, body *decad.Body, opt decad.CopyOption) *decad.Body {
		c, err := decad.Copy(t.Context(), body, opt)
		require.NoError(t, err)
		return c
	}
	first, second := r3.Vec{X: 10}, r3.Vec{X: 20}
	cases := []struct {
		name string
		ring float64
		// copy makes the body checked against the drilled block.
		copy func(t *testing.T, sc scene) *decad.Body
		// separated says whether the (drilled, copy) pair is decided clear.
		separated bool
		// masterClear says whether the (drilled, master) pair is decided clear.
		masterClear bool
	}{
		{"exact lift move into the second bore", 20, func(t *testing.T, sc scene) *decad.Body {
			return copyOf(t, sc.master, decad.WithCopyLiftTranslation(sc.lift, first, second))
		}, true, true},
		{"exact move along the top, ring 0.2 above it", 11, func(t *testing.T, sc scene) *decad.Body {
			return copyOf(t, sc.master, decad.WithCopyLiftTranslation(sc.lift, first, second))
		}, true, true},
		{"exact move lifted 0.3 off the top", 11, func(t *testing.T, sc scene) *decad.Body {
			return copyOf(t, sc.master, decad.WithCopyLiftTranslation(sc.lift, first, r3.Vec{X: 20, Z: 0.3}))
		}, true, true},
		{"exact move lowered 0.21, ring 0.01 into the top", 11, func(t *testing.T, sc scene) *decad.Body {
			return copyOf(t, sc.master, decad.WithCopyLiftTranslation(sc.lift, first, r3.Vec{X: 20, Z: -0.21}))
		}, false, true},
		{"exact move lowered 1, ring across the top plane", 11, func(t *testing.T, sc scene) *decad.Body {
			return copyOf(t, sc.master, decad.WithCopyLiftTranslation(sc.lift, first, r3.Vec{X: 20, Z: -1}))
		}, false, true},
		{"float placement of the same move", 11, func(t *testing.T, sc scene) *decad.Body {
			move, err := r3.Translation(sc.lift.Basis().EX.Scale(10))
			require.NoError(t, err)
			return copyOf(t, sc.master, decad.WithCopyPlacement(move))
		}, false, true},
		{"exact move 0.01 off the bore", 20, func(t *testing.T, sc scene) *decad.Body {
			return copyOf(t, sc.master, decad.WithCopyLiftTranslation(sc.lift, first, r3.Vec{X: 20.01}))
		}, false, true},
		{"exact move under another lift", 11, func(t *testing.T, sc scene) *decad.Body {
			return copyOf(t, sc.master, decad.WithCopyLiftTranslation(liftAbout(t, 0.15+1e-9), first, second))
		}, false, true},
		{"exact move that sinks the ring into the block", 9.5, func(t *testing.T, sc scene) *decad.Body {
			return copyOf(t, sc.master, decad.WithCopyLiftTranslation(sc.lift, first, second))
		}, false, false},
		{"copy of a copy", 11, func(t *testing.T, sc scene) *decad.Body {
			away := copyOf(t, sc.master, decad.WithCopyLiftTranslation(sc.lift, first, r3.Vec{X: 20, Y: 40}))
			return copyOf(t, away, decad.WithCopyLiftTranslation(sc.lift, r3.Vec{X: 20, Y: 40}, second))
		}, false, true},
		{"exact move of a refined master", 11, func(t *testing.T, sc scene) *decad.Body {
			refined, err := decad.Refine(t.Context(), sc.master, units.Millimeters(0.05))
			require.NoError(t, err)
			return copyOf(t, refined, decad.WithCopyLiftTranslation(sc.lift, first, second))
		}, true, true},
	}
	// Groups own separate documents; their cases run serially and remove their
	// temporary bodies before the next case uses that document.
	for _, ring := range []float64{20, 11, 9.5} {
		t.Run(fmt.Sprintf("ring=%g", ring), func(t *testing.T) {
			t.Parallel()
			sc := build(t, ring)
			before := sc.doc.Bodies()
			for _, tc := range cases {
				if tc.ring != ring {
					continue
				}
				t.Run(tc.name, func(t *testing.T) {
					live := sc.doc.Bodies()
					require.Len(t, live, len(before), "the previous case left temporary bodies")
					for _, body := range before {
						require.True(t, slices.Contains(live, body), "a fixture body is no longer live")
					}
					t.Cleanup(func() {
						for _, body := range sc.doc.Bodies() {
							if !slices.Contains(before, body) {
								require.NoError(t, decad.Remove(body))
							}
						}
						live := sc.doc.Bodies()
						refined := tc.name == "exact move of a refined master"
						want := len(before)
						if refined {
							want-- // Refine consumes the master; this is the group's last case.
						}
						require.Len(t, live, want, "temporary bodies remain after cleanup")
						for _, body := range before {
							if refined && body == sc.master {
								require.False(t, slices.Contains(live, body), "Refine must retire the master")
								continue
							}
							require.True(t, slices.Contains(live, body), "a fixture body is no longer live")
						}
					})
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
		})
	}
}

// TestVerifyHistoryCopyEndCap moves a set screw along a revolved spindle by
// exact copies. The master is the Union of a Ø2 screw whose seat lies in the
// plane x = 4, tangent to the spindle at y = 20, and a ring far from the
// spindle. The end-cap separator decides the master: within the screw's slab
// across the spindle's axis, the spindle reaches no further than x = 4. An
// exact copy is read as the master moved by its vector, so the screw's cap
// plane and its slab move with it: a copy at y = 30 or 0.01 mm off the
// spindle is decided, and a copy over the collar or 0.01 mm into the spindle
// is not.
//
// Leg shown to fail: reading the slab at the master's place reports the copy
// over the collar separated, and flipping the shift along the cap normal
// reports the copy 0.01 mm into the spindle separated.
func TestVerifyHistoryCopyEndCap(t *testing.T) {
	t.Parallel()
	from := r3.Vec{Y: 20}
	for _, tc := range []struct {
		name      string
		to        r3.Vec
		separated bool
	}{
		{"exact move along the spindle", r3.Vec{Y: 30}, true},
		{"exact move 0.01 into the spindle", r3.Vec{X: -0.01, Y: 30}, false},
		{"exact move over the collar", r3.Vec{Y: 11}, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			spindle := collarSpindle(t, doc)
			master, err := decad.Union(t.Context(), xScrew(t, doc, 4, from.Y, 0), hookRing(t, doc, 20))
			require.NoError(t, err)
			copied, err := decad.Copy(t.Context(), master, decad.WithCopyLiftTranslation(r3.Identity(), from, tc.to))
			require.NoError(t, err)
			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			masterInterferences, masterDiagnostics := pairFindings(report, spindle, master)
			require.Zero(t, masterInterferences)
			require.Zero(t, masterDiagnostics, "the screw's cap plane decides the master")
			interferences, diagnostics := pairFindings(report, spindle, copied)
			if tc.separated {
				require.Zero(t, interferences)
				require.Zero(t, diagnostics, "the copy is the master moved along the spindle")
				return
			}
			require.Positive(t, interferences+diagnostics, "the copy must be reported or left undecided")
		})
	}
}
