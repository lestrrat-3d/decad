package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// beadBody revolves a half disk off the axis a full turn: the chord ρ = 6
// from z = 0 to z = 10 under the arc of radius 5 about (5, 6).
func beadBody(t *testing.T) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o, end, c := s.CreatePoint(0, 6), s.CreatePoint(10, 6), s.CreatePoint(5, 6)
	for _, p := range []*sketch.Point{o, end, c} {
		s.Fix(p)
	}
	s.CreateLine(o, end)
	s.CreateArc(c, end, o)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := New().Revolve(s, s.Profiles()[0], revolveAxisU, FullRevolution{})
	require.NoError(t, err)
	return body
}

// TestRevolveShellOpeningSectionDelta covers docs/shell-opening-design.md
// §8's charge: a side opening's rim cut is a float solve wherever the removed
// carrier is not one of §2.4's exact pairs, and the wall publishes its
// enclosed reach, with every other float cut's, as the revolve's
// whole-section displacement. A cut whose enclosure is the held float — a
// right-angle rim on axis-aligned walks, or the slanted cone's (4, 6.5) —
// publishes none.
//
// Legs shown to fail before this fixture was accepted: charging an opening
// end nothing leaves the bead's two √24 cuts uncharged, so both bead rows
// publish zero; and enclosing an opening end as the kept walk's offset foot,
// instead of the rim cut, measures the slanted cone's exact cut against the
// wrong point and publishes a 6 mm displacement.
func TestRevolveShellOpeningSectionDelta(t *testing.T) {
	t.Parallel()
	ring := [][2]float64{{0, 5}, {20, 5}, {20, 10}, {0, 10}}
	cylinder := [][2]float64{{0, 0}, {20, 0}, {20, 10}, {0, 10}}
	slanted := [][2]float64{{0, 0}, {6, 0}, {6, 5}, {2, 8}, {0, 8}}
	cone := [][2]float64{{0, 0}, {20, 0}, {0, 10}}
	cylinderAt := func(r float64) func(*Face) bool {
		return func(f *Face) bool {
			c, ok := f.Surface().(Cylinder)
			return ok && c.Radius.Mag() == r
		}
	}
	planeAt := func(u float64) func(*Face) bool {
		return func(f *Face) bool {
			p, ok := f.Surface().(Plane)
			return ok && math.Abs(p.Frame.N().X) > 0.5 && p.Frame.Origin().X == u
		}
	}
	isCone := func(f *Face) bool { _, ok := f.Surface().(Cone); return ok }
	isTorus := func(f *Face) bool { _, ok := f.Surface().(Torus); return ok }
	for _, tc := range []struct {
		name      string
		pts       [][2]float64
		remove    []func(*Face) bool
		t         float64
		outward   bool
		displaced bool
	}{
		{"ring without its outer skin", ring, []func(*Face) bool{cylinderAt(10)}, 1, false, false},
		{"cylinder without one end", cylinder, []func(*Face) bool{planeAt(20)}, 2, false, false},
		{"slanted, top disc removed", slanted, []func(*Face) bool{planeAt(6)}, 1, false, true},
		{"slanted, cone and top removed", slanted, []func(*Face) bool{isCone, planeAt(6)}, 1.5, false, false},
		{"cone without its base", cone, []func(*Face) bool{planeAt(0)}, 1, false, true},
		{"bead without its arc", nil, []func(*Face) bool{isTorus}, 1, false, true},
		{"bead without its arc, outward", nil, []func(*Face) bool{isTorus}, 1, true, true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			var body *Body
			if tc.pts == nil {
				body = beadBody(t)
			} else {
				body = shellMeridianBody(t, tc.pts, FullRevolution{})
			}
			var refs []FacePredicate
			for _, pick := range tc.remove {
				var hit *Face
				for _, f := range body.Faces() {
					if pick(f) {
						require.Nil(t, hit)
						hit = f
					}
				}
				require.NotNil(t, hit)
				refs = append(refs, FaceCreatedBy(hit.Origins()[0]))
			}
			sel := Faces(refs[0])
			for _, r := range refs[1:] {
				sel = sel.Or(r)
			}
			var opts []ShellOption
			if tc.outward {
				opts = append(opts, WithShellSense(Outward))
			}
			shelled, err := body.Shell(t.Context(), sel, units.Millimeters(tc.t), opts...)
			require.NoError(t, err)
			rp := shelled.payload.(revolvePayload)
			if !tc.displaced {
				require.Zero(t, rp.sectionDelta, `an exact rim publishes no displacement`)
				require.False(t, rp.sectionWhole)
				return
			}
			require.Positive(t, rp.sectionDelta)
			require.True(t, rp.sectionWhole)
		})
	}
}
