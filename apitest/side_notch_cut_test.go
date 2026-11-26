package apitest_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestSideNotchCutKeepsEarlierResults pins the routes general-boolean §3 A9
// runs after. A notch across a plate's side, across its corner or round across
// its side, and a pocket across one of two holes, keep the mesh result: the mesh
// path builds them, and A9 runs only once it has refused. A free pocket in a
// one-hole plate keeps the stacked blind cut, which A7 does not take: A7's cut
// slab still needs one outer and one hole. Each volume holds its closed form.
//
// The closed forms. A 4x4 notch, x 6..14, y -2..2 from z 6, removes 4x4x4 = 64
// mm^3 of 4000; the corner notch x 6..14, y 6..14 removes the same 64; a disc
// r=3 about (10, 0) from z 6 removes half its area over 4, 18*pi. The two-hole
// plate loses 2x(4x4x10) = 320 to its holes and the pocket x 2..6, y -1..1
// from z 6 the 2x2 beside the right hole over 4, 16 more. The one-hole plate
// loses 160 and the pocket x 2..6, y 4..6 from z 6 another 4x2x4 = 32.
func TestSideNotchCutKeepsEarlierResults(t *testing.T) {
	t.Parallel()
	plate := func(t *testing.T, d *decad.Document) *decad.Body { return boxBody(t, d, -10, -10, 10, 10, 10) }
	holed := func(t *testing.T, d *decad.Document, both bool) *decad.Body {
		part, err := decad.Cut(plate(t, d), boxBodyAtZ(t, d, -8, -2, -4, 2, 0, 10)).Do(t.Context())
		require.NoError(t, err)
		if !both {
			return part
		}
		part, err = decad.Cut(part, boxBodyAtZ(t, d, 4, -2, 8, 2, 0, 10)).Do(t.Context())
		require.NoError(t, err)
		return part
	}
	for _, tc := range []struct {
		name         string
		target, tool func(*testing.T, *decad.Document) *decad.Body
		k, p         string
		// mesh is a mesh result; otherwise the stacked blind cut's payload.
		mesh bool
	}{
		{name: "rect-across-side", target: plate, k: "3936", p: "0", mesh: true,
			tool: func(t *testing.T, d *decad.Document) *decad.Body { return boxBodyAtZ(t, d, 6, -2, 14, 2, 6, 6) }},
		{name: "rect-across-corner", target: plate, k: "3936", p: "0", mesh: true,
			tool: func(t *testing.T, d *decad.Document) *decad.Body { return boxBodyAtZ(t, d, 6, 6, 14, 14, 6, 6) }},
		{name: "round-across-side", target: plate, k: "4000", p: "-18", mesh: true,
			tool: func(t *testing.T, d *decad.Document) *decad.Body { return circleBodyAtZ(t, d, 10, 3, 6, 6) }},
		{name: "pocket-across-one-of-two-holes", k: "3664", p: "0", mesh: true,
			target: func(t *testing.T, d *decad.Document) *decad.Body { return holed(t, d, true) },
			tool:   func(t *testing.T, d *decad.Document) *decad.Body { return boxBodyAtZ(t, d, 2, -1, 6, 1, 6, 6) }},
		{name: "free-pocket-in-one-hole-plate", k: "3808", p: "0",
			target: func(t *testing.T, d *decad.Document) *decad.Body { return holed(t, d, false) },
			tool:   func(t *testing.T, d *decad.Document) *decad.Body { return boxBodyAtZ(t, d, 2, 4, 6, 6, 6, 6) }},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			part, err := decad.Cut(tc.target(t, doc), tc.tool(t, doc)).Do(t.Context())
			require.NoError(t, err)
			if tc.mesh {
				require.True(t, tookMeshPath(part), "class %s", decadtest.PayloadClass(part))
			} else {
				require.Equal(t, "decad.stackedPrismPayload", decadtest.PayloadClass(part))
			}
			volume, err := part.Volume()
			require.NoError(t, err)
			gap := new(big.Float).SetPrec(volumeRefPrec).SetFloat64(volumeMM(t, volume))
			gap.Sub(gap, arcVolume(t, tc.k, tc.p, "0")).Abs(gap)
			require.LessOrEqual(t, gap.Cmp(new(big.Float).SetFloat64(boundMM3(t, volume))), 0,
				"volume %v misses the closed form by %s", volumeMM(t, volume), gap.Text('g', 6))
		})
	}
}

// forkOptions vary leverForkSketch's fork and notch.
type forkOptions struct {
	// slots adds that many holes of four oblique lines below the fork end, up
	// to len(forkSlots).
	slots int
	// wideWall draws the notch's walls up and out to (±2.2, 44), so each wall
	// crosses its corner round instead of leaving into the void.
	wideWall bool
}

// forkSlots are the slot holes forkOptions.slots adds, counter-clockwise; each
// has area 4 by the shoelace formula.
var forkSlots = [2][4][2]float64{
	{{-1, 31}, {1.5, 31.5}, {1, 33}, {-1.5, 32.5}},
	{{-1, 34}, {1.5, 34.5}, {1, 36}, {-1.5, 35.5}},
}

// leverForkSketch draws either the fork end of a watch lever (notch false) or
// the notch cut into it (notch true). The fork's gap round G and the notch
// semicircle B both end at a = (r, ky), where G touches B from inside and the
// corner round K touches B from outside; the notch's walls start there too.
// The values are the lever's own solved floats.
func leverForkSketch(notch bool, opts forkOptions) func(*sketch.Sketch) {
	const (
		ky  = 38.874650636459869
		r   = 1.6676711944252536
		gc  = 1.1338355972126268
		dr  = 0.59999999999999998
		tip = 42.149999999999999
		kc  = 2.3338355972126268
	)
	return func(s *sketch.Sketch) {
		p := func(x, y float64) *sketch.Point {
			pt := s.CreatePoint(x, y)
			s.Fix(pt)
			return pt
		}
		if notch {
			top := 1.0
			if opts.wideWall {
				top = 2.2
			}
			b2, a2 := p(-r, ky), p(r, ky)
			u1, u2 := p(top, 44), p(-top, 44)
			s.CreateArc(p(0, ky), b2, a2)
			s.CreateLine(a2, u1)
			s.CreateLine(u1, u2)
			s.CreateLine(u2, b2)
			return
		}
		a0, a1, a2 := p(3, 30), p(3, ky), p(r, ky)
		b0, b1, b2 := p(-3, 30), p(-3, ky), p(-r, ky)
		d1, d2, d3, d4 := p(dr, ky), p(dr, tip), p(-dr, tip), p(-dr, ky)
		s.CreateLine(a0, a1)
		s.CreateArc(p(kc, ky), a1, a2)
		s.CreateArc(p(gc, ky), d1, a2)
		s.CreateLine(d1, d2)
		s.CreateArc(p(0, tip), d2, d3)
		s.CreateLine(d3, d4)
		s.CreateArc(p(-gc, ky), b2, d4)
		s.CreateArc(p(-kc, ky), b2, b1)
		s.CreateLine(b1, b0)
		s.CreateLine(b0, a0)
		for _, slot := range forkSlots[:opts.slots] {
			var c [4]*sketch.Point
			for i, v := range slot {
				c[i] = p(v[0], v[1])
			}
			for i := range c {
				s.CreateLine(c[i], c[(i+1)%4])
			}
		}
	}
}

// forkBody extrudes, from z by depth, the profile of draw that has the given
// number of holes.
func forkBody(t *testing.T, doc *decad.Document, z float64, depth units.Value, holes int,
	draw func(*sketch.Sketch)) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), z)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	draw(s)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var profile *sketch.Profile
	for _, p := range s.Profiles() {
		if p.Valid && len(p.Holes) == holes {
			require.Nil(t, profile, "one region has %d holes", holes)
			profile = p
		}
	}
	require.NotNil(t, profile)
	body, err := decad.Extrude(doc, s, profile, decad.Distance{D: depth, Dir: decad.Along}).Do(t.Context())
	require.NoError(t, err)
	return body
}

// TestSideNotchCutTangentFork cuts the swisslever lever's notch into its fork
// through general-boolean §3 A9, with no, one and two oblique slot holes. The
// notch semicircle meets the fork outline only at the recorded vertices a and
// a', where it is tangent to the corner round from outside and the gap round
// from inside; the mesh path refuses that contact, and A9 builds it. The result
// is a BRep whose one reflex floor arc is the edge Reflex selects. Its volume is
// the fork's area over the full height less the notch's overlap with it over the
// notched band, less the slot over the full height; the fork and overlap areas
// are read from sketch's arrangement of the same records.
//
// Shown red: without A9's fallback in booleandispatch both cases refuse with
// the mesh path's contact refusal.
func TestSideNotchCutTangentFork(t *testing.T) {
	t.Parallel()
	mm := units.Millimeters
	// The fork's area and its overlap with the notch, from one scene of both.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	leverForkSketch(false, forkOptions{})(s)
	leverForkSketch(true, forkOptions{})(s)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 3)
	var forkLessNotch, overlap float64
	for _, p := range profiles {
		switch {
		case p.Area > 40:
			forkLessNotch = p.Area
		case p.Area < 9:
			overlap = p.Area
		}
	}
	require.InDelta(t, 50.273480, forkLessNotch, 1e-5)
	require.InDelta(t, 7.969198, overlap, 1e-5)

	for _, tc := range []struct {
		name  string
		slots int
	}{
		{name: "plain"},
		{name: "one-slot", slots: 1},
		{name: "two-slots", slots: 2},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			fork := forkBody(t, doc, 0, mm(6.5), tc.slots, leverForkSketch(false, forkOptions{slots: tc.slots}))
			notch := forkBody(t, doc, 2.5, mm(5), 0, leverForkSketch(true, forkOptions{}))
			part, err := decad.Cut(fork, notch).Do(t.Context())
			require.NoError(t, err)
			require.Equal(t, "decad.brepPayload", decadtest.PayloadClass(part))

			want := (forkLessNotch+overlap)*2.5 + forkLessNotch*4
			want -= float64(4*tc.slots) * 6.5
			volume, err := part.Volume()
			require.NoError(t, err)
			require.InDelta(t, want, volumeMM(t, volume), 1e-9+boundMM3(t, volume))

			_, err = part.Tessellate(t.Context(), mm(0.05), decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)

			edges, err := decad.Edges(decad.Reflex(), decad.Circular()).SelectEdges(part)
			require.NoError(t, err)
			require.Len(t, edges, 1)
		})
	}
}

// TestSideNotchCutDeclinesObliqueCut keeps the mesh path's refusal for a fork
// notch whose oblique walls cross the corner rounds: A9 states no cut vertex on
// an oblique line, so the pair is left to the refusal the mesh path gave (the
// tangent contact at a and a' stays).
//
// Shown red: without the oblique-fragment miss in stackedunion's scene the
// pair builds a BRep.
func TestSideNotchCutDeclinesObliqueCut(t *testing.T) {
	t.Parallel()
	mm := units.Millimeters
	doc := decad.New()
	fork := forkBody(t, doc, 0, mm(6.5), 0, leverForkSketch(false, forkOptions{}))
	notch := forkBody(t, doc, 2.5, mm(5), 0, leverForkSketch(true, forkOptions{wideWall: true}))
	part, err := decad.Cut(fork, notch).Do(t.Context())
	if err == nil {
		require.True(t, tookMeshPath(part), "class %s", decadtest.PayloadClass(part))
		return
	}
	require.ErrorIs(t, err, decad.ErrUnsupported)
}
