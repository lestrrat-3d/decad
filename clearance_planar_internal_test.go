package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These tests pin the three proof legs of planarPairVerdict's displacement
// band (clearance_planar.go) on fixtures where each leg decides the answer.
// Each leg has been deleted once and watched fail:
//
//   - the delta subtraction in planarDisjointResult: with it gone, the
//     shallow-gap case reads pairDisjoint;
//   - the PlanarDeepVertex gate on a held overlap: with it gone, the
//     shallow-overlap case reads pairOverlapping;
//   - the zero-delta gate on a held touch: with it gone, the displaced
//     touching case reads pairTouching.
//
// The branch stands a million millimetres from the origin, so its held
// vertices round by about 1e-10 mm (mitredSweepPayload.delta) while the
// part's arm, placed by exact dyadic entries, has a held gap set to the ulp
// of a 40 mm height, about 7e-15 mm: the two scales are far enough apart to
// place a held gap, or a held overlap, well inside the band.

// planarPartAndBranch builds mtilt's overhanging part and a mitred branch
// whose vertical tip ends at height top, both shifted by shift, and returns
// them with the branch's own held displacement.
func planarPartAndBranch(t *testing.T, doc *Document, shift r3.Vec, top float64) (*Body, *Body, float64) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	corners := [][2]float64{{0, 0}, {40, 0}, {40, 10}, {10, 10}, {10, 40}, {40, 40}, {40, 50}, {0, 50}}
	pts := make([]*sketch.Point, len(corners))
	for i, c := range corners {
		pts[i] = s.CreatePoint(c[0], c[1])
		s.Fix(pts[i])
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	part, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(20), Dir: Along})
	require.NoError(t, err)
	turn, err := r3.FromBasis(r3.Basis{EX: r3.NewVec(1, 0, 0), EY: r3.NewVec(0, 0, 1), EZ: r3.NewVec(0, -1, 0)},
		r3.NewVec(0, 20, 0).Add(shift))
	require.NoError(t, err)
	part, err = part.Placed(t.Context(), turn)
	require.NoError(t, err)

	ps, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	poly, err := ps.CreatePolygon(0, 0, 16, 2)
	require.NoError(t, err)
	ps.Fix(poly.Center)
	for _, v := range poly.Vertices {
		ps.Fix(v)
	}
	_, err = ps.Solve(t.Context())
	require.NoError(t, err)
	lean := math.Tan(30 * math.Pi / 180)
	path := []r3.Vec{
		r3.NewVec(46, 10, 0), r3.NewVec(46, 10, 14),
		r3.NewVec(46-12*lean, 10, 26), r3.NewVec(46-12*lean, 10, top-2), r3.NewVec(46-12*lean, 10, top),
	}
	radii := []float64{2.0, 1.8, 1.4, 1.0, 0.5}
	segments := make([]PathSegment, 0, len(path)-1)
	factors := make([]units.Value, 0, len(path)-1)
	for i := 1; i < len(path); i++ {
		segments = append(segments, LineTo{End: path[i].Sub(path[0])})
		factors = append(factors, units.Scalar(radii[i]/radii[0]))
	}
	spatial, err := NewPath(r3.Vec{}, segments...)
	require.NoError(t, err)
	branch, err := doc.Sweep(t.Context(), ps, ps.Profiles()[0], spatial, WithMitredJoins(), WithSectionScale(factors...))
	require.NoError(t, err)
	move, err := r3.Translation(path[0].Add(shift))
	require.NoError(t, err)
	branch, err = branch.Placed(t.Context(), move)
	require.NoError(t, err)
	payload, ok := branch.payload.(mitredSweepPayload)
	require.True(t, ok)
	return part, branch, payload.delta
}

func TestPlanarPairVerdictDisplacementBand(t *testing.T) {
	t.Parallel()
	far := r3.NewVec(1e6, 0, 0)
	// Read the branch's displacement once off a reference build: every
	// fixture below moves only the tip's height, which does not change the
	// rounding of the far coordinates that set it.
	refDoc := New()
	_, _, delta := planarPartAndBranch(t, refDoc, far, 39)
	require.Greater(t, delta, 1e-11, "the far placement rounds the held vertices well past a 40 mm ulp")
	require.Less(t, delta, 1e-8)
	ulp := math.Nextafter(40, math.Inf(1)) - 40
	require.Less(t, 100*ulp, delta, "the held gap can be set far inside the band")

	cases := []struct {
		name    string
		top     float64
		verdict pairVerdict
	}{
		{"clear gap, ten times the band", 40 - 10*delta, pairDisjoint},
		{"shallow gap, inside the band", 40 - delta/4, pairUndecided},
		{"shallow overlap, inside the band", 40 + delta/4, pairUndecided},
		{"deep overlap", 41, pairOverlapping},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := New()
			part, branch, got := planarPartAndBranch(t, doc, far, tc.top)
			require.InDelta(t, delta, got, delta/10, "the tip height does not move the displacement")
			res, ok, err := planarPairVerdict(t.Context(), part, branch)
			require.NoError(t, err)
			require.True(t, ok)
			require.Equal(t, tc.verdict, res.verdict)
			if tc.verdict != pairDisjoint {
				require.Zero(t, res.lo)
				require.Zero(t, res.hi)
				return
			}
			// The true gap is 40 − top up to the branch's own rounding, and
			// the proven interval holds it while staying clear of zero.
			want := 40 - tc.top
			require.Positive(t, res.lo)
			require.LessOrEqual(t, res.lo, want)
			require.GreaterOrEqual(t, res.hi, want)
			require.LessOrEqual(t, res.hi-res.lo, 4*got, "the interval is widened by the summed displacement and the root enclosure alone")
			require.False(t, res.exact)
			require.Positive(t, res.diam)
		})
	}
}

// TestPlanarPairVerdictTouchNeedsZeroDisplacement pins the touching gate: a
// held touch between a displaced branch and the part is undecided, since
// the held boundary may sit on either side of the true one, while an exact
// faceted body (a zero-bound Union of two boxes) flush against an exact
// prism is a certified touch with an Exact zero gap.
func TestPlanarPairVerdictTouchNeedsZeroDisplacement(t *testing.T) {
	t.Parallel()

	t.Run("displaced branch on the arm", func(t *testing.T) {
		t.Parallel()
		doc := New()
		part, branch, delta := planarPartAndBranch(t, doc, r3.Vec{}, 40)
		require.Positive(t, delta)
		res, ok, err := planarPairVerdict(t.Context(), part, branch)
		require.NoError(t, err)
		require.True(t, ok)
		require.Equal(t, pairUndecided, res.verdict)
	})

	t.Run("exact mitred column under an exact prism lid", func(t *testing.T) {
		t.Parallel()
		doc := New()
		// A one-span square column swept straight up holds float vertices
		// only, so its displacement is exactly zero, and the lid rests
		// flush on its top cap: a certified touch with an Exact zero gap.
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		r := s.CreateRectangle(-2, -2, 2, 2)
		for _, p := range []*sketch.Point{r.A, r.B, r.C, r.D} {
			s.Fix(p)
		}
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		up, err := NewPath(r3.Vec{}, LineTo{End: r3.NewVec(0, 0, 10)})
		require.NoError(t, err)
		column, err := doc.Sweep(t.Context(), s, s.Profiles()[0], up, WithMitredJoins())
		require.NoError(t, err)
		payload, ok := column.payload.(mitredSweepPayload)
		require.True(t, ok)
		require.Zero(t, payload.delta, "every held vertex is its own rational")
		lid := planarBox(t, doc, -1, -1, 10, 1, 1, 12)
		res, ok, err := planarPairVerdict(t.Context(), column, lid)
		require.NoError(t, err)
		require.True(t, ok)
		require.Equal(t, pairTouching, res.verdict)
		require.True(t, res.exact)
		require.Zero(t, res.lo)
		require.Zero(t, res.hi)
		require.Positive(t, res.diam)
	})
}

// planarBox extrudes the axis-aligned box [x0,x1]×[y0,y1]×[z0,z1] from an XY
// sketch, placing it by an exact translation when z0 is not zero.
func planarBox(t *testing.T, doc *Document, x0, y0, z0, x1, y1, z1 float64) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	r := s.CreateRectangle(x0, y0, x1, y1)
	for _, p := range []*sketch.Point{r.A, r.B, r.C, r.D} {
		s.Fix(p)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(z1 - z0), Dir: Along})
	require.NoError(t, err)
	if z0 == 0 {
		return body
	}
	lift, err := r3.Translation(r3.NewVec(0, 0, z0))
	require.NoError(t, err)
	body, err = body.Placed(t.Context(), lift)
	require.NoError(t, err)
	return body
}
