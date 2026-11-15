package retainedblend

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// The band of the tests lies in the plane z = 0: 0 ≤ x ≤ 1 (the main plane
// and the one further plane) and 0 ≤ y (the end plane), with every held vertex
// bounded by 0.001 and a rounding of 0.001, so a vertex within 0.002 of a
// plane is undecided on it.

// flatBand is that band, cutting the facet through verts.
func flatBand(verts ...r3.Vec) (*work, *wallBand, faceLoops) {
	beta := make([]float64, len(verts))
	for i := range beta {
		beta[i] = 0.001
	}
	w := &work{
		fp:    facetedrecord.Payload{Tris: [][3]int{{0, 1, 2}}, FaceOf: []int{0}},
		verts: verts, beta: beta, tau: 0.001, cuts: map[[2]int]int{},
	}
	side := func(n, o r3.Vec, sign float64) *bandEnd { return &bandEnd{g: plane{n: n, o: o}, sign: sign} }
	b := &wallBand{
		main:   side(r3.NewVec(1, 0, 0), r3.NewVec(1, 0, 0), -1),
		ends:   [2]*bandEnd{side(r3.NewVec(0, 1, 0), r3.Vec{}, 1), nil},
		others: []*bandEnd{side(r3.NewVec(1, 0, 0), r3.Vec{}, 1)},
	}
	return w, b, faceLoops{facets: []int{0}}
}

// TestCutBandMembershipByEveryPlane reads each held vertex against every band
// plane (retained-edge-blend §4.3): a plane that puts a vertex outside by more
// than its bound proves it outside, so a vertex within its bound of another
// plane is never read against it.
func TestCutBandMembershipByEveryPlane(t *testing.T) {
	t.Parallel()

	t.Run("a facet outside one plane is emitted unchanged", func(t *testing.T) {
		t.Parallel()
		// The corner (-5, 0) lies on the end plane y = 0 within its bound, but all
		// three corners lie 4 to 6 outside x ≥ 0.
		w, b, f := flatBand(r3.NewVec(-5, 0, 0), r3.NewVec(-4, -3, 0), r3.NewVec(-6, -3, 0))
		cuts, err := w.cutBand(f, b)
		require.NoError(t, err)
		require.Equal(t, [][3]int{{0, 1, 2}}, w.out)
		require.Len(t, w.verts, 3, "the facet makes no vertex")
		require.Empty(t, cuts.removed)
	})

	t.Run("a facet on or outside the main plane is emitted unchanged", func(t *testing.T) {
		t.Parallel()
		w, b, f := flatBand(r3.NewVec(1, 0, 0), r3.NewVec(1, 5, 0), r3.NewVec(3, 2, 0))
		// The first two corners lie on the main plane by construction; the first
		// is also within its bound of the end plane y = 0.
		b.main.on = map[int]bool{0: true, 1: true}
		_, err := w.cutBand(f, b)
		require.NoError(t, err)
		require.Equal(t, [][3]int{{0, 1, 2}}, w.out)
		require.Len(t, w.verts, 3)
	})

	t.Run("a corner another plane drops is clipped before the plane that leaves it undecided", func(t *testing.T) {
		t.Parallel()
		// The corner (-0.5, 0) lies on y = 0 within its bound and 0.5 outside
		// x ≥ 0. Applied first, y ≥ 0 would refuse it; x ≥ 0 drops it, and y ≥ 0
		// then reads only the two cut points and the corners inside.
		w, b, f := flatBand(r3.NewVec(-0.5, 0, 0), r3.NewVec(0.6, 2, 0), r3.NewVec(0.6, 4, 0))
		cuts, err := w.cutBand(f, b)
		require.NoError(t, err)
		require.Len(t, w.verts, 5, "two cut points on x = 0")
		for _, p := range w.verts[3:] {
			require.InDelta(t, 0, p.X, 1e-12)
		}
		require.Equal(t, map[int]bool{1: true, 2: true}, cuts.removed)
		// What stays outside is the corner triangle: base 0.909 on x = 0, height 0.5.
		area := 0.0
		for _, tri := range w.out {
			a, c, d := w.verts[tri[0]], w.verts[tri[1]], w.verts[tri[2]]
			area += c.Sub(a).Cross(d.Sub(a)).Len() / 2
		}
		require.InDelta(t, 0.25/1.1, area, 1e-12)
	})

	t.Run("a corner no other plane drops is read by its sign", func(t *testing.T) {
		t.Parallel()
		// (0.5, 0.0015) lies on y = 0 within its bound and inside every other
		// plane: no order of the planes decides it, so the end plane reads it by the
		// sign of its offset and charges the offset (cyl_band_sign_internal_test.go).
		w, b, f := flatBand(r3.NewVec(0.5, 0.0015, 0), r3.NewVec(0.6, 2, 0), r3.NewVec(0.7, 4, 0))
		cuts, err := w.cutBand(f, b)
		require.NoError(t, err)
		require.Equal(t, map[int]bool{0: true, 1: true, 2: true}, cuts.removed)
		require.InDelta(t, 0.0025, w.beta[0], 1e-12)
	})

	t.Run("a corner two planes leave undecided keeps the plain text", func(t *testing.T) {
		t.Parallel()
		// (0.0005, 0.0005) lies within its bound of both x = 0 and y = 0.
		w, b, f := flatBand(r3.NewVec(0.0005, 0.0005, 0), r3.NewVec(0.6, 2, 0), r3.NewVec(0.7, 4, 0))
		_, err := w.cutBand(f, b)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "a vertex of the wall lies on an end face's plane")
	})

	t.Run("a vertex outside a plane is not removed by one that leaves it undecided", func(t *testing.T) {
		t.Parallel()
		// (-0.5, 0) lies on y = 0 within its bound and outside x ≥ 0: it is not in
		// the band, and the plane that cannot read it does not refuse it.
		w, b, f := flatBand(r3.NewVec(-0.5, 0, 0), r3.NewVec(0.5, 3, 0), r3.NewVec(-0.5, 3, 0))
		cuts, err := w.cutBand(f, b)
		require.NoError(t, err)
		require.Equal(t, map[int]bool{1: true}, cuts.removed)
	})
}
