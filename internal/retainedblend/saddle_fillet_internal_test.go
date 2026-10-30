package retainedblend

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/crossbore"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestChainArcs(t *testing.T) {
	t.Parallel()
	// Three facets' arcs around a window, clockwise: 1 → (corner 10) → 2,
	// 2 → 3, 3 → 1, and a held sliver whose merged entry and exit make no arc.
	arcs := map[int][]int{1: {1, 10, 2}, 2: {2, 3}, 3: {3, 1}}
	ring, err := chainArcs(arcs, nil)
	require.NoError(t, err)
	require.Equal(t, []int{3, 2, 10, 1}, ring, "the ring runs counter-clockwise")

	_, err = chainArcs(map[int][]int{1: {1, 2}, 2: {2, 1}, 5: {5, 6}, 6: {6, 5}}, nil)
	require.ErrorIs(t, err, decaderr.ErrUnsupported, "two loops")
	_, err = chainArcs(map[int][]int{1: {1, 2}, 2: {2, 3}}, nil)
	require.ErrorIs(t, err, decaderr.ErrUnsupported, "an open chain")

	ring, err = chainArcs(map[int][]int{}, []int{7, 6, 5})
	require.NoError(t, err)
	require.Equal(t, []int{5, 6, 7}, ring, "a window inside one facet is its corners")
}

func TestPoseHand(t *testing.T) {
	t.Parallel()
	require.InDelta(t, 1.0, poseHand(crossbore.Pose{Axis: [3]int{0, 1, 2}, Sign: [3]float64{1, 1, 1}}), 0)
	require.InDelta(t, 1.0, poseHand(crossbore.Pose{Axis: [3]int{0, 1, 2}, Sign: [3]float64{-1, -1, 1}}), 0)
	require.InDelta(t, 1.0, poseHand(crossbore.Pose{Axis: [3]int{2, 0, 1}, Sign: [3]float64{1, 1, 1}}), 0)
	// The Revolve shaft on x with a bore on z: local X is world z.
	require.InDelta(t, -1.0, poseHand(crossbore.Pose{Axis: [3]int{2, 1, 0}, Sign: [3]float64{1, 1, 1}}), 0)
	require.InDelta(t, -1.0, poseHand(crossbore.Pose{Axis: [3]int{0, 1, 2}, Sign: [3]float64{-1, 1, 1}}), 0)
}

func TestMonotoneRing(t *testing.T) {
	t.Parallel()
	ring, err := monotoneRing([]ringPt{{idx: 1, ang: 4}, {idx: 2, ang: 5}, {idx: 3, ang: 1}, {idx: 4, ang: 2.5}}, "ring")
	require.NoError(t, err)
	require.Equal(t, 3, ring[0].idx, "the ring starts at its least angle")

	_, err = monotoneRing([]ringPt{{idx: 1, ang: 1}, {idx: 2, ang: 3}, {idx: 3, ang: 2}, {idx: 4, ang: 5}}, "ring")
	require.ErrorIs(t, err, decaderr.ErrUnsupported, "a ring that turns back")
	_, err = monotoneRing([]ringPt{{idx: 1, ang: 0}, {idx: 2, ang: 0.5}, {idx: 3, ang: 1 + math.Pi}}, "ring")
	require.ErrorIs(t, err, decaderr.ErrUnsupported, "a gap of half a turn")
}

// TestSaddleRecognitionKeepsTheChamferRule compares looksLikeSaddle with the
// rule the R7 chamfer read before σ: a convex edge, exactly one face
// Reversed, the other the shaft of radius R and the Reversed one the bore of
// radius r, 2r² < R². Every pair of radii, Reversed flags and Convex flag
// must give the same answer and the same shaft. A concave pin root inside a
// tube's bore (both flags and Convex flipped from a cross hole's mouth) has the
// mouth's σ and must still refuse.
func TestSaddleRecognitionKeepsTheChamferRule(t *testing.T) {
	t.Parallel()
	before := func(e *topo.Edge) (*topo.Face, bool) {
		if !e.Convex || e.Faces[0].Reversed == e.Faces[1].Reversed {
			return nil, false
		}
		shaft, bore := e.Faces[0], e.Faces[1]
		if shaft.Reversed {
			shaft, bore = bore, shaft
		}
		R, _ := shaft.Surface.(surfacegeom.Cylinder).Radius.In(units.Millimeter)
		r, _ := bore.Surface.(surfacegeom.Cylinder).Radius.In(units.Millimeter)
		return shaft, 2*r*r < R*R
	}
	v := &topo.Vertex{}
	for _, radii := range [][2]float64{{10, 3}, {3, 10}, {10, 7.5}, {7.5, 10}, {5, 5}} {
		for flags := range 8 {
			convex, rev0, rev1 := flags&1 != 0, flags&2 != 0, flags&4 != 0
			faces := []*topo.Face{
				{Surface: surfacegeom.Cylinder{Axis: r3.NewVec(1, 0, 0), Radius: units.Millimeters(radii[0])},
					RetainedTag: true, Reversed: rev0},
				{Surface: surfacegeom.Cylinder{Origin: r3.NewVec(40, 0, 0), Axis: r3.NewVec(0, 0, 1),
					Radius: units.Millimeters(radii[1])}, RetainedTag: true, Reversed: rev1},
			}
			e := &topo.Edge{Start: v, End: v, Faces: faces, Convex: convex}
			name := fmt.Sprintf("radii %v convex %v reversed %v %v", radii, convex, rev0, rev1)
			wantShaft, want := before(e)
			require.Equal(t, want, looksLikeSaddle(e), name)
			if want {
				shaft, _, _, _ := saddleTags(e)
				require.Same(t, wantShaft, shaft, name)
			}
		}
	}
}

// TestSaddleFilletRecognition checks the fillet's admission set on every
// pair of radii, Reversed flags and Convex flag: the convex mouth of a cross
// hole (the larger face not Reversed, the bore Reversed), two crossing bores
// (both Reversed, convex) and a boss's root on a rod (neither Reversed,
// concave). The concave pin root inside a tube's bore, and every edge whose
// ball lies inside the bore, refuse.
func TestSaddleFilletRecognition(t *testing.T) {
	t.Parallel()
	v := &topo.Vertex{}
	for _, radii := range [][2]float64{{10, 3}, {3, 10}, {10, 7.5}, {5, 5}} {
		for flags := range 8 {
			convex, rev0, rev1 := flags&1 != 0, flags&2 != 0, flags&4 != 0
			faces := []*topo.Face{
				{Surface: surfacegeom.Cylinder{Axis: r3.NewVec(1, 0, 0), Radius: units.Millimeters(radii[0])},
					RetainedTag: true, Reversed: rev0},
				{Surface: surfacegeom.Cylinder{Origin: r3.NewVec(40, 0, 0), Axis: r3.NewVec(0, 0, 1),
					Radius: units.Millimeters(radii[1])}, RetainedTag: true, Reversed: rev1},
			}
			e := &topo.Edge{Start: v, End: v, Faces: faces, Convex: convex}
			name := fmt.Sprintf("radii %v convex %v reversed %v %v", radii, convex, rev0, rev1)
			bigRev, boreRev := rev0, rev1
			if radii[1] > radii[0] {
				bigRev, boreRev = rev1, rev0
			}
			want := radii[0] != radii[1] && 2*min(radii[0], radii[1])*min(radii[0], radii[1]) <
				max(radii[0], radii[1])*max(radii[0], radii[1])
			switch {
			case convex && !bigRev && boreRev, convex && bigRev && boreRev, !convex && !bigRev && !boreRev:
			default:
				want = false
			}
			require.Equal(t, want, looksLikeSaddleFillet(e), name)
			if !want {
				continue
			}
			_, _, key, ok := saddleTags(e)
			require.True(t, ok, name)
			require.Equal(t, convex && !bigRev, key.built(), "%s: the chamfer builds only the mouth", name)
			require.Equal(t, bigRev || !convex, canalOf(key, 1).Outside, "%s: the ball's side of the larger cylinder", name)
		}
	}
}
