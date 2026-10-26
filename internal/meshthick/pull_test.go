package meshthick_test

import (
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshthick"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func undercuts(t *testing.T, v []r3.Vec, tris [][3]int, pull r3.Vec) []int {
	t.Helper()
	budget := proofbound.NewWorkBudget(t.Context())
	tree, err := meshthick.NewTree(budget, v, tris)
	require.NoError(t, err)
	got, err := meshthick.Undercuts(budget, tree, pull, 1e-9)
	require.NoError(t, err)
	return got
}

func TestUndercutsOfABoxAreNoneAlongAnyAxis(t *testing.T) {
	t.Parallel()
	v, tris := box(10, 10, 10)
	for _, pull := range []r3.Vec{r3.NewVec(0, 0, 1), r3.NewVec(1, 0, 0), r3.NewVec(0, -1, 0)} {
		require.Empty(t, undercuts(t, v, tris, pull), "a box releases along %v", pull)
	}
}

// extrudePolygon extrudes the simple polygon poly, given as (x, z) points in
// either winding, 10 mm along y, and returns the closed mesh wound outward. The
// caps are triangulated by ear clipping.
func extrudePolygon(poly [][2]float64) ([]r3.Vec, [][3]int) {
	n := len(poly)
	var verts []r3.Vec
	for _, p := range poly {
		verts = append(verts, r3.NewVec(p[0], 0, p[1])) // the near cap, y = 0
	}
	for _, p := range poly {
		verts = append(verts, r3.NewVec(p[0], 10, p[1])) // the far cap, y = 10
	}
	area := 0.0
	for i, p := range poly {
		q := poly[(i+1)%n]
		area += p[0]*q[1] - q[0]*p[1]
	}
	idx := make([]int, n)
	for i := range idx {
		idx[i] = i
	}
	if area < 0 {
		slices.Reverse(idx) // counter-clockwise in (x, z)
	}
	cross := func(a, b, c int) float64 {
		return (poly[b][0]-poly[a][0])*(poly[c][1]-poly[a][1]) - (poly[b][1]-poly[a][1])*(poly[c][0]-poly[a][0])
	}
	inside := func(p, a, b, c int) bool {
		return cross(a, b, p) >= 0 && cross(b, c, p) >= 0 && cross(c, a, p) >= 0
	}
	var caps [][3]int
	ring := slices.Clone(idx)
	for len(ring) > 3 {
		for i := range ring {
			a, b, c := ring[(i+len(ring)-1)%len(ring)], ring[i], ring[(i+1)%len(ring)]
			if cross(a, b, c) <= 0 {
				continue
			}
			ear := true
			for _, p := range ring {
				if p != a && p != b && p != c && inside(p, a, b, c) {
					ear = false
					break
				}
			}
			if ear {
				caps = append(caps, [3]int{a, b, c})
				ring = slices.Delete(ring, i, i+1)
				break
			}
		}
	}
	caps = append(caps, [3]int{ring[0], ring[1], ring[2]})
	var tris [][3]int
	for _, c := range caps {
		tris = append(tris, c, [3]int{c[0] + n, c[2] + n, c[1] + n}) // the far cap faces the other way
	}
	for i := range idx {
		a, b := idx[i], idx[(i+1)%n]
		tris = append(tris, [3]int{a, b + n, b}, [3]int{a, a + n, b + n})
	}
	// Orient outward: the signed volume must be positive.
	vol := 0.0
	for _, t := range tris {
		vol += verts[t[0]].Dot(verts[t[1]].Cross(verts[t[2]])) / 6
	}
	if vol < 0 {
		for i := range tris {
			tris[i][1], tris[i][2] = tris[i][2], tris[i][1]
		}
	}
	return verts, tris
}

// overhang is a C-shaped prism in the XZ plane: a base, an arm 4 mm above it and
// a post joining them at the left, open to +x between base and arm.
func overhang() ([]r3.Vec, [][3]int) {
	return extrudePolygon([][2]float64{{0, 0}, {10, 0}, {10, 4}, {2, 4}, {2, 8}, {10, 8}, {10, 10}, {0, 10}})
}

func TestUndercutsFindTheFacesAnArmHidesFromThePull(t *testing.T) {
	t.Parallel()
	v, tris := overhang()
	// Pulling along +x the base's top face (z = 4, looking up) is released along
	// +z: the arm above blocks it. The arm's underside (z = 8, looking down) is
	// blocked by the base below.
	got := undercuts(t, v, tris, r3.NewVec(0, 0, 1))
	require.Len(t, got, 4, "two faces, each of two triangles: the base's inner top and the arm's underside")
	for _, i := range got {
		tri := tris[i]
		centre := v[tri[0]].Add(v[tri[1]]).Add(v[tri[2]]).Scale(1.0 / 3)
		require.True(t, centre.Z == 4 || centre.Z == 8, "a blocked triangle lies in z = 4 or z = 8: %v", centre)
		require.Greater(t, centre.X, 2.0)
	}
	// Pulling along +x releases the C's open side: no face is blocked in x.
	require.Empty(t, undercuts(t, v, tris, r3.NewVec(1, 0, 0)))
}

func TestOverhangFixtureEnclosesTheCShape(t *testing.T) {
	t.Parallel()
	v, tris := overhang()
	vol := 0.0
	for _, tri := range tris {
		vol += v[tri[0]].Dot(v[tri[1]].Cross(v[tri[2]])) / 6
	}
	require.InDelta(t, 680, vol, 1e-9, "the C's area, 100 less the 8 x 4 gap and the 2 mm post's notch, is 68 over 10 mm")
}
