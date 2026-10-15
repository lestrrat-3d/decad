package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"slices"
	"sort"

	"github.com/lestrrat-3d/decad/internal/meshbool"
)

type spiralGraphPoint struct{ y, z *big.Rat }

func spiralGraphLift(y, z float64) spiralGraphPoint {
	return spiralGraphPoint{lfF(y), lfF(z)}
}

func spiralGraphCross(a, b spiralGraphPoint) *big.Rat {
	return lfSub(lfMul(a.y, b.z), lfMul(a.z, b.y))
}

func spiralGraphDot(a, b spiralGraphPoint) *big.Rat {
	return lfAdd(lfMul(a.y, b.y), lfMul(a.z, b.z))
}

func spiralGraphSectionOrder(p0, p1, q0, q1 spiralGraphPoint) bool {
	c0, c2 := spiralGraphCross(p0, q0), spiralGraphCross(p1, q1)
	c1 := lfAdd(spiralGraphCross(p0, q1), spiralGraphCross(p1, q0))
	return c0.Sign() >= 0 && c1.Sign() >= 0 && c2.Sign() >= 0 &&
		(c0.Sign() > 0 || c1.Sign() > 0 || c2.Sign() > 0)
}

// spiralGraphOrder proves the two section edges of one Revolve cell stay in
// angular order for every X between its rings. The degree-two Bernstein
// coefficients are exact lifts of the actual held mesh coordinates.
func spiralGraphOrder(a, b, c, d spiralGraphPoint, diagonalAC bool) bool {
	if spiralGraphCross(a, d).Sign() <= 0 || spiralGraphCross(b, c).Sign() <= 0 {
		return false
	}
	m0, m1 := a, c
	if !diagonalAC {
		m0, m1 = d, b
	}
	if !spiralGraphSectionOrder(a, b, m0, m1) ||
		!spiralGraphSectionOrder(m0, m1, d, c) {
		return false
	}
	for _, pair := range [][2]spiralGraphPoint{{a, b}, {a, c}, {a, d},
		{b, c}, {b, d}, {c, d}} {
		if spiralGraphDot(pair[0], pair[1]).Sign() <= 0 {
			return false
		}
	}
	return true
}

type spiralGraphTri [3]int

func spiralSortedGraphTri(a, b, c int) spiralGraphTri {
	t := spiralGraphTri{a, b, c}
	slices.Sort(t[:])
	return t
}

func spiralBlankFaceGroup(blank *Body, source *spiralBlankSource, edge int) int {
	if blank == nil || source == nil || edge < 0 || edge >= len(source.EdgeIndex) {
		return -1
	}
	role := fmt.Sprintf("side(0,%d)", source.EdgeIndex[edge])
	group := -1
	for i, face := range blank.Faces() {
		for _, origin := range face.Origins() {
			if origin.Role == role {
				if group >= 0 {
					return -1
				}
				group = i
			}
		}
	}
	return group
}

// spiralHeldBlankCellGraph authenticates one finite straight meridian's
// actual held Revolve mesh as a complete radial graph. The source matcher
// establishes the world-X six-line profile; this gate checks the emitted
// rings and fixed cell diagonal rather than assuming float ring homothety.
func spiralHeldBlankCellGraph(ctx context.Context, blank *Body, held *meshbool.BoolMesh,
	source *spiralBlankSource, edge int) (bool, error) {
	if ctx == nil || blank == nil || held == nil || source == nil ||
		edge < 0 || edge >= len(source.EdgeIndex) {
		return false, nil
	}
	group := spiralBlankFaceGroup(blank, source, edge)
	if group < 0 || len(held.Src) != len(held.Tris) {
		return false, nil
	}
	var ringX [2]float64
	var rings [2][]int
	seenX := map[float64]struct{}{}
	actual := map[spiralGraphTri]int{}
	for i, tri := range held.Tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return false, err
			}
		}
		if held.Src[i] != group {
			continue
		}
		actual[spiralSortedGraphTri(tri[0], tri[1], tri[2])]++
		for _, index := range tri {
			if index < 0 || index >= len(held.Verts) {
				return false, nil
			}
			v := held.Verts[index]
			if math.IsNaN(v.X) || math.IsInf(v.X, 0) ||
				math.IsNaN(v.Y) || math.IsNaN(v.Z) ||
				math.IsInf(v.Y, 0) || math.IsInf(v.Z, 0) {
				return false, nil
			}
			seenX[v.X] = struct{}{}
		}
	}
	if len(seenX) != 2 {
		return false, nil
	}
	i := 0
	for x := range seenX {
		ringX[i] = x
		i++
	}
	if ringX[0] > ringX[1] {
		ringX[0], ringX[1] = ringX[1], ringX[0]
	}
	seen := map[int]bool{}
	for tri := range actual {
		for _, index := range tri {
			if seen[index] {
				continue
			}
			seen[index] = true
			v := held.Verts[index]
			which := 0
			if v.X == ringX[1] {
				which = 1
			} else if v.X != ringX[0] {
				return false, nil
			}
			rings[which] = append(rings[which], index)
		}
	}
	n := len(rings[0])
	if n < 8 || n > 4096 || len(rings[1]) != n || len(actual) != 2*n {
		return false, nil
	}
	for k := range rings {
		sort.Slice(rings[k], func(i, j int) bool {
			a, b := held.Verts[rings[k][i]], held.Verts[rings[k][j]]
			// Start at +Y and order the two half-planes by exact signs.
			// No rounded angle decides the claimed full turn.
			upperA := a.Z > 0 || a.Z == 0 && a.Y > 0
			upperB := b.Z > 0 || b.Z == 0 && b.Y > 0
			if upperA != upperB {
				return upperA
			}
			return spiralGraphCross(spiralGraphLift(a.Y, a.Z),
				spiralGraphLift(b.Y, b.Z)).Sign() > 0
		})
		for i := range rings[k] {
			a, b := held.Verts[rings[k][i]], held.Verts[rings[k][(i+1)%n]]
			pa, pb := spiralGraphLift(a.Y, a.Z), spiralGraphLift(b.Y, b.Z)
			if spiralGraphCross(pa, pb).Sign() <= 0 || spiralGraphDot(pa, pb).Sign() <= 0 {
				return false, nil
			}
		}
	}
	for i := range n {
		j := (i + 1) % n
		a, b := rings[0][i], rings[1][i]
		c, d := rings[1][j], rings[0][j]
		ac := actual[spiralSortedGraphTri(a, b, c)] == 1 &&
			actual[spiralSortedGraphTri(a, c, d)] == 1
		bd := actual[spiralSortedGraphTri(a, b, d)] == 1 &&
			actual[spiralSortedGraphTri(b, c, d)] == 1
		if ac == bd {
			return false, nil
		}
		av, bv, cv, dv := held.Verts[a], held.Verts[b], held.Verts[c], held.Verts[d]
		if !spiralGraphOrder(spiralGraphLift(av.Y, av.Z),
			spiralGraphLift(bv.Y, bv.Z), spiralGraphLift(cv.Y, cv.Z),
			spiralGraphLift(dv.Y, dv.Z), ac) {
			return false, nil
		}
	}
	return true, nil
}
