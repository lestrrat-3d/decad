package spiralsector

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// blankFaceGroup returns the index, in blank's face order, of the one face
// whose origins carry the Revolve role of the source edge, or -1 when no face
// or more than one face carries it.
func blankFaceGroup(blank *topo.Body, source *facetedrecord.SpiralBlankSource, edge int) int {
	if blank == nil || source == nil || edge < 0 || edge >= len(source.EdgeIndex) {
		return -1
	}
	role := fmt.Sprintf("side(0,%d)", source.EdgeIndex[edge])
	group := -1
	for i, face := range topobuild.Faces(blank) {
		for _, origin := range face.Origins {
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

// HeldBlankCellGraph authenticates one finite straight meridian's
// actual held Revolve mesh as a complete radial graph. The source matcher
// establishes the world-X six-line profile; this gate checks the emitted
// rings and fixed cell diagonal rather than assuming float ring homothety.
func HeldBlankCellGraph(ctx context.Context, blank *topo.Body, held *meshbool.BoolMesh,
	source *facetedrecord.SpiralBlankSource, edge int) (bool, error) {
	if ctx == nil || blank == nil || held == nil || source == nil ||
		edge < 0 || edge >= len(source.EdgeIndex) {
		return false, nil
	}
	group := blankFaceGroup(blank, source, edge)
	if group < 0 || len(held.Src) != len(held.Tris) {
		return false, nil
	}
	var ringX [2]float64
	var rings [2][]int
	seenX := map[float64]struct{}{}
	actual := map[GraphTri]int{}
	for i, tri := range held.Tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return false, err
			}
		}
		if held.Src[i] != group {
			continue
		}
		actual[SortedGraphTri(tri[0], tri[1], tri[2])]++
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
			return GraphCross(GraphLift(a.Y, a.Z), GraphLift(b.Y, b.Z)).Sign() > 0
		})
		for i := range rings[k] {
			a, b := held.Verts[rings[k][i]], held.Verts[rings[k][(i+1)%n]]
			pa, pb := GraphLift(a.Y, a.Z), GraphLift(b.Y, b.Z)
			if GraphCross(pa, pb).Sign() <= 0 || GraphDot(pa, pb).Sign() <= 0 {
				return false, nil
			}
		}
	}
	for i := range n {
		j := (i + 1) % n
		a, b := rings[0][i], rings[1][i]
		c, d := rings[1][j], rings[0][j]
		ac := actual[SortedGraphTri(a, b, c)] == 1 && actual[SortedGraphTri(a, c, d)] == 1
		bd := actual[SortedGraphTri(a, b, d)] == 1 && actual[SortedGraphTri(b, c, d)] == 1
		if ac == bd {
			return false, nil
		}
		av, bv, cv, dv := held.Verts[a], held.Verts[b], held.Verts[c], held.Verts[d]
		if !GraphOrder(GraphLift(av.Y, av.Z), GraphLift(bv.Y, bv.Z),
			GraphLift(cv.Y, cv.Z), GraphLift(dv.Y, dv.Z), ac) {
			return false, nil
		}
	}
	return true, nil
}

// HeldBlankRadialAllowance bounds the actual held face's radius at
// each X against its exact source meridian. The caller separately proves
// this face is a full radial graph with HeldBlankCellGraph.
func HeldBlankRadialAllowance(ctx context.Context, blank *topo.Body,
	held *meshbool.BoolMesh, source *facetedrecord.SpiralBlankSource, edge int) (float64, bool, error) {
	if ctx == nil || blank == nil || held == nil || source == nil ||
		edge < 0 || edge >= len(source.EdgeIndex) || len(held.Src) != len(held.Tris) {
		return 0, false, nil
	}
	group := blankFaceGroup(blank, source, edge)
	if group < 0 {
		return 0, false, nil
	}
	cycle := [6]planePoint{source.ToeAxis, source.HeelAxis, source.HeelOuter,
		source.Ded, source.ToeRoot, source.ToeOuter}
	a, z := cycle[edge], cycle[(edge+1)%len(cycle)]
	if a.U == z.U {
		return 0, false, nil
	}
	m := seamRatDiv(seamRatSub(seamRatF(z.V), seamRatF(a.V)), seamRatSub(seamRatF(z.U), seamRatF(a.U)))
	b := seamRatSub(seamRatF(a.V), seamRatMul(m, seamRatF(a.U)))
	limit := new(big.Rat)
	seen := false
	for i, tri := range held.Tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return 0, false, err
			}
		}
		if held.Src[i] != group {
			continue
		}
		seen = true
		var points [3]r3.Vec
		for k, index := range tri {
			if index < 0 || index >= len(held.Verts) ||
				!proofbound.FiniteVec(held.Verts[index]) {
				return 0, false, nil
			}
			points[k] = held.Verts[index]
		}
		triangleLimit, ok := TriangleMeridianError(points, m, b)
		if !ok {
			return 0, false, nil
		}
		if triangleLimit.Cmp(limit) > 0 {
			limit = triangleLimit
		}
	}
	if !seen {
		return 0, false, nil
	}
	allowance := proofbound.RatFloatUp(limit)
	return allowance, !proofbound.IsNonFinite(allowance), nil
}
