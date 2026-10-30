// Package meshbody reads an imported triangle mesh: its connectivity, its
// signed volume and its area, in exact rational arithmetic
// (agent-docs/mesh-import-design.md). It touches no body or document.
package meshbody

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// Analysis is what a mesh's triangles say about it.
type Analysis struct {
	// Vertices and Triangles count the input.
	Vertices, Triangles int
	// DegenerateTriangles counts triangles of zero area, including those that
	// name a vertex twice. They carry no area or volume and join no shell.
	DegenerateTriangles int
	// BoundaryEdges counts undirected edges that exactly one triangle uses.
	BoundaryEdges int
	// NonManifoldEdges counts undirected edges that three or more triangles use.
	NonManifoldEdges int
	// MisorientedEdges counts undirected edges that exactly two triangles use
	// in the same direction.
	MisorientedEdges int
	// Shells counts connected groups of non-degenerate triangles, joined
	// across shared edges.
	Shells int
	// SignedVolume is the exact sum of signed tetrahedron volumes about the
	// origin. It is the enclosed volume of a closed, consistently oriented
	// mesh whose triangles wind counter-clockwise seen from outside.
	SignedVolume *big.Rat
	// AreaLow and AreaHigh are exact rationals that enclose the surface area.
	AreaLow, AreaHigh *big.Rat
}

// Closed reports a mesh whose every edge is used by exactly two triangles in
// opposite directions.
func (a Analysis) Closed() bool {
	return a.Triangles > a.DegenerateTriangles && a.BoundaryEdges == 0 && a.NonManifoldEdges == 0 && a.MisorientedEdges == 0
}

type edge [2]int

type edgeUse struct {
	first             int // dense live-triangle index of the first use
	forward, backward uint8
}

// Analyze reads the mesh. Every vertex must be finite and every triangle index
// in range; anything else is ErrUnsupported.
func Analyze(ctx context.Context, verts []r3.Vec, tris [][3]int) (Analysis, error) {
	budget := proofbound.NewWorkBudget(ctx)
	out := Analysis{Vertices: len(verts), Triangles: len(tris), SignedVolume: new(big.Rat),
		AreaLow: new(big.Rat), AreaHigh: new(big.Rat)}
	exact := make([]proofarith.DyV3, len(verts))
	for i, v := range verts {
		if err := budget.Step(); err != nil {
			return Analysis{}, err
		}
		if !proofbound.FiniteVec(v) {
			return Analysis{}, fmt.Errorf(`%w: vertex %d is not finite`, decaderr.ErrUnsupported, i)
		}
		exact[i] = proofarith.DyVec(v)
	}
	// A closed mesh uses an edge twice. This is only a capacity hint;
	// open or non-manifold inputs grow the same table as needed.
	edges := make(map[edge]edgeUse, len(tris)+len(tris)/2)
	parent := make([]int, 0, len(tris))
	find := func(x int) int {
		for parent[x] != x {
			parent[x] = parent[parent[x]]
			x = parent[x]
		}
		return x
	}
	var volume proofarith.DyAcc
	var areaLow, areaHigh proofarith.DyAcc
	for ti, tri := range tris {
		if err := budget.Step(); err != nil {
			return Analysis{}, err
		}
		for _, index := range tri {
			if index < 0 || index >= len(verts) {
				return Analysis{}, fmt.Errorf(`%w: triangle %d names vertex %d of %d`,
					decaderr.ErrUnsupported, ti, index, len(verts))
			}
		}
		a, b, c := exact[tri[0]], exact[tri[1]], exact[tri[2]]
		// Finite float coordinates and their products are exact dyadics.
		// Keep intermediate vectors there instead of reducing a rational
		// after every scalar operation.
		normal := proofarith.DvCross(proofarith.DvSub(b, a), proofarith.DvSub(c, a))
		if tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0] ||
			(normal[0].IsZero() && normal[1].IsZero() && normal[2].IsZero()) {
			out.DegenerateTriangles++
			continue
		}
		component := len(parent)
		parent = append(parent, component)
		out.Shells++
		for k := range tri {
			a, b := tri[k], tri[(k+1)%3]
			forward := a < b
			if !forward {
				a, b = b, a
			}
			key := edge{a, b}
			use, seen := edges[key]
			if !seen {
				use.first = component
			} else if left, right := find(component), find(use.first); left != right {
				parent[left] = right
				out.Shells--
			}
			// Reports distinguish one, two and at least three uses. Saturate
			// at three so even a heavily repeated edge cannot overflow.
			if use.forward+use.backward < 3 {
				if forward {
					use.forward++
				} else {
					use.backward++
				}
			}
			edges[key] = use
		}
		// a·((b-a)×(c-a)) = a·(b×c), so the area normal also gives
		// the signed tetrahedron numerator. Sum the dyadics exactly before
		// dividing once, instead of reducing one rational per triangle.
		volume.AddDot(a, normal)
		lowFloat, highFloat := proofarith.DvNormBounds(normal)
		if math.IsNaN(lowFloat) || math.IsInf(lowFloat, 0) ||
			math.IsNaN(highFloat) || math.IsInf(highFloat, 0) {
			// A triangle's area past the largest float has no float bound.
			return Analysis{}, fmt.Errorf(`%w: triangle %d is too large to bound its area`, decaderr.ErrUnsupported, ti)
		}
		areaLow.Add(proofarith.DyOfFinite(lowFloat))
		areaHigh.Add(proofarith.DyOfFinite(highFloat))
	}
	out.SignedVolume.Quo(volume.Rat(), big.NewRat(6, 1))
	out.AreaLow.Set(areaLow.Rat())
	out.AreaHigh.Set(areaHigh.Rat())
	half := big.NewRat(1, 2)
	out.AreaLow.Mul(out.AreaLow, half)
	out.AreaHigh.Mul(out.AreaHigh, half)
	countEdges(&out, edges)
	return out, nil
}

// countEdges classifies every undirected edge by how many triangles use it
// and in which directions.
func countEdges(out *Analysis, edges map[edge]edgeUse) {
	for _, use := range edges {
		switch total := use.forward + use.backward; {
		case total == 1:
			out.BoundaryEdges++
		case total > 2:
			out.NonManifoldEdges++
		case use.forward == 2 || use.backward == 2:
			out.MisorientedEdges++
		}
	}
}

// Interval publishes the exact interval [low, high] as its nearest midpoint
// float and a bound that covers both ends. ok is false when either number
// does not fit a float.
func Interval(low, high *big.Rat) (value, bound float64, ok bool) {
	mid := new(big.Rat).Quo(new(big.Rat).Add(low, high), big.NewRat(2, 1))
	value, _ = mid.Float64()
	if math.IsInf(value, 0) || math.IsNaN(value) {
		return 0, 0, false
	}
	held := proofarith.FloatRat(value)
	spread := proofbound.RatMax(new(big.Rat).Abs(new(big.Rat).Sub(low, held)),
		new(big.Rat).Abs(new(big.Rat).Sub(high, held)))
	bound = proofbound.RatFloatUp(spread)
	if math.IsInf(bound, 0) || math.IsNaN(bound) {
		return 0, 0, false
	}
	return value, bound, true
}
