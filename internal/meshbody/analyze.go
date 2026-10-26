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

func ratVec(v r3.Vec) [3]*big.Rat {
	return [3]*big.Rat{proofarith.FloatRat(v.X), proofarith.FloatRat(v.Y), proofarith.FloatRat(v.Z)}
}

func sub(a, b [3]*big.Rat) [3]*big.Rat {
	return [3]*big.Rat{new(big.Rat).Sub(a[0], b[0]), new(big.Rat).Sub(a[1], b[1]), new(big.Rat).Sub(a[2], b[2])}
}

func dot(a, b [3]*big.Rat) *big.Rat {
	out := new(big.Rat)
	for i := range a {
		out.Add(out, new(big.Rat).Mul(a[i], b[i]))
	}
	return out
}

func cross(a, b [3]*big.Rat) [3]*big.Rat {
	term := func(i, j int) *big.Rat {
		return new(big.Rat).Sub(new(big.Rat).Mul(a[i], b[j]), new(big.Rat).Mul(a[j], b[i]))
	}
	return [3]*big.Rat{term(1, 2), term(2, 0), term(0, 1)}
}

// Analyze reads the mesh. Every vertex must be finite and every triangle index
// in range; anything else is ErrUnsupported.
func Analyze(ctx context.Context, verts []r3.Vec, tris [][3]int) (Analysis, error) {
	budget := proofbound.NewWorkBudget(ctx)
	out := Analysis{Vertices: len(verts), Triangles: len(tris), SignedVolume: new(big.Rat),
		AreaLow: new(big.Rat), AreaHigh: new(big.Rat)}
	exact := make([][3]*big.Rat, len(verts))
	for i, v := range verts {
		if err := budget.Step(); err != nil {
			return Analysis{}, err
		}
		if !proofbound.FiniteVec(v) {
			return Analysis{}, fmt.Errorf(`%w: vertex %d is not finite`, decaderr.ErrUnsupported, i)
		}
		exact[i] = ratVec(v)
	}
	directed := make(map[edge]int, 3*len(tris))
	live := make([]int, 0, len(tris))
	six := big.NewRat(6, 1)
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
		normal := cross(sub(b, a), sub(c, a))
		lengthSquared := dot(normal, normal)
		if tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0] || lengthSquared.Sign() == 0 {
			out.DegenerateTriangles++
			continue
		}
		live = append(live, ti)
		for k := range tri {
			directed[edge{tri[k], tri[(k+1)%3]}]++
		}
		out.SignedVolume.Add(out.SignedVolume, new(big.Rat).Quo(dot(a, cross(b, c)), six))
		low := proofarith.FloatRat(proofbound.RatSqrtDown(lengthSquared))
		high := proofarith.FloatRat(proofbound.RatSqrtUp(lengthSquared))
		out.AreaLow.Add(out.AreaLow, low)
		out.AreaHigh.Add(out.AreaHigh, high)
	}
	half := big.NewRat(1, 2)
	out.AreaLow.Mul(out.AreaLow, half)
	out.AreaHigh.Mul(out.AreaHigh, half)
	countEdges(&out, directed)
	out.Shells = shells(tris, live)
	return out, nil
}

// countEdges classifies every undirected edge by how many triangles use it
// and in which directions.
func countEdges(out *Analysis, directed map[edge]int) {
	for e, forward := range directed {
		back := directed[edge{e[1], e[0]}]
		if e[0] > e[1] && back > 0 {
			continue // the pair is read once, from its smaller end
		}
		switch total := forward + back; {
		case total == 1:
			out.BoundaryEdges++
		case total > 2:
			out.NonManifoldEdges++
		case forward == 2 || back == 2:
			out.MisorientedEdges++
		}
	}
}

// shells counts the groups of live triangles joined across shared edges.
func shells(tris [][3]int, live []int) int {
	parent := make(map[int]int, len(live))
	for _, ti := range live {
		parent[ti] = ti
	}
	find := func(x int) int {
		for parent[x] != x {
			parent[x] = parent[parent[x]]
			x = parent[x]
		}
		return x
	}
	owner := make(map[edge]int, 3*len(live))
	for _, ti := range live {
		tri := tris[ti]
		for k := range tri {
			a, b := tri[k], tri[(k+1)%3]
			if a > b {
				a, b = b, a
			}
			if other, ok := owner[edge{a, b}]; ok {
				parent[find(ti)] = find(other)
				continue
			}
			owner[edge{a, b}] = ti
		}
	}
	roots := make(map[int]struct{})
	for _, ti := range live {
		roots[find(ti)] = struct{}{}
	}
	return len(roots)
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
