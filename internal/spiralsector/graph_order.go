package spiralsector

import (
	"math/big"
	"slices"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// GraphPoint is an exact lift of one held mesh meridian point.
type GraphPoint struct{ y, z *big.Rat }

// GraphLift lifts one rounded mesh coordinate pair to exact rationals.
func GraphLift(y, z float64) GraphPoint {
	return GraphPoint{proofarith.FloatRat(y), proofarith.FloatRat(z)}
}

// GraphCross returns the exact meridian cross product.
func GraphCross(a, b GraphPoint) *big.Rat {
	return proofarith.SubRat(new(big.Rat),
		proofarith.MulRat(new(big.Rat), a.y, b.z),
		proofarith.MulRat(new(big.Rat), a.z, b.y))
}

// GraphDot returns the exact meridian dot product.
func GraphDot(a, b GraphPoint) *big.Rat {
	return proofarith.AddRat(new(big.Rat),
		proofarith.MulRat(new(big.Rat), a.y, b.y),
		proofarith.MulRat(new(big.Rat), a.z, b.z))
}

func graphSectionOrder(p0, p1, q0, q1 GraphPoint) bool {
	c0, c2 := GraphCross(p0, q0), GraphCross(p1, q1)
	c1 := proofarith.AddRat(new(big.Rat), GraphCross(p0, q1), GraphCross(p1, q0))
	return c0.Sign() >= 0 && c1.Sign() >= 0 && c2.Sign() >= 0 &&
		(c0.Sign() > 0 || c1.Sign() > 0 || c2.Sign() > 0)
}

// GraphOrder proves the two section edges of one Revolve cell stay in
// angular order for every X between its rings. The degree-two Bernstein
// coefficients are exact lifts of the actual held mesh coordinates.
func GraphOrder(a, b, c, d GraphPoint, diagonalAC bool) bool {
	if GraphCross(a, d).Sign() <= 0 || GraphCross(b, c).Sign() <= 0 {
		return false
	}
	m0, m1 := a, c
	if !diagonalAC {
		m0, m1 = d, b
	}
	if !graphSectionOrder(a, b, m0, m1) ||
		!graphSectionOrder(m0, m1, d, c) {
		return false
	}
	for _, pair := range [][2]GraphPoint{{a, b}, {a, c}, {a, d},
		{b, c}, {b, d}, {c, d}} {
		if GraphDot(pair[0], pair[1]).Sign() <= 0 {
			return false
		}
	}
	return true
}

// GraphTri is an unordered held-mesh triangle key after sorting.
type GraphTri [3]int

// SortedGraphTri makes a canonical held-mesh triangle key.
func SortedGraphTri(a, b, c int) GraphTri {
	t := GraphTri{a, b, c}
	slices.Sort(t[:])
	return t
}
