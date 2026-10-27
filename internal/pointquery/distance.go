package pointquery

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

type ratVec [3]*big.Rat

func exactVec(v r3.Vec) ratVec {
	return ratVec{proof.FloatRat(v.X), proof.FloatRat(v.Y), proof.FloatRat(v.Z)}
}

func subtract(a, b ratVec) ratVec {
	return ratVec{
		new(big.Rat).Sub(a[0], b[0]),
		new(big.Rat).Sub(a[1], b[1]),
		new(big.Rat).Sub(a[2], b[2]),
	}
}

func dot(a, b ratVec) *big.Rat {
	out := new(big.Rat)
	for i := range a {
		out.Add(out, new(big.Rat).Mul(a[i], b[i]))
	}
	return out
}

func cross(a, b ratVec) ratVec {
	return ratVec{
		new(big.Rat).Sub(new(big.Rat).Mul(a[1], b[2]), new(big.Rat).Mul(a[2], b[1])),
		new(big.Rat).Sub(new(big.Rat).Mul(a[2], b[0]), new(big.Rat).Mul(a[0], b[2])),
		new(big.Rat).Sub(new(big.Rat).Mul(a[0], b[1]), new(big.Rat).Mul(a[1], b[0])),
	}
}

// boxDistanceSquared is a lower bound used only to skip a triangle whose
// closed axis-aligned box cannot beat the current exact minimum.
func boxDistanceSquared(p, a, b, c ratVec) *big.Rat {
	result := new(big.Rat)
	for i := range p {
		lo, hi := a[i], a[i]
		for _, v := range [2]*big.Rat{b[i], c[i]} {
			if v.Cmp(lo) < 0 {
				lo = v
			}
			if v.Cmp(hi) > 0 {
				hi = v
			}
		}
		gap := new(big.Rat)
		switch {
		case p[i].Cmp(lo) < 0:
			gap.Sub(lo, p[i])
		case p[i].Cmp(hi) > 0:
			gap.Sub(p[i], hi)
		}
		result.Add(result, new(big.Rat).Mul(gap, gap))
	}
	return result
}

// boxDistanceSquaredLower rounds every positive gap, square and partial sum
// downward. It bounds the exact box distance even when an operation
// overflows: NextDown then gives MaxFloat64, below the exact result.
// Comparing this lower bound with an upward-rounded minimum can only skip
// triangles the exact box test would skip. Unresolved cases keep that test.
func boxDistanceSquaredLower(p, a, b, c r3.Vec) float64 {
	var result float64
	for _, axis := range [3][4]float64{
		{p.X, a.X, b.X, c.X},
		{p.Y, a.Y, b.Y, c.Y},
		{p.Z, a.Z, b.Z, c.Z},
	} {
		lo, hi := min(axis[1], axis[2], axis[3]), max(axis[1], axis[2], axis[3])
		var gap float64
		switch {
		case axis[0] < lo:
			gap = proof.NextDown(lo - axis[0])
		case axis[0] > hi:
			gap = proof.NextDown(axis[0] - hi)
		default:
			continue
		}
		gap = max(0, gap)
		squared := max(0, proof.NextDown(float64(gap*gap)))
		result = max(0, proof.NextDown(result+squared))
	}
	return result
}

// checkFiniteVertices spends one budget step per vertex, in order, and
// rejects the first non-finite vertex.
func checkFiniteVertices(budget *proofbound.WorkBudget, verts []r3.Vec) error {
	for _, v := range verts {
		if err := budget.Step(); err != nil {
			return err
		}
		if !proofbound.FiniteVec(v) {
			return fmt.Errorf(`%w: point query mesh has a non-finite vertex`, decaderr.ErrUnsupported)
		}
	}
	return nil
}

// lazyVertices converts a vertex to its exact rational form the first time a
// triangle needs it. The float filter rejects most triangles before then.
type lazyVertices struct {
	verts []r3.Vec
	exact []ratVec
}

func newLazyVertices(verts []r3.Vec) *lazyVertices {
	return &lazyVertices{verts: verts, exact: make([]ratVec, len(verts))}
}

// at returns vertex i, which must be in range and finite, as exact rationals.
func (l *lazyVertices) at(i int) ratVec {
	if l.exact[i][0] == nil {
		l.exact[i] = exactVec(l.verts[i])
	}
	return l.exact[i]
}

// MeshDistanceSquared returns the exact squared distance from p to the held
// triangle set, which may be a selected face patch. All coordinates are finite
// float64 values interpreted as exact dyadics. The caller must provide a
// verified embedded mesh or a subset of its triangles.
func MeshDistanceSquared(ctx context.Context, p r3.Vec, verts []r3.Vec, tris [][3]int) (*big.Rat, error) {
	if len(tris) == 0 {
		return nil, fmt.Errorf(`%w: point query mesh has no triangles`, decaderr.ErrUnsupported)
	}
	budget := proofbound.NewWorkBudget(ctx)
	point := proof.DyVec(p)
	if err := checkFiniteVertices(budget, verts); err != nil {
		return nil, err
	}
	var minimum dyDistance
	haveMinimum := false
	var minimumUpper float64
	for _, tri := range tris {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		for _, index := range tri {
			if index < 0 || index >= len(verts) {
				return nil, fmt.Errorf(`%w: point query mesh has an invalid triangle index`, decaderr.ErrUnsupported)
			}
		}
		if haveMinimum && boxDistanceSquaredLower(p, verts[tri[0]], verts[tri[1]], verts[tri[2]]) >= minimumUpper {
			continue
		}
		a, b, c := proof.DyVec(verts[tri[0]]), proof.DyVec(verts[tri[1]]), proof.DyVec(verts[tri[2]])
		if haveMinimum && proof.DyCmp(proof.DyMul(boxDyDistanceSquared(point, a, b, c), minimum.d), minimum.n) >= 0 {
			continue
		}
		distance, err := triangleDyDistanceSquared(point, a, b, c)
		if err != nil {
			return nil, err
		}
		if !haveMinimum || distance.cmp(minimum) < 0 {
			minimum = distance
			haveMinimum = true
			minimumUpper = proof.DyQuoFloatUp(minimum.n, minimum.d)
		}
	}
	return minimum.rat(), nil
}
