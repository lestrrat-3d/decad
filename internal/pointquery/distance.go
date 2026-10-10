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

func segmentDistanceSquared(p, a, b ratVec) *big.Rat {
	axis := subtract(b, a)
	lengthSquared := dot(axis, axis)
	fromStart := subtract(p, a)
	if lengthSquared.Sign() == 0 {
		return dot(fromStart, fromStart)
	}
	along := dot(fromStart, axis)
	if along.Sign() <= 0 {
		return dot(fromStart, fromStart)
	}
	if along.Cmp(lengthSquared) >= 0 {
		fromEnd := subtract(p, b)
		return dot(fromEnd, fromEnd)
	}
	parallelSquared := new(big.Rat).Quo(new(big.Rat).Mul(along, along), lengthSquared)
	return new(big.Rat).Sub(dot(fromStart, fromStart), parallelSquared)
}

func triangleDistanceSquared(p, a, b, c ratVec) (*big.Rat, error) {
	normal := cross(subtract(b, a), subtract(c, a))
	normalSquared := dot(normal, normal)
	if normalSquared.Sign() == 0 {
		return nil, fmt.Errorf(`%w: point query mesh contains a degenerate triangle`, decaderr.ErrUnsupported)
	}
	triangle := [3]ratVec{a, b, c}
	projectionInside := true
	for i := range triangle {
		start, end := triangle[i], triangle[(i+1)%3]
		side := dot(cross(subtract(end, start), subtract(p, start)), normal)
		if side.Sign() < 0 {
			projectionInside = false
			break
		}
	}
	if projectionInside {
		offset := dot(subtract(p, a), normal)
		return new(big.Rat).Quo(new(big.Rat).Mul(offset, offset), normalSquared), nil
	}
	var minimum *big.Rat
	for i := range triangle {
		distance := segmentDistanceSquared(p, triangle[i], triangle[(i+1)%3])
		if minimum == nil || distance.Cmp(minimum) < 0 {
			minimum = distance
		}
	}
	return minimum, nil
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

// MeshDistanceSquared returns the exact squared distance from p to the held
// triangle set, which may be a selected face patch. All coordinates are finite
// float64 values interpreted as exact dyadics. The caller must provide a
// verified embedded mesh or a subset of its triangles.
func MeshDistanceSquared(ctx context.Context, p r3.Vec, verts []r3.Vec, tris [][3]int) (*big.Rat, error) {
	if len(tris) == 0 {
		return nil, fmt.Errorf(`%w: point query mesh has no triangles`, decaderr.ErrUnsupported)
	}
	budget := proofbound.NewWorkBudget(ctx)
	point := exactVec(p)
	converted := make([]ratVec, len(verts))
	for i, v := range verts {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		if !proofbound.FiniteVec(v) {
			return nil, fmt.Errorf(`%w: point query mesh has a non-finite vertex`, decaderr.ErrUnsupported)
		}
		converted[i] = exactVec(v)
	}
	var minimum *big.Rat
	for _, tri := range tris {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		for _, index := range tri {
			if index < 0 || index >= len(converted) {
				return nil, fmt.Errorf(`%w: point query mesh has an invalid triangle index`, decaderr.ErrUnsupported)
			}
		}
		a, b, c := converted[tri[0]], converted[tri[1]], converted[tri[2]]
		if minimum != nil && boxDistanceSquared(point, a, b, c).Cmp(minimum) >= 0 {
			continue
		}
		distance, err := triangleDistanceSquared(point, a, b, c)
		if err != nil {
			return nil, err
		}
		if minimum == nil || distance.Cmp(minimum) < 0 {
			minimum = distance
		}
	}
	return minimum, nil
}
