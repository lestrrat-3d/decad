package facemesh

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
)

// ErrDropped marks a triangulation that left a loop point out of every triangle (trim TG5). It wraps
// decaderr.ErrUnsupported.
var ErrDropped = fmt.Errorf("%w: the triangulation dropped a boundary point", decaderr.ErrUnsupported)

// ErrMargin marks a triangle that fails the orientation margin (trim TG5). It wraps decaderr.ErrUnsupported.
var ErrMargin = fmt.Errorf("%w: a triangle fails the orientation margin", decaderr.ErrUnsupported)

// Loop is one boundary loop: held points with their bounds, in the face's walk order (outer counter-clockwise
// about the outward normal, holes clockwise).
type Loop struct {
	Points []r3.Vec
	Bounds []float64
}

func ratOfVec(v r3.Vec) ([3]*big.Rat, bool) {
	x, okx := proofbound.RatOf(v.X)
	y, oky := proofbound.RatOf(v.Y)
	z, okz := proofbound.RatOf(v.Z)
	return [3]*big.Rat{x, y, z}, okx && oky && okz
}

func sub(a, b [3]*big.Rat) [3]*big.Rat {
	return [3]*big.Rat{new(big.Rat).Sub(a[0], b[0]), new(big.Rat).Sub(a[1], b[1]), new(big.Rat).Sub(a[2], b[2])}
}

func cross(a, b [3]*big.Rat) [3]*big.Rat {
	c := func(i, j int) *big.Rat {
		return new(big.Rat).Sub(new(big.Rat).Mul(a[i], b[j]), new(big.Rat).Mul(a[j], b[i]))
	}
	return [3]*big.Rat{c(1, 2), c(2, 0), c(0, 1)}
}

func dot(a, b [3]*big.Rat) *big.Rat {
	s := new(big.Rat)
	for i := range a {
		s.Add(s, new(big.Rat).Mul(a[i], b[i]))
	}
	return s
}

// lenUp is an upper bound on |a| as an exact rational (a float whose square is at least a·a).
func lenUp(a [3]*big.Rat) (*big.Rat, bool) {
	return proofbound.RatOf(proofbound.RatSqrtUp(dot(a, a)))
}

// Planar triangulates a planar face. n is the exact outward normal (anasurf.Plane.N); u, v is the tag frame used only
// to project for internal/triangulation.Triangulate. When (u×v)·n < 0 it projects with (v, u) instead, so the outer
// loop is counter-clockwise in the projection, as Triangulate requires. Indices refer to the concatenated loop
// points. It refuses
// (ErrUnsupported, TG5) when Triangulate drops a loop point, or when a triangle (a, b, c) fails the orientation
// margin: s = n·((b−a)×(c−a)) exact must satisfy s > |n|_up·(2β(|b−a|_up + |c−a|_up) + 4β²), β the largest of its
// corner bounds.
func Planar(ctx context.Context, n [3]*big.Rat, u, v r3.Vec, loops []Loop) ([][3]int, error) {
	ur, okU := ratOfVec(u)
	vr, okV := ratOfVec(v)
	if !okU || !okV || len(loops) == 0 {
		return nil, fmt.Errorf("%w: a face has no usable frame or boundary", decaderr.ErrUnsupported)
	}
	if dot(cross(ur, vr), n).Sign() < 0 {
		u, v = v, u
	}
	var pts3 []r3.Vec
	var beta []float64
	var pts2 []triangulation.Point2
	idx := make([][]int, len(loops))
	for li, loop := range loops {
		if len(loop.Points) != len(loop.Bounds) || len(loop.Points) < 3 {
			return nil, fmt.Errorf("%w: a face loop has too few points", decaderr.ErrUnsupported)
		}
		for pi, p := range loop.Points {
			idx[li] = append(idx[li], len(pts3))
			pts3 = append(pts3, p)
			beta = append(beta, loop.Bounds[pi])
			pts2 = append(pts2, triangulation.Point2{U: p.Dot(u), V: p.Dot(v)})
		}
	}
	tris, err := triangulation.Triangulate(ctx, pts2, idx)
	if err != nil {
		return nil, err
	}
	if err := checkCoverage(len(pts3), tris); err != nil {
		return nil, err
	}
	nUp, ok := lenUp(n)
	if !ok {
		return nil, fmt.Errorf("%w: the face normal has no bound", decaderr.ErrUnsupported)
	}
	for k, t := range tris {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		if err := checkMargin(n, nUp, pts3, beta, t); err != nil {
			return nil, fmt.Errorf("%w: triangle %d", err, k)
		}
	}
	return tris, nil
}

// checkCoverage refuses a triangulation that leaves a loop point out of every triangle.
func checkCoverage(points int, tris [][3]int) error {
	used := make([]bool, points)
	for _, t := range tris {
		for _, i := range t {
			used[i] = true
		}
	}
	for i, ok := range used {
		if !ok {
			return fmt.Errorf("%w: point %d", ErrDropped, i)
		}
	}
	return nil
}

// checkMargin applies the orientation margin of Planar to one triangle.
func checkMargin(n [3]*big.Rat, nUp *big.Rat, pts []r3.Vec, beta []float64, t [3]int) error {
	a, okA := ratOfVec(pts[t[0]])
	b, okB := ratOfVec(pts[t[1]])
	c, okC := ratOfVec(pts[t[2]])
	if !okA || !okB || !okC {
		return ErrMargin
	}
	ab, ac := sub(b, a), sub(c, a)
	s := dot(n, cross(ab, ac))
	bmax := max(beta[t[0]], beta[t[1]], beta[t[2]])
	br, okBeta := proofbound.RatOf(bmax)
	abUp, ok1 := lenUp(ab)
	acUp, ok2 := lenUp(ac)
	if !okBeta || !ok1 || !ok2 || bmax < 0 {
		return ErrMargin
	}
	// 2β(|b−a| + |c−a|) + 4β²
	need := new(big.Rat).Add(abUp, acUp)
	need.Mul(need, br)
	need.Mul(need, big.NewRat(2, 1))
	need.Add(need, new(big.Rat).Mul(big.NewRat(4, 1), new(big.Rat).Mul(br, br)))
	need.Mul(need, nUp)
	if s.Cmp(need) <= 0 {
		return ErrMargin
	}
	return nil
}
