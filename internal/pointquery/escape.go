package pointquery

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// This file proves a point outside a body by a free path (agent-docs/point-containment-design.md §2, "Free path"):
// a chain of overlapping balls, each clear of the held boundary by more than the boundary bound B, from the point to
// beyond the held mesh's box widened by B. Neither the held boundary nor the true one meets the chain, so both
// solids give the point the membership they give the chain's far end, which lies outside both.

const (
	// maxEscapeSteps is the most balls one direction's chain takes.
	maxEscapeSteps = 64
	// maxEscapeScans is the most exact mesh distances one query spends on chains.
	maxEscapeScans = 128
	// escapeShrink abandons a direction whose ball shrinks below this fraction of the first one: the chain then
	// runs into the boundary, and its balls would only shrink further.
	escapeShrink = 1.0 / 8
)

// clearRadius is a lower bound on L − B, L the distance whose square is distanceSquared and B the boundary bound.
// Both roundings point downward.
func clearRadius(distanceSquared *big.Rat, boundaryBound float64) float64 {
	return proof.NextDown(proofbound.RatSqrtDown(distanceSquared) - boundaryBound)
}

// escapeOutside reports whether a free path proves p outside the held solid and the true one. distanceSquared is
// p's exact squared distance to the held mesh, and boundaryBound the two-sided Hausdorff bound B between the held
// and the true boundaries.
func escapeOutside(ctx context.Context, p r3.Vec, verts []r3.Vec, tris [][3]int, boundaryBound float64,
	distanceSquared *big.Rat) (bool, error) {
	r0 := clearRadius(distanceSquared, boundaryBound)
	if !(r0 > 0) || len(verts) == 0 {
		return false, nil
	}
	lo, hi := verts[0], verts[0]
	for _, v := range verts[1:] {
		lo = r3.NewVec(math.Min(lo.X, v.X), math.Min(lo.Y, v.Y), math.Min(lo.Z, v.Z))
		hi = r3.NewVec(math.Max(hi.X, v.X), math.Max(hi.Y, v.Y), math.Max(hi.Z, v.Z))
	}
	b := proof.FloatRat(boundaryBound)
	beyond := func(q r3.Vec) bool {
		for _, c := range [3][3]float64{{q.X, lo.X, hi.X}, {q.Y, lo.Y, hi.Y}, {q.Z, lo.Z, hi.Z}} {
			x := proof.FloatRat(c[0])
			if x.Cmp(new(big.Rat).Sub(proof.FloatRat(c[1]), b)) < 0 || x.Cmp(new(big.Rat).Add(proof.FloatRat(c[2]), b)) > 0 {
				return true
			}
		}
		return false
	}
	scans := 0
	for _, d := range escapeDirections(p, verts, tris) {
		q, r := p, r0
		for range maxEscapeSteps {
			if beyond(q) {
				return true, nil
			}
			if scans >= maxEscapeScans {
				return false, nil
			}
			next := q.Add(d.Scale(r * (1 - 0x1p-10)))
			ds, err := MeshDistanceSquared(ctx, next, verts, tris)
			if err != nil {
				return false, err
			}
			scans++
			rn := clearRadius(ds, boundaryBound)
			if !(rn > r0*escapeShrink) {
				break
			}
			// The two balls overlap: |next − q| < r + rn, compared exactly.
			gap := subtract(exactVec(next), exactVec(q))
			reach := new(big.Rat).Add(proof.FloatRat(r), proof.FloatRat(rn))
			if dot(gap, gap).Cmp(new(big.Rat).Mul(reach, reach)) >= 0 {
				break
			}
			q, r = next, rn
		}
	}
	return false, nil
}

// escapeDirections are the unit directions a free path tries from p: the six axis directions, and both ways along
// each edge of the held triangle nearest p, which follows a bore's or a slot's wall out of it. The choice only
// steers the search; every chain is checked exactly.
func escapeDirections(p r3.Vec, verts []r3.Vec, tris [][3]int) []r3.Vec {
	var dirs []r3.Vec
	nearest, best := -1, math.Inf(1)
	for i, t := range tris {
		if lb := boxDistanceSquaredLower(p, verts[t[0]], verts[t[1]], verts[t[2]]); lb < best {
			nearest, best = i, lb
		}
	}
	if nearest >= 0 {
		t := tris[nearest]
		for k := range 3 {
			if d, ok := verts[t[(k+1)%3]].Sub(verts[t[k]]).Normalize(); ok {
				dirs = append(dirs, d, d.Scale(-1))
			}
		}
	}
	return append(dirs, []r3.Vec{{X: 1}, {X: -1}, {Y: 1}, {Y: -1}, {Z: 1}, {Z: -1}}...)
}
