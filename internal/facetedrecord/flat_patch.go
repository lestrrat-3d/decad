package facetedrecord

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// FlatPatchExtent bounds how far the true patch of a built flat patch f lies from f's Plane tag T: the extent a mesh
// Boolean states as the group's ExtentDelta when it keeps the tag of a face that states a tilt
// (agent-docs/faceted-source-retention-design.md §3.2, A2, Claim F). It is false for any face the claim does not
// cover: one not marked topo.Face.FlatPatch, a retained or denoted face, one that states an AxialDelta, a tilt that
// is not finite and positive, a loop count other than one, or a length, bound or coordinate that is not finite.
//
// Claim F. Write θ = min(1, NormalBound), P = Σ (Length + LengthBound) over f's edges and, for each vertex v of its
// loop, d_v = dist(v, T) + Bound(v). Every point of the true patch lies within min_v d_v + θ·P/2 of T.
//
// Proof. The true patch lies in a plane P*, whose unit normal lies within NormalBound of T's (topo.Face.FlatPatch).
// Two unit normals at a distance s make an angle whose sine is at most min(1, s), so on P* the distance to T is an
// affine function whose gradient has length at most θ. An affine function takes its largest value over a planar
// region on the region's boundary. That boundary is one closed curve, the true loop, of length at most P, so every
// point q of it lies within P/2 of the true vertex v* of any v along the curve, and so within P/2 of v* in space.
// Then dist(q, T) ≤ dist(v*, T) + θ·P/2 ≤ dist(v, T) + Bound(v) + θ·P/2. ∎
//
// dist(v, T) is read exactly in rationals from the held position and T's frame normal and origin, and every sum and
// product rounds up.
func FlatPatchExtent(f *topo.Face) (float64, bool) {
	pl, ok := f.Surface.(surfacegeom.Plane)
	if !ok || !f.FlatPatch || f.RetainedTag || f.Denoted != nil || f.HasAxialDelta || len(f.Loops) != 1 {
		return 0, false
	}
	if !(f.NormalBound > 0) || math.IsInf(f.NormalBound, 0) {
		return 0, false
	}
	n, o := pl.Frame.N(), pl.Frame.Origin()
	if !proofbound.FiniteVec(n) || !proofbound.FiniteVec(o) {
		return 0, false
	}
	nr := ratVec(n)
	norm := proofbound.RatSqrtDown(dotRat(nr, nr))
	if !(norm > 0) {
		return 0, false
	}
	or := ratVec(o)
	perimeter := 0.0
	nearest := math.Inf(1)
	for _, ce := range f.Loops[0].CoEdges {
		e := ce.Edge
		if e == nil || e.Start == nil || e.End == nil || e.LengthUnbounded {
			return 0, false
		}
		if !(e.Length >= 0) || !(e.LengthBound >= 0) || math.IsInf(e.Length, 0) || math.IsInf(e.LengthBound, 0) {
			return 0, false
		}
		perimeter = proofbound.AbsSumUpper(perimeter, e.Length, e.LengthBound)
		for _, v := range []*topo.Vertex{e.Start, e.End} {
			bound := v.Bound.Base()
			if !proofbound.FiniteVec(v.Position) || !(bound >= 0) || math.IsInf(bound, 0) {
				return 0, false
			}
			off := dotRat(nr, subRat(ratVec(v.Position), or))
			d := proofbound.DivUpper(proofbound.RatFloatUp(off.Abs(off)), norm)
			nearest = math.Min(nearest, proofbound.AbsSumUpper(d, bound))
		}
	}
	theta := math.Min(1, f.NormalBound)
	extent := proofbound.AbsSumUpper(nearest, proofbound.ProductUpper(theta, proofbound.DivUpper(perimeter, 2)))
	if !(extent > 0) || math.IsInf(extent, 0) {
		return 0, false
	}
	return extent, true
}

func ratVec(v r3.Vec) [3]*big.Rat {
	return [3]*big.Rat{new(big.Rat).SetFloat64(v.X), new(big.Rat).SetFloat64(v.Y), new(big.Rat).SetFloat64(v.Z)}
}

func subRat(a, b [3]*big.Rat) [3]*big.Rat {
	var out [3]*big.Rat
	for i := range out {
		out[i] = new(big.Rat).Sub(a[i], b[i])
	}
	return out
}

func dotRat(a, b [3]*big.Rat) *big.Rat {
	sum := new(big.Rat)
	for i := range a {
		sum.Add(sum, new(big.Rat).Mul(a[i], b[i]))
	}
	return sum
}
