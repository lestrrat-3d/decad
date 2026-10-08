package tessellation

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// StoreMax folds per-vertex store displacements and refuses a sample whose
// recorded point has no finite enclosure.
func StoreMax(store []float64) (float64, error) {
	worst := 0.0
	for _, d := range store {
		if proofbound.IsNonFinite(d) {
			return 0, fmt.Errorf(`%w: a chorded boundary sample states no enclosure of the point its own record denotes, so this mesh can publish no displacement bound for the faces that meet it`, decaderr.ErrUnsupported)
		}
		worst = math.Max(worst, d)
	}
	return worst, nil
}

// FaceBounds composes each source face's trim, store, section and axial
// displacements over the facets that name it.
func FaceBounds[F comparable](triangles [][3]int, source []F, trim, axial map[F]float64,
	store []float64, section float64) (map[F]float64, error) {
	faceStore := map[F]float64{}
	for i, f := range source {
		for _, v := range triangles[i] {
			faceStore[f] = math.Max(faceStore[f], store[v])
		}
	}
	bounds := make(map[F]float64, len(faceStore))
	for f, s := range faceStore {
		bound := proofbound.UpRound(trim[f] + s + section + axial[f])
		if proofbound.IsNonFinite(bound) {
			return nil, fmt.Errorf(`%w: a face's composed displacement is not finite, so this mesh can state no bound for it`, decaderr.ErrUnsupported)
		}
		bounds[f] = bound
	}
	return bounds, nil
}

// StoreAreaAllow sums the coordinate-movement area allowance per facet.
func StoreAreaAllow(vertices []r3.Vec, triangles [][3]int, store []float64) float64 {
	total := 0.0
	for _, tri := range triangles {
		d := math.Max(store[tri[0]], math.Max(store[tri[1]], store[tri[2]]))
		if d <= 0 {
			continue
		}
		a, b, c := vertices[tri[0]], vertices[tri[1]], vertices[tri[2]]
		total = proofbound.AbsSumUpper(total, proofbound.PerturbedTriangleAreaAllow(a, b, c, d))
	}
	return total
}

// FaceAreaUpper bounds each source face's true patch area using its held
// facets and their per-vertex store displacements.
func FaceAreaUpper[F comparable](vertices []r3.Vec, triangles [][3]int, source []F,
	store []float64) map[F]float64 {
	out := map[F]float64{}
	for i, f := range source {
		tri := triangles[i]
		a, b, c := vertices[tri[0]], vertices[tri[1]], vertices[tri[2]]
		d := math.Max(store[tri[0]], math.Max(store[tri[1]], store[tri[2]]))
		held := b.Sub(a).Cross(c.Sub(a)).Len() / 2
		out[f] = proofbound.AbsSumUpper(out[f], held, proofbound.PerturbedTriangleAreaAllow(a, b, c, d))
	}
	return out
}

// SymDiff sums an analytic payload's occupied-volume terms and refuses a
// non-finite proof.
func SymDiff(terms []float64) (float64, error) {
	total := proofbound.AbsSumUpper(terms...)
	if proofbound.IsNonFinite(total) {
		return 0, fmt.Errorf(`%w: this mesh states no finite bound on the volume it and the body it stands for differ by, so no boolean may consume it`, decaderr.ErrUnsupported)
	}
	return total, nil
}
