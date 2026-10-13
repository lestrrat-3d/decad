package proofbound

import "github.com/lestrrat-3d/r3"

// PerturbedTriangleTermsForTest exposes perturbedTriangleTerms: per triangle
// the held area, lu, lv, and whether the triangle was measurable.
func PerturbedTriangleTermsForTest(
	budget *WorkBudget,
	verts []r3.Vec,
	tris [][3]int,
	workers int,
) ([][3]float64, []bool, error) {
	terms, err := perturbedTriangleTerms(budget, verts, tris, workers)
	if err != nil {
		return nil, nil, err
	}
	vals := make([][3]float64, len(terms))
	oks := make([]bool, len(terms))
	for i, term := range terms {
		vals[i] = [3]float64{term.area, term.lu, term.lv}
		oks[i] = term.ok
	}
	return vals, oks, nil
}
