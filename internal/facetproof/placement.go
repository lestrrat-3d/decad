package facetproof

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// PlacedMesh carries the held geometry and bounds after a faceted placement.
type PlacedMesh struct {
	Verts       []r3.Vec
	Tris        [][3]int
	VertexBound []float64
	MeshBound   float64
	VolSymDiff  float64
	AreaSlack   float64
}

// PlaceMesh rounds each exact affine image to its nearest float, keeps its
// held facets embedded, and charges the stored basis's exact stretch and
// determinant before each vertex's own rounding and embedding correction.
func PlaceMesh(ctx context.Context, budget *proofbound.WorkBudget, verts []r3.Vec, tris [][3]int,
	vertexBound []float64, volSymDiff, areaSlack, stretch, volumeCharge float64,
	delta r3.Transform) (PlacedMesh, error) {
	if proofbound.IsNonFinite(volSymDiff) || proofbound.IsNonFinite(areaSlack) ||
		proofbound.IsNonFinite(stretch) || proofbound.IsNonFinite(volumeCharge) ||
		volSymDiff < 0 || areaSlack < 0 || stretch < 0 || volumeCharge < 0 ||
		len(verts) != len(vertexBound) {
		return PlacedMesh{}, fmt.Errorf("%w: placement has no finite source proof", decaderr.ErrUnsupported)
	}
	next := PlacedMesh{Verts: make([]r3.Vec, len(verts)), Tris: tris}
	tr := delta.Translation()
	maxTrans := math.Max(math.Abs(tr.X), math.Max(math.Abs(tr.Y), math.Abs(tr.Z)))
	vertexAllow := make([]float64, len(verts))
	exact := make([]proof.Xpt, len(verts))
	basis := delta.Basis()
	for i, v := range verts {
		if err := budget.Step(); err != nil {
			return PlacedMesh{}, err
		}
		vertexAllow[i] = proofbound.RigidRoundAllow(math.Max(math.Abs(v.X), math.Max(math.Abs(v.Y), math.Abs(v.Z))), maxTrans)
		exact[i] = meshbool.ExactRigidImage(basis, tr, v)
		next.Verts[i] = exact[i].Vec()
		if !proofbound.FiniteVec(next.Verts[i]) {
			return PlacedMesh{}, fmt.Errorf("%w: a placed vertex is not representable", decaderr.ErrUnsupported)
		}
	}
	if err := budget.Err(); err != nil {
		return PlacedMesh{}, err
	}
	if delta.IsReflection() {
		next.Tris = make([][3]int, len(tris))
		for i, t := range tris {
			if err := budget.Step(); err != nil {
				return PlacedMesh{}, err
			}
			next.Tris[i] = [3]int{t[0], t[2], t[1]}
		}
	}
	moved, err := KeepPlacedEmbedded(ctx, exact, next.Verts, next.Tris)
	if err != nil {
		return PlacedMesh{}, err
	}
	allow := 0.0
	next.VertexBound = make([]float64, len(vertexBound))
	lengthStretch := proofbound.AbsSumUpper(1, stretch)
	for i, beta := range vertexBound {
		a := vertexAllow[i]
		if moved != nil {
			a = math.Max(a, moved[i])
		}
		allow = math.Max(allow, a)
		next.VertexBound[i] = proofbound.AbsSumUpper(proofbound.ProductUpper(lengthStretch, beta), a)
	}
	next.MeshBound = FacetBoundMax(next.Tris, next.VertexBound)
	areaUpper, err := proofbound.PerturbedAreaUpperContext(ctx, next.Verts, next.Tris, allow)
	if err != nil {
		return PlacedMesh{}, err
	}
	next.VolSymDiff = proofbound.AbsSumUpper(
		proofbound.ProductUpper(proofbound.AbsSumUpper(1, volumeCharge), volSymDiff),
		proofbound.SweptVolumeAllow(allow, areaUpper))
	areaRound := 0.0
	for _, tri := range next.Tris {
		if err := budget.Step(); err != nil {
			return PlacedMesh{}, err
		}
		a, b, c := next.Verts[tri[0]], next.Verts[tri[1]], next.Verts[tri[2]]
		u, v := proofbound.HeldDelta(b, a), proofbound.HeldDelta(c, a)
		term := proofbound.AbsSumUpper(
			proofbound.ProductUpper(allow,
				proofbound.AbsSumUpper(proofbound.DvLenUpper(u), proofbound.DvLenUpper(v))),
			proofbound.ProductUpper(2, proofbound.ProductUpper(allow, allow)))
		areaRound = proofbound.AbsSumUpper(areaRound, term)
	}
	// True and held tangent planes can gain opposite area changes under the
	// same nearly orthonormal map. Charge each against its own area: if their
	// old difference is E and held area is H, the new difference is at most
	// (1+e)E + 2eH before held-coordinate rounding.
	heldArea := MeshAreaUpper(verts, tris)
	next.AreaSlack = proofbound.AbsSumUpper(areaSlack,
		proofbound.ProductUpper(stretch, areaSlack),
		proofbound.ProductUpper(2, proofbound.ProductUpper(stretch, heldArea)), areaRound)
	if proofbound.IsNonFinite(next.MeshBound) || proofbound.IsNonFinite(next.VolSymDiff) ||
		proofbound.IsNonFinite(next.AreaSlack) {
		return PlacedMesh{}, fmt.Errorf("%w: placement has no finite image proof", decaderr.ErrUnsupported)
	}
	return next, nil
}

// FacetBoundMax is the largest held facet bound over the mesh.
func FacetBoundMax(tris [][3]int, beta []float64) float64 {
	out := 0.0
	for _, t := range tris {
		out = max(out, beta[t[0]], beta[t[1]], beta[t[2]])
	}
	return out
}
