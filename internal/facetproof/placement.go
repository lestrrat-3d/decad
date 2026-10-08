package facetproof

import (
	"context"
	"math"

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
}

// PlaceMesh applies the delta placement, keeps its held facets embedded, and
// charges each vertex's own input rounding and any embedding correction.
func PlaceMesh(ctx context.Context, budget *proofbound.WorkBudget, verts []r3.Vec, tris [][3]int,
	vertexBound []float64, volSymDiff float64, delta r3.Transform) (PlacedMesh, error) {
	next := PlacedMesh{Verts: make([]r3.Vec, len(verts)), Tris: tris}
	tr := delta.Translation()
	maxTrans := math.Max(math.Abs(tr.X), math.Max(math.Abs(tr.Y), math.Abs(tr.Z)))
	vertexAllow := make([]float64, len(verts))
	for i, v := range verts {
		if err := budget.Step(); err != nil {
			return PlacedMesh{}, err
		}
		vertexAllow[i] = proofbound.RigidRoundAllow(math.Max(math.Abs(v.X), math.Max(math.Abs(v.Y), math.Abs(v.Z))), maxTrans)
		next.Verts[i] = delta.Apply(v)
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
	moved, err := KeepPlacedEmbedded(ctx, verts, next.Verts, next.Tris, delta)
	if err != nil {
		return PlacedMesh{}, err
	}
	allow := 0.0
	next.VertexBound = make([]float64, len(vertexBound))
	for i, beta := range vertexBound {
		a := vertexAllow[i]
		if moved != nil {
			a = math.Max(a, moved[i])
		}
		allow = math.Max(allow, a)
		next.VertexBound[i] = proofbound.AbsSumUpper(beta, a)
	}
	next.MeshBound = FacetBoundMax(next.Tris, next.VertexBound)
	areaUpper, err := proofbound.PerturbedAreaUpperContext(ctx, next.Verts, next.Tris, allow)
	if err != nil {
		return PlacedMesh{}, err
	}
	next.VolSymDiff = proofbound.AbsSumUpper(volSymDiff, proofbound.SweptVolumeAllow(allow, areaUpper))
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
