package planarsnapshot

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/pair/planar"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// FacetedInput contains the held mesh and optional exact source of a Boolean body.
type FacetedInput struct {
	Verts            []r3.Vec
	Tris             [][3]int
	FaceOf           []int
	ExactSourceVerts []r3.Vec
	ExactSourceTris  [][3]int
	MeshBound        float64
	VolSymDiff       float64
	Transform        r3.Transform
}

// FacetedSolid reads the exact source under a translation-only placement when
// its held vertices certify that source; otherwise it reads the held mesh.
func FacetedSolid(budget *proofbound.WorkBudget, in FacetedInput) (planar.PlanarSolid, proofarith.Dyadic, bool, error) {
	none := proofarith.DyZero()
	if len(in.Verts) == 0 || len(in.Tris) == 0 {
		return planar.PlanarSolid{}, none, false, nil
	}
	if in.MeshBound != 0 || in.VolSymDiff != 0 {
		if math.IsNaN(in.MeshBound) || math.IsInf(in.MeshBound, 0) || in.MeshBound < 0 {
			return planar.PlanarSolid{}, none, false, nil
		}
		solid, ok, err := facetedSourceSolid(budget, in)
		if err != nil || ok {
			return solid, none, ok, err
		}
		if in.MeshBound == 0 {
			return planar.PlanarSolid{}, none, false, nil
		}
	}
	solid, ok, err := HeldSolid(budget, in.Verts, in.Tris, in.FaceOf)
	if err != nil || !ok {
		return planar.PlanarSolid{}, none, false, err
	}
	return solid, proofarith.MustDyOf(in.MeshBound), true, nil
}

func facetedSourceSolid(budget *proofbound.WorkBudget, in FacetedInput) (planar.PlanarSolid, bool, error) {
	if !translationOnly(in.Transform) || len(in.ExactSourceVerts) != len(in.Verts) ||
		len(in.ExactSourceTris) != len(in.Tris) {
		return planar.PlanarSolid{}, false, nil
	}
	for i, tri := range in.Tris {
		if tri != in.ExactSourceTris[i] {
			return planar.PlanarSolid{}, false, nil
		}
	}
	solid, ok, err := HeldSolid(budget, in.ExactSourceVerts, in.Tris, in.FaceOf)
	if err != nil || !ok {
		return planar.PlanarSolid{}, false, err
	}
	bound := proofarith.MustDyOf(in.MeshBound)
	for i := range solid.Verts {
		if err := budget.Step(); err != nil {
			return planar.PlanarSolid{}, false, err
		}
		if !proofbound.FiniteVec(in.Verts[i]) {
			return planar.PlanarSolid{}, false, nil
		}
		solid.Verts[i] = transformPoint(in.Transform, solid.Verts[i])
		difference := proofarith.DvSub(solid.Verts[i], proofarith.DyVec(in.Verts[i]))
		if proofarith.DyCmp(proofarith.DvDot(difference, difference), proofarith.DyMul(bound, bound)) > 0 {
			return planar.PlanarSolid{}, false, nil
		}
	}
	return solid, true, nil
}

// HeldSolid lifts a held triangle mesh to an exact snapshot. A face map with
// the wrong length leaves Faces unset.
func HeldSolid(budget *proofbound.WorkBudget, verts []r3.Vec, tris [][3]int,
	faceOf []int) (planar.PlanarSolid, bool, error) {
	solid := planar.PlanarSolid{Verts: make([]proofarith.DyV3, len(verts)),
		Tris: append([][3]int(nil), tris...)}
	if len(faceOf) == len(tris) {
		solid.Faces = append([]int(nil), faceOf...)
	}
	for i, v := range verts {
		if err := budget.Step(); err != nil {
			return planar.PlanarSolid{}, false, err
		}
		if !proofbound.FiniteVec(v) {
			return planar.PlanarSolid{}, false, nil
		}
		solid.Verts[i] = proofarith.DyVec(v)
	}
	return solid, true, nil
}

func translationOnly(t r3.Transform) bool {
	return t.IsValid() && proofbound.FiniteVec(t.Translation()) && t.Basis() == r3.Identity().Basis()
}

func scaleVec(v proofarith.DyV3, s proofarith.Dyadic) proofarith.DyV3 {
	return proofarith.DyV3{proofarith.DyMul(v[0], s), proofarith.DyMul(v[1], s), proofarith.DyMul(v[2], s)}
}

func transformPoint(t r3.Transform, p proofarith.DyV3) proofarith.DyV3 {
	b := t.Basis()
	return proofarith.DvAdd(proofarith.DyVec(t.Translation()), proofarith.DvAdd(scaleVec(proofarith.DyVec(b.EX), p[0]),
		proofarith.DvAdd(scaleVec(proofarith.DyVec(b.EY), p[1]), scaleVec(proofarith.DyVec(b.EZ), p[2]))))
}
