package tessellation

import (
	"context"
	"math"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// ReplayProveStackedConservative charges the stacked volume terms. Its facet
// area uses |u x v| <= |u|_1 |v|_1, so no unmetered dyadic root is needed.
// The caller authenticates the rings, faces, and source table to one body.
func ReplayProveStackedConservative[F comparable](ctx context.Context,
	in StackedProofInput[F], budget *proofbound.ReplayBudget) (StackedProof, error) {
	if budget == nil || len(in.Rings) != len(in.ColumnHeights) ||
		len(in.SlabHeights) != len(in.SlabColumns) || len(in.Triangles) != len(in.Source) ||
		len(in.Rings) > 4096 || len(in.Faces) > 4096 || len(in.Triangles) > 1<<18 ||
		in.IsPlanar == nil || proofbound.IsNonFinite(in.SectionDelta) || in.SectionDelta < 0 ||
		proofbound.IsNonFinite(in.StoreMax) || in.StoreMax < 0 ||
		proofbound.IsNonFinite(in.AreaSlack) || in.AreaSlack < 0 {
		return StackedProof{}, decaderr.ErrUnsupported
	}
	termCap := 2*len(in.SlabHeights) + len(in.Faces) + 1
	if err := budget.ReserveArray(ctx, uint64(termCap), uint64(unsafe.Sizeof(float64(0)))); err != nil {
		return StackedProof{}, err
	}
	out := StackedProof{AreaSlack: in.AreaSlack, Terms: make([]float64, 0, termCap)}
	allWalks, allPerimeter := 0, 0.0
	for ci, ring := range in.Rings {
		if err := budget.SpendWork(ctx, 24); err != nil {
			return StackedProof{}, err
		}
		if ring.Walks < 0 || proofbound.IsNonFinite(ring.PerimeterUpper) || ring.PerimeterUpper < 0 ||
			proofbound.IsNonFinite(ring.SegmentArea) || ring.SegmentArea < 0 ||
			proofbound.IsNonFinite(in.ColumnHeights[ci]) || in.ColumnHeights[ci] < 0 {
			return StackedProof{}, decaderr.ErrUnsupported
		}
		allWalks += ring.Walks
		allPerimeter = proofbound.AbsSumUpper(allPerimeter, ring.PerimeterUpper)
		if in.SectionDelta > 0 {
			out.AreaSlack = proofbound.AbsSumUpper(out.AreaSlack,
				proofbound.ProductUpper(proofbound.SectionDisplacementLength(in.SectionDelta, ring.Walks),
					in.ColumnHeights[ci]))
		}
	}
	for _, face := range in.Faces {
		if err := budget.SpendWork(ctx, 8); err != nil {
			return StackedProof{}, err
		}
		if in.IsPlanar(face) && in.SectionDelta > 0 {
			out.AreaSlack = proofbound.AbsSumUpper(out.AreaSlack,
				proofbound.SectionDisplacementArea(in.SectionDelta, allWalks, allPerimeter))
		}
	}
	if err := budget.ReserveArray(ctx, uint64(len(in.Faces)), uint64(unsafe.Sizeof(float64(0)))); err != nil {
		return StackedProof{}, err
	}
	storeArea, faceArea, totalArea := 0.0, make([]float64, len(in.Faces)), 0.0
	for i, tri := range in.Triangles {
		if err := budget.SpendWork(ctx, uint64(128+8*len(in.Faces))); err != nil {
			return StackedProof{}, err
		}
		for _, v := range tri {
			if v < 0 || v >= len(in.Vertices) || v >= len(in.Store) ||
				!proofbound.FiniteVec(in.Vertices[v]) ||
				proofbound.IsNonFinite(in.Store[v]) || in.Store[v] < 0 {
				return StackedProof{}, decaderr.ErrUnsupported
			}
		}
		a, b, c := in.Vertices[tri[0]], in.Vertices[tri[1]], in.Vertices[tri[2]]
		lu, lv := replayFacetEdgeL1(a, b), replayFacetEdgeL1(a, c)
		held := proofbound.ProductUpper(.5, proofbound.ProductUpper(lu, lv))
		local := math.Max(in.Store[tri[0]], math.Max(in.Store[tri[1]], in.Store[tri[2]]))
		if local > in.StoreMax {
			return StackedProof{}, decaderr.ErrUnsupported
		}
		storeArea = proofbound.AbsSumUpper(storeArea, replayFacetAllow(local, lu, lv))
		area := proofbound.AbsSumUpper(held, replayFacetAllow(local, lu, lv))
		totalArea = proofbound.AbsSumUpper(totalArea, held,
			replayFacetAllow(in.StoreMax, lu, lv))
		found := false
		for j, face := range in.Faces {
			if in.Source[i] == face {
				faceArea[j] = proofbound.AbsSumUpper(faceArea[j], area)
				found = true
				break
			}
		}
		if !found {
			return StackedProof{}, decaderr.ErrUnsupported
		}
	}
	out.AreaSlack = proofbound.AbsSumUpper(out.AreaSlack, storeArea)
	for k, columns := range in.SlabColumns {
		if err := budget.SpendWork(ctx, 16); err != nil {
			return StackedProof{}, err
		}
		if proofbound.IsNonFinite(in.SlabHeights[k]) || in.SlabHeights[k] < 0 {
			return StackedProof{}, decaderr.ErrUnsupported
		}
		segments, perimeter, walks := 0.0, 0.0, 0
		for _, column := range columns {
			if err := budget.SpendWork(ctx, 16); err != nil {
				return StackedProof{}, err
			}
			if column < 0 || column >= len(in.Rings) {
				return StackedProof{}, decaderr.ErrUnsupported
			}
			ring := in.Rings[column]
			segments = proofbound.AbsSumUpper(segments, ring.SegmentArea)
			perimeter = proofbound.AbsSumUpper(perimeter, ring.PerimeterUpper)
			walks += ring.Walks
		}
		height := in.SlabHeights[k]
		out.Terms = append(out.Terms, proofbound.ProductUpper(height, segments),
			proofbound.ProductUpper(height,
				proofbound.SectionDisplacementArea(in.SectionDelta, walks, perimeter)))
	}
	for j, face := range in.Faces {
		if err := budget.SpendWork(ctx, 16); err != nil {
			return StackedProof{}, err
		}
		axial, found := in.FaceAxial[face]
		if found && in.IsPlanar(face) {
			if proofbound.IsNonFinite(axial) || axial < 0 {
				return StackedProof{}, decaderr.ErrUnsupported
			}
			out.Terms = append(out.Terms, proofbound.ProductUpper(axial, faceArea[j]))
		}
	}
	out.Terms = append(out.Terms, proofbound.SweptVolumeAllow(in.StoreMax, totalArea))
	if proofbound.IsNonFinite(out.AreaSlack) || proofbound.IsNonFinite(totalArea) {
		return StackedProof{}, decaderr.ErrUnsupported
	}
	return out, nil
}

func replayFacetEdgeL1(a, b r3.Vec) float64 {
	return proofbound.AbsSumUpper(a.X, b.X, a.Y, b.Y, a.Z, b.Z)
}

func replayFacetAllow(delta, lu, lv float64) float64 {
	return proofbound.AbsSumUpper(
		proofbound.ProductUpper(delta, proofbound.AbsSumUpper(lu, lv)),
		proofbound.ProductUpper(2, proofbound.ProductUpper(delta, delta)))
}
