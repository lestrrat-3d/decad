package tessellation

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// StackedRing is one column's shared wall and planar-patch chording.
type StackedRing[F any] struct {
	Samples        []sectionrecord.Point2
	Bottom, Top    []int
	Faces          []F
	Sag            float64
	SegmentArea    float64
	Walks          int
	PerimeterUpper float64
}

// StackedPatchLoop identifies one column ring in a planar patch.
type StackedPatchLoop struct {
	Column      int
	Top, Outer  bool
	ColumnOuter bool
}

// StackedPatchMesh carries the facets and trim displacement of one patch.
type StackedPatchMesh struct {
	Triangles [][3]int
	Sag       float64
}

// StackedSlabClearance checks only the rings sharing one slab's material.
func StackedSlabClearance[F any](ctx context.Context, rings []StackedRing[F], columns []int,
	clearance func(context.Context, []sectionrecord.Point2, [][]int, []float64) error,
) error {
	var points []sectionrecord.Point2
	var indices [][]int
	var sags []float64
	for _, column := range columns {
		ring := rings[column]
		start := len(points)
		points = append(points, ring.Samples...)
		idx := make([]int, len(ring.Samples))
		for i := range idx {
			idx[i] = start + i
		}
		indices = append(indices, idx)
		sags = append(sags, ring.Sag)
	}
	return clearance(ctx, points, indices, sags)
}

// StackedPatchTriangles triangulates a planar patch over its already shared
// ring vertices. A patch joining multiple columns checks their clearance.
func StackedPatchTriangles[F any](ctx context.Context, rings []StackedRing[F], loops []StackedPatchLoop,
	reverse bool,
	clearance func(context.Context, []sectionrecord.Point2, [][]int, []float64) error,
	triangulate func(context.Context, []sectionrecord.Point2, [][]int) ([][3]int, error),
) (StackedPatchMesh, error) {
	var points []sectionrecord.Point2
	var vertices []int
	var indexLoops [][]int
	var sags []float64
	out := StackedPatchMesh{}
	for _, loop := range loops {
		ring := rings[loop.Column]
		start := len(points)
		points = append(points, ring.Samples...)
		if loop.Top {
			vertices = append(vertices, ring.Top...)
		} else {
			vertices = append(vertices, ring.Bottom...)
		}
		idx := make([]int, len(ring.Samples))
		for i := range idx {
			idx[i] = start + i
		}
		if loop.ColumnOuter != loop.Outer {
			for a, z := 0, len(idx)-1; a < z; a, z = a+1, z-1 {
				idx[a], idx[z] = idx[z], idx[a]
			}
		}
		indexLoops = append(indexLoops, idx)
		sags = append(sags, ring.Sag)
		out.Sag = math.Max(out.Sag, ring.Sag)
	}
	if len(loops) > 1 {
		if err := clearance(ctx, points, indexLoops, sags); err != nil {
			return StackedPatchMesh{}, err
		}
	}
	tris, err := triangulate(ctx, points, indexLoops)
	if err != nil {
		return StackedPatchMesh{}, err
	}
	out.Triangles = make([][3]int, len(tris))
	for i, tri := range tris {
		a, b, c := vertices[tri[0]], vertices[tri[1]], vertices[tri[2]]
		if reverse {
			b, c = c, b
		}
		out.Triangles[i] = [3]int{a, b, c}
	}
	return out, nil
}

// StackedProofInput names the column and slab heights and the shared mesh
// record used for a stacked solid's area and occupied-volume proofs.
type StackedProofInput[F comparable] struct {
	Rings         []StackedRing[F]
	ColumnHeights []float64
	SlabHeights   []float64
	SlabColumns   [][]int
	SectionDelta  float64
	Faces, Source []F
	FaceAxial     map[F]float64
	IsPlanar      func(F) bool
	Vertices      []r3.Vec
	Triangles     [][3]int
	Store         []float64
	StoreMax      float64
	AreaSlack     float64
}

// StackedProof carries the completed area slack and the non-cancelling terms
// whose sum bounds the stacked solid's occupied-volume difference.
type StackedProof struct {
	AreaSlack float64
	Terms     []float64
}

// ProveStacked composes the column, slab, level and stored-coordinate terms.
func ProveStacked[F comparable](in StackedProofInput[F]) StackedProof {
	out := StackedProof{AreaSlack: in.AreaSlack}
	var allWalks int
	var allPerimeter float64
	for ci, ring := range in.Rings {
		allWalks += ring.Walks
		allPerimeter = proofbound.AbsSumUpper(allPerimeter, ring.PerimeterUpper)
		if in.SectionDelta > 0 {
			out.AreaSlack = proofbound.AbsSumUpper(out.AreaSlack,
				proofbound.ProductUpper(proofbound.SectionDisplacementLength(in.SectionDelta, ring.Walks),
					in.ColumnHeights[ci]))
		}
	}
	for _, face := range in.Faces {
		if in.IsPlanar(face) && in.SectionDelta > 0 {
			out.AreaSlack = proofbound.AbsSumUpper(out.AreaSlack,
				proofbound.SectionDisplacementArea(in.SectionDelta, allWalks, allPerimeter))
		}
	}
	out.AreaSlack = proofbound.AbsSumUpper(out.AreaSlack,
		StoreAreaAllow(in.Vertices, in.Triangles, in.Store))
	areaUpper := FaceAreaUpper(in.Vertices, in.Triangles, in.Source, in.Store)
	out.Terms = make([]float64, 0, len(in.Rings)*len(in.SlabHeights)+len(in.FaceAxial)+2)
	for k, columns := range in.SlabColumns {
		var segments, perimeter float64
		walks := 0
		for _, column := range columns {
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
	for face, axial := range in.FaceAxial {
		if in.IsPlanar(face) {
			out.Terms = append(out.Terms, proofbound.ProductUpper(axial, areaUpper[face]))
		}
	}
	out.Terms = append(out.Terms, proofbound.SweptVolumeAllow(in.StoreMax,
		proofbound.PerturbedAreaUpper(in.Vertices, in.Triangles, in.StoreMax)))
	return out
}
