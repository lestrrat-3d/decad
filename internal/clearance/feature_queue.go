package clearance

import (
	"cmp"
	"slices"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// FeatureKind identifies a boundary pair in the sorted clearance walk.
type FeatureKind uint8

const (
	FaceFace FeatureKind = iota
	FaceEdge
	EdgeFace
	EdgeEdge
)

// FeatureCell names one pair and its lower box-distance bound.
type FeatureCell struct {
	LowerBound float64
	Kind       FeatureKind
	I, J       int
}

// FeatureSet is one body's boundary features in the clearance walk.
type FeatureSet struct {
	Faces    []*CFace
	Edges    []*CEdge
	Vertices []r3.Vec
}

// FeatureVisitor evaluates the cells the queue admits. Side 0 names a's
// vertex against b; side 1 names b's vertex against a.
type FeatureVisitor struct {
	Vertex   func(side int, budget *proofbound.WorkBudget, v r3.Vec, sink *CellSink) error
	FaceFace func(a, b *CFace, sink *CellSink)
	FaceEdge func(f *CFace, e *CEdge, sink *CellSink)
	EdgeEdge func(a, b *CEdge, sink *CellSink)
	Err      func() error
}

// EnumerateFeatures walks the vertex tiers before the sorted face and edge
// cells. One budget counts every outer visit, every queued pair, and the
// visitor's inner vertex work.
func EnumerateFeatures(budget *proofbound.WorkBudget, tolerance float64, a, b FeatureSet,
	sink *CellSink, visit FeatureVisitor) error {
	check := func() error {
		if err := visit.Err(); err != nil {
			return err
		}
		return budget.Step()
	}
	for _, va := range a.Vertices {
		for _, vb := range b.Vertices {
			if err := check(); err != nil {
				return err
			}
			sink.CandidateDist(tolerance, 1, PointPointDist(va, vb), va, vb)
		}
	}
	for _, va := range a.Vertices {
		if err := check(); err != nil {
			return err
		}
		if err := visit.Vertex(0, budget, va, sink); err != nil {
			return err
		}
	}
	for _, vb := range b.Vertices {
		if err := check(); err != nil {
			return err
		}
		if err := visit.Vertex(1, budget, vb, sink); err != nil {
			return err
		}
	}
	cells, err := FeatureCells(budget, a.Faces, b.Faces, a.Edges, b.Edges)
	if err != nil {
		return err
	}
	for _, cell := range cells {
		if err := check(); err != nil {
			return err
		}
		if sink.Pruned(cell.LowerBound) {
			continue
		}
		switch cell.Kind {
		case FaceFace:
			visit.FaceFace(a.Faces[cell.I], b.Faces[cell.J], sink)
		case FaceEdge:
			visit.FaceEdge(a.Faces[cell.I], b.Edges[cell.J], sink)
		case EdgeFace:
			visit.FaceEdge(b.Faces[cell.I], a.Edges[cell.J], sink)
		case EdgeEdge:
			visit.EdgeEdge(a.Edges[cell.I], b.Edges[cell.J], sink)
		}
	}
	if err := visit.Err(); err != nil {
		return err
	}
	return budget.Err()
}

// FeatureCells queues face and edge pairs in ascending box-distance order.
// Stable sorting preserves the face-face, face-edge, edge-face, edge-edge
// order for ties. The caller's budget also counts every queued pair.
func FeatureCells(budget *proofbound.WorkBudget, aFaces, bFaces []*CFace,
	aEdges, bEdges []*CEdge) ([]FeatureCell, error) {
	n := len(aFaces)*(len(bFaces)+len(bEdges)) + len(bFaces)*len(aEdges) + len(aEdges)*len(bEdges)
	cells := make([]FeatureCell, 0, n)
	push := func(kind FeatureKind, i, j int, boxA, boxB [2]r3.Vec) error {
		if err := budget.Step(); err != nil {
			return err
		}
		cells = append(cells, FeatureCell{LowerBound: ClrBoxDist(boxA, boxB), Kind: kind, I: i, J: j})
		return nil
	}
	for i, fa := range aFaces {
		for j, fb := range bFaces {
			if err := push(FaceFace, i, j, fa.Box, fb.Box); err != nil {
				return nil, err
			}
		}
	}
	for i, fa := range aFaces {
		for j, eb := range bEdges {
			if err := push(FaceEdge, i, j, fa.Box, eb.Box); err != nil {
				return nil, err
			}
		}
	}
	for i, fb := range bFaces {
		for j, ea := range aEdges {
			if err := push(EdgeFace, i, j, fb.Box, ea.Box); err != nil {
				return nil, err
			}
		}
	}
	for i, ea := range aEdges {
		for j, eb := range bEdges {
			if err := push(EdgeEdge, i, j, ea.Box, eb.Box); err != nil {
				return nil, err
			}
		}
	}
	slices.SortStableFunc(cells, func(x, y FeatureCell) int { return cmp.Compare(x.LowerBound, y.LowerBound) })
	return cells, nil
}
