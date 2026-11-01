package booleaneval

import (
	"context"
	"math"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/booloperand"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessdispatch"
	"github.com/lestrrat-3d/decad/internal/topo"
)

type replayMesher struct{ budget *proofbound.ReplayBudget }

type retainedReplayMesher struct {
	replayMesher
	target           *RetainedTarget
	refined          *topo.Mesh
	refinedChord     float64
	preparedBody     *topo.Body
	preparedMesh     *topo.Mesh
	preparedChord    uint64
	preparedRetained bool
	prepared         *meshbool.ReplayGeometry
	preparedFaces    []*topo.Face
}

func (m *retainedReplayMesher) prepareTarget(ctx context.Context, body *topo.Body, mesh *topo.Mesh,
	chord float64, src []int, retained bool, b *proofbound.ReplayBudget) (*meshbool.BoolMesh, error) {
	match := m.prepared != nil && m.preparedBody == body && m.preparedMesh == mesh &&
		m.preparedChord == math.Float64bits(chord) && m.preparedRetained == retained && m.budget == b &&
		len(m.preparedFaces) == len(mesh.Source)
	if match {
		for i, face := range mesh.Source {
			if err := b.SpendWork(ctx, 1); err != nil {
				return nil, err
			}
			if face != m.preparedFaces[i] {
				match = false
				break
			}
		}
	}
	if match {
		bm, ok, err := m.prepared.View(ctx, mesh.Vertices, mesh.Triangles, src, b)
		if err != nil {
			return nil, err
		}
		if ok {
			return finishBoolMeshReplay(ctx, mesh, bm, retained, b)
		}
	}
	bm, err := PrepBoolMeshReplay(ctx, mesh, src, retained, b)
	if err != nil {
		return nil, err
	}
	geometry, err := meshbool.CaptureReplayGeometry(ctx, bm, b)
	if err != nil {
		return nil, err
	}
	if err := b.ReserveRecords(ctx, uint64(len(mesh.Source))); err != nil {
		return nil, err
	}
	if err := b.ReserveArray(ctx, uint64(len(mesh.Source)), uint64(unsafe.Sizeof((*topo.Face)(nil)))); err != nil {
		return nil, err
	}
	if err := b.SpendWork(ctx, uint64(len(mesh.Source))); err != nil {
		return nil, err
	}
	faces := append([]*topo.Face(nil), mesh.Source...)
	m.preparedBody, m.preparedMesh, m.preparedChord, m.preparedRetained, m.prepared =
		body, mesh, math.Float64bits(chord), retained, geometry
	m.preparedFaces = faces
	return bm, nil
}

func (m *retainedReplayMesher) TessellateOperand(ctx context.Context, body *topo.Body, chord float64) (*topo.Mesh, bool, error) {
	if m.target != nil && body == m.target.body {
		floor, restating := booloperand.HeldFloor(body)
		if !(m.target.chord <= max(chord, floor)) {
			ok, err := retainedTargetWithinChord(ctx, m.target.mesh, max(chord, floor), m.budget)
			if err != nil {
				return nil, false, err
			}
			if !ok {
				request := max(chord, floor)
				if m.refined != nil && math.Float64bits(m.refinedChord) == math.Float64bits(request) {
					if err := m.budget.SpendWork(ctx, 1); err != nil {
						return nil, false, err
					}
					return m.refined, restating, nil
				}
				mesh, _, err := m.replayMesher.TessellateOperand(ctx, body, request)
				if err != nil {
					return nil, false, err
				}
				m.refined, m.refinedChord = mesh, request
				return mesh, restating, nil
			}
		}
		if err := m.budget.SpendWork(ctx, 1); err != nil {
			return nil, false, err
		}
		return m.target.mesh, restating, nil
	}
	return m.replayMesher.TessellateOperand(ctx, body, chord)
}

// retainedTargetWithinChord checks the authenticated VerifyAll mesh's actual
// displacement proofs when its original request was coarser than this request.
func retainedTargetWithinChord(ctx context.Context, mesh *topo.Mesh, chord float64, budget *proofbound.ReplayBudget) (bool, error) {
	valid := func(v float64) bool { return v >= 0 && !math.IsInf(v, 0) && !math.IsNaN(v) }
	if mesh == nil || budget == nil || !(chord > 0) || proofbound.IsNonFinite(chord) || !mesh.BoundaryOK || !mesh.SymDiffOK ||
		!valid(mesh.Bound) || mesh.Bound > chord || !valid(mesh.CoordBound) || !valid(mesh.AreaSlack) || !valid(mesh.VolSymDiff) ||
		len(mesh.Triangles) == 0 || len(mesh.Source) != len(mesh.Triangles) ||
		(len(mesh.VertexBound) != 0 && len(mesh.VertexBound) != len(mesh.Vertices)) {
		return false, nil
	}
	for _, bound := range mesh.VertexBound {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return false, err
		}
		if !valid(bound) || bound > chord || bound > mesh.Bound {
			return false, nil
		}
	}
	for i, tri := range mesh.Triangles {
		if err := budget.SpendWork(ctx, 4); err != nil {
			return false, err
		}
		for _, vi := range tri {
			if vi < 0 || vi >= len(mesh.Vertices) {
				return false, nil
			}
		}
		if mesh.Source[i] == nil {
			return false, nil
		}
		bound, ok, err := mesh.ReplayFaceBound(ctx, mesh.Source[i], budget)
		if err != nil {
			return false, err
		}
		if !ok || !valid(bound) || bound > chord || bound > mesh.Bound {
			return false, nil
		}
	}
	return true, nil
}

func (m replayMesher) TessellateOperand(ctx context.Context, b *topo.Body, chord float64) (*topo.Mesh, bool, error) {
	if b == nil || m.budget == nil {
		return nil, false, errBrakeEvalBoundedMesherNeedsAnOperandAndBudget
	}
	floor, restating := booloperand.HeldFloor(b)
	chord = max(chord, floor)
	var mesh *topo.Mesh
	var err error
	switch b.Payload.(type) {
	case facetedrecord.Payload:
		mesh, err = tessdispatch.FacetedReplay(ctx, b, chord, m.budget)
	case prismrecord.Payload:
		mesh, err = tessdispatch.RectanglePrismReplay(ctx, b, chord, m.budget)
	default:
		return nil, false, errBrakeEvalBoundedMesherDoesNotSupportThisOperand
	}
	return mesh, restating, err
}

func operandChordReplay(ctx context.Context, b *topo.Body, pairTol, dPair, chordScale float64, budget *proofbound.ReplayBudget) (float64, error) {
	share, err := booloperand.ChordShareReplay(ctx, b, budget)
	if err != nil {
		return 0, err
	}
	_, restating := booloperand.HeldFloor(b)
	return meshbool.OperandChord(pairTol, dPair*chordScale, share.Floor*chordScale, restating), nil
}

func facesReplay(ctx context.Context, b *topo.Body, budget *proofbound.ReplayBudget) ([]*topo.Face, error) {
	var count uint64
	for _, l := range b.Lumps {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return nil, err
		}
		if l == nil {
			return nil, errBrakeEvalNilLump
		}
		for _, s := range l.Shells {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return nil, err
			}
			if s == nil {
				return nil, errBrakeEvalNilShell
			}
			n := uint64(len(s.Faces))
			if n > proofbound.MaxMultiwayRecordsPerCall-count {
				return nil, budget.ReserveRecords(ctx, proofbound.MaxMultiwayRecordsPerCall+1)
			}
			count += n
		}
	}
	if err := budget.SpendWork(ctx, count); err != nil {
		return nil, err
	}
	if err := budget.ReserveRecords(ctx, count); err != nil {
		return nil, err
	}
	out := make([]*topo.Face, 0, int(count))
	for _, l := range b.Lumps {
		for _, s := range l.Shells {
			out = append(out, s.Faces...)
		}
	}
	return out, nil
}
