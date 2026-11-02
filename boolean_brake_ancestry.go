package decad

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

const brakeSourceNodeLimit = 65536

type brakeSourceRemapper struct {
	ctx                    context.Context //nolint:containedctx // The value lives only for one bounded replay call.
	budget                 *proofbound.ReplayBudget
	private                [2]*topo.Body
	resultID               producerID
	faces                  [2][]*topo.Face
	original, mapped, path []*facetedrecord.SourcePatch
}

func brakeOrderedFaces(ctx context.Context, body *topo.Body, budget *proofbound.ReplayBudget) ([]*topo.Face, error) {
	if body == nil {
		return nil, errBrakeMissingReplaySourceBody
	}
	n := 0
	for _, lump := range body.Lumps {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return nil, err
		}
		if lump == nil {
			return nil, errBrakeMissingReplaySourceLump
		}
		for _, shell := range lump.Shells {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return nil, err
			}
			if shell == nil || len(shell.Faces) > brakeSourceNodeLimit-n {
				return nil, errBrakeReplaySourceFaceLimit
			}
			n += len(shell.Faces)
		}
	}
	if err := budget.ReserveRecords(ctx, uint64(n)); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, uint64(n), uint64(unsafe.Sizeof((*topo.Face)(nil)))); err != nil {
		return nil, err
	}
	if err := budget.SpendWork(ctx, uint64(n)); err != nil {
		return nil, err
	}
	faces := make([]*topo.Face, 0, n)
	for _, lump := range body.Lumps {
		for _, shell := range lump.Shells {
			faces = append(faces, shell.Faces...)
		}
	}
	for _, face := range faces {
		if face == nil {
			return nil, errBrakeMissingReplaySourceFace
		}
	}
	return faces, nil
}

func (r *brakeSourceRemapper) lookup(p *facetedrecord.SourcePatch) (int, error) {
	for i, previous := range r.original {
		if err := r.budget.SpendWork(r.ctx, 1); err != nil {
			return -1, err
		}
		if p == previous {
			return i, nil
		}
	}
	return -1, nil
}

func (r *brakeSourceRemapper) strip(root *facetedrecord.SourcePatch) (*facetedrecord.SourcePatch, error) {
	if root == nil {
		return nil, errBrakeMissingReplaySource
	}
	r.path = r.path[:0]
	p := root
	var parent *facetedrecord.SourcePatch
	for p != nil {
		index, err := r.lookup(p)
		if err != nil {
			return nil, err
		}
		if index >= 0 {
			parent = r.mapped[index]
			break
		}
		if len(r.original)+len(r.path) >= brakeSourceNodeLimit {
			return nil, errBrakeReplaySourceAncestryLimit
		}
		for _, prior := range r.path {
			if err := r.budget.SpendWork(r.ctx, 1); err != nil {
				return nil, err
			}
			if prior == p {
				return nil, errBrakeCyclicReplaySourceAncestry
			}
		}
		r.path = append(r.path, p)
		p = p.Parent
	}
	for i := len(r.path) - 1; i >= 0; i-- { //nolint:modernize // Keep the reverse scan and its budgeted visit order explicit.
		node := r.path[i]
		if node.Producer == r.resultID {
			return nil, errBrakeFinalResultAppearsInItsSourceAncestry
		}
		private := -1
		for k, body := range r.private {
			if node.Producer == body.Origin.Producer {
				private = k
				break
			}
		}
		if private >= 0 {
			// A private C' face is Union operand 1; the joined face is Cut operand 0.
			wantOperand := 1 - private
			if node.Blend || node.Operand != wantOperand || node.Ordinal < 0 || node.Ordinal >= len(r.faces[private]) || r.faces[private][node.Ordinal].Source != node.Parent {
				return nil, errBrakePrivateSourceFaceIdentityChanged
			}
		} else if node.Parent == parent {
			parent = node
		} else {
			if err := r.budget.ReserveRecords(r.ctx, 1); err != nil {
				return nil, err
			}
			if err := r.budget.ReserveArray(r.ctx, 1, uint64(unsafe.Sizeof(facetedrecord.SourcePatch{}))); err != nil {
				return nil, err
			}
			nodeCopy := *node
			nodeCopy.Parent = parent
			parent = &nodeCopy
		}
		r.original = append(r.original, node)
		r.mapped = append(r.mapped, parent)
	}
	return parent, nil
}

// stripBrakePrivateSources removes only authenticated private producer nodes.
// Public nodes retain their fields and the original shared-parent identity.
func stripBrakePrivateSources(ctx context.Context, result, placed, joined *topo.Body, ref producerID, budget *proofbound.ReplayBudget) error {
	if result == nil || placed == nil || joined == nil || budget == nil || ref < 0 || ref > producerID(int(^uint(0)>>1)-2) ||
		placed.Origin.Producer != ref || joined.Origin.Producer != ref+1 || result.Origin.Producer != ref+2 {
		return errBrakePrivateReplayProducerIdentitiesChanged
	}
	payload, ok := result.Payload.(facetedrecord.Payload)
	if !ok || len(payload.Groups) > brakeSourceNodeLimit {
		return errBrakeReplaySourcePayloadUnavailable
	}
	r := brakeSourceRemapper{ctx: ctx, budget: budget, private: [2]*topo.Body{placed, joined}, resultID: result.Origin.Producer}
	var err error
	for i, body := range r.private {
		if r.faces[i], err = brakeOrderedFaces(ctx, body, budget); err != nil {
			return err
		}
	}
	faces, err := brakeOrderedFaces(ctx, result, budget)
	if err != nil {
		return err
	}
	for _, a := range [][2]uint64{{3 * brakeSourceNodeLimit, uint64(unsafe.Sizeof((*facetedrecord.SourcePatch)(nil)))}, {uint64(len(payload.Groups)), uint64(unsafe.Sizeof(facetedrecord.Group{}))}, {uint64(len(faces)), uint64(unsafe.Sizeof((*facetedrecord.SourcePatch)(nil)))}} {
		if err := budget.ReserveRecords(ctx, a[0]); err != nil {
			return err
		}
		if err := budget.ReserveArray(ctx, a[0], a[1]); err != nil {
			return err
		}
	}
	r.original = make([]*facetedrecord.SourcePatch, 0, brakeSourceNodeLimit)
	r.mapped = make([]*facetedrecord.SourcePatch, 0, brakeSourceNodeLimit)
	r.path = make([]*facetedrecord.SourcePatch, 0, brakeSourceNodeLimit)
	if err := budget.SpendWork(ctx, uint64(len(payload.Groups))+uint64(len(faces))); err != nil {
		return err
	}
	groups := make([]facetedrecord.Group, len(payload.Groups))
	copy(groups, payload.Groups)
	for i := range groups {
		groups[i].Source, err = r.strip(payload.Groups[i].Source)
		if err != nil {
			return err
		}
	}
	newFaces := make([]*facetedrecord.SourcePatch, len(faces))
	for i, face := range faces {
		found := false
		for j, old := range payload.Groups {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return err
			}
			if old.Source == face.Source {
				newFaces[i] = groups[j].Source
				found = true
				break
			}
		}
		if !found {
			return errBrakeFinalFaceSourceMappingIsIncomplete
		}
	}
	if err := budget.SpendWork(ctx, uint64(len(faces))); err != nil {
		return err
	}
	payload.Groups = groups
	result.Payload = payload
	for i, face := range faces {
		face.Source = newFaces[i]
	}
	return nil
}
