package stackedrecord

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// ReplayBlindFaceKind identifies the builder operation that created a face.
type ReplayBlindFaceKind uint8

const (
	ReplayBlindWall ReplayBlindFaceKind = iota + 1
	ReplayBlindCap
	ReplayBlindPatch
)

// ReplayBlindFaceEvent records one actual face at its construction site.
type ReplayBlindFaceEvent struct {
	Face                          *topo.Face
	Kind                          ReplayBlindFaceKind
	Column, Segment, Slab, Region int
	Loop, Start, End              int
	PatchIndex                    int
	Top, Floor                    bool
}

// ReplayBlindFaceLog owns the precharged face events for one private build.
type ReplayBlindFaceLog struct {
	budget   *proofbound.ReplayBudget
	faces    []ReplayBlindFaceEvent
	snapshot []blindFaceSnapshot
	levels   [4]prismplacement.ReplayLevelIdentity
	body     *topo.Body
	sealed   bool
}

// NewReplayBlindFaceLog reserves capacity before construction emits events.
func NewReplayBlindFaceLog(ctx context.Context, budget *proofbound.ReplayBudget, capacity int) (*ReplayBlindFaceLog, error) {
	if budget == nil || capacity <= 0 || capacity > 4096 {
		return nil, decaderr.ErrUnsupported
	}
	if err := budget.SpendWork(ctx, uint64(capacity)+32); err != nil {
		return nil, err
	}
	if err := budget.ReserveRecords(ctx, uint64(capacity)+1); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(ReplayBlindFaceLog{}))); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, uint64(capacity), uint64(unsafe.Sizeof(ReplayBlindFaceEvent{}))); err != nil {
		return nil, err
	}
	return &ReplayBlindFaceLog{budget: budget, faces: make([]ReplayBlindFaceEvent, 0, capacity)}, nil
}

// record appends one builder event without growing the reserved storage.
func (l *ReplayBlindFaceLog) record(ctx context.Context, event ReplayBlindFaceEvent) error {
	if l == nil || l.budget == nil {
		return decaderr.ErrUnsupported
	}
	if err := l.budget.SpendWork(ctx, 32); err != nil {
		return err
	}
	if l.sealed || event.Face == nil || len(l.faces) == cap(l.faces) {
		return decaderr.ErrUnsupported
	}
	switch event.Kind {
	case ReplayBlindWall:
		if event.Column < 0 || event.Segment < 0 {
			return decaderr.ErrUnsupported
		}
	case ReplayBlindCap:
		if event.Slab < 0 || event.Region < 0 {
			return decaderr.ErrUnsupported
		}
	case ReplayBlindPatch:
		if event.PatchIndex < 0 {
			return decaderr.ErrUnsupported
		}
	default:
		return decaderr.ErrUnsupported
	}
	l.faces = append(l.faces, event)
	return nil
}

// Seal binds the complete event list to the built body's exact face pointers.
func (l *ReplayBlindFaceLog) Seal(ctx context.Context, body *topo.Body) (bool, error) {
	if l == nil || l.budget == nil || l.sealed || body == nil || len(l.faces) != cap(l.faces) {
		return false, nil
	}
	ok, err := l.checkFaces(ctx, body)
	if err != nil || !ok {
		return false, err
	}
	snapshot, ok, err := snapshotBlindFaces(ctx, l.faces, l.budget)
	if err != nil || !ok {
		return false, err
	}
	payload, ok := body.Payload.(Payload)
	if !ok || len(payload.Slabs) != 2 {
		return false, nil
	}
	var levels [4]prismplacement.ReplayLevelIdentity
	for i, level := range [4]prismplacement.LevelDenotation{payload.Slabs[0].Z0Denoted, payload.Slabs[0].Z1Denoted, payload.Slabs[1].Z0Denoted, payload.Slabs[1].Z1Denoted} {
		levels[i], ok, err = prismplacement.CaptureReplayLevelIdentity(ctx, level, l.budget)
		if err != nil || !ok {
			return false, err
		}
	}
	l.snapshot = snapshot
	l.levels = levels
	l.body, l.sealed = body, true
	return true, nil
}

func (l *ReplayBlindFaceLog) checkFaces(ctx context.Context, body *topo.Body) (bool, error) {
	count := 0
	for _, lump := range body.Lumps {
		if err := l.budget.SpendWork(ctx, 32); err != nil {
			return false, err
		}
		if lump == nil {
			return false, nil
		}
		for _, shell := range lump.Shells {
			if err := l.budget.SpendWork(ctx, 32); err != nil {
				return false, err
			}
			if shell == nil {
				return false, nil
			}
			for _, face := range shell.Faces {
				if err := l.budget.SpendWork(ctx, 32); err != nil {
					return false, err
				}
				if face == nil || face.Body != body {
					return false, nil
				}
				matches := 0
				for _, event := range l.faces {
					if err := l.budget.SpendWork(ctx, 32); err != nil {
						return false, err
					}
					if event.Face == face {
						matches++
					}
				}
				if matches != 1 {
					return false, nil
				}
				count++
				if count > len(l.faces) {
					return false, nil
				}
			}
		}
	}
	if count == 0 || count != len(l.faces) {
		return false, nil
	}
	for _, event := range l.faces {
		matches := 0
		for _, lump := range body.Lumps {
			if err := l.budget.SpendWork(ctx, 32); err != nil {
				return false, err
			}
			for _, shell := range lump.Shells {
				if err := l.budget.SpendWork(ctx, 32); err != nil {
					return false, err
				}
				for _, face := range shell.Faces {
					if err := l.budget.SpendWork(ctx, 32); err != nil {
						return false, err
					}
					if face == event.Face {
						matches++
					}
				}
			}
		}
		if matches != 1 {
			return false, nil
		}
	}
	if err := ctx.Err(); err != nil {
		return false, err
	}
	return true, nil
}

// Len and Event return construction records only after a successful seal.
func (l *ReplayBlindFaceLog) Len() int {
	if l == nil || !l.sealed {
		return 0
	}
	return len(l.faces)
}

func (l *ReplayBlindFaceLog) Event(index int) (ReplayBlindFaceEvent, bool) {
	if l == nil || !l.sealed || index < 0 || index >= len(l.faces) {
		return ReplayBlindFaceEvent{}, false
	}
	return l.faces[index], true
}

// Matches binds a sealed log to its body and its original shared budget.
func (l *ReplayBlindFaceLog) Matches(body *topo.Body, budget *proofbound.ReplayBudget) bool {
	return l != nil && l.sealed && l.body == body && l.budget == budget
}

// Recheck verifies that all sealed faces still belong to the same body.
func (l *ReplayBlindFaceLog) Recheck(ctx context.Context, body *topo.Body, budget *proofbound.ReplayBudget) (bool, error) {
	if !l.Matches(body, budget) {
		return false, nil
	}
	ok, err := l.checkFaces(ctx, body)
	if err != nil || !ok {
		return false, err
	}
	ok, err = recheckBlindFaces(ctx, l.snapshot, budget)
	if err != nil || !ok {
		return false, err
	}
	payload, ok := body.Payload.(Payload)
	if !ok || len(payload.Slabs) != 2 {
		return false, nil
	}
	for i, level := range [4]prismplacement.LevelDenotation{payload.Slabs[0].Z0Denoted, payload.Slabs[0].Z1Denoted, payload.Slabs[1].Z0Denoted, payload.Slabs[1].Z1Denoted} {
		ok, err = l.levels[i].Matches(ctx, level, budget)
		if err != nil || !ok {
			return false, err
		}
	}
	return true, nil
}
