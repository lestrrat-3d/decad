package topo

import (
	"context"
	"fmt"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// FaceBoundBudget supplies the private replay's shared allocation and work limits.
type FaceBoundBudget interface {
	ReserveRecords(context.Context, uint64) error
	ReserveArray(context.Context, uint64, uint64) error
	SpendWork(context.Context, uint64) error
}

type faceBoundEntry struct {
	face  *Face
	bound float64
}

// FaceBoundTable holds replay-only face bounds in a fixed-capacity table.
type FaceBoundTable struct {
	slots        []faceBoundEntry
	count, limit int
}

// These refusals are constructed once so rejected replay does not allocate.
var (
	errFaceBoundTable1  = fmt.Errorf("%w: invalid face bound capacity", decaderr.ErrUnsupported)
	errFaceBoundTable2  = fmt.Errorf("%w: invalid face bound lookup", decaderr.ErrUnsupported)
	errFaceBoundTable3  = fmt.Errorf("%w: invalid face bound insertion", decaderr.ErrUnsupported)
	errFaceBoundTable4  = fmt.Errorf("%w: face bound capacity exceeded", decaderr.ErrUnsupported)
	errFaceBoundTable5  = fmt.Errorf("%w: invalid face bound iteration", decaderr.ErrUnsupported)
	errFaceBoundTable6  = fmt.Errorf("%w: missing face bound table", decaderr.ErrUnsupported)
	errFaceBoundTable7  = fmt.Errorf("%w: invalid mesh face bound lookup", decaderr.ErrUnsupported)
	errFaceBoundTable8  = fmt.Errorf("%w: ambiguous mesh face bounds", decaderr.ErrUnsupported)
	errFaceBoundTable9  = fmt.Errorf("%w: missing mesh face bounds", decaderr.ErrUnsupported)
	errFaceBoundTable10 = fmt.Errorf("%w: invalid mesh face bounds", decaderr.ErrUnsupported)
)

// NewFaceBoundTable reserves a table before allocating it. Its maximum number
// of entries is fixed by limit; it never grows after construction.
func NewFaceBoundTable(ctx context.Context, limit int, budget FaceBoundBudget) (*FaceBoundTable, error) {
	if limit < 0 || budget == nil {
		return nil, errFaceBoundTable1
	}
	n := limit
	if err := budget.ReserveRecords(ctx, uint64(n)+1); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, uint64(n), uint64(unsafe.Sizeof(faceBoundEntry{}))); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(FaceBoundTable{}))); err != nil {
		return nil, err
	}
	if err := budget.SpendWork(ctx, uint64(n)); err != nil {
		return nil, err
	}
	return &FaceBoundTable{slots: make([]faceBoundEntry, n), limit: limit}, nil
}

// Lookup returns a face's bound and charges the complete table before searching.
func (t *FaceBoundTable) Lookup(ctx context.Context, face *Face, budget FaceBoundBudget) (float64, bool, error) {
	if budget == nil {
		return 0, false, errFaceBoundTable2
	}
	return t.LookupWork(face, func() error { return budget.SpendWork(ctx, 1) })
}

// LookupWork charges the complete table before searching, so insertion order
// cannot change a successful lookup's work charge.
func (t *FaceBoundTable) LookupWork(face *Face, step func() error) (float64, bool, error) {
	if t == nil || face == nil || step == nil {
		return 0, false, errFaceBoundTable2
	}
	for i := 0; i <= t.count; i++ {
		if err := step(); err != nil {
			return 0, false, err
		}
	}
	for i := range t.count {
		if t.slots[i].face == face {
			return t.slots[i].bound, true, nil
		}
	}
	return 0, false, nil
}

// Set writes one face bound, refusing a new entry once the fixed limit is hit.
func (t *FaceBoundTable) Set(ctx context.Context, face *Face, bound float64, budget FaceBoundBudget) error {
	if t == nil || face == nil || budget == nil {
		return errFaceBoundTable3
	}
	if err := budget.SpendWork(ctx, uint64(t.count)+1); err != nil {
		return err
	}
	for i := range t.count {
		if t.slots[i].face == face {
			t.slots[i].bound = bound
			return nil
		}
	}
	if t.count >= t.limit {
		return errFaceBoundTable4
	}
	t.slots[t.count] = faceBoundEntry{face: face, bound: bound}
	t.count++
	return nil
}

// Len returns the number of stored faces.
func (t *FaceBoundTable) Len() int {
	if t == nil {
		return 0
	}
	return t.count
}

// Each visits every entry and propagates a failed work charge or visitor.
func (t *FaceBoundTable) Each(ctx context.Context, budget FaceBoundBudget, visit func(*Face, float64) error) error {
	if t == nil || budget == nil || visit == nil {
		return errFaceBoundTable5
	}
	for _, e := range t.slots[:t.count] {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return err
		}
		if e.face != nil {
			if err := visit(e.face, e.bound); err != nil {
				return err
			}
		}
	}
	return nil
}

// Clone reserves and copies the whole fixed table without changing its slots.
func (t *FaceBoundTable) Clone(ctx context.Context, budget FaceBoundBudget) (*FaceBoundTable, error) {
	if t == nil {
		return nil, errFaceBoundTable6
	}
	out, err := NewFaceBoundTable(ctx, t.limit, budget)
	if err != nil {
		return nil, err
	}
	if err := budget.SpendWork(ctx, uint64(len(t.slots))); err != nil {
		return nil, err
	}
	copy(out.slots, t.slots)
	out.count = t.count
	return out, nil
}

// ReplayFaceBound reads either the fixed replay table or a legacy map.
func (m *Mesh) ReplayFaceBound(ctx context.Context, face *Face, budget FaceBoundBudget) (float64, bool, error) {
	if m == nil || budget == nil {
		return 0, false, errFaceBoundTable7
	}
	if m.ReplayFaceBounds != nil {
		if m.FaceBound != nil {
			return 0, false, errFaceBoundTable8
		}
		return m.ReplayFaceBounds.Lookup(ctx, face, budget)
	}
	if m.FaceBound == nil {
		return 0, false, errFaceBoundTable9
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return 0, false, err
	}
	value, ok := m.FaceBound[face]
	return value, ok, nil
}

// ReplayFaceBoundCount returns the number of face-bound entries in either form.
func (m *Mesh) ReplayFaceBoundCount() int {
	if m == nil {
		return 0
	}
	if m.ReplayFaceBounds != nil {
		return m.ReplayFaceBounds.Len()
	}
	return len(m.FaceBound)
}

// EachReplayFaceBound visits the stored bounds while charging the replay.
func (m *Mesh) EachReplayFaceBound(ctx context.Context, budget FaceBoundBudget, visit func(*Face, float64) error) error {
	if m == nil || budget == nil || visit == nil {
		return errFaceBoundTable10
	}
	if m.ReplayFaceBounds != nil {
		if m.FaceBound != nil {
			return errFaceBoundTable8
		}
		return m.ReplayFaceBounds.Each(ctx, budget, visit)
	}
	if m.FaceBound == nil {
		return errFaceBoundTable9
	}
	for face, bound := range m.FaceBound {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return err
		}
		if err := visit(face, bound); err != nil {
			return err
		}
	}
	return nil
}
