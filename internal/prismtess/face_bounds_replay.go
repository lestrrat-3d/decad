package prismtess

import (
	"context"
	"fmt"
	"math"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// These refusals are constructed once so rejected replay does not allocate.
var (
	errFaceBoundsReplay1 = fmt.Errorf("%w: invalid prism face bounds", decaderr.ErrUnsupported)
	errFaceBoundsReplay2 = fmt.Errorf("%w: invalid prism vertex", decaderr.ErrBooleanFailed)
	errFaceBoundsReplay3 = fmt.Errorf("%w: nonfinite prism face bound", decaderr.ErrUnsupported)
	errFaceBoundsReplay4 = fmt.Errorf("%w: invalid bounded stacked face bounds", decaderr.ErrUnsupported)
	errFaceBoundsReplay5 = fmt.Errorf("%w: missing bounded stacked faces", decaderr.ErrUnsupported)
	errFaceBoundsReplay6 = fmt.Errorf("%w: invalid bounded stacked face", decaderr.ErrUnsupported)
	errFaceBoundsReplay7 = fmt.Errorf("%w: duplicate stacked face bound", decaderr.ErrUnsupported)
	errFaceBoundsReplay8 = fmt.Errorf("%w: missing stacked face bound", decaderr.ErrUnsupported)
	errFaceBoundsReplay9 = fmt.Errorf("%w: unused stacked face bound", decaderr.ErrUnsupported)
)

// faceBoundsReplay composes the same per-face bounds without a map.
func faceBoundsReplay(ctx context.Context, mesh *topo.Mesh, inputs []tessellation.PrismReplayFaceBound[*topo.Face],
	store []float64, section float64, budget *proofbound.ReplayBudget) error {
	if mesh == nil || budget == nil || len(mesh.Triangles) != len(mesh.Source) {
		return errFaceBoundsReplay1
	}
	faceStore, err := topo.NewFaceBoundTable(ctx, len(mesh.Source), budget)
	if err != nil {
		return err
	}
	for i, face := range mesh.Source {
		previous, _, err := faceStore.Lookup(ctx, face, budget)
		if err != nil {
			return err
		}
		for _, v := range mesh.Triangles[i] {
			if v < 0 || v >= len(store) {
				return errFaceBoundsReplay2
			}
			if store[v] < 0 || proofbound.IsNonFinite(store[v]) {
				return decaderr.ErrUnsupported
			}
			previous = math.Max(previous, store[v])
		}
		if err := faceStore.Set(ctx, face, previous, budget); err != nil {
			return err
		}
	}
	out, err := topo.NewFaceBoundTable(ctx, faceStore.Len(), budget)
	if err != nil {
		return err
	}
	boundMax := 0.0
	err = faceStore.Each(ctx, budget, func(face *topo.Face, s float64) error {
		trim, axial := 0.0, 0.0
		for _, input := range inputs {
			if err := budget.SpendWork(ctx, 16); err != nil {
				return err
			}
			if input.Face == face {
				trim, axial = input.Trim, input.Axial
				break
			}
		}
		bound := proofbound.AbsSumUpper(trim, s, section, axial)
		if proofbound.IsNonFinite(bound) {
			return errFaceBoundsReplay3
		}
		if err := out.Set(ctx, face, bound, budget); err != nil {
			return err
		}
		boundMax = max(boundMax, bound)
		return nil
	})
	if err != nil {
		return err
	}
	mesh.Bound = boundMax
	mesh.ReplayFaceBounds = out
	return nil
}

// verifiedFaceBoundsReplay composes the existing bounded face table for a
// source-authenticated stacked mesh, including its section displacement.
func verifiedFaceBoundsReplay(ctx context.Context, mesh *topo.Mesh, inputs []tessellation.PrismReplayFaceBound[*topo.Face], store []float64, section float64, b *proofbound.ReplayBudget) error {
	if b == nil {
		return decaderr.ErrUnsupported
	}
	if err := b.SpendWork(ctx, 32); err != nil {
		return err
	}
	if mesh == nil || len(store) != len(mesh.Vertices) || section < 0 || proofbound.IsNonFinite(section) {
		return errFaceBoundsReplay4
	}
	if len(inputs) == 0 || len(mesh.Source) != len(mesh.Triangles) {
		return errFaceBoundsReplay5
	}
	if err := b.ReserveRecords(ctx, uint64(len(inputs))); err != nil {
		return err
	}
	if err := b.ReserveArray(ctx, uint64(len(inputs)), uint64(unsafe.Sizeof(false))); err != nil {
		return err
	}
	seen := make([]bool, len(inputs))
	for _, x := range inputs {
		if err := b.SpendWork(ctx, 8); err != nil {
			return err
		}
		if x.Face == nil || x.Trim < 0 || x.Axial < 0 || proofbound.IsNonFinite(x.Trim) || proofbound.IsNonFinite(x.Axial) {
			return errFaceBoundsReplay6
		}
	}
	for _, f := range mesh.Source {
		match := -1
		for i, x := range inputs {
			if err := b.SpendWork(ctx, 8); err != nil {
				return err
			}
			if x.Face == f {
				if match != -1 {
					return errFaceBoundsReplay7
				}
				match = i
			}
		}
		if match < 0 {
			return errFaceBoundsReplay8
		}
		seen[match] = true
	}
	for _, ok := range seen {
		if err := b.SpendWork(ctx, 8); err != nil {
			return err
		}
		if !ok {
			return errFaceBoundsReplay9
		}
	}
	return faceBoundsReplay(ctx, mesh, inputs, store, section, b)
}
