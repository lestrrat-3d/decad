package stackedrecord

import (
	"context"
	"fmt"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

type replayFaceUse struct {
	edge *topo.Edge
	face int
}

// lumpsReplay groups faces by exact shared Edge identity with charged arrays
// and comparisons. Quadratic use-pair work is explicit and stops at the meter.
func lumpsReplay(ctx context.Context, alloc topo.Alloc, faces []*topo.Face,
	anchors map[*topo.Face]struct{}, budget *proofbound.ReplayBudget) ([]*topo.Lump, error) {
	if budget == nil {
		return nil, decaderr.ErrUnsupported
	}
	count := 0
	for _, face := range faces {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return nil, err
		}
		if face == nil {
			return nil, decaderr.ErrUnsupported
		}
		for _, loop := range face.Loops {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return nil, err
			}
			if loop == nil || len(loop.CoEdges) > int(^uint(0)>>1)-count {
				return nil, decaderr.ErrUnsupported
			}
			count += len(loop.CoEdges)
		}
	}
	if err := budget.ReserveRecords(ctx, uint64(len(faces)+count)); err != nil {
		return nil, err
	}
	for _, array := range [][2]uint64{
		{uint64(len(faces)), uint64(unsafe.Sizeof(int(0)))},
		{uint64(len(faces)), uint64(unsafe.Sizeof(uint8(0)))},
		{uint64(count), uint64(unsafe.Sizeof(replayFaceUse{}))},
		{uint64(len(faces)), uint64(unsafe.Sizeof(int(0)))},
		{uint64(len(faces)), uint64(unsafe.Sizeof(int(0)))},
		{uint64(len(faces)), uint64(unsafe.Sizeof(int(0)))},
	} {
		if err := budget.ReserveArray(ctx, array[0], array[1]); err != nil {
			return nil, err
		}
	}
	parent := make([]int, len(faces))
	rank := make([]uint8, len(faces))
	uses := make([]replayFaceUse, 0, count)
	rootToGroup := make([]int, len(faces))
	memberCount := make([]int, len(faces))
	groupForFace := make([]int, len(faces))
	for i, face := range faces {
		if err := budget.SpendWork(ctx, 2); err != nil {
			return nil, err
		}
		parent[i], rootToGroup[i] = i, -1
		for _, loop := range face.Loops {
			for _, coedge := range loop.CoEdges {
				if err := budget.SpendWork(ctx, 1); err != nil {
					return nil, err
				}
				if coedge.Edge == nil {
					return nil, decaderr.ErrUnsupported
				}
				uses = append(uses, replayFaceUse{edge: coedge.Edge, face: i})
			}
		}
	}
	root := func(i int) (int, error) {
		for parent[i] != i {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return 0, err
			}
			i = parent[i]
		}
		return i, nil
	}
	for i := range uses {
		for j := i + 1; j < len(uses); j++ {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return nil, err
			}
			if uses[i].edge != uses[j].edge || uses[i].face == uses[j].face {
				continue
			}
			a, err := root(uses[i].face)
			if err != nil {
				return nil, err
			}
			b, err := root(uses[j].face)
			if err != nil {
				return nil, err
			}
			if a != b {
				if rank[a] < rank[b] {
					a, b = b, a
				}
				parent[b] = a
				if rank[a] == rank[b] {
					rank[a]++
				}
			}
		}
	}
	groupsCount := 0
	for i := range faces {
		r, err := root(i)
		if err != nil {
			return nil, err
		}
		if err := budget.SpendWork(ctx, 1); err != nil {
			return nil, err
		}
		if rootToGroup[r] < 0 {
			rootToGroup[r] = groupsCount
			groupsCount++
		}
		g := rootToGroup[r]
		groupForFace[i] = g
		memberCount[g]++
	}
	if err := budget.ReserveRecords(ctx, uint64(groupsCount+len(faces))); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, uint64(groupsCount), uint64(unsafe.Sizeof([]*topo.Face{}))); err != nil {
		return nil, err
	}
	groups := make([][]*topo.Face, groupsCount)
	for g := range groups {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return nil, err
		}
		if err := budget.ReserveArray(ctx, uint64(memberCount[g]), uint64(unsafe.Sizeof((*topo.Face)(nil)))); err != nil {
			return nil, err
		}
		groups[g] = make([]*topo.Face, 0, memberCount[g])
	}
	for i, face := range faces {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return nil, err
		}
		g := groupForFace[i]
		groups[g] = append(groups[g], face)
	}
	for _, size := range []uint64{uint64(unsafe.Sizeof((*topo.Lump)(nil))),
		uint64(unsafe.Sizeof((*topo.FaceShell)(nil)))} {
		if err := budget.ReserveArray(ctx, uint64(groupsCount), size); err != nil {
			return nil, err
		}
	}
	outer := make([]*topo.Lump, 0, groupsCount)
	voids := make([]*topo.FaceShell, 0, groupsCount)
	for _, group := range groups {
		anchored, open := false, false
		for _, face := range group {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return nil, err
			}
			if _, found := anchors[face]; found {
				anchored = true
			}
			for _, loop := range face.Loops {
				if err := budget.SpendWork(ctx, 1); err != nil {
					return nil, err
				}
				for _, ce := range loop.CoEdges {
					if err := budget.SpendWork(ctx, 1); err != nil {
						return nil, err
					}
					if len(ce.Edge.Faces) == 1 {
						open = true
					}
				}
			}
		}
		if err := budget.ReserveRecords(ctx, 1); err != nil {
			return nil, err
		}
		if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(topo.FaceShell{}))); err != nil {
			return nil, err
		}
		shell := alloc.NewFaceShell()
		shell.Open, shell.Faces = open, group
		if !anchored {
			shell.Void = true
			voids = append(voids, shell)
			continue
		}
		if err := budget.ReserveRecords(ctx, 2); err != nil {
			return nil, err
		}
		if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(topo.Lump{}))); err != nil {
			return nil, err
		}
		if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof((*topo.FaceShell)(nil)))); err != nil {
			return nil, err
		}
		lump := alloc.NewLump()
		lump.Shells = []*topo.FaceShell{shell}
		outer = append(outer, lump)
	}
	if len(voids) == 0 {
		return outer, nil
	}
	if len(outer) != 1 {
		return nil, fmt.Errorf("%w: a stacked body with %d outer shells and %d cavities needs a nesting proof this evaluator does not run",
			decaderr.ErrUnsupported, len(outer), len(voids))
	}
	if err := budget.ReserveRecords(ctx, uint64(len(voids))); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, uint64(len(voids)+1), uint64(unsafe.Sizeof((*topo.FaceShell)(nil)))); err != nil {
		return nil, err
	}
	outer[0].Shells = append(outer[0].Shells, voids...)
	return outer, nil
}
