package meshbool

import (
	"context"
	"math"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// replayParityScan uses the original facet order and rejects only with the
// same per-facet box. It avoids constructing the unmetered grid and caches.
type replayParityNode struct {
	box    ParityFacetBox
	lo, hi int
}
type replayParityAxis struct {
	owner     *proofbound.ReplayBudget
	nodes     []replayParityNode
	projected []Xp2
}

func replayParityTree(ctx context.Context, pm *ParityMesh, axis, u, v int, b *proofbound.WorkBudget) (*replayParityAxis, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if b.ReplayID == nil {
		return nil, errReplayMissingReplayParityOwner
	}
	if old := pm.replayAxes[axis]; old != nil && old.owner == b.ReplayID {
		return old, nil
	}
	n := len(pm.Tris)
	maxInt := int(^uint(0) >> 1)
	if len(pm.Verts) > maxInt-3 || n > (maxInt-len(pm.Verts)-3)/8 {
		return nil, errReplayReplayParitySize
	}
	// Nodes, projected vertices, temporary stack, and the tree object.
	if err := b.Reserve(uint64(8*n + len(pm.Verts) + 3)); err != nil {
		return nil, err
	}
	if _, err := b.StepN(uint64(8*n + len(pm.Verts) + 2)); err != nil {
		return nil, err
	}
	if err := b.ReserveBytesMul(uint64(4*n+1), uint64(unsafe.Sizeof(replayParityNode{}))); err != nil {
		return nil, err
	}
	if err := b.ReserveBytesMul(uint64(len(pm.Verts)), uint64(unsafe.Sizeof(Xp2{}))); err != nil {
		return nil, err
	}
	if err := b.ReserveBytes(512); err != nil {
		return nil, err
	}
	if err := b.ReserveBytesMul(uint64(4*n+1), uint64(unsafe.Sizeof(int(0)))); err != nil {
		return nil, err
	}
	tree := &replayParityAxis{owner: b.ReplayID, nodes: make([]replayParityNode, 4*n+1), projected: make([]Xp2, len(pm.Verts))}
	stack := make([]int, 0, 4*n+1)
	stack = append(stack, 1)
	tree.nodes[1].hi = n
	for len(stack) > 0 {
		idx := stack[len(stack)-1]
		stack = stack[:len(stack)-1]
		if _, err := b.StepN(64); err != nil {
			return nil, err
		}
		if idx < 0 {
			idx = -idx
			node := &tree.nodes[idx]
			a, c := tree.nodes[idx*2].box, tree.nodes[idx*2+1].box
			node.box = ParityFacetBox{MinU: math.Min(a.MinU, c.MinU), MaxU: math.Max(a.MaxU, c.MaxU), MinV: math.Min(a.MinV, c.MinV), MaxV: math.Max(a.MaxV, c.MaxV), Built: true, Nondegenerate: a.Nondegenerate && c.Nondegenerate}
			continue
		}
		node := &tree.nodes[idx]
		if node.hi-node.lo == 1 {
			node.box = replayParityBox(pm, u, v, node.lo)
			continue
		}
		mid := node.lo + (node.hi-node.lo)/2
		tree.nodes[2*idx].lo, tree.nodes[2*idx].hi = node.lo, mid
		tree.nodes[2*idx+1].lo, tree.nodes[2*idx+1].hi = mid, node.hi
		stack = append(stack, -idx, 2*idx+1, 2*idx)
	}
	pm.replayAxes[axis] = tree
	return tree, nil
}

func replayParityScan(ctx context.Context, p proof.Xpt, pm *ParityMesh, b *proofbound.WorkBudget) (bool, bool, error) {
	if len(pm.exactVerts) != len(pm.Verts) || len(pm.exactNorms) != len(pm.Tris) {
		return false, false, errReplayReplayParityRequiresPreparedExactFacets
	}
	meter := replayExactWork{b}
	for _, ray := range AxisRays {
		if len(pm.Tris) == 0 {
			return false, false, nil
		}
		pa, err := replayProjection(ctx, p, ray.U, ray.V, meter)
		if err != nil {
			return false, false, err
		}
		tree, err := replayParityTree(ctx, pm, ray.Axis, ray.U, ray.V, b)
		if err != nil {
			return false, false, err
		}
		if err := b.Reserve(uint64(len(tree.nodes))); err != nil {
			return false, false, err
		}
		if _, err := b.StepN(uint64(len(tree.nodes))); err != nil {
			return false, false, err
		}
		if err := b.ReserveBytesMul(uint64(len(tree.nodes)), uint64(unsafe.Sizeof(int(0)))); err != nil {
			return false, false, err
		}
		crossings := 0
		ambiguous, boundary := false, false
		visit := func(index int) error {
			if err := b.Step(); err != nil {
				return err
			}
			node := &tree.nodes[index]
			if !pm.Unfiltered && pa.FloatFinite && node.box.Rejects(pa.Fu, pa.Fv) {
				return nil
			}
			ti := node.lo
			tri := pm.Tris[ti]
			var q [3]Xp2
			for i, vi := range tri {
				if tree.projected[vi].U == nil {
					tree.projected[vi], err = replayProjection(ctx, pm.exactVerts[vi], ray.U, ray.V, meter)
					if err != nil {
						return err
					}
				}
				q[i] = tree.projected[vi]
			}
			s1, err := replayCross2(ctx, b, q[0], q[1], pa)
			if err != nil {
				return err
			}
			s2, err := replayCross2(ctx, b, q[1], q[2], pa)
			if err != nil {
				return err
			}
			if s1*s2 < 0 {
				return nil
			}
			s3, err := replayCross2(ctx, b, q[2], q[0], pa)
			if err != nil {
				return err
			}
			if (s1 < 0 || s2 < 0 || s3 < 0) && (s1 > 0 || s2 > 0 || s3 > 0) {
				return nil
			}
			if s1 == 0 || s2 == 0 || s3 == 0 {
				ambiguous = true
				return nil
			}
			normal := pm.exactNorms[ti]
			axis := XIntCoordOf(normal, ray.Axis).Sign()
			if axis == 0 {
				ambiguous = true
				return nil
			}
			side, err := proof.ReplayXdotDiffSign(ctx, normal, pm.exactVerts[tri[0]], p, meter)
			if err != nil {
				return err
			}
			if side == 0 {
				boundary = true
				return nil
			}
			if side*axis*ray.Dir > 0 {
				crossings++
			}
			return nil
		}
		stack := make([]int, 0, len(tree.nodes))
		stack = append(stack, 1)
		for len(stack) > 0 {
			index := stack[len(stack)-1]
			stack = stack[:len(stack)-1]
			node := &tree.nodes[index]
			if node.hi-node.lo > 1 {
				if err := b.Step(); err != nil {
					return false, false, err
				}
				if !pm.Unfiltered && pa.FloatFinite && node.box.Rejects(pa.Fu, pa.Fv) {
					continue
				}
				stack = append(stack, index*2+1, index*2)
				continue
			}
			if err := visit(index); err != nil {
				return false, false, err
			}
			if ambiguous || boundary {
				break
			}
		}

		if boundary {
			return false, true, nil
		}
		if !ambiguous {
			return crossings%2 == 1, false, nil
		}
	}
	return false, false, errBrakeParityRays
}

// replayParityBox accepts only the already-proven floating determinant filter.
// Uncertain and degenerate facets remain candidates for the exact classifier.
func replayParityBox(pm *ParityMesh, u, v, ti int) ParityFacetBox {
	tri := pm.Tris[ti]
	a, c, d := pm.Verts[tri[0]], pm.Verts[tri[1]], pm.Verts[tri[2]]
	au, av := CoordOf(a, u), CoordOf(a, v)
	cu, cv := CoordOf(c, u), CoordOf(c, v)
	du, dv := CoordOf(d, u), CoordOf(d, v)
	box := ParityFacetBox{MinU: math.Min(au, math.Min(cu, du)), MaxU: math.Max(au, math.Max(cu, du)), MinV: math.Min(av, math.Min(cv, dv)), MaxV: math.Max(av, math.Max(cv, dv)), Built: true}
	if math.IsNaN(box.MinU) || math.IsNaN(box.MaxU) || math.IsNaN(box.MinV) || math.IsNaN(box.MaxV) || math.IsInf(box.MinU, 0) || math.IsInf(box.MaxU, 0) || math.IsInf(box.MinV, 0) || math.IsInf(box.MaxV, 0) {
		return box
	}
	left, right := (cu-au)*(dv-av), (cv-av)*(du-au)
	bound := ParityAreaErrCoef*(math.Abs(left)+math.Abs(right)) + ParityAreaFloor
	det := left - right
	box.Nondegenerate = det > bound || det < -bound
	return box
}
