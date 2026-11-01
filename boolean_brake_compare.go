package decad

import (
	"context"
	"math"
	"reflect"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

type brakeVisit struct {
	a, b uintptr
	typ  reflect.Type
}

type brakeComparePair struct{ a, b reflect.Value }

// brakeCompare uses explicitly metered stacks and visit records. Map payloads
// refuse before reflect creates map-iteration scratch.
func brakeCompare(ctx context.Context, x, y any, budget *proofbound.ReplayBudget) (bool, error) {
	var stack []brakeComparePair
	var seen []brakeVisit
	push := func(p brakeComparePair) error {
		if len(stack) == cap(stack) {
			n := max(1, 2*cap(stack))
			if err := budget.ReserveRecords(ctx, uint64(n)); err != nil {
				return err
			}
			if err := budget.ReserveArray(ctx, uint64(n), uint64(unsafe.Sizeof(brakeComparePair{}))); err != nil {
				return err
			}
			if err := budget.SpendWork(ctx, uint64(len(stack))); err != nil {
				return err
			}
			next := make([]brakeComparePair, len(stack), n)
			copy(next, stack)
			stack = next
		}
		stack = append(stack, p)
		return nil
	}
	if err := push(brakeComparePair{reflect.ValueOf(x), reflect.ValueOf(y)}); err != nil {
		return false, err
	}
	for len(stack) > 0 {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return false, err
		}
		p := stack[len(stack)-1]
		stack = stack[:len(stack)-1]
		a, b := p.a, p.b
		if !a.IsValid() || !b.IsValid() {
			if a.IsValid() != b.IsValid() {
				return false, nil
			}
			continue
		}
		if a.Type() != b.Type() {
			return false, nil
		}
		switch a.Kind() {
		case reflect.Interface, reflect.Pointer:
			if a.IsNil() || b.IsNil() {
				if a.IsNil() != b.IsNil() {
					return false, nil
				}
				continue
			}
			if a.Kind() == reflect.Pointer {
				if a.Pointer() == b.Pointer() {
					continue
				}
				k := brakeVisit{a.Pointer(), b.Pointer(), a.Type()}
				found := false
				for _, v := range seen {
					if err := budget.SpendWork(ctx, 1); err != nil {
						return false, err
					}
					if v == k {
						found = true
						break
					}
				}
				if found {
					continue
				}
				if len(seen) == cap(seen) {
					n := max(1, 2*cap(seen))
					if err := budget.ReserveRecords(ctx, uint64(n)); err != nil {
						return false, err
					}
					if err := budget.ReserveArray(ctx, uint64(n), uint64(unsafe.Sizeof(brakeVisit{}))); err != nil {
						return false, err
					}
					if err := budget.SpendWork(ctx, uint64(len(seen))); err != nil {
						return false, err
					}
					next := make([]brakeVisit, len(seen), n)
					copy(next, seen)
					seen = next
				}
				seen = append(seen, k)
			}
			if err := push(brakeComparePair{a.Elem(), b.Elem()}); err != nil {
				return false, err
			}
		case reflect.Struct:
			for i := a.NumField() - 1; i >= 0; i-- {
				if err := budget.SpendWork(ctx, 1); err != nil {
					return false, err
				}
				if err := push(brakeComparePair{a.Field(i), b.Field(i)}); err != nil {
					return false, err
				}
			}
		case reflect.Array, reflect.Slice:
			if a.Len() != b.Len() || (a.Kind() == reflect.Slice && a.IsNil() != b.IsNil()) {
				return false, nil
			}
			for i := a.Len() - 1; i >= 0; i-- {
				if err := budget.SpendWork(ctx, 1); err != nil {
					return false, err
				}
				if err := push(brakeComparePair{a.Index(i), b.Index(i)}); err != nil {
					return false, err
				}
			}
		case reflect.Map:
			return false, errBrakeReplayComparisonDoesNotAcceptMaps
		case reflect.Float64:
			if math.Float64bits(a.Float()) != math.Float64bits(b.Float()) {
				return false, nil
			}
		case reflect.Float32:
			if math.Float32bits(float32(a.Float())) != math.Float32bits(float32(b.Float())) {
				return false, nil
			}
		case reflect.Bool:
			if a.Bool() != b.Bool() {
				return false, nil
			}
		case reflect.Int, reflect.Int8, reflect.Int16, reflect.Int32, reflect.Int64:
			if a.Int() != b.Int() {
				return false, nil
			}
		case reflect.Uint, reflect.Uint8, reflect.Uint16, reflect.Uint32, reflect.Uint64, reflect.Uintptr:
			if a.Uint() != b.Uint() {
				return false, nil
			}
		case reflect.String:
			if err := budget.SpendWork(ctx, uint64(max(len(a.String()), len(b.String()))/64)+1); err != nil {
				return false, err
			}
			if a.String() != b.String() {
				return false, nil
			}
		default:
			return false, nil
		}
	}
	return true, nil
}

// brakeBodiesEqual compares the retained payload, measurements and complete
// ordered face/loop/edge records without traversing cyclic body-owner links.
func brakeBodiesEqual(ctx context.Context, a, b *topo.Body, budget *proofbound.ReplayBudget) (bool, error) {
	if a == nil || b == nil || budget == nil {
		return false, nil
	}
	if a.Doc != b.Doc || a.Solid != b.Solid || a.Kind != b.Kind || a.PrivateShift != nil || b.PrivateShift != nil || len(a.Lumps) != len(b.Lumps) {
		return false, nil
	}
	for i, lump := range a.Lumps {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return false, err
		}
		other := b.Lumps[i]
		if lump == nil || other == nil || len(lump.Shells) != len(other.Shells) {
			return false, nil
		}
		for j, shell := range lump.Shells {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return false, err
			}
			otherShell := other.Shells[j]
			if shell == nil || otherShell == nil || shell.Void != otherShell.Void || shell.Open != otherShell.Open || len(shell.Faces) != len(otherShell.Faces) {
				return false, nil
			}
		}
	}
	for _, pair := range [][2]any{{a.Payload, b.Payload}, {a.Volume, b.Volume}, {a.VolumeLineage, b.VolumeLineage}, {a.Area, b.Area}, {a.Centroid, b.Centroid}, {a.Bounds, b.Bounds}, {a.OperationWarnings, b.OperationWarnings}, {a.Origin, b.Origin}} {
		ok, err := brakeCompare(ctx, pair[0], pair[1], budget)
		if err != nil || !ok {
			return ok, err
		}
	}
	// Each body in this route has at most 65,536 faces; charge traversals and
	// both flattened lists before calling the existing ordered face reader.
	var counts [2]int
	for bi, body := range [2]*topo.Body{a, b} {
		for _, l := range body.Lumps {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return false, err
			}
			if l == nil {
				return false, nil
			}
			for _, s := range l.Shells {
				if err := budget.SpendWork(ctx, 1); err != nil {
					return false, err
				}
				if s == nil {
					return false, nil
				}
				if len(s.Faces) > int(^uint(0)>>1)-counts[bi] {
					return false, errBrakeReplayFaceCount
				}
				counts[bi] += len(s.Faces)
				if err := budget.ReserveRecords(ctx, 2*uint64(len(s.Faces))); err != nil {
					return false, err
				}
				if err := budget.SpendWork(ctx, uint64(len(s.Faces))); err != nil {
					return false, err
				}
			}
		}
	}
	var all [2][]*topo.Face
	for i, body := range [2]*topo.Body{a, b} {
		if err := budget.ReserveArray(ctx, uint64(counts[i]), uint64(unsafe.Sizeof((*topo.Face)(nil)))); err != nil {
			return false, err
		}
		all[i] = make([]*topo.Face, 0, counts[i])
		for _, l := range body.Lumps {
			for _, sh := range l.Shells {
				all[i] = append(all[i], sh.Faces...)
			}
		}
	}
	fa, fb := all[0], all[1]
	if len(fa) != len(fb) {
		return false, nil
	}
	for i := range fa {
		if fa[i] == nil || fb[i] == nil {
			return false, nil
		}
		x, y := *fa[i], *fb[i]
		x.Body, y.Body = nil, nil
		x.Owner, y.Owner = nil, nil
		lx, ly := x.Loops, y.Loops
		x.Loops, y.Loops = nil, nil
		ok, err := brakeCompare(ctx, x, y, budget)
		if err != nil || !ok {
			return ok, err
		}
		if len(lx) != len(ly) {
			return false, nil
		}
		for k := range lx {
			if lx[k].Outer != ly[k].Outer || len(lx[k].CoEdges) != len(ly[k].CoEdges) {
				return false, nil
			}
			for j := range lx[k].CoEdges {
				e, f := lx[k].CoEdges[j], ly[k].CoEdges[j]
				if e.Forward != f.Forward || e.Edge == nil || f.Edge == nil {
					return false, nil
				}
				if len(e.Edge.Faces) != len(f.Edge.Faces) {
					return false, nil
				}
				for faceIndex, face := range e.Edge.Faces {
					found := false
					for index, candidate := range fa {
						if err := budget.SpendWork(ctx, 1); err != nil {
							return false, err
						}
						if candidate == face {
							found = fb[index] == f.Edge.Faces[faceIndex]
							break
						}
					}
					if !found {
						return false, nil
					}
				}
				ex, ey := *e.Edge, *f.Edge
				ex.Owner, ey.Owner = nil, nil
				ex.Faces, ey.Faces = nil, nil
				vx, vy, wx, wy := ex.Start, ey.Start, ex.End, ey.End
				ex.Start, ey.Start, ex.End, ey.End = nil, nil, nil, nil
				ok, err := brakeCompare(ctx, ex, ey, budget)
				if err != nil || !ok {
					return ok, err
				}
				if vx == nil || vy == nil || wx == nil || wy == nil {
					return false, nil
				}
				for _, pair := range [][2]*topo.Vertex{{vx, vy}, {wx, wy}} {
					v1, v2 := *pair[0], *pair[1]
					v1.Owner, v2.Owner = nil, nil
					ok, err := brakeCompare(ctx, v1, v2, budget)
					if err != nil || !ok {
						return ok, err
					}
				}
			}
		}
	}
	return true, nil
}
