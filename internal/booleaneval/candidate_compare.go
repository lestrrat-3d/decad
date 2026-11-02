package booleaneval

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"reflect"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

type capVisit struct {
	a, b uintptr
	typ  reflect.Type
}

type capComparePair struct{ a, b reflect.Value }

// capCompare uses explicitly metered stacks and visit records. Map payloads
// refuse before reflect creates map-iteration scratch.
func capCompare(ctx context.Context, x, y any, budget *proofbound.ReplayBudget) (bool, error) {
	var stack []capComparePair
	var seen []capVisit
	push := func(p capComparePair) error {
		if len(stack) == cap(stack) {
			n := max(1, 2*cap(stack))
			if err := budget.ReserveRecords(ctx, uint64(n)); err != nil {
				return err
			}
			if err := budget.ReserveArray(ctx, uint64(n), uint64(unsafe.Sizeof(capComparePair{}))); err != nil {
				return err
			}
			if err := budget.SpendWork(ctx, uint64(len(stack))); err != nil {
				return err
			}
			next := make([]capComparePair, len(stack), n)
			copy(next, stack)
			stack = next
		}
		stack = append(stack, p)
		return nil
	}
	if err := push(capComparePair{reflect.ValueOf(x), reflect.ValueOf(y)}); err != nil {
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
				if a.Type() == reflect.TypeFor[*big.Rat]() {
					if !a.CanInterface() || !b.CanInterface() {
						return false, fmt.Errorf("%w: inaccessible candidate exact level", decaderr.ErrUnsupported)
					}
					x, xok := reflect.TypeAssert[*big.Rat](a)
					y, yok := reflect.TypeAssert[*big.Rat](b)
					if !xok || !yok {
						return false, fmt.Errorf("%w: invalid candidate exact level", decaderr.ErrUnsupported)
					}
					if x.Denom().Sign() <= 0 || y.Denom().Sign() <= 0 || x.Num().BitLen() > 4096 || x.Denom().BitLen() > 4096 || y.Num().BitLen() > 4096 || y.Denom().BitLen() > 4096 {
						return false, fmt.Errorf("%w: candidate exact level width", decaderr.ErrUnsupported)
					}
					cmp, err := proof.ReplayRatCompare(ctx, x, y, budget)
					if err != nil {
						return false, err
					}
					if cmp != 0 {
						return false, nil
					}
					continue
				}
				if a.Pointer() == b.Pointer() {
					continue
				}
				k := capVisit{a.Pointer(), b.Pointer(), a.Type()}
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
					if err := budget.ReserveArray(ctx, uint64(n), uint64(unsafe.Sizeof(capVisit{}))); err != nil {
						return false, err
					}
					if err := budget.SpendWork(ctx, uint64(len(seen))); err != nil {
						return false, err
					}
					next := make([]capVisit, len(seen), n)
					copy(next, seen)
					seen = next
				}
				seen = append(seen, k)
			}
			if err := push(capComparePair{a.Elem(), b.Elem()}); err != nil {
				return false, err
			}
		case reflect.Struct:
			for i := a.NumField() - 1; i >= 0; i-- {
				if err := budget.SpendWork(ctx, 1); err != nil {
					return false, err
				}
				if err := push(capComparePair{a.Field(i), b.Field(i)}); err != nil {
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
				if err := push(capComparePair{a.Index(i), b.Index(i)}); err != nil {
					return false, err
				}
			}
		case reflect.Map:
			return false, fmt.Errorf("%w: replay comparison does not accept maps", decaderr.ErrUnsupported)
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
