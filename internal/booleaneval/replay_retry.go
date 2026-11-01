package booleaneval

import (
	"context"
	"errors"
	"math"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/gaptol"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

type replayContactKey struct {
	x, y, z  int64
	decisive bool
}

func replayContactKeyOf(c meshbool.TouchContact) replayContactKey {
	return replayContactKey{int64(math.Round(c.NormalA.X * 1e3)), int64(math.Round(c.NormalA.Y * 1e3)), int64(math.Round(c.NormalA.Z * 1e3)), c.Decisive}
}
func mergeReplayContacts(ctx context.Context, b *proofbound.ReplayBudget, held, more []meshbool.TouchContact) ([]meshbool.TouchContact, error) {
	if len(more) > int(^uint(0)>>1)-len(held) {
		return nil, errBrakeEvalReplayContactSize
	}
	capacity := len(held) + len(more)
	if err := b.SpendWork(ctx, uint64(capacity)); err != nil {
		return nil, err
	}
	if err := b.ReserveRecords(ctx, uint64(capacity)); err != nil {
		return nil, err
	}
	if err := b.ReserveArray(ctx, uint64(capacity), uint64(unsafe.Sizeof(meshbool.TouchContact{}))); err != nil {
		return nil, err
	}
	out := make([]meshbool.TouchContact, len(held), capacity)
	copy(out, held)
	for _, c := range more {
		if proofbound.IsNonFinite(c.Slack) || !proofbound.FiniteVec(c.NormalA) {
			return nil, errBrakeEvalNonfiniteReplayContact
		}
		key := replayContactKeyOf(c)
		duplicate := false
		for _, old := range out {
			if err := b.SpendWork(ctx, 1); err != nil {
				return nil, err
			}
			if replayContactKeyOf(old) == key {
				duplicate = true
				break
			}
		}
		if !duplicate {
			out = append(out, c)
		}
	}
	return out, nil
}
func replayShiftDirections(ctx context.Context, b *proofbound.ReplayBudget, op meshbool.OperationKind, contacts []meshbool.TouchContact) ([]r3.Vec, error) {
	n := len(contacts)
	if uint64(n) > proofbound.MaxMultiwayRecordsPerCall/4 {
		return nil, errBrakeEvalReplayContactCount
	}
	if err := b.ReserveRecords(ctx, uint64(2*n+len(shiftWeights))); err != nil {
		return nil, err
	}
	if err := b.ReserveArray(ctx, uint64(n), uint64(unsafe.Sizeof(false))); err != nil {
		return nil, err
	}
	if err := b.ReserveArray(ctx, uint64(n), uint64(unsafe.Sizeof(meshbool.TouchContact{}))); err != nil {
		return nil, err
	}
	if err := b.ReserveArray(ctx, uint64(len(shiftWeights)), uint64(unsafe.Sizeof(r3.Vec{}))); err != nil {
		return nil, err
	}
	wanted := make([]bool, n)
	rest := make([]meshbool.TouchContact, 0, n)
	out := make([]r3.Vec, 0, len(shiftWeights))
	decisive := false
	for i, c := range contacts {
		if err := b.SpendWork(ctx, 1); err != nil {
			return nil, err
		}
		if c.Decisive {
			wanted[i], decisive = true, true
		}
	}
	if !decisive && op != meshbool.OpUnion {
		for i := range wanted {
			wanted[i] = true
		}
	}
	sigma := 1.0
	if op == meshbool.OpUnion {
		sigma = -1
	}
	tie := sigma
	var pull r3.Vec
	weight := func(c meshbool.TouchContact) float64 { return math.Max(c.Slack, 1e-6) }
	for i, c := range contacts {
		if err := b.SpendWork(ctx, 1); err != nil {
			return nil, err
		}
		if wanted[i] {
			pull = pull.Add(c.NormalA.Scale(sigma * weight(c)))
			tie = 1
		} else {
			rest = append(rest, c)
		}
	}
	for i := 1; i < len(rest); i++ {
		v := rest[i]
		j := i
		for j > 0 {
			if err := b.SpendWork(ctx, 1); err != nil {
				return nil, err
			}
			if !(weight(v) > weight(rest[j-1])) {
				break
			}
			rest[j] = rest[j-1]
			j--
		}
		rest[j] = v
	}
	for _, c := range rest {
		if err := b.SpendWork(ctx, 1); err != nil {
			return nil, err
		}
		sense := tie
		switch lean := pull.Dot(c.NormalA); {
		case lean > 0:
			sense = 1
		case lean < 0:
			sense = -1
		}
		pull = pull.Add(c.NormalA.Scale(sense * weight(c)))
	}
	base, hasBase := pull.Normalize()
	for _, w := range shiftWeights {
		if err := b.SpendWork(ctx, uint64(n)+1); err != nil {
			return nil, err
		}
		u := w
		if hasBase {
			u = base.Add(w.Scale(shiftGeneric))
		}
		u, ok := u.Normalize()
		if ok && shiftOpens(u, sigma, wanted, contacts) {
			out = append(out, u)
		}
	}
	return out, nil
}
func replayRetryShifted(ctx context.Context, refusal error, op meshbool.OperationKind, pc pieces, attempt func(*ReplayShifts) (Evaluation, error), b *proofbound.ReplayBudget) (Evaluation, error) {
	var touch *meshbool.TouchContactError
	if !errors.As(refusal, &touch) || !(gaptol.Reach(ctx) > 0) {
		return Evaluation{}, refusal
	}
	if pc.count != 1 {
		return Evaluation{}, refusal
	}
	for _, p := range pc.ofFacet {
		if err := b.SpendWork(ctx, 1); err != nil {
			return Evaluation{}, err
		}
		if p != 0 {
			return Evaluation{}, errBrakeEvalReplayContactPieceMismatch
		}
	}
	contacts, err := mergeReplayContacts(ctx, b, nil, touch.Contacts)
	if err != nil {
		return Evaluation{}, err
	}
	initial := contacts
	reach := gaptol.Reach(ctx)
	grow := func(shrink bool) (Evaluation, error) {
		if err := b.SpendWork(ctx, uint64(len(initial))); err != nil {
			return Evaluation{}, err
		}
		if touch.What != meshbool.TouchCoplanar || len(initial) == 0 || !growOpens(initial) {
			return Evaluation{}, refusal
		}
		if fixed, ok := ctx.Value(authenticatedGrowthKey{}).(*AuthenticatedGrowth); ok && fixed != nil {
			if op != meshbool.OpCut || shrink || fixed.used {
				return Evaluation{}, refusal
			}
			for _, c := range initial {
				if c.Slack != 0 {
					return Evaluation{}, refusal
				}
			}
			if err := fixed.validate(ctx, fixed.target, b); err != nil {
				return Evaluation{}, err
			}
			fixed.used = true
			shifts, e := newReplayShifts(ctx, b)
			if e != nil {
				return Evaluation{}, e
			}
			if e = shifts.set(ctx, pieceShift{Grow: math.Float64frombits(fixed.growBits)}); e != nil {
				return Evaluation{}, e
			}
			ev, e := attempt(shifts)
			if errors.Is(e, errShiftDeclined) {
				return Evaluation{}, refusal
			}
			if _, ok := errors.AsType[*meshbool.TouchContactError](e); ok {
				return Evaluation{}, refusal
			}
			return ev, e
		}
		for _, step := range shiftSteps {
			amount := step * reach
			if op == meshbool.OpIntersect || shrink {
				amount = -amount
			}
			shifts, e := newReplayShifts(ctx, b)
			if e != nil {
				return Evaluation{}, e
			}
			if e = shifts.set(ctx, pieceShift{Grow: amount}); e != nil {
				return Evaluation{}, e
			}
			ev, e := attempt(shifts)
			if errors.Is(e, errShiftDeclined) {
				continue
			}
			if _, ok := errors.AsType[*meshbool.TouchContactError](e); ok {
				continue
			}
			return ev, e
		}
		return Evaluation{}, refusal
	}
	flush := len(initial) > 0
	for _, c := range initial {
		if err := b.SpendWork(ctx, 1); err != nil {
			return Evaluation{}, err
		}
		if c.Decisive || c.Slack > 0 {
			flush = false
		}
	}
	if op == meshbool.OpUnion && flush {
		ev, e := grow(true)
		if e != refusal {
			return ev, e
		}
	}
	for range shiftRounds {
		dirs, e := replayShiftDirections(ctx, b, op, contacts)
		if e != nil {
			return Evaluation{}, e
		}
		if len(dirs) == 0 {
			return grow(false)
		}
		limit := len(shiftSteps) * len(shiftWeights)
		if err := b.ReserveArray(ctx, uint64(limit), uint64(unsafe.Sizeof(r3.Vec{}))); err != nil {
			return Evaluation{}, err
		}
		tried := make([]r3.Vec, 0, limit)
		grown := false
	steps:
		for _, step := range shiftSteps {
			for variant := range shiftWeights {
				if err := b.SpendWork(ctx, uint64(len(contacts))+1); err != nil {
					return Evaluation{}, err
				}
				dir := dirs[variant%len(dirs)]
				length := math.Max(step*reach, shiftNeed(dir, contacts))
				if length > reach {
					continue
				}
				move := snapShift(dir.Scale(length))
				duplicate := false
				for _, old := range tried {
					if err := b.SpendWork(ctx, 1); err != nil {
						return Evaluation{}, err
					}
					if math.Float64bits(old.X) == math.Float64bits(move.X) && math.Float64bits(old.Y) == math.Float64bits(move.Y) && math.Float64bits(old.Z) == math.Float64bits(move.Z) {
						duplicate = true
						break
					}
				}
				if duplicate {
					continue
				}
				tried = append(tried, move)
				shifts, e := newReplayShifts(ctx, b)
				if e != nil {
					return Evaluation{}, e
				}
				if e = shifts.set(ctx, pieceShift{Move: move}); e != nil {
					return Evaluation{}, e
				}
				ev, e := attempt(shifts)
				if errors.Is(e, errShiftDeclined) {
					continue
				}
				if again, ok := errors.AsType[*meshbool.TouchContactError](e); ok {
					more, e := mergeReplayContacts(ctx, b, contacts, again.Contacts)
					if e != nil {
						return Evaluation{}, e
					}
					if len(more) > len(contacts) {
						contacts, grown = more, true
						break steps
					}
					continue
				}
				return ev, e
			}
		}
		if !grown {
			break
		}
	}
	return grow(false)
}
