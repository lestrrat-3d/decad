package meshbool

import (
	"context"
	"math/big"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// replayContactSeparated proves the existing first plane-side rejection using
// bounded arithmetic. False means later contact branches are still required.
func replayContactSigns(ctx context.Context, ta, tb [3]r3.Vec, xta, xtb [3]proof.Xpt, na, nb proof.Xpt,
	budget proof.ReplayKeyBudget) (sa, sb [3]int, separated bool, err error) {
	for i := range 3 {
		sb[i], err = proof.ReplayOrientSignPrepared(ctx, ta[0], ta[1], ta[2], tb[i], xta[0], xtb[i], na, budget)
		if err != nil {
			return
		}
	}
	if AllOneSide(sb) {
		separated = true
		return
	}
	for i := range 3 {
		sa[i], err = proof.ReplayOrientSignPrepared(ctx, tb[0], tb[1], tb[2], ta[i], xtb[0], xta[i], nb, budget)
		if err != nil {
			return
		}
	}
	separated = AllOneSide(sa)
	return
}

// replayContactRunner serializes contact candidates and charges orientation.
// Non-separated branches still require their own bounded arithmetic.
type replayProductBudget struct {
	*proofbound.ReplayBudget
	arena   *proof.ReplayProductArena
	scratch replayProductScratch
}

// replayProductScratch envelopes only the arena's raw allocation sizes.
// Other proof helpers already include their own conservative envelopes.
type replayProductScratch struct{ *proofbound.ReplayBudget }

func (b *replayProductScratch) ReserveBytes(ctx context.Context, bytes uint64) error {
	return b.ReserveArray(ctx, bytes, 1)
}

// ReserveProductSmall covers one fixed-size allocation, including its size class.
func (b *replayProductScratch) ReserveProductSmall(ctx context.Context, size uint64) error {
	if size > 128 {
		return errReplayProductObjectExceedsItsFixedEnvelope
	}
	return b.ReplayBudget.ReserveBytes(ctx, 2*size)
}

func (b *replayProductBudget) ProductDifference(ctx context.Context, a, c, d, e *big.Int) (*big.Int, error) {
	return b.arena.ProductDifference(ctx, a, c, d, e, &b.scratch)
}

func replayContactRunner(budget *proofbound.ReplayBudget) ContactBatchRunner {
	return func(ctx context.Context, ma, mb *BoolMesh, pairs []ContactPair, results []ContactBatchResult, workers int) error {
		if err := budget.ReserveRecords(ctx, 2); err != nil {
			return err
		}
		if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(proof.ReplayProductArena{}))); err != nil {
			return err
		}
		if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(replayProductBudget{}))); err != nil {
			return err
		}
		arena := &proof.ReplayProductArena{}
		productBudget := &replayProductBudget{ReplayBudget: budget, arena: arena, scratch: replayProductScratch{budget}}
		return replayContactRunnerWithArena(budget, productBudget)(ctx, ma, mb, pairs, results, workers)
	}
}

func replayContactRunnerWithArena(budget *proofbound.ReplayBudget, productBudget *replayProductBudget) ContactBatchRunner {
	return replayContactRunnerWithPlanes(budget, productBudget, nil, nil)
}

func replayContactRunnerWithPlanes(budget *proofbound.ReplayBudget, productBudget *replayProductBudget, planes []proof.FloatOrientationPlane, sat []heldSATFacet) ContactBatchRunner {
	return func(ctx context.Context, ma, mb *BoolMesh, pairs []ContactPair, results []ContactBatchResult, _ int) error {
		for i, pair := range pairs {
			if err := ctx.Err(); err != nil {
				return err
			}
			for _, item := range []struct {
				mesh  *BoolMesh
				facet int
			}{{ma, pair.I}, {mb, pair.J}} {
				if !item.mesh.FnormsReady[item.facet] {
					v, err := proof.ReplayXptVec(ctx, item.mesh.Norms[item.facet], budget)
					if err != nil {
						return err
					}
					item.mesh.Fnorms[item.facet] = v
					item.mesh.FnormsReady[item.facet] = true
				}
			}
			ta, tb := TriCorners(ma, pair.I), TriCorners(mb, pair.J)
			xa, xb := XtriCorners(ma, pair.I), XtriCorners(mb, pair.J)
			na, nb := ma.Norms[pair.I], mb.Norms[pair.J]
			var sa, sb [3]int
			var separated bool
			var err error
			if planes != nil {
				sa, sb, separated, err = replayHeldCachedContactSigns(ctx, ta, tb, xa, xb, na, nb, &planes[pair.I], &planes[pair.J], budget)
			} else {
				sa, sb, separated, err = replayContactSigns(ctx, ta, tb, xa, xb, na, nb, budget)
			}
			if err != nil {
				return err
			}
			if separated {
				results[i] = ContactBatchResult{Contact: TriContact{Kind: ContactNone, EdgeA: -1, EdgeB: -1}}
				continue
			}
			var contact TriContact
			if CountZero(sa) == 3 || CountZero(sb) == 3 {
				shared := -1
				if planes != nil && sat != nil && ma == mb && len(pairs) == 1 {
					if err := budget.SpendWork(ctx, 32); err != nil {
						return err
					}
					shared = 0
					for _, av := range ma.Tris[pair.I] {
						for _, bv := range mb.Tris[pair.J] {
							if av == bv {
								shared++
							}
						}
					}
				}
				if shared == 0 {
					separated, e := heldSATSeparated(ctx, ta, tb, &sat[pair.I], &sat[pair.J], na, productBudget)
					if e != nil {
						return e
					}
					if separated {
						results[i] = ContactBatchResult{Contact: TriContact{Kind: ContactNone, EdgeA: -1, EdgeB: -1}}
						continue
					}
				}
				contact, err = replayCoplanarContactHeld(ctx, ta, tb, xa, xb, na, nb, productBudget, shared == 1)
			} else {
				sharedOne := false
				if planes != nil && ma == mb && len(pairs) == 1 {
					if err := budget.SpendWork(ctx, 32); err != nil {
						return err
					}
					shared := 0
					for _, av := range ma.Tris[pair.I] {
						for _, bv := range mb.Tris[pair.J] {
							if av == bv {
								shared++
							}
						}
					}
					sharedOne = shared == 1
				}
				contact, err = replayCrossingContactHeld(ctx, ta, tb, xa, xb, na, nb, sa, sb, ma.Fnorms[pair.I], mb.Fnorms[pair.J], budget, sharedOne)
			}
			results[i] = ContactBatchResult{Contact: contact, Err: err}
			if err != nil {
				return nil //nolint:nilerr // The ordered executor reads the error from results[i].
			}
		}
		return ctx.Err()
	}
}

// replayNearContactRunner uses the bounded contact classifier and the
// fixed-size floating distance calculation, in candidate order.
func replayNearContactRunner(slack float64, budget *proofbound.ReplayBudget) ContactBatchRunner {
	classify := replayContactRunner(budget)
	return func(ctx context.Context, ma, mb *BoolMesh, pairs []ContactPair, results []ContactBatchResult, _ int) error {
		for i, pair := range pairs {
			if err := classify(ctx, ma, mb, pairs[i:i+1], results[i:i+1], 1); err != nil {
				return err
			}
			if results[i].Err != nil {
				return nil //nolint:nilerr // The ordered executor reads the error from results[i].
			}
			if results[i].Contact.Kind == ContactNone {
				far, err := triTriDistanceExceedsReplay(ctx, TriCorners(ma, pair.I), TriCorners(mb, pair.J), slack, budget)
				if err != nil {
					return err
				}
				results[i].Measured = true
				results[i].Far = far
			}
		}
		return ctx.Err()
	}
}
