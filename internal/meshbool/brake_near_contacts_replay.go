package meshbool

import (
	"context"
	"math"
	"sort"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func gatherNearContactsShared(ctx context.Context, bmA *BoolMesh, fis []int, bmB *BoolMesh, fjs []int, slack float64, memo *ContactMemo, run ContactBatchRunner, limit int, budget *proofbound.WorkBudget) (NearContacts, bool, error) {
	return gatherNearContactsSharedReplay(ctx, bmA, fis, bmB, fjs, slack, memo, run, limit, budget, nil)
}

func gatherNearContactsSharedReplay(ctx context.Context, bmA *BoolMesh, fis []int, bmB *BoolMesh, fjs []int, slack float64, memo *ContactMemo, run ContactBatchRunner, limit int, budget *proofbound.WorkBudget, replay *proofbound.ReplayBudget) (NearContacts, bool, error) {
	var nc NearContacts
	var seenA, seenB map[int]bool
	var fixedA, fixedB []bool
	if replay != nil {
		for _, n := range []int{len(bmA.Tris), len(bmB.Tris)} {
			if err := replay.ReserveRecords(ctx, uint64(n)); err != nil {
				return NearContacts{}, false, err
			}
			if err := replay.SpendWork(ctx, uint64(n)); err != nil {
				return NearContacts{}, false, err
			}
			if err := replay.ReserveArray(ctx, uint64(n), uint64(unsafe.Sizeof(false))); err != nil {
				return NearContacts{}, false, err
			}
		}
		fixedA = make([]bool, len(bmA.Tris))
		fixedB = make([]bool, len(bmB.Tris))
	} else {
		seenA = map[int]bool{}
		seenB = map[int]bool{}
	}
	work := 0
	if err := budget.Reserve(1); err != nil {
		return NearContacts{}, false, err
	}
	if err := budget.ReserveBytesMul(1, uint64(unsafe.Sizeof(ContactBatchExecutor{}))+256); err != nil {
		return NearContacts{}, false, err
	}
	contacts := NewContactBatchExecutor(ctx, bmA, bmB, memo, ContactWorkers(ctx), nil)
	if replay != nil {
		if limit < 0 || uint64(limit) > ^uint64(0)/4 {
			return NearContacts{}, false, errBrakeContactBatchCapacity
		}
		if err := replay.ReserveRecords(ctx, 4*uint64(limit)); err != nil {
			return NearContacts{}, false, err
		}
		if err := replay.SpendWork(ctx, 4*uint64(limit)); err != nil {
			return NearContacts{}, false, err
		}
		contacts.NoMemoStore = true
		contacts.NoMemoLookup = true
		contacts.Workers = 1
		for _, size := range []uintptr{unsafe.Sizeof(ContactBatchEntry{}), unsafe.Sizeof(ContactPair{}), unsafe.Sizeof(ContactBatchResult{}), unsafe.Sizeof(TriContact{})} {
			if err := replay.ReserveArray(ctx, uint64(limit), uint64(size)); err != nil {
				return NearContacts{}, false, err
			}
		}
		contacts.Batch = make([]ContactBatchEntry, 0, limit)
		contacts.Misses = make([]ContactPair, 0, limit)
		contacts.Results = make([]ContactBatchResult, 0, limit)
		contacts.memoHits = make([]TriContact, 0, limit)
	}
	contacts.Run = run
	contacts.Limit = limit
	contacts.ConsumeResult = func(pair ContactPair, r ContactBatchResult) error {
		i, j := pair.I, pair.J
		c := r.Contact
		if c.Kind == ContactRegion {
			return ErrContactBatchStop
		}
		if c.Kind == ContactNone {
			far := r.Far
			if !r.Measured { // a memo hit: no worker measured it
				far = TriTriDistanceExceeds(TriCorners(bmA, i), TriCorners(bmB, j), slack)
			}
			if far {
				return nil
			}
		}
		if replay != nil {
			var err error
			if c.Kind == ContactSegment {
				nc.Spans, err = appendWork(nc.Spans, []ContactSpan{{I: i, J: j, P0: c.P0, P1: c.P1}}, budget)
				if err != nil {
					return err
				}
			}
			if !fixedA[i] {
				fixedA[i] = true
				nc.CloseA, err = appendWork(nc.CloseA, []int{i}, budget)
				if err != nil {
					return err
				}
			}
			if !fixedB[j] {
				fixedB[j] = true
				nc.CloseB, err = appendWork(nc.CloseB, []int{j}, budget)
				if err != nil {
					return err
				}
			}
		} else {
			if c.Kind == ContactSegment {
				nc.Spans = append(nc.Spans, ContactSpan{I: i, J: j, P0: c.P0, P1: c.P1})
			}
			if !seenA[i] {
				seenA[i] = true
				nc.CloseA = append(nc.CloseA, i)
			}
			if !seenB[j] {
				seenB[j] = true
				nc.CloseB = append(nc.CloseB, j)
			}
		}
		return nil
	}
	// A face pair usually queues few contacts, so keep its initial buffers
	// small without changing when the ordered batch is flushed.
	contacts.initialCapacity = contactBatchInitialCapacity
	useTree := len(fis) > 0 && len(fjs) > 0 && len(fis)*len(fjs) > 16384 && slack >= 0 && !math.IsNaN(slack)
	var tree heldBoxTree
	var treeStack []int
	var candidateIDs []int
	nextPoll := 256
	if useTree {
		if budget != nil {
			if _, err := budget.StepN(uint64(len(fjs))); err != nil {
				return NearContacts{}, false, err
			}
		}
		if err := budget.Reserve(uint64(5 * len(fjs))); err != nil {
			return NearContacts{}, false, err
		}
		if err := budget.ReserveBytesMul(uint64(len(fjs)), uint64(unsafe.Sizeof([2]r3.Vec{}))); err != nil {
			return NearContacts{}, false, err
		}
		boxes := make([][2]r3.Vec, len(fjs))
		for position, j := range fjs {
			boxes[position] = bmB.Boxes[j]
		}
		if budget == nil {
			tree = newQueryBoxTree(boxes)
		} else {
			var err error
			tree, err = newBoxTreeWork(boxes, budget)
			if err != nil {
				return NearContacts{}, false, err
			}
			if err := budget.Reserve(uint64(len(tree.nodes))); err != nil {
				return NearContacts{}, false, err
			}
			if err := budget.ReserveBytesMul(uint64(len(tree.nodes)), uint64(unsafe.Sizeof(int(0)))); err != nil {
				return NearContacts{}, false, err
			}
			if replay != nil {
				if _, err := budget.StepN(uint64(len(tree.nodes))); err != nil {
					return NearContacts{}, false, err
				}
			}
			treeStack = make([]int, 0, len(tree.nodes))
		}
		if err := budget.Reserve(uint64(len(fjs))); err != nil {
			return NearContacts{}, false, err
		}
		if err := budget.ReserveBytesMul(uint64(len(fjs)), uint64(unsafe.Sizeof(int(0)))); err != nil {
			return NearContacts{}, false, err
		}
		if replay != nil {
			if _, err := budget.StepN(uint64(len(fjs))); err != nil {
				return NearContacts{}, false, err
			}
		}
		candidateIDs = make([]int, 0, len(fjs))
	}
	for _, i := range fis {
		if useTree {
			candidateIDs = candidateIDs[:0]
			box := expandBoxForSlack(bmA.Boxes[i], slack)
			collect := func(position int) error {
				candidateIDs = append(candidateIDs, position)
				return nil
			}
			var err error
			if budget == nil {
				err = tree.visitOverlaps(box, nil, collect)
			} else {
				err = tree.visitOverlapsWork(box, budget.Step, treeStack, collect)
			}
			if err != nil {
				return NearContacts{}, false, err
			}
			if budget == nil {
				sort.Ints(candidateIDs)
			} else {
				for i := 1; i < len(candidateIDs); i++ {
					for j := i; j > 0; j-- {
						if err := budget.Step(); err != nil {
							return NearContacts{}, false, err
						}
						if candidateIDs[j] >= candidateIDs[j-1] {
							break
						}
						candidateIDs[j], candidateIDs[j-1] = candidateIDs[j-1], candidateIDs[j]
					}
				}
			}
			rowStart := work
			for _, position := range candidateIDs {
				if budget != nil {
					if err := budget.Step(); err != nil {
						return NearContacts{}, false, err
					}
				}
				target := rowStart + position + 1
				for nextPoll <= target {
					if err := ctx.Err(); err != nil {
						return NearContacts{}, false, err
					}
					nextPoll += 256
				}
				j := fjs[position]
				if !BoxesWithin(bmA.Boxes[i], bmB.Boxes[j], slack) {
					continue
				}
				if budget != nil {
					if _, err := budget.StepN(3); err != nil {
						return NearContacts{}, false, err
					}
				}
				if err := contacts.Add(i, j); err != nil {
					if err == ErrContactBatchStop {
						return NearContacts{}, true, nil
					}
					return NearContacts{}, false, err
				}
			}
			rowEnd := rowStart + len(fjs)
			for nextPoll <= rowEnd {
				if err := ctx.Err(); err != nil {
					return NearContacts{}, false, err
				}
				nextPoll += 256
			}
			work = rowEnd
			continue
		}
		for _, j := range fjs {
			if budget != nil {
				if err := budget.Step(); err != nil {
					return NearContacts{}, false, err
				}
			}
			work++
			if work%256 == 0 {
				if err := ctx.Err(); err != nil {
					return NearContacts{}, false, err
				}
			}
			if !BoxesWithin(bmA.Boxes[i], bmB.Boxes[j], slack) {
				continue
			}
			if budget != nil {
				if _, err := budget.StepN(3); err != nil {
					return NearContacts{}, false, err
				}
			}
			if err := contacts.Add(i, j); err != nil {
				if err == ErrContactBatchStop {
					return NearContacts{}, true, nil
				}
				return NearContacts{}, false, err
			}
		}
	}
	if err := contacts.Done(); err != nil {
		if err == ErrContactBatchStop {
			return NearContacts{}, true, nil
		}
		return NearContacts{}, false, err
	}
	return nc, false, nil
}
