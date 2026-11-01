package prismboolean

import (
	"context"
	"strings"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// ReplayPublishBlindUpstream binds each output face's own role to the exact
// source faces in ledger. It stages all copies and rechecks the ledger before
// changing the uncommitted result. The caller publishes the body separately.
func ReplayPublishBlindUpstream(ctx context.Context, in BlindFaceLedgerInput,
	ledger *BlindFaceLedger, budget *proofbound.ReplayBudget) (bool, error) {
	if ledger == nil || ledger.published != nil || budget == nil || !ledger.Matches(in.Output, budget) ||
		in.Output == nil || ledger.Len() == 0 || ledger.Len() > 4096 {
		return false, nil
	}
	if err := budget.ReserveRecords(ctx, uint64(ledger.Len())); err != nil {
		return false, err
	}
	if err := budget.ReserveArray(ctx, uint64(ledger.Len()), uint64(unsafe.Sizeof(topo.UpstreamEntry{}))); err != nil {
		return false, err
	}
	table := make(topo.UpstreamRoles, ledger.Len())
	if err := budget.ReserveArray(ctx, uint64(ledger.Len()), uint64(unsafe.Sizeof((*topo.Face)(nil)))); err != nil {
		return false, err
	}
	faces := make([]*topo.Face, ledger.Len())
	for i := range table {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return false, err
		}
		entry, ok := ledger.Entry(i)
		if !ok || entry.Output == nil || entry.Output.Body != in.Output ||
			entry.Immediate == nil || entry.Leaf == nil || len(entry.Output.Origins) == 0 ||
			len(entry.Output.Origins) > 32 || len(entry.Output.Upstream) != 0 {
			return false, nil
		}
		faces[i] = entry.Output
		if err := budget.ReserveArray(ctx, uint64(len(entry.Output.Origins)), uint64(unsafe.Sizeof(""))); err != nil {
			return false, err
		}
		roles := make([]string, len(entry.Output.Origins))
		for j, own := range entry.Output.Origins {
			if own.Producer != in.Output.Origin.Producer || len(own.Role) == 0 || len(own.Role) > 256 {
				return false, nil
			}
			if err := budget.ReserveBundle(ctx, uint64(len(own.Role))+1, 1, 2*uint64(len(own.Role))+128); err != nil {
				return false, err
			}
			roles[j] = strings.Clone(own.Role)
		}
		refs := []provenance.Ref(nil)
		for _, source := range []*topo.Face{entry.Immediate, entry.Leaf} {
			// This route authenticates only direct producer faces. A prior
			// Boolean's inherited roles need their own immutable history seal.
			if len(source.Upstream) != 0 {
				return false, nil
			}
			count := len(source.Origins) + len(source.Upstream)
			if count > 4096-len(refs) {
				return false, nil
			}
			for _, group := range [][]provenance.Ref{source.Origins, source.Upstream} {
				for _, ref := range group {
					if len(ref.Role) == 0 || len(ref.Role) > 256 {
						return false, nil
					}
					duplicate := false
					for _, old := range refs {
						if err := budget.SpendWork(ctx, uint64(len(old.Role)+len(ref.Role))+1); err != nil {
							return false, err
						}
						if old == ref {
							duplicate = true
							break
						}
					}
					if duplicate {
						continue
					}
					if err := budget.ReserveBundle(ctx, uint64(len(ref.Role))+1, 1, 2*uint64(len(ref.Role))+128); err != nil {
						return false, err
					}
					if err := reserveBlindRefAppend(ctx, len(refs), cap(refs), budget); err != nil {
						return false, err
					}
					ref.Role = strings.Clone(ref.Role)
					refs = append(refs, ref)
				}
			}
		}
		if len(refs) == 0 {
			return false, nil
		}
		table[i] = topo.UpstreamEntry{Roles: roles, Origins: refs}
	}
	for i := range table {
		for j := range i {
			if err := budget.SpendWork(ctx, uint64(len(table[i].Roles))*257+1); err != nil {
				return false, err
			}
			if sameBlindRoles(table[i].Roles, table[j].Roles) {
				return false, nil
			}
		}
	}
	for _, row := range table {
		if err := budget.ReserveRecords(ctx, uint64(len(row.Origins))); err != nil {
			return false, err
		}
		if err := budget.ReserveArray(ctx, uint64(len(row.Origins)), uint64(unsafe.Sizeof(provenance.Ref{}))); err != nil {
			return false, err
		}
	}
	payload, ok := in.Output.Payload.(stackedrecord.Payload)
	if !ok || len(payload.Upstream) != 0 {
		return false, nil
	}
	checked, err := ledger.Recheck(ctx, in, budget)
	if err != nil || !checked {
		return false, err
	}
	witness, err := newBlindPublishedWitness(ctx, in, ledger, table, budget)
	if err != nil || witness == nil {
		return false, err
	}
	if err := ctx.Err(); err != nil {
		return false, err
	}
	for i, face := range faces {
		face.Upstream = append([]provenance.Ref(nil), table[i].Origins...)
	}
	payload.Upstream = table
	in.Output.Payload = payload
	ledger.published = witness
	return true, nil
}

func reserveBlindRefAppend(ctx context.Context, length, capacity int, budget *proofbound.ReplayBudget) error {
	if length < capacity {
		return nil
	}
	newCap := max(1, 2*capacity)
	return budget.ReserveArray(ctx, uint64(newCap), uint64(unsafe.Sizeof(provenance.Ref{})))
}

func sameBlindRoles(a, b []string) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if a[i] != b[i] {
			return false
		}
	}
	return true
}
