package prismboolean

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
)

type blindPublishedFace struct {
	face, immediate, leaf                        *topo.Face
	immediateBody, leafBody                      *topo.Body
	own, immediateOrigins, leafOrigins, upstream []provenance.Ref
}

// BlindPublishedWitness retains the ledger binding independently of the
// published slices. It is created and attached before the Cut is committed.
type BlindPublishedWitness struct {
	output, target, group *topo.Body
	leaves                []*topo.Body
	faces                 []blindPublishedFace
}

func cloneWitnessRefs(ctx context.Context, refs []provenance.Ref, b *proofbound.ReplayBudget) ([]provenance.Ref, error) {
	if err := b.ReserveRecords(ctx, uint64(len(refs))); err != nil {
		return nil, err
	}
	if err := b.ReserveArray(ctx, uint64(len(refs)), uint64(unsafe.Sizeof(provenance.Ref{}))); err != nil {
		return nil, err
	}
	if err := b.SpendWork(ctx, uint64(len(refs))); err != nil {
		return nil, err
	}
	return append([]provenance.Ref(nil), refs...), nil
}

func newBlindPublishedWitness(ctx context.Context, in BlindFaceLedgerInput, ledger *BlindFaceLedger,
	table topo.UpstreamRoles, b *proofbound.ReplayBudget) (*BlindPublishedWitness, error) {
	if err := b.ReserveRecords(ctx, uint64(1+len(in.Leaves)+len(table))); err != nil {
		return nil, err
	}
	if err := b.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(BlindPublishedWitness{}))); err != nil {
		return nil, err
	}
	if err := b.ReserveArray(ctx, uint64(len(in.Leaves)), uint64(unsafe.Sizeof((*topo.Body)(nil)))); err != nil {
		return nil, err
	}
	if err := b.ReserveArray(ctx, uint64(len(table)), uint64(unsafe.Sizeof(blindPublishedFace{}))); err != nil {
		return nil, err
	}
	if err := b.SpendWork(ctx, uint64(1+len(in.Leaves)+len(table))); err != nil {
		return nil, err
	}
	w := &BlindPublishedWitness{output: in.Output, target: in.Target, group: in.Group,
		leaves: append([]*topo.Body(nil), in.Leaves...), faces: make([]blindPublishedFace, len(table))}
	for i := range table {
		entry, ok := ledger.Entry(i)
		if !ok || entry.Output == nil || entry.Immediate == nil || entry.Leaf == nil {
			return nil, nil //nolint:nilnil // A nil result without an error means the bounded proof declined.
		}
		f := &w.faces[i]
		f.face, f.immediate, f.leaf = entry.Output, entry.Immediate, entry.Leaf
		f.immediateBody, f.leafBody = entry.Immediate.Body, entry.Leaf.Body
		var err error
		if f.own, err = cloneWitnessRefs(ctx, entry.Output.Origins, b); err != nil {
			return nil, err
		}
		if f.immediateOrigins, err = cloneWitnessRefs(ctx, entry.Immediate.Origins, b); err != nil {
			return nil, err
		}
		if f.leafOrigins, err = cloneWitnessRefs(ctx, entry.Leaf.Origins, b); err != nil {
			return nil, err
		}
		if f.upstream, err = cloneWitnessRefs(ctx, table[i].Origins, b); err != nil {
			return nil, err
		}
	}
	return w, nil
}

func sameWitnessRefs(ctx context.Context, actual, expected []provenance.Ref, b *proofbound.ReplayBudget) (bool, error) {
	if len(actual) != len(expected) {
		return false, nil
	}
	for i := range expected {
		if len(actual[i].Role) > 256 || len(expected[i].Role) > 256 {
			return false, nil
		}
		if err := b.SpendWork(ctx, uint64(len(actual[i].Role)+len(expected[i].Role))+1); err != nil {
			return false, err
		}
		if actual[i] != expected[i] {
			return false, nil
		}
	}
	return true, nil
}

// Recheck authenticates the published roles, source faces, output faces and
// ordered leaves against the pre-commit ledger witness under the caller meter.
func (w *BlindPublishedWitness) Recheck(ctx context.Context, output, target, group *topo.Body,
	leaves []*topo.Body, b *proofbound.ReplayBudget) (bool, error) {
	if w == nil || b == nil {
		return false, nil
	}
	if err := b.SpendWork(ctx, 64); err != nil {
		return false, err
	}
	if output == nil || output != w.output || target != w.target || group != w.group || len(leaves) != len(w.leaves) {
		return false, nil
	}
	for i := range leaves {
		if err := b.SpendWork(ctx, 1); err != nil {
			return false, err
		}
		if leaves[i] != w.leaves[i] {
			return false, nil
		}
	}
	payload, ok := output.Payload.(stackedrecord.Payload)
	if !ok || len(payload.Upstream) != len(w.faces) {
		return false, nil
	}
	var seen int
	for _, lump := range output.Lumps {
		if err := b.SpendWork(ctx, 1); err != nil {
			return false, err
		}
		if lump == nil {
			return false, nil
		}
		for _, shell := range lump.Shells {
			if err := b.SpendWork(ctx, 1); err != nil {
				return false, err
			}
			if shell == nil {
				return false, nil
			}
			for _, face := range shell.Faces {
				found := 0
				for i := range w.faces {
					if err := b.SpendWork(ctx, 1); err != nil {
						return false, err
					}
					if face == w.faces[i].face {
						found++
					}
				}
				if found != 1 {
					return false, nil
				}
				seen++
			}
		}
	}
	if seen != len(w.faces) {
		return false, nil
	}
	for _, expected := range w.faces {
		count := 0
		for _, lump := range output.Lumps {
			if err := b.SpendWork(ctx, 1); err != nil {
				return false, err
			}
			for _, shell := range lump.Shells {
				if err := b.SpendWork(ctx, 1); err != nil {
					return false, err
				}
				for _, face := range shell.Faces {
					if err := b.SpendWork(ctx, 1); err != nil {
						return false, err
					}
					if face == expected.face {
						count++
					}
				}
			}
		}
		if count != 1 {
			return false, nil
		}
	}
	for i, f := range w.faces {
		if err := b.SpendWork(ctx, 32); err != nil {
			return false, err
		}
		if f.face == nil || f.face.Body != output || f.face.Source != nil || f.face.Denoted != nil ||
			f.immediate == nil || f.leaf == nil ||
			f.immediate.Body != f.immediateBody || f.leaf.Body != f.leafBody ||
			len(f.immediate.Upstream) != 0 || len(f.leaf.Upstream) != 0 {
			return false, nil
		}
		for _, pair := range [5][2][]provenance.Ref{
			{f.face.Origins, f.own}, {f.face.Upstream, f.upstream},
			{f.immediate.Origins, f.immediateOrigins}, {f.leaf.Origins, f.leafOrigins},
			{payload.Upstream[i].Origins, f.upstream},
		} {
			equal, err := sameWitnessRefs(ctx, pair[0], pair[1], b)
			if err != nil || !equal {
				return false, err
			}
		}
		roles := payload.Upstream[i].Roles
		if len(roles) != len(f.own) {
			return false, nil
		}
		for j, role := range roles {
			if len(role) > 256 {
				return false, nil
			}
			if err := b.SpendWork(ctx, uint64(len(role)+len(f.own[j].Role))+1); err != nil {
				return false, err
			}
			if role != f.own[j].Role {
				return false, nil
			}
		}
		for _, source := range [...]*topo.Face{f.face, f.immediate, f.leaf} {
			if source.Body == nil {
				return false, nil
			}
			ok, err := source.Body.ReplayPrismBuiltFace(ctx, source, b)
			if err != nil || !ok {
				return false, err
			}
		}
	}
	return true, ctx.Err()
}
