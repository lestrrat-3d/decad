package booleaneval

import (
	"context"
	"fmt"
	"math/big"
	"slices"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

type candidateSliceIdentity struct {
	data             unsafe.Pointer
	length, capacity int
}

type candidatePayloadSnapshot struct {
	payload    facetedrecord.Payload
	identity   [8]candidateSliceIdentity
	capSources []*topo.CapRecord
}

func candidateSliceID[T any](s []T) candidateSliceIdentity {
	return candidateSliceIdentity{data: unsafe.Pointer(unsafe.SliceData(s)), length: len(s), capacity: cap(s)}
}

func candidatePayloadIDs(p facetedrecord.Payload) [8]candidateSliceIdentity {
	return [8]candidateSliceIdentity{candidateSliceID(p.Verts), candidateSliceID(p.VertexBound), candidateSliceID(p.Tris),
		candidateSliceID(p.Src), candidateSliceID(p.Groups), candidateSliceID(p.FaceOf),
		candidateSliceID(p.ExactSourceVerts), candidateSliceID(p.ExactSourceTris)}
}

func candidateSnapshotSlice[T any](ctx context.Context, src []T, b *proofbound.ReplayBudget) ([]T, error) {
	if err := b.SpendWork(ctx, uint64(len(src))); err != nil {
		return nil, err
	}
	if err := b.ReserveRecords(ctx, uint64(len(src))); err != nil {
		return nil, err
	}
	if err := b.ReserveArray(ctx, uint64(len(src)), uint64(unsafe.Sizeof(*new(T)))); err != nil {
		return nil, err
	}
	return slices.Clone(src), nil
}

func snapshotCandidatePayload(ctx context.Context, p facetedrecord.Payload, b *proofbound.ReplayBudget) (candidatePayloadSnapshot, error) {
	out := candidatePayloadSnapshot{payload: p, identity: candidatePayloadIDs(p)}
	var err error
	if out.payload.Verts, err = candidateSnapshotSlice(ctx, p.Verts, b); err != nil {
		return candidatePayloadSnapshot{}, err
	}
	if out.payload.VertexBound, err = candidateSnapshotSlice(ctx, p.VertexBound, b); err != nil {
		return candidatePayloadSnapshot{}, err
	}
	if out.payload.Tris, err = candidateSnapshotSlice(ctx, p.Tris, b); err != nil {
		return candidatePayloadSnapshot{}, err
	}
	if out.payload.Src, err = candidateSnapshotSlice(ctx, p.Src, b); err != nil {
		return candidatePayloadSnapshot{}, err
	}
	if out.payload.FaceOf, err = candidateSnapshotSlice(ctx, p.FaceOf, b); err != nil {
		return candidatePayloadSnapshot{}, err
	}
	if out.payload.ExactSourceVerts, err = candidateSnapshotSlice(ctx, p.ExactSourceVerts, b); err != nil {
		return candidatePayloadSnapshot{}, err
	}
	if out.payload.ExactSourceTris, err = candidateSnapshotSlice(ctx, p.ExactSourceTris, b); err != nil {
		return candidatePayloadSnapshot{}, err
	}
	if out.payload.Groups, err = candidateSnapshotSlice(ctx, p.Groups, b); err != nil {
		return candidatePayloadSnapshot{}, err
	}
	if err := b.ReserveRecords(ctx, uint64(len(p.Groups))); err != nil {
		return candidatePayloadSnapshot{}, err
	}
	if err := b.ReserveArray(ctx, uint64(len(p.Groups)), uint64(unsafe.Sizeof((*topo.CapRecord)(nil)))); err != nil {
		return candidatePayloadSnapshot{}, err
	}
	out.capSources = make([]*topo.CapRecord, len(p.Groups))
	for i := range p.Groups {
		out.capSources[i] = p.Groups[i].CapRecord
		if out.payload.Groups[i].Origins, err = candidateSnapshotSlice(ctx, p.Groups[i].Origins, b); err != nil {
			return candidatePayloadSnapshot{}, err
		}
		if out.payload.Groups[i].Upstream, err = candidateSnapshotSlice(ctx, p.Groups[i].Upstream, b); err != nil {
			return candidatePayloadSnapshot{}, err
		}
		if p.Groups[i].Source != nil {
			if err := b.ReserveRecords(ctx, 1); err != nil {
				return candidatePayloadSnapshot{}, err
			}
			if err := b.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(facetedrecord.SourcePatch{}))); err != nil {
				return candidatePayloadSnapshot{}, err
			}
			v := *p.Groups[i].Source
			out.payload.Groups[i].Source = &v
		}
		if p.Groups[i].CapRecord != nil {
			if err := b.ReserveRecords(ctx, 1); err != nil {
				return candidatePayloadSnapshot{}, err
			}
			if err := b.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(topo.CapRecord{}))); err != nil {
				return candidatePayloadSnapshot{}, err
			}
			v := *p.Groups[i].CapRecord
			if v.ZDenoted != nil {
				v.ZDenoted, err = copyCandidateRat(ctx, v.ZDenoted, b)
				if err != nil {
					return candidatePayloadSnapshot{}, err
				}
			}
			out.payload.Groups[i].CapRecord = &v
		}
	}
	return out, nil
}

func copyCandidateRat(ctx context.Context, q *big.Rat, b *proofbound.ReplayBudget) (*big.Rat, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if q == nil || b == nil || q.Denom().Sign() <= 0 || q.Num().BitLen() > 4096 || q.Denom().BitLen() > 4096 {
		return nil, fmt.Errorf("%w: candidate exact level width", decaderr.ErrUnsupported)
	}
	n, d := uint64(len(q.Num().Bits())), uint64(len(q.Denom().Bits()))
	if n > ^uint64(0)-4 || d > ^uint64(0)-4 || n+d > ^uint64(0)-8 {
		return nil, fmt.Errorf("%w: candidate exact level allocation overflow", decaderr.ErrUnsupported)
	}
	if err := b.SpendWork(ctx, n+d+8); err != nil {
		return nil, err
	}
	if err := b.ReserveRecords(ctx, 3); err != nil {
		return nil, err
	}
	if err := b.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(big.Rat{}))); err != nil {
		return nil, err
	}
	for _, count := range []uint64{n + 4, d + 4} {
		if err := b.ReserveArray(ctx, count, uint64(unsafe.Sizeof(big.Word(0)))); err != nil {
			return nil, err
		}
	}
	return new(big.Rat).Set(q), nil
}

func (s candidatePayloadSnapshot) unchanged(ctx context.Context, p facetedrecord.Payload, b *proofbound.ReplayBudget) (bool, error) {
	if err := b.SpendWork(ctx, 8); err != nil {
		return false, err
	}
	if s.identity != candidatePayloadIDs(p) {
		return false, nil
	}
	if len(s.capSources) != len(p.Groups) {
		return false, nil
	}
	if err := b.SpendWork(ctx, uint64(len(p.Groups))); err != nil {
		return false, err
	}
	for i := range p.Groups {
		if s.capSources[i] != p.Groups[i].CapRecord {
			return false, nil
		}
	}
	return capCompare(ctx, s.payload, p, b)
}
