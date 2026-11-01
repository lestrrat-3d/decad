package decad

import (
	"context"
	"slices"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

type brakeSliceIdentity struct {
	data             unsafe.Pointer
	length, capacity int
}

type brakeToolSnapshot struct {
	payload  facetedrecord.Payload
	identity [8]brakeSliceIdentity
}

func brakeSliceID[T any](s []T) brakeSliceIdentity {
	return brakeSliceIdentity{data: unsafe.Pointer(unsafe.SliceData(s)), length: len(s), capacity: cap(s)}
}

func brakePayloadIDs(p facetedrecord.Payload) [8]brakeSliceIdentity {
	return [8]brakeSliceIdentity{brakeSliceID(p.Verts), brakeSliceID(p.VertexBound), brakeSliceID(p.Tris),
		brakeSliceID(p.Src), brakeSliceID(p.Groups), brakeSliceID(p.FaceOf),
		brakeSliceID(p.ExactSourceVerts), brakeSliceID(p.ExactSourceTris)}
}

func brakeSnapshotSlice[T any](ctx context.Context, src []T, b *proofbound.ReplayBudget) ([]T, error) {
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

func snapshotBrakeTool(ctx context.Context, p facetedrecord.Payload, b *proofbound.ReplayBudget) (brakeToolSnapshot, error) {
	out := brakeToolSnapshot{payload: p, identity: brakePayloadIDs(p)}
	var err error
	if out.payload.Verts, err = brakeSnapshotSlice(ctx, p.Verts, b); err != nil {
		return brakeToolSnapshot{}, err
	}
	if out.payload.VertexBound, err = brakeSnapshotSlice(ctx, p.VertexBound, b); err != nil {
		return brakeToolSnapshot{}, err
	}
	if out.payload.Tris, err = brakeSnapshotSlice(ctx, p.Tris, b); err != nil {
		return brakeToolSnapshot{}, err
	}
	if out.payload.Src, err = brakeSnapshotSlice(ctx, p.Src, b); err != nil {
		return brakeToolSnapshot{}, err
	}
	if out.payload.FaceOf, err = brakeSnapshotSlice(ctx, p.FaceOf, b); err != nil {
		return brakeToolSnapshot{}, err
	}
	if out.payload.ExactSourceVerts, err = brakeSnapshotSlice(ctx, p.ExactSourceVerts, b); err != nil {
		return brakeToolSnapshot{}, err
	}
	if out.payload.ExactSourceTris, err = brakeSnapshotSlice(ctx, p.ExactSourceTris, b); err != nil {
		return brakeToolSnapshot{}, err
	}
	if out.payload.Groups, err = brakeSnapshotSlice(ctx, p.Groups, b); err != nil {
		return brakeToolSnapshot{}, err
	}
	for i := range p.Groups {
		if out.payload.Groups[i].Origins, err = brakeSnapshotSlice(ctx, p.Groups[i].Origins, b); err != nil {
			return brakeToolSnapshot{}, err
		}
		if out.payload.Groups[i].Upstream, err = brakeSnapshotSlice(ctx, p.Groups[i].Upstream, b); err != nil {
			return brakeToolSnapshot{}, err
		}
	}
	return out, nil
}

func (s brakeToolSnapshot) unchanged(ctx context.Context, p facetedrecord.Payload, b *proofbound.ReplayBudget) (bool, error) {
	if err := b.SpendWork(ctx, 8); err != nil {
		return false, err
	}
	if s.identity != brakePayloadIDs(p) {
		return false, nil
	}
	return brakeCompare(ctx, s.payload, p, b)
}
