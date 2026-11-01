package topo

import (
	"context"
	"math/big"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
)

// SealPrismBuildFacesReplay reserves the topology and cap snapshot storage
// before the ordinary construction seal copies any of it.
func (b *Body) SealPrismBuildFacesReplay(ctx context.Context, faces []*Face,
	budget proof.ReplayKeyBudget) (bool, error) {
	if budget == nil {
		return false, nil
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return false, err
	}
	if b == nil || b.prismBuildSeal != nil || len(faces) == 0 || len(faces) > prismSealLimit {
		return false, nil
	}
	var loops, edges, caps uint64
	var rationalBytes uint64
	for _, f := range faces {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return false, err
		}
		if f == nil || f.Body != b || len(f.Origins) != 1 {
			return false, nil
		}
		if f.CapRecord != nil {
			caps++
			if rat := f.CapRecord.ZDenoted; rat != nil {
				if err := budget.SpendWork(ctx, 2); err != nil {
					return false, err
				}
				if rat.Num().BitLen() > 4096 || rat.Denom().BitLen() > 4096 {
					return false, nil
				}
				rationalBytes += uint64(unsafe.Sizeof(big.Rat{})) +
					uint64((rat.Num().BitLen()+7)/8+(rat.Denom().BitLen()+7)/8)*4 + 256
			}
		}
		for _, l := range f.Loops {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return false, err
			}
			loops++
			if l == nil || loops > prismSealLimit {
				return false, nil
			}
			for range l.CoEdges {
				if err := budget.SpendWork(ctx, 1); err != nil {
					return false, err
				}
				edges++
				if edges > prismSealLimit {
					return false, nil
				}
			}
		}
	}
	fcount := uint64(len(faces))
	// Every slice and cap clone in SealPrismBuildFaces is covered here.
	// The extra per-record room covers allocator headers and slice growth.
	bytes := uint64(unsafe.Sizeof(prismBuildSeal{})) +
		fcount*(uint64(unsafe.Sizeof(prismSealedFace{}))+uint64(unsafe.Sizeof(surfacegeom.Cylinder{}))+64) +
		loops*(uint64(unsafe.Sizeof(prismSealedLoop{}))+64) +
		edges*(uint64(unsafe.Sizeof(prismSealedEdge{}))+64) +
		caps*(uint64(unsafe.Sizeof(CapRecord{}))+64) + rationalBytes
	if err := budget.ReserveRecords(ctx, 1+2*fcount+loops+edges+caps); err != nil {
		return false, err
	}
	if err := budget.ReserveBytes(ctx, bytes); err != nil {
		return false, err
	}
	var spendErr error
	spend := func(n uint64) error {
		spendErr = budget.SpendWork(ctx, n)
		return spendErr
	}
	ok := b.SealPrismBuildFaces(faces, spend)
	if spendErr != nil {
		return false, spendErr
	}
	return ok, nil
}

// SealPrismBuildPayloadReplay reserves the supported profile snapshot before
// the construction seal copies its profile, loop and segment slices.
func (b *Body) SealPrismBuildPayloadReplay(ctx context.Context, frame r3.Frame, xform r3.Transform,
	z0, z1, z0Delta, z1Delta, sectionDelta float64,
	z0Denoted, z1Denoted prismplacement.LevelDenotation, profiles []momentinput.Profile,
	budget proof.ReplayKeyBudget) (bool, error) {
	if budget == nil {
		return false, nil
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return false, err
	}
	if b == nil || b.prismBuildSeal == nil || b.prismBuildSeal.payload != nil ||
		len(profiles) == 0 || z0Delta != 0 || z1Delta != 0 {
		return false, nil
	}
	z0Bytes, ok := prismplacement.LevelDenotationSnapshotCost(z0Denoted)
	if !ok {
		return false, nil
	}
	z1Bytes, ok := prismplacement.LevelDenotationSnapshotCost(z1Denoted)
	if !ok {
		return false, nil
	}
	var loops, holes, segments uint64
	for _, p := range profiles {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return false, err
		}
		holes += uint64(len(p.Holes))
		loops += 1 + uint64(len(p.Holes))
		if loops > prismSealLimit {
			return false, nil
		}
		for li := -1; li < len(p.Holes); li++ {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return false, err
			}
			loop := p.Outer
			if li >= 0 {
				loop = p.Holes[li]
			}
			for _, seg := range loop.Segments {
				if err := budget.SpendWork(ctx, 1); err != nil {
					return false, err
				}
				segments++
				if segments > prismSealLimit {
					return false, nil
				}
				switch seg.(type) {
				case sectionrecord.LineSeg, sectionrecord.CircleSeg, sectionrecord.ArcSeg:
				default:
					return false, nil
				}
			}
		}
	}
	pcount := uint64(len(profiles))
	bytes := uint64(unsafe.Sizeof(prismPayloadSeal{})) +
		pcount*(uint64(unsafe.Sizeof(momentinput.Profile{}))+64) +
		holes*(uint64(unsafe.Sizeof(sectionrecord.LoopRecord{}))+64) +
		segments*(uint64(unsafe.Sizeof(sectionrecord.CurveSegment(nil)))+64) + z0Bytes + z1Bytes
	if err := budget.ReserveRecords(ctx, 1+pcount+holes+loops+segments+2); err != nil {
		return false, err
	}
	if err := budget.ReserveBytes(ctx, bytes); err != nil {
		return false, err
	}
	var spendErr error
	spend := func(n uint64) error {
		spendErr = budget.SpendWork(ctx, n)
		return spendErr
	}
	ok = b.SealPrismBuildPayload(frame, xform, z0, z1, z0Delta, z1Delta, sectionDelta,
		z0Denoted, z1Denoted, profiles, spend)
	if spendErr != nil {
		return false, spendErr
	}
	return ok, nil
}
