package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// spiralBandOneRootBox contains the complete true blank-root/fitted-flank
// seam from the finite toe root ring through the second LoftFaces cap. The
// first endpoint is isolated on the authenticated root branch. The second is
// its certified crossing on cap 2, so neither endpoint is inferred from a
// held mesh vertex.
func spiralBandOneRootBox(ctx context.Context, proof *spiralFirstJoinSource,
	side int) ([3]loftFaceInterval, bool, error) {
	var empty [3]loftFaceInterval
	if ctx == nil || proof == nil || side < 0 || side >= len(proof.Flank) ||
		proof.Blank == nil || proof.Tooth == nil || len(proof.Tooth.Maps) != 9 {
		return empty, false, nil
	}
	if err := ctx.Err(); err != nil {
		return empty, false, err
	}
	toeRoot := lfF(proof.Blank.toeRoot.U)
	cap2 := loftface.CapXRange(proof.Tooth.Maps[2], proof.Flank[side], proof.Crossings[side][2])
	if cap2.Lo == nil || cap2.Hi == nil || toeRoot.Cmp(cap2.Lo) >= 0 ||
		cap2.Hi.Cmp(lfF(proof.Blank.ded.U)) >= 0 {
		return empty, false, nil
	}
	low, ok := spiralIsolateJunction(ctx, proof.Tooth.Maps, proof.Flank[side],
		proof.RootM, proof.RootB, proof.Crossings[side], toeRoot)
	if !ok || low.Band != 1 || low.THi.Cmp(lfOne()) >= 0 {
		return empty, false, ctx.Err()
	}
	q := loftface.MeridianQRows(proof.Tooth.Maps[1], proof.Tooth.Maps[2],
		proof.Flank[side], proof.RootM, proof.RootB)
	sLo, sHi := lfZero(), lfOne()
	if !spiralJunctionRootBracket(q, sLo, sHi, low.TLo, lfOne()) {
		return empty, false, ctx.Err()
	}
	for step := range 64 {
		if step%8 == 0 {
			if err := ctx.Err(); err != nil {
				return empty, false, err
			}
		}
		mid := lfMul(lfAdd(sLo, sHi), big.NewRat(1, 2))
		if polynomial.BernsteinRowsSignAt(q, mid, low.TLo, lfOne(), -1) {
			sLo = mid
			continue
		}
		if polynomial.BernsteinRowsSignAt(q, mid, low.TLo, lfOne(), 1) {
			sHi = mid
			continue
		}
		break
	}
	box, ok := loftface.JunctionBox(proof.Tooth.Maps[1], proof.Tooth.Maps[2],
		proof.Flank[side], sLo, sHi, low.TLo, lfOne())
	if err := ctx.Err(); err != nil {
		return empty, false, err
	}
	if !ok || box[0].Lo.Cmp(toeRoot) > 0 || box[0].Hi.Cmp(cap2.Hi) < 0 {
		return empty, false, nil
	}
	// J>0 in the source certificate orders X along this single branch.
	box[0] = loftFaceInterval{Lo: toeRoot, Hi: cap2.Hi}
	return box, true, nil
}

// spiralRootFlankWholeBound checks every point of the true first-band seam
// against the complete held switched path, and conversely. The caller must
// first authenticate that held is the one complete blank-root/fitted-flank
// path and minBound is no larger than every published curve bound on it.
// Strict held X order and source J>0 let each exact time slab use the same
// X interval on both curves. An endpoint box covers the held tail after the
// source seam reaches cap 2.
func spiralRootFlankWholeBound(ctx context.Context, proof *spiralFirstJoinSource,
	side int, held []r3.Vec, minBound float64,
	prepared ...*spiralRootFlankSourceBoxes) (float64, bool, error) {
	if ctx == nil || proof == nil || side < 0 || side >= len(proof.Flank) ||
		len(held) < 2 || len(held) > 4097 || proofbound.IsNonFinite(minBound) || minBound <= 0 {
		return 0, false, nil
	}
	if err := ctx.Err(); err != nil {
		return 0, false, err
	}
	if len(prepared) > 1 {
		return 0, false, nil
	}
	if len(prepared) == 1 {
		if prepared[0] == nil || prepared[0].proof != proof || prepared[0].side != side {
			return 0, false, nil
		}
		return spiralRootFlankHeldAgainstSource(ctx, held, minBound, prepared[0])
	}
	_, ok, err := spiralBandOneRootBox(ctx, proof, side)
	if err != nil || !ok {
		return 0, false, err
	}
	if held[0].X > held[len(held)-1].X {
		reversed := make([]r3.Vec, len(held))
		for i, p := range held {
			reversed[len(held)-1-i] = p
		}
		held = reversed
	}
	for i, p := range held {
		if !proofbound.FiniteVec(p) || i > 0 && lfF(held[i-1].X).Cmp(lfF(p.X)) >= 0 {
			return 0, false, nil
		}
	}
	toeRoot := lfF(proof.Blank.toeRoot.U)
	cap2 := loftface.CapXRange(proof.Tooth.Maps[2], proof.Flank[side], proof.Crossings[side][2])
	if lfF(held[0].X).Cmp(toeRoot) != 0 || lfF(held[len(held)-1].X).Cmp(cap2.Hi) < 0 {
		return 0, false, nil
	}
	start := proof.Junctions[side][0]
	if start.Band != 1 || start.TLo.Sign() < 0 || start.TLo.Cmp(lfOne()) >= 0 {
		return 0, false, nil
	}
	first, second := proof.Tooth.Maps[1], proof.Tooth.Maps[2]
	span := proof.Flank[side]
	q := loftface.MeridianQRows(first, second, span, proof.RootM, proof.RootB)
	width := lfSub(lfOne(), start.TLo)
	const slabs = 16
	var largest float64
	for slab := range slabs {
		if err := ctx.Err(); err != nil {
			return 0, false, err
		}
		a := lfAdd(start.TLo, lfMul(width, big.NewRat(int64(slab), slabs)))
		z := lfAdd(start.TLo, lfMul(width, big.NewRat(int64(slab+1), slabs)))
		if !spiralJunctionRootBracket(q, lfZero(), lfOne(), a, z) {
			return 0, false, nil
		}
		rootA := loftface.IsolateRootN(polynomial.BernsteinRowsAt(q, a), 64)
		rootZ := loftface.IsolateRootN(polynomial.BernsteinRowsAt(q, z), 64)
		xA := loftface.BandXRange(first, second, span, rootA, a)
		xZ := loftface.BandXRange(first, second, span, rootZ, z)
		if xA.Hi.Cmp(xZ.Lo) >= 0 {
			return 0, false, nil
		}
		xLo, xHi := xA.Lo, xZ.Hi
		if slab == 0 {
			xLo = toeRoot
		}
		if slab == slabs-1 {
			xHi = cap2.Hi
		}
		if xLo.Cmp(xHi) >= 0 {
			return 0, false, nil
		}
		sLo, sHi := lfZero(), lfOne()
		for step := range 64 {
			if step%8 == 0 {
				if err := ctx.Err(); err != nil {
					return 0, false, err
				}
			}
			mid := lfMul(lfAdd(sLo, sHi), big.NewRat(1, 2))
			if polynomial.BernsteinRowsSignAt(q, mid, a, z, -1) {
				sLo = mid
				continue
			}
			if polynomial.BernsteinRowsSignAt(q, mid, a, z, 1) {
				sHi = mid
				continue
			}
			break
		}
		trueBox, yes := loftface.JunctionBox(first, second, span, sLo, sHi, a, z)
		if !yes || trueBox[0].Lo.Cmp(xLo) > 0 || trueBox[0].Hi.Cmp(xHi) < 0 {
			return 0, false, nil
		}
		trueBox[0] = loftFaceInterval{Lo: xLo, Hi: xHi}
		heldBox, yes := spiralHeldXStrip(held, xLo, xHi)
		if !yes {
			return 0, false, nil
		}
		bound, yes := spiralRootFlankBoxesBound(trueBox, heldBox, minBound)
		if !yes {
			return bound, false, nil
		}
		largest = max(largest, bound)
	}
	endpoint, ok := loftface.JunctionBox(second, second, span,
		proof.Crossings[side][2].Lo, proof.Crossings[side][2].Hi, lfZero(), lfZero())
	if !ok || endpoint[0].Lo.Cmp(cap2.Lo) > 0 || endpoint[0].Hi.Cmp(cap2.Hi) < 0 {
		return 0, false, nil
	}
	tail, ok := spiralHeldXStrip(held, cap2.Lo, lfF(held[len(held)-1].X))
	if !ok {
		return 0, false, nil
	}
	bound, ok := spiralRootFlankBoxesBound(endpoint, tail, minBound)
	if err := ctx.Err(); err != nil {
		return 0, false, err
	}
	if !ok {
		return bound, false, nil
	}
	return max(largest, bound), true, nil
}

// The source slab boxes depend only on one authenticated first-tooth record.
// A multi-body Union computes them once before validating its independent held
// switch chains. The record pointer and side bind the cache to that source.
type spiralRootFlankSourceBoxes struct {
	proof    *spiralFirstJoinSource
	side     int
	toeRoot  *big.Rat
	cap2     loftFaceInterval
	slabs    [16][3]loftFaceInterval
	endpoint [3]loftFaceInterval
}

func spiralPrepareRootFlankSource(ctx context.Context, proof *spiralFirstJoinSource,
	side int) (*spiralRootFlankSourceBoxes, bool, error) {
	if ctx == nil || proof == nil || side < 0 || side >= len(proof.Flank) ||
		proof.Blank == nil || proof.Tooth == nil || len(proof.Tooth.Maps) != 9 {
		return nil, false, nil
	}
	if err := ctx.Err(); err != nil {
		return nil, false, err
	}
	rootBox, ok, err := spiralBandOneRootBox(ctx, proof, side)
	if err != nil || !ok {
		return nil, false, err
	}
	toeRoot := lfF(proof.Blank.toeRoot.U)
	cap2 := loftface.CapXRange(proof.Tooth.Maps[2], proof.Flank[side], proof.Crossings[side][2])
	if rootBox[0].Lo.Cmp(toeRoot) != 0 || rootBox[0].Hi.Cmp(cap2.Hi) != 0 {
		return nil, false, nil
	}
	start := proof.Junctions[side][0]
	if cap2.Lo == nil || cap2.Hi == nil ||
		start.Band != 1 || start.TLo.Sign() < 0 || start.TLo.Cmp(lfOne()) >= 0 {
		return nil, false, nil
	}
	first, second := proof.Tooth.Maps[1], proof.Tooth.Maps[2]
	span := proof.Flank[side]
	q := loftface.MeridianQRows(first, second, span, proof.RootM, proof.RootB)
	width := lfSub(lfOne(), start.TLo)
	plan := &spiralRootFlankSourceBoxes{
		proof: proof, side: side, toeRoot: toeRoot, cap2: cap2,
	}
	for slab := range plan.slabs {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		a := lfAdd(start.TLo, lfMul(width, big.NewRat(int64(slab), 16)))
		z := lfAdd(start.TLo, lfMul(width, big.NewRat(int64(slab+1), 16)))
		if !spiralJunctionRootBracket(q, lfZero(), lfOne(), a, z) {
			return nil, false, nil
		}
		rootA := loftface.IsolateRootN(polynomial.BernsteinRowsAt(q, a), 64)
		rootZ := loftface.IsolateRootN(polynomial.BernsteinRowsAt(q, z), 64)
		xA := loftface.BandXRange(first, second, span, rootA, a)
		xZ := loftface.BandXRange(first, second, span, rootZ, z)
		if xA.Hi.Cmp(xZ.Lo) >= 0 {
			return nil, false, nil
		}
		xLo, xHi := xA.Lo, xZ.Hi
		if slab == 0 {
			xLo = toeRoot
		}
		if slab == len(plan.slabs)-1 {
			xHi = cap2.Hi
		}
		if xLo.Cmp(xHi) >= 0 {
			return nil, false, nil
		}
		sLo, sHi := lfZero(), lfOne()
		for step := range 64 {
			if step%8 == 0 {
				if err := ctx.Err(); err != nil {
					return nil, false, err
				}
			}
			mid := lfMul(lfAdd(sLo, sHi), big.NewRat(1, 2))
			if polynomial.BernsteinRowsSignAt(q, mid, a, z, -1) {
				sLo = mid
				continue
			}
			if polynomial.BernsteinRowsSignAt(q, mid, a, z, 1) {
				sHi = mid
				continue
			}
			break
		}
		trueBox, yes := loftface.JunctionBox(first, second, span, sLo, sHi, a, z)
		if !yes || trueBox[0].Lo.Cmp(xLo) > 0 || trueBox[0].Hi.Cmp(xHi) < 0 {
			return nil, false, nil
		}
		trueBox[0] = loftFaceInterval{Lo: xLo, Hi: xHi}
		plan.slabs[slab] = trueBox
	}
	endpoint, ok := loftface.JunctionBox(second, second, span,
		proof.Crossings[side][2].Lo, proof.Crossings[side][2].Hi, lfZero(), lfZero())
	if !ok || endpoint[0].Lo.Cmp(cap2.Lo) > 0 || endpoint[0].Hi.Cmp(cap2.Hi) < 0 {
		return nil, false, nil
	}
	plan.endpoint = endpoint
	return plan, true, ctx.Err()
}

func spiralRootFlankHeldAgainstSource(ctx context.Context, held []r3.Vec,
	minBound float64, plan *spiralRootFlankSourceBoxes) (float64, bool, error) {
	if held[0].X > held[len(held)-1].X {
		reversed := make([]r3.Vec, len(held))
		for i, p := range held {
			reversed[len(held)-1-i] = p
		}
		held = reversed
	}
	for i, p := range held {
		if !proofbound.FiniteVec(p) || i > 0 && lfF(held[i-1].X).Cmp(lfF(p.X)) >= 0 {
			return 0, false, nil
		}
	}
	if lfF(held[0].X).Cmp(plan.toeRoot) != 0 ||
		lfF(held[len(held)-1].X).Cmp(plan.cap2.Hi) < 0 {
		return 0, false, nil
	}
	var largest float64
	for _, source := range plan.slabs {
		if err := ctx.Err(); err != nil {
			return 0, false, err
		}
		heldBox, yes := spiralHeldXStrip(held, source[0].Lo, source[0].Hi)
		if !yes {
			return 0, false, nil
		}
		bound, yes := spiralRootFlankBoxesBound(source, heldBox, minBound)
		if !yes {
			return bound, false, nil
		}
		largest = max(largest, bound)
	}
	tail, ok := spiralHeldXStrip(held, plan.cap2.Lo, lfF(held[len(held)-1].X))
	if !ok {
		return 0, false, nil
	}
	bound, ok := spiralRootFlankBoxesBound(plan.endpoint, tail, minBound)
	if err := ctx.Err(); err != nil {
		return 0, false, err
	}
	if !ok {
		return bound, false, nil
	}
	return max(largest, bound), true, nil
}

// spiralHeldXStrip encloses every straight held edge portion in a closed X
// interval. The caller already checked that all path vertices strictly
// increase in X, so exact endpoint interpolation covers the full strip.
func spiralHeldXStrip(held []r3.Vec, xLo, xHi *big.Rat) ([3]loftFaceInterval, bool) {
	var box [3]loftFaceInterval
	if xLo == nil || xHi == nil || xLo.Cmp(xHi) > 0 {
		return box, false
	}
	seen := false
	for i := 1; i < len(held); i++ {
		a, z := held[i-1], held[i]
		ax, zx := lfF(a.X), lfF(z.X)
		lo, hi := xLo, xHi
		if lo.Cmp(ax) < 0 {
			lo = ax
		}
		if hi.Cmp(zx) > 0 {
			hi = zx
		}
		if lo.Cmp(hi) > 0 {
			continue
		}
		for _, x := range []*big.Rat{lo, hi} {
			t := lfDiv(lfSub(x, ax), lfSub(zx, ax))
			coordinates := [3]*big.Rat{x,
				lfAdd(lfF(a.Y), lfMul(t, lfSub(lfF(z.Y), lfF(a.Y)))),
				lfAdd(lfF(a.Z), lfMul(t, lfSub(lfF(z.Z), lfF(a.Z))))}
			for axis, coordinate := range coordinates {
				if !seen || coordinate.Cmp(box[axis].Lo) < 0 {
					box[axis].Lo = coordinate
				}
				if !seen || coordinate.Cmp(box[axis].Hi) > 0 {
					box[axis].Hi = coordinate
				}
			}
			seen = true
		}
	}
	return box, seen
}

func spiralRootFlankBoxesBound(trueBox, heldBox [3]loftFaceInterval,
	minBound float64) (float64, bool) {
	squared := new(big.Rat)
	for axis := range trueBox {
		a, b := trueBox[axis], heldBox[axis]
		if a.Lo == nil || a.Hi == nil || b.Lo == nil || b.Hi == nil ||
			a.Lo.Cmp(a.Hi) > 0 || b.Lo.Cmp(b.Hi) > 0 {
			return 0, false
		}
		lo := lfAbs(lfSub(a.Lo, b.Hi))
		hi := lfAbs(lfSub(a.Hi, b.Lo))
		if hi.Cmp(lo) > 0 {
			lo = hi
		}
		squared = lfAdd(squared, lfMul(lo, lo))
	}
	bound := proofbound.RatSqrtUp(squared)
	return bound, !proofbound.IsNonFinite(bound) && bound < minBound
}
