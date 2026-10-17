package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/spiralsector"
	"github.com/lestrrat-3d/r3"
)

// spiralHeelSameRayBound applies only after another gate proves that the
// source and held seams cover the same angular interval. A held point lies
// within radialError of the blank's exact linear heel meridian at its X.
func spiralHeelSameRayBound(held spiralsector.HeldSwitch, trueX loftFaceInterval,
	meridianSlope *big.Rat, radialError, stitchRound float64) (float64, bool) {
	if held.X.Lo == nil || held.X.Hi == nil || trueX.Lo == nil || trueX.Hi == nil ||
		meridianSlope == nil || held.X.Lo.Cmp(held.X.Hi) > 0 ||
		trueX.Lo.Cmp(trueX.Hi) > 0 || proofbound.IsNonFinite(radialError) ||
		proofbound.IsNonFinite(stitchRound) || radialError < 0 || stitchRound < 0 ||
		proofbound.IsNonFinite(held.MinBeta) || held.MinBeta <= 0 {
		return 0, false
	}
	dx := new(big.Rat)
	for _, a := range []*big.Rat{held.X.Lo, held.X.Hi} {
		for _, b := range []*big.Rat{trueX.Lo, trueX.Hi} {
			d := lfAbs(lfSub(a, b))
			if d.Cmp(dx) > 0 {
				dx = d
			}
		}
	}
	absSlope := lfAbs(meridianSlope)
	stitchRadial := lfMul(lfAdd(big.NewRat(1, 1), absSlope), lfF(stitchRound))
	dr := lfAdd(lfMul(absSlope, dx), lfAdd(lfF(radialError), stitchRadial))
	distance := proofbound.RatSqrtUp(lfAdd(lfMul(dx, dx), lfMul(dr, dr)))
	return distance, !proofbound.IsNonFinite(distance) && distance < held.MinBeta
}

// spiralHeelTailBoxBound encloses a whole source seam tail by one held
// endpoint. The source proof must certify that every tail point lies in box.
func spiralHeelTailBoxBound(box [3]loftFaceInterval, endpoint r3.Vec,
	curveBound float64) (float64, bool) {
	if !proofbound.FiniteVec(endpoint) || proofbound.IsNonFinite(curveBound) || curveBound <= 0 {
		return 0, false
	}
	return spiralPointBoxDistance(box,
		[3]*big.Rat{lfF(endpoint.X), lfF(endpoint.Y), lfF(endpoint.Z)}, curveBound)
}

func spiralPointBoxDistance(box [3]loftFaceInterval,
	coordinates [3]*big.Rat, curveBound float64) (float64, bool) {
	if proofbound.IsNonFinite(curveBound) || curveBound <= 0 {
		return 0, false
	}
	squared := new(big.Rat)
	for axis, interval := range box {
		if coordinates[axis] == nil || interval.Lo == nil || interval.Hi == nil ||
			interval.Lo.Cmp(interval.Hi) > 0 {
			return 0, false
		}
		lo := lfAbs(lfSub(interval.Lo, coordinates[axis]))
		hi := lfAbs(lfSub(interval.Hi, coordinates[axis]))
		if hi.Cmp(lo) > 0 {
			lo = hi
		}
		squared = lfAdd(squared, lfMul(lo, lo))
	}
	distance := proofbound.RatSqrtUp(squared)
	return distance, !proofbound.IsNonFinite(distance) && distance < curveBound
}

// spiralHeldMonotoneTailBound covers every held edge between one chain end
// and the uncertain source endpoint slope. Strict angular order and Y>0 make
// the slope cut unique. Squared distance to any fixed source endpoint is
// convex along each straight held edge, so its maximum is at a vertex or
// the exact cut where this conservative tail ends.
func spiralHeldMonotoneTailBound(ctx context.Context, held *meshbool.StitchedMesh,
	chain spiralsector.HeldSwitch, sourceEndpoint [3]loftFaceInterval,
	tailAtLow bool) (float64, bool, error) {
	if ctx == nil || held == nil || len(chain.Vertices) < 2 ||
		proofbound.IsNonFinite(chain.MinBeta) || chain.MinBeta <= 0 {
		return 0, false, nil
	}
	sourceSlope, ok := spiralSeamEndpointSlope(sourceEndpoint)
	if !ok {
		return 0, false, nil
	}
	start, step := 0, 1
	if tailAtLow != chain.Increasing {
		start, step = len(chain.Vertices)-1, -1
	}
	target := sourceSlope.Lo
	if tailAtLow {
		target = sourceSlope.Hi
	}
	point := func(index int) ([3]*big.Rat, bool) {
		v := held.Verts[chain.Vertices[index]]
		if !proofbound.FiniteVec(v) || v.Y <= 0 {
			return [3]*big.Rat{}, false
		}
		return [3]*big.Rat{lfF(v.X), lfF(v.Y), lfF(v.Z)}, true
	}
	first, ok := point(start)
	if !ok {
		return 0, false, nil
	}
	firstSlope := lfDiv(first[2], first[1])
	if tailAtLow && firstSlope.Cmp(sourceSlope.Lo) >= 0 ||
		!tailAtLow && firstSlope.Cmp(sourceSlope.Hi) <= 0 {
		return 0, false, nil
	}
	maxDistance := 0.0
	for index, walked := start, 0; index+step >= 0 && index+step < len(chain.Vertices); index, walked = index+step, walked+1 {
		if walked%128 == 0 {
			if err := ctx.Err(); err != nil {
				return 0, false, err
			}
		}
		p, pOK := point(index)
		q, qOK := point(index + step)
		if !pOK || !qOK {
			return 0, false, nil
		}
		distance, within := spiralPointBoxDistance(sourceEndpoint, p, chain.MinBeta)
		if !within {
			return distance, false, nil
		}
		maxDistance = max(maxDistance, distance)
		pSlope, qSlope := lfDiv(p[2], p[1]), lfDiv(q[2], q[1])
		if tailAtLow && pSlope.Cmp(qSlope) >= 0 ||
			!tailAtLow && pSlope.Cmp(qSlope) <= 0 {
			return 0, false, nil
		}
		crossed := tailAtLow && qSlope.Cmp(target) >= 0 ||
			!tailAtLow && qSlope.Cmp(target) <= 0
		if !crossed {
			continue
		}
		numerator := lfSub(lfMul(target, p[1]), p[2])
		denominator := lfSub(lfSub(q[2], p[2]),
			lfMul(target, lfSub(q[1], p[1])))
		if denominator.Sign() == 0 {
			return 0, false, nil
		}
		t := lfDiv(numerator, denominator)
		if t.Sign() < 0 || t.Cmp(lfOne()) > 0 {
			return 0, false, nil
		}
		var cut [3]*big.Rat
		for axis := range cut {
			cut[axis] = lfAdd(p[axis], lfMul(t, lfSub(q[axis], p[axis])))
		}
		distance, within = spiralPointBoxDistance(sourceEndpoint, cut, chain.MinBeta)
		if !within {
			return distance, false, nil
		}
		if err := ctx.Err(); err != nil {
			return 0, false, err
		}
		return max(maxDistance, distance), true, nil
	}
	return 0, false, nil
}

// spiralHeldShortCapProof compares every complete short cap/flank or
// cap/connector switch with its authenticated source crossing. Global held
// group ids come from the original two operands' face order, not a gallery
// layout. Exact source-cell and held-edge boxes cover both curves.
func spiralHeldShortCapProof(ctx context.Context, held *meshbool.StitchedMesh,
	blank, tooth *Body, proof *spiralFirstJoinSource,
	short [2][4]spiralShortCapFamily) (float64, bool, error) {
	if ctx == nil || held == nil || blank == nil || tooth == nil || proof == nil ||
		proof.Tooth == nil {
		return 0, false, nil
	}
	groups, ok := spiralJoinGroups(blank, tooth, proof)
	if !ok {
		return 0, false, nil
	}
	roles := [...]int{0, 2, 3, 5}
	maxDistance := 0.0
	// Every switch below reads the same unchanged held mesh, so its edge
	// table is built once, at the first switch, and reused within this call.
	var edges map[spiralsector.HeldEdge]spiralsector.HeldEdgeOwners
	for end := range 2 {
		if err := ctx.Err(); err != nil {
			return 0, false, err
		}
		blankCap := groups.blankToe
		if end == 1 {
			blankCap = groups.blankHeel
		}
		for index, role := range roles {
			family := short[end][index]
			if family.group != role || len(family.pieces) != 1 ||
				family.band < 0 || family.band >= len(proof.Tooth.Maps)-1 ||
				family.band != proof.Junctions[0][end].Band ||
				family.band != proof.Junctions[1][end].Band {
				return 0, false, nil
			}
			origin, found := spiralOriginalSideRef(proof.Tooth, family.band, role)
			if !found {
				return 0, false, nil
			}
			face := spiralUniqueFaceWithOrigin(tooth, origin)
			if face == nil {
				return 0, false, nil
			}
			toothFace := -1
			for i, live := range tooth.Faces() {
				if live == face {
					toothFace = i
				}
			}
			if toothFace < 0 {
				return 0, false, nil
			}
			if edges == nil {
				built, ok, err := spiralsector.HeldEdges(ctx, held)
				if err != nil || !ok {
					return 0, false, err
				}
				edges = built
			}
			chain, ok, err := spiralsector.HeldSwitchFrom(ctx, held, edges,
				blankCap, len(blank.Faces())+toothFace, false)
			if err != nil || !ok {
				return 0, false, err
			}
			ordered := make([]r3.Vec, len(chain.Vertices))
			for i, vertex := range chain.Vertices {
				ordered[i] = held.Verts[vertex]
			}
			distance, accepted, err := spiralShortHeldBound(ctx,
				proof, family, ordered, chain.MinBeta)
			if err != nil || !accepted {
				return distance, false, err
			}
			maxDistance = max(maxDistance, distance)
		}
	}
	return maxDistance, true, nil
}

func spiralSeamEndpointSlope(box [3]loftFaceInterval) (loftFaceInterval, bool) {
	if box[1].Lo == nil || box[1].Hi == nil || box[2].Lo == nil || box[2].Hi == nil ||
		box[1].Lo.Sign() <= 0 || box[1].Lo.Cmp(box[1].Hi) > 0 ||
		box[2].Lo.Cmp(box[2].Hi) > 0 {
		return loftFaceInterval{}, false
	}
	var out loftFaceInterval
	for _, z := range []*big.Rat{box[2].Lo, box[2].Hi} {
		for _, y := range []*big.Rat{box[1].Lo, box[1].Hi} {
			slope := lfDiv(z, y)
			if out.Lo == nil || slope.Cmp(out.Lo) < 0 {
				out.Lo = slope
			}
			if out.Hi == nil || slope.Cmp(out.Hi) > 0 {
				out.Hi = slope
			}
		}
	}
	return out, true
}

func spiralConeSlopeBox(seam *spiralHeelRootSeam,
	slope loftFaceInterval) ([3]loftFaceInterval, bool) {
	var result [3]loftFaceInterval
	if seam == nil || seam.ConeC == nil || seam.X.Lo == nil || seam.X.Hi == nil ||
		seam.Slope.Lo == nil || seam.Slope.Hi == nil ||
		slope.Lo == nil || slope.Hi == nil || slope.Lo.Cmp(slope.Hi) > 0 ||
		slope.Lo.Cmp(seam.Slope.Lo) < 0 || slope.Hi.Cmp(seam.Slope.Hi) > 0 {
		return result, false
	}
	rLo, rHi := lfSub(seam.ConeC, seam.X.Hi), lfSub(seam.ConeC, seam.X.Lo)
	if rLo.Sign() <= 0 || rLo.Cmp(rHi) > 0 {
		return result, false
	}
	absLo, absHi := lfAbs(slope.Lo), lfAbs(slope.Hi)
	minimum, maximum := absLo, absHi
	if absHi.Cmp(minimum) < 0 {
		minimum = absHi
	}
	if absLo.Cmp(maximum) > 0 {
		maximum = absLo
	}
	if slope.Lo.Sign() <= 0 && slope.Hi.Sign() >= 0 {
		minimum = lfZero()
	}
	yLoDen := proofbound.RatSqrtUp(lfAdd(lfOne(), lfMul(maximum, maximum)))
	yHiDen := proofbound.RatSqrtDown(lfAdd(lfOne(), lfMul(minimum, minimum)))
	if proofbound.IsNonFinite(yLoDen) || proofbound.IsNonFinite(yHiDen) ||
		yLoDen <= 0 || yHiDen <= 0 {
		return result, false
	}
	y := loftFaceInterval{Lo: lfDiv(rLo, lfF(yLoDen)), Hi: lfDiv(rHi, lfF(yHiDen))}
	var z loftFaceInterval
	for _, s := range []*big.Rat{slope.Lo, slope.Hi} {
		for _, value := range []*big.Rat{y.Lo, y.Hi} {
			product := lfMul(s, value)
			if z.Lo == nil || product.Cmp(z.Lo) < 0 {
				z.Lo = product
			}
			if z.Hi == nil || product.Cmp(z.Hi) > 0 {
				z.Hi = product
			}
		}
	}
	return [3]loftFaceInterval{seam.X, y, z}, true
}

// The source seam is a continuous unique crossing of the authenticated
// root arc and finite heel cap. The held cap/root chain is an angular graph.
// On their shared angular interval, points on the same ray differ by the
// exact cap meridian and held blank radial error. The two unmatched tails
// are enclosed by an endpoint box and a source-wide cap box, respectively.
func spiralHeldHeelSeamProof(ctx context.Context, held *meshbool.StitchedMesh,
	blankHeel, toothRoot int, seam *spiralHeelRootSeam,
	radialError float64) (float64, bool, error) {
	if seam == nil || seam.X.Lo == nil || seam.X.Hi == nil ||
		seam.Slope.Lo == nil || seam.Slope.Hi == nil || held == nil ||
		proofbound.IsNonFinite(radialError) || radialError < 0 {
		return 0, false, nil
	}
	chain, ok, err := spiralsector.HeldHeelSwitch(ctx, held, blankHeel, toothRoot)
	if err != nil || !ok {
		return 0, false, err
	}
	low, high := 0, len(chain.Vertices)-1
	if !chain.Increasing {
		low, high = high, low
	}
	if len(chain.Vertices) < 3 {
		return 0, false, nil
	}
	slopeAt := func(index int) *big.Rat {
		point := held.Verts[chain.Vertices[index]]
		return lfDiv(lfF(point.Z), lfF(point.Y))
	}
	heldLow, heldHigh := slopeAt(low), slopeAt(high)
	if heldLow.Cmp(heldHigh) >= 0 {
		return 0, false, nil
	}
	end0, ok0 := spiralSeamEndpointSlope(seam.Ends[0])
	end1, ok1 := spiralSeamEndpointSlope(seam.Ends[1])
	if !ok0 || !ok1 {
		return 0, false, nil
	}
	sourceLow, sourceHigh := end0, end1
	lowBox, highBox := seam.Ends[0], seam.Ends[1]
	if end1.Hi.Cmp(end0.Lo) < 0 {
		sourceLow, sourceHigh = end1, end0
		lowBox, highBox = seam.Ends[1], seam.Ends[0]
	}
	if sourceLow.Hi.Cmp(sourceHigh.Lo) >= 0 ||
		seam.Slope.Lo.Cmp(sourceLow.Lo) > 0 ||
		seam.Slope.Hi.Cmp(sourceHigh.Hi) < 0 {
		return 0, false, nil
	}
	inner, ok := spiralHeelSameRayBound(chain, seam.X, big.NewRat(-1, 1),
		radialError, held.Round)
	if !ok {
		return inner, false, nil
	}
	var endpoint [3]loftFaceInterval
	var highAbsIndex int
	var minAbsSlope *big.Rat
	var tailAtLow bool
	if seam.Negative {
		// The pinion has a true low-slope tail and a held high-slope tail.
		if sourceLow.Hi.Cmp(heldLow) >= 0 || seam.Slope.Hi.Cmp(heldHigh) >= 0 {
			return 0, false, nil
		}
		endpoint = highBox
		highAbsIndex = low
		minAbsSlope = lfAbs(heldLow)
	} else {
		// The driving member has a held low-slope tail and a true high-slope tail.
		if heldLow.Cmp(sourceLow.Lo) >= 0 || heldHigh.Cmp(sourceHigh.Lo) >= 0 ||
			seam.Slope.Lo.Cmp(heldLow) <= 0 {
			return 0, false, nil
		}
		endpoint = lowBox
		highAbsIndex = high
		minAbsSlope = heldHigh
		tailAtLow = true
	}
	heldTail, ok, err := spiralHeldMonotoneTailBound(ctx, held, chain, endpoint, tailAtLow)
	if err != nil || !ok {
		return heldTail, false, err
	}
	box, ok := spiralHeelTailBox(seam, minAbsSlope)
	if !ok {
		return 0, false, nil
	}
	trueTail, ok := spiralHeelTailBoxBound(box,
		held.Verts[chain.Vertices[highAbsIndex]], chain.MinBeta)
	if !ok {
		return trueTail, false, nil
	}
	return max(inner, heldTail, trueTail), true, nil
}

// spiralHeldToeSeamProof matches the complete toe cap/root switch to the
// authenticated toe source crossing. The source and held angular intervals
// overlap. Their only nonoverlap is one held end edge and the opposite source
// end; the latter stays on the exact blank cap cone.
func spiralHeldToeSeamProof(ctx context.Context, held *meshbool.StitchedMesh,
	blankToe, toothRoot int, seam *spiralToeRootSeam,
	radialError float64) (float64, bool, error) {
	if seam == nil || seam.X.Lo == nil || seam.X.Hi == nil ||
		seam.Slope.Lo == nil || seam.Slope.Hi == nil || held == nil ||
		proofbound.IsNonFinite(radialError) || radialError < 0 {
		return 0, false, nil
	}
	chain, ok, err := spiralsector.HeldHeelSwitch(ctx, held, blankToe, toothRoot)
	if err != nil || !ok {
		return 0, false, err
	}
	if len(chain.Vertices) < 3 {
		return 0, false, nil
	}
	low, high := 0, len(chain.Vertices)-1
	if !chain.Increasing {
		low, high = high, low
	}
	slopeAt := func(index int) *big.Rat {
		point := held.Verts[chain.Vertices[index]]
		return lfDiv(lfF(point.Z), lfF(point.Y))
	}
	heldLow, heldHigh := slopeAt(low), slopeAt(high)
	if heldLow.Cmp(heldHigh) >= 0 {
		return 0, false, nil
	}
	end0, ok0 := spiralSeamEndpointSlope(seam.Ends[0])
	end1, ok1 := spiralSeamEndpointSlope(seam.Ends[1])
	if !ok0 || !ok1 {
		return 0, false, nil
	}
	sourceLow, sourceHigh := end0, end1
	lowBox, highBox := seam.Ends[0], seam.Ends[1]
	if end1.Hi.Cmp(end0.Lo) < 0 {
		sourceLow, sourceHigh = end1, end0
		lowBox, highBox = seam.Ends[1], seam.Ends[0]
	}
	if sourceLow.Hi.Cmp(sourceHigh.Lo) >= 0 ||
		seam.Slope.Lo.Cmp(sourceLow.Lo) > 0 ||
		seam.Slope.Hi.Cmp(sourceHigh.Hi) < 0 {
		return 0, false, nil
	}
	inner, ok := spiralHeelSameRayBound(chain, seam.X, big.NewRat(-1, 1),
		radialError, held.Round)
	if !ok {
		return inner, false, nil
	}
	var endpoint [3]loftFaceInterval
	var tailAtLow bool
	var sourceTail loftFaceInterval
	var opposite int
	switch {
	case heldLow.Cmp(sourceLow.Lo) < 0 && sourceLow.Hi.Cmp(heldHigh) < 0 &&
		heldHigh.Cmp(sourceHigh.Lo) < 0 && seam.Slope.Lo.Cmp(heldLow) > 0:
		// The held chain extends below the source and the source extends above it.
		endpoint = lowBox
		tailAtLow = true
		sourceTail = loftFaceInterval{Lo: heldHigh, Hi: seam.Slope.Hi}
		opposite = high
	case seam.Slope.Hi.Cmp(heldHigh) < 0 && sourceLow.Hi.Cmp(heldLow) < 0 &&
		heldLow.Cmp(sourceHigh.Lo) < 0 && sourceHigh.Hi.Cmp(heldHigh) < 0:
		// The source extends below the held chain and the held chain extends above it.
		endpoint = highBox
		sourceTail = loftFaceInterval{Lo: seam.Slope.Lo, Hi: heldLow}
		opposite = low
	default:
		return 0, false, nil
	}
	heldTail, ok, err := spiralHeldMonotoneTailBound(ctx, held, chain, endpoint, tailAtLow)
	if err != nil || !ok {
		return heldTail, false, err
	}
	box, ok := spiralConeSlopeBox(seam, sourceTail)
	if !ok {
		return 0, false, nil
	}
	trueTail, ok := spiralHeelTailBoxBound(box,
		held.Verts[chain.Vertices[opposite]], chain.MinBeta)
	if !ok {
		return trueTail, false, nil
	}
	return max(inner, heldTail, trueTail), true, nil
}
