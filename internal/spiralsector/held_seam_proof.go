package spiralsector

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func spiralConeSlopeBox(seam *spiralHeelRootSeam,
	slope loftFaceInterval) ([3]loftFaceInterval, bool) {
	var result [3]loftFaceInterval
	if seam == nil || seam.ConeC == nil || seam.X.Lo == nil || seam.X.Hi == nil ||
		seam.Slope.Lo == nil || seam.Slope.Hi == nil ||
		slope.Lo == nil || slope.Hi == nil || slope.Lo.Cmp(slope.Hi) > 0 ||
		slope.Lo.Cmp(seam.Slope.Lo) < 0 || slope.Hi.Cmp(seam.Slope.Hi) > 0 {
		return result, false
	}
	rLo, rHi := seamRatSub(seam.ConeC, seam.X.Hi), seamRatSub(seam.ConeC, seam.X.Lo)
	if rLo.Sign() <= 0 || rLo.Cmp(rHi) > 0 {
		return result, false
	}
	absLo, absHi := seamRatAbs(slope.Lo), seamRatAbs(slope.Hi)
	minimum, maximum := absLo, absHi
	if absHi.Cmp(minimum) < 0 {
		minimum = absHi
	}
	if absLo.Cmp(maximum) > 0 {
		maximum = absLo
	}
	if slope.Lo.Sign() <= 0 && slope.Hi.Sign() >= 0 {
		minimum = seamRatZero()
	}
	yLoDen := proofbound.RatSqrtUp(seamRatAdd(seamRatOne(), seamRatMul(maximum, maximum)))
	yHiDen := proofbound.RatSqrtDown(seamRatAdd(seamRatOne(), seamRatMul(minimum, minimum)))
	if proofbound.IsNonFinite(yLoDen) || proofbound.IsNonFinite(yHiDen) ||
		yLoDen <= 0 || yHiDen <= 0 {
		return result, false
	}
	y := loftFaceInterval{Lo: seamRatDiv(rLo, seamRatF(yLoDen)), Hi: seamRatDiv(rHi, seamRatF(yHiDen))}
	var z loftFaceInterval
	for _, s := range []*big.Rat{slope.Lo, slope.Hi} {
		for _, value := range []*big.Rat{y.Lo, y.Hi} {
			product := seamRatMul(s, value)
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
func HeldHeelSeamProof(ctx context.Context, held *meshbool.StitchedMesh,
	blankHeel, toothRoot int, seam *spiralHeelRootSeam,
	radialError float64) (float64, bool, error) {
	if seam == nil || seam.X.Lo == nil || seam.X.Hi == nil ||
		seam.Slope.Lo == nil || seam.Slope.Hi == nil || held == nil ||
		proofbound.IsNonFinite(radialError) || radialError < 0 {
		return 0, false, nil
	}
	chain, ok, err := HeldHeelSwitch(ctx, held, blankHeel, toothRoot)
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
		return seamRatDiv(seamRatF(point.Z), seamRatF(point.Y))
	}
	heldLow, heldHigh := slopeAt(low), slopeAt(high)
	if heldLow.Cmp(heldHigh) >= 0 {
		return 0, false, nil
	}
	end0, ok0 := SeamEndpointSlope(seam.Ends[0])
	end1, ok1 := SeamEndpointSlope(seam.Ends[1])
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
	inner, ok := HeelSameRayBound(chain, seam.X, big.NewRat(-1, 1),
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
		minAbsSlope = seamRatAbs(heldLow)
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
	heldTail, ok, err := HeldMonotoneTailBound(ctx, held, chain, endpoint, tailAtLow)
	if err != nil || !ok {
		return heldTail, false, err
	}
	box, ok := SpiralHeelTailBox(seam, minAbsSlope)
	if !ok {
		return 0, false, nil
	}
	trueTail, ok := HeelTailBoxBound(box,
		held.Verts[chain.Vertices[highAbsIndex]], chain.MinBeta)
	if !ok {
		return trueTail, false, nil
	}
	return max(inner, heldTail, trueTail), true, nil
}

// HeldToeSeamProof matches the complete toe cap/root switch to the
// authenticated toe source crossing. The source and held angular intervals
// overlap. Their only nonoverlap is one held end edge and the opposite source
// end; the latter stays on the exact blank cap cone.
func HeldToeSeamProof(ctx context.Context, held *meshbool.StitchedMesh,
	blankToe, toothRoot int, seam *spiralHeelRootSeam,
	radialError float64) (float64, bool, error) {
	if seam == nil || seam.X.Lo == nil || seam.X.Hi == nil ||
		seam.Slope.Lo == nil || seam.Slope.Hi == nil || held == nil ||
		proofbound.IsNonFinite(radialError) || radialError < 0 {
		return 0, false, nil
	}
	chain, ok, err := HeldHeelSwitch(ctx, held, blankToe, toothRoot)
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
		return seamRatDiv(seamRatF(point.Z), seamRatF(point.Y))
	}
	heldLow, heldHigh := slopeAt(low), slopeAt(high)
	if heldLow.Cmp(heldHigh) >= 0 {
		return 0, false, nil
	}
	end0, ok0 := SeamEndpointSlope(seam.Ends[0])
	end1, ok1 := SeamEndpointSlope(seam.Ends[1])
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
	inner, ok := HeelSameRayBound(chain, seam.X, big.NewRat(-1, 1),
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
	heldTail, ok, err := HeldMonotoneTailBound(ctx, held, chain, endpoint, tailAtLow)
	if err != nil || !ok {
		return heldTail, false, err
	}
	box, ok := spiralConeSlopeBox(seam, sourceTail)
	if !ok {
		return 0, false, nil
	}
	trueTail, ok := HeelTailBoxBound(box,
		held.Verts[chain.Vertices[opposite]], chain.MinBeta)
	if !ok {
		return trueTail, false, nil
	}
	return max(inner, heldTail, trueTail), true, nil
}
