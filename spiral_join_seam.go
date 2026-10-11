package decad

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// spiralHeldSwitch is a complete, simple source-face switch on the held
// Boolean boundary. The ordered vertices make both sides of an angular chart
// available to the separate exact-source correspondence proof.
type spiralHeldSwitch struct {
	vertices   []int
	x          loftFaceInterval
	box        [3]loftFaceInterval
	slope      [2]*big.Rat
	minBeta    float64
	increasing bool
}

// spiralHeldHeelSwitch accepts only one angularly ordered cap/root chain.
// A closed output audit remains the caller's prerequisite; this gate checks
// the local face identities and chain topology before any smaller source-
// specific contact allowance may be used.
func spiralHeldHeelSwitch(ctx context.Context, held *meshbool.StitchedMesh,
	blankHeel, toothRoot int) (spiralHeldSwitch, bool, error) {
	return spiralHeldSwitchPath(ctx, held, blankHeel, toothRoot, true)
}

func spiralHeldShortSwitch(ctx context.Context, held *meshbool.StitchedMesh,
	blankCap, toothSide int) (spiralHeldSwitch, bool, error) {
	return spiralHeldSwitchPath(ctx, held, blankCap, toothSide, false)
}

func spiralHeldSwitchPath(ctx context.Context, held *meshbool.StitchedMesh,
	blankHeel, toothRoot int, monotone bool) (spiralHeldSwitch, bool, error) {
	if ctx == nil || held == nil || blankHeel < 0 || toothRoot < 0 || blankHeel == toothRoot ||
		len(held.Tris) == 0 || len(held.Tris) > 1<<17 ||
		len(held.Src) != len(held.Tris) || len(held.VertexBound) != len(held.Verts) {
		return spiralHeldSwitch{}, false, nil
	}
	type edge struct{ lo, hi int }
	owners := make(map[edge][2]int, len(held.Tris)*3/2)
	counts := make(map[edge]uint8, len(held.Tris)*3/2)
	for i, tri := range held.Tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return spiralHeldSwitch{}, false, err
			}
		}
		for side := range 3 {
			a, b := tri[side], tri[(side+1)%3]
			if a < 0 || b < 0 || a >= len(held.Verts) || b >= len(held.Verts) || a == b {
				return spiralHeldSwitch{}, false, nil
			}
			if a > b {
				a, b = b, a
			}
			key := edge{a, b}
			if counts[key] >= 2 {
				return spiralHeldSwitch{}, false, nil
			}
			owner := owners[key]
			owner[counts[key]] = held.Src[i]
			owners[key] = owner
			counts[key]++
		}
	}
	adj := make(map[int][]int)
	seamEdges := 0
	seen := 0
	for key, pair := range owners {
		if seen%256 == 0 {
			if err := ctx.Err(); err != nil {
				return spiralHeldSwitch{}, false, err
			}
		}
		seen++
		if counts[key] != 2 {
			return spiralHeldSwitch{}, false, nil
		}
		if pair[0] != blankHeel || pair[1] != toothRoot {
			if pair[0] != toothRoot || pair[1] != blankHeel {
				continue
			}
		}
		adj[key.lo] = append(adj[key.lo], key.hi)
		adj[key.hi] = append(adj[key.hi], key.lo)
		seamEdges++
	}
	if seamEdges < 1 || seamEdges > 4096 {
		return spiralHeldSwitch{}, false, nil
	}
	starts := make([]int, 0, 2)
	for vertex, neighbors := range adj {
		switch len(neighbors) {
		case 1:
			starts = append(starts, vertex)
		case 2:
		default:
			return spiralHeldSwitch{}, false, nil
		}
	}
	if len(starts) != 2 || len(adj) != seamEdges+1 {
		return spiralHeldSwitch{}, false, nil
	}
	ordered := make([]int, 0, len(adj))
	previous, current := -1, starts[0]
	sign := 0
	minBeta := math.Inf(1)
	var x loftFaceInterval
	var box [3]loftFaceInterval
	for len(ordered) < len(adj) {
		if len(ordered)%128 == 0 {
			if err := ctx.Err(); err != nil {
				return spiralHeldSwitch{}, false, err
			}
		}
		p := held.Verts[current]
		beta := held.VertexBound[current]
		if !proofbound.FiniteVec(p) || p.Y <= 0 ||
			proofbound.IsNonFinite(beta) || beta < 0 {
			return spiralHeldSwitch{}, false, nil
		}
		if len(ordered) == 0 {
			x = loftFaceInterval{lo: lfF(p.X), hi: lfF(p.X)}
		} else {
			if lfF(p.X).Cmp(x.lo) < 0 {
				x.lo = lfF(p.X)
			}
			if lfF(p.X).Cmp(x.hi) > 0 {
				x.hi = lfF(p.X)
			}
		}
		for axis, coordinate := range [...]float64{p.X, p.Y, p.Z} {
			value := lfF(coordinate)
			if box[axis].lo == nil || value.Cmp(box[axis].lo) < 0 {
				box[axis].lo = value
			}
			if box[axis].hi == nil || value.Cmp(box[axis].hi) > 0 {
				box[axis].hi = value
			}
		}
		minBeta = min(minBeta, beta)
		ordered = append(ordered, current)
		next := -1
		for _, neighbor := range adj[current] {
			if neighbor != previous {
				next = neighbor
				break
			}
		}
		if next < 0 {
			break
		}
		q := held.Verts[next]
		if !proofbound.FiniteVec(q) || q.Y <= 0 {
			return spiralHeldSwitch{}, false, nil
		}
		cross := lfSub(lfMul(lfF(p.Y), lfF(q.Z)), lfMul(lfF(p.Z), lfF(q.Y))).Sign()
		if cross == 0 || monotone && sign != 0 && cross != sign {
			return spiralHeldSwitch{}, false, nil
		}
		sign = cross
		previous, current = current, next
	}
	if len(ordered) != len(adj) || ordered[len(ordered)-1] != starts[1] ||
		proofbound.IsNonFinite(minBeta) || minBeta <= 0 {
		return spiralHeldSwitch{}, false, nil
	}
	first, last := held.Verts[ordered[0]], held.Verts[ordered[len(ordered)-1]]
	return spiralHeldSwitch{vertices: ordered, x: x, box: box,
		slope:   [2]*big.Rat{lfDiv(lfF(first.Z), lfF(first.Y)), lfDiv(lfF(last.Z), lfF(last.Y))},
		minBeta: minBeta, increasing: sign > 0}, true, nil
}

// spiralHeelSameRayBound applies only after another gate proves that the
// source and held seams cover the same angular interval. A held point lies
// within radialError of the blank's exact linear heel meridian at its X.
func spiralHeelSameRayBound(held spiralHeldSwitch, trueX loftFaceInterval,
	meridianSlope *big.Rat, radialError, stitchRound float64) (float64, bool) {
	if held.x.lo == nil || held.x.hi == nil || trueX.lo == nil || trueX.hi == nil ||
		meridianSlope == nil || held.x.lo.Cmp(held.x.hi) > 0 ||
		trueX.lo.Cmp(trueX.hi) > 0 || proofbound.IsNonFinite(radialError) ||
		proofbound.IsNonFinite(stitchRound) || radialError < 0 || stitchRound < 0 ||
		proofbound.IsNonFinite(held.minBeta) || held.minBeta <= 0 {
		return 0, false
	}
	dx := new(big.Rat)
	for _, a := range []*big.Rat{held.x.lo, held.x.hi} {
		for _, b := range []*big.Rat{trueX.lo, trueX.hi} {
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
	return distance, !proofbound.IsNonFinite(distance) && distance < held.minBeta
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
		if coordinates[axis] == nil || interval.lo == nil || interval.hi == nil ||
			interval.lo.Cmp(interval.hi) > 0 {
			return 0, false
		}
		lo := lfAbs(lfSub(interval.lo, coordinates[axis]))
		hi := lfAbs(lfSub(interval.hi, coordinates[axis]))
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
	chain spiralHeldSwitch, sourceEndpoint [3]loftFaceInterval,
	tailAtLow bool) (float64, bool, error) {
	if ctx == nil || held == nil || len(chain.vertices) < 2 ||
		proofbound.IsNonFinite(chain.minBeta) || chain.minBeta <= 0 {
		return 0, false, nil
	}
	sourceSlope, ok := spiralSeamEndpointSlope(sourceEndpoint)
	if !ok {
		return 0, false, nil
	}
	start, step := 0, 1
	if tailAtLow != chain.increasing {
		start, step = len(chain.vertices)-1, -1
	}
	target := sourceSlope.lo
	if tailAtLow {
		target = sourceSlope.hi
	}
	point := func(index int) ([3]*big.Rat, bool) {
		v := held.Verts[chain.vertices[index]]
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
	if tailAtLow && firstSlope.Cmp(sourceSlope.lo) >= 0 ||
		!tailAtLow && firstSlope.Cmp(sourceSlope.hi) <= 0 {
		return 0, false, nil
	}
	maxDistance := 0.0
	for index, walked := start, 0; index+step >= 0 && index+step < len(chain.vertices); index, walked = index+step, walked+1 {
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
		distance, within := spiralPointBoxDistance(sourceEndpoint, p, chain.minBeta)
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
		distance, within = spiralPointBoxDistance(sourceEndpoint, cut, chain.minBeta)
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
		proof.tooth == nil {
		return 0, false, nil
	}
	groups, ok := spiralJoinGroups(blank, tooth, proof)
	if !ok {
		return 0, false, nil
	}
	roles := [...]int{0, 2, 3, 5}
	maxDistance := 0.0
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
				family.band < 0 || family.band >= len(proof.tooth.maps)-1 ||
				family.band != proof.junctions[0][end].band ||
				family.band != proof.junctions[1][end].band {
				return 0, false, nil
			}
			origin, found := spiralOriginalSideRef(proof.tooth, family.band, role)
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
			chain, ok, err := spiralHeldShortSwitch(ctx, held,
				blankCap, len(blank.Faces())+toothFace)
			if err != nil || !ok {
				return 0, false, err
			}
			ordered := make([]r3.Vec, len(chain.vertices))
			for i, vertex := range chain.vertices {
				ordered[i] = held.Verts[vertex]
			}
			distance, accepted, err := spiralShortHeldBound(ctx,
				proof, family, ordered, chain.minBeta)
			if err != nil || !accepted {
				return distance, false, err
			}
			maxDistance = max(maxDistance, distance)
		}
	}
	return maxDistance, true, nil
}

func spiralSeamEndpointSlope(box [3]loftFaceInterval) (loftFaceInterval, bool) {
	if box[1].lo == nil || box[1].hi == nil || box[2].lo == nil || box[2].hi == nil ||
		box[1].lo.Sign() <= 0 || box[1].lo.Cmp(box[1].hi) > 0 ||
		box[2].lo.Cmp(box[2].hi) > 0 {
		return loftFaceInterval{}, false
	}
	var out loftFaceInterval
	for _, z := range []*big.Rat{box[2].lo, box[2].hi} {
		for _, y := range []*big.Rat{box[1].lo, box[1].hi} {
			slope := lfDiv(z, y)
			if out.lo == nil || slope.Cmp(out.lo) < 0 {
				out.lo = slope
			}
			if out.hi == nil || slope.Cmp(out.hi) > 0 {
				out.hi = slope
			}
		}
	}
	return out, true
}

func spiralConeSlopeBox(seam *spiralHeelRootSeam,
	slope loftFaceInterval) ([3]loftFaceInterval, bool) {
	var result [3]loftFaceInterval
	if seam == nil || seam.coneC == nil || seam.x.lo == nil || seam.x.hi == nil ||
		seam.slope.lo == nil || seam.slope.hi == nil ||
		slope.lo == nil || slope.hi == nil || slope.lo.Cmp(slope.hi) > 0 ||
		slope.lo.Cmp(seam.slope.lo) < 0 || slope.hi.Cmp(seam.slope.hi) > 0 {
		return result, false
	}
	rLo, rHi := lfSub(seam.coneC, seam.x.hi), lfSub(seam.coneC, seam.x.lo)
	if rLo.Sign() <= 0 || rLo.Cmp(rHi) > 0 {
		return result, false
	}
	absLo, absHi := lfAbs(slope.lo), lfAbs(slope.hi)
	minimum, maximum := absLo, absHi
	if absHi.Cmp(minimum) < 0 {
		minimum = absHi
	}
	if absLo.Cmp(maximum) > 0 {
		maximum = absLo
	}
	if slope.lo.Sign() <= 0 && slope.hi.Sign() >= 0 {
		minimum = lfZero()
	}
	yLoDen := proofbound.RatSqrtUp(lfAdd(lfOne(), lfMul(maximum, maximum)))
	yHiDen := proofbound.RatSqrtDown(lfAdd(lfOne(), lfMul(minimum, minimum)))
	if proofbound.IsNonFinite(yLoDen) || proofbound.IsNonFinite(yHiDen) ||
		yLoDen <= 0 || yHiDen <= 0 {
		return result, false
	}
	y := loftFaceInterval{lo: lfDiv(rLo, lfF(yLoDen)), hi: lfDiv(rHi, lfF(yHiDen))}
	var z loftFaceInterval
	for _, s := range []*big.Rat{slope.lo, slope.hi} {
		for _, value := range []*big.Rat{y.lo, y.hi} {
			product := lfMul(s, value)
			if z.lo == nil || product.Cmp(z.lo) < 0 {
				z.lo = product
			}
			if z.hi == nil || product.Cmp(z.hi) > 0 {
				z.hi = product
			}
		}
	}
	return [3]loftFaceInterval{seam.x, y, z}, true
}

// The source seam is a continuous unique crossing of the authenticated
// root arc and finite heel cap. The held cap/root chain is an angular graph.
// On their shared angular interval, points on the same ray differ by the
// exact cap meridian and held blank radial error. The two unmatched tails
// are enclosed by an endpoint box and a source-wide cap box, respectively.
func spiralHeldHeelSeamProof(ctx context.Context, held *meshbool.StitchedMesh,
	blankHeel, toothRoot int, seam *spiralHeelRootSeam,
	radialError float64) (float64, bool, error) {
	if seam == nil || seam.x.lo == nil || seam.x.hi == nil ||
		seam.slope.lo == nil || seam.slope.hi == nil || held == nil ||
		proofbound.IsNonFinite(radialError) || radialError < 0 {
		return 0, false, nil
	}
	chain, ok, err := spiralHeldHeelSwitch(ctx, held, blankHeel, toothRoot)
	if err != nil || !ok {
		return 0, false, err
	}
	low, high := 0, len(chain.vertices)-1
	if !chain.increasing {
		low, high = high, low
	}
	if len(chain.vertices) < 3 {
		return 0, false, nil
	}
	slopeAt := func(index int) *big.Rat {
		point := held.Verts[chain.vertices[index]]
		return lfDiv(lfF(point.Z), lfF(point.Y))
	}
	heldLow, heldHigh := slopeAt(low), slopeAt(high)
	if heldLow.Cmp(heldHigh) >= 0 {
		return 0, false, nil
	}
	end0, ok0 := spiralSeamEndpointSlope(seam.ends[0])
	end1, ok1 := spiralSeamEndpointSlope(seam.ends[1])
	if !ok0 || !ok1 {
		return 0, false, nil
	}
	sourceLow, sourceHigh := end0, end1
	lowBox, highBox := seam.ends[0], seam.ends[1]
	if end1.hi.Cmp(end0.lo) < 0 {
		sourceLow, sourceHigh = end1, end0
		lowBox, highBox = seam.ends[1], seam.ends[0]
	}
	if sourceLow.hi.Cmp(sourceHigh.lo) >= 0 ||
		seam.slope.lo.Cmp(sourceLow.lo) > 0 ||
		seam.slope.hi.Cmp(sourceHigh.hi) < 0 {
		return 0, false, nil
	}
	inner, ok := spiralHeelSameRayBound(chain, seam.x, big.NewRat(-1, 1),
		radialError, held.Round)
	if !ok {
		return inner, false, nil
	}
	var endpoint [3]loftFaceInterval
	var highAbsIndex int
	var minAbsSlope *big.Rat
	var tailAtLow bool
	if seam.negative {
		// The pinion has a true low-slope tail and a held high-slope tail.
		if sourceLow.hi.Cmp(heldLow) >= 0 || seam.slope.hi.Cmp(heldHigh) >= 0 {
			return 0, false, nil
		}
		endpoint = highBox
		highAbsIndex = low
		minAbsSlope = lfAbs(heldLow)
	} else {
		// The driving member has a held low-slope tail and a true high-slope tail.
		if heldLow.Cmp(sourceLow.lo) >= 0 || heldHigh.Cmp(sourceHigh.lo) >= 0 ||
			seam.slope.lo.Cmp(heldLow) <= 0 {
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
		held.Verts[chain.vertices[highAbsIndex]], chain.minBeta)
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
	if seam == nil || seam.x.lo == nil || seam.x.hi == nil ||
		seam.slope.lo == nil || seam.slope.hi == nil || held == nil ||
		proofbound.IsNonFinite(radialError) || radialError < 0 {
		return 0, false, nil
	}
	chain, ok, err := spiralHeldHeelSwitch(ctx, held, blankToe, toothRoot)
	if err != nil || !ok {
		return 0, false, err
	}
	if len(chain.vertices) < 3 {
		return 0, false, nil
	}
	low, high := 0, len(chain.vertices)-1
	if !chain.increasing {
		low, high = high, low
	}
	slopeAt := func(index int) *big.Rat {
		point := held.Verts[chain.vertices[index]]
		return lfDiv(lfF(point.Z), lfF(point.Y))
	}
	heldLow, heldHigh := slopeAt(low), slopeAt(high)
	if heldLow.Cmp(heldHigh) >= 0 {
		return 0, false, nil
	}
	end0, ok0 := spiralSeamEndpointSlope(seam.ends[0])
	end1, ok1 := spiralSeamEndpointSlope(seam.ends[1])
	if !ok0 || !ok1 {
		return 0, false, nil
	}
	sourceLow, sourceHigh := end0, end1
	lowBox, highBox := seam.ends[0], seam.ends[1]
	if end1.hi.Cmp(end0.lo) < 0 {
		sourceLow, sourceHigh = end1, end0
		lowBox, highBox = seam.ends[1], seam.ends[0]
	}
	if sourceLow.hi.Cmp(sourceHigh.lo) >= 0 ||
		seam.slope.lo.Cmp(sourceLow.lo) > 0 ||
		seam.slope.hi.Cmp(sourceHigh.hi) < 0 {
		return 0, false, nil
	}
	inner, ok := spiralHeelSameRayBound(chain, seam.x, big.NewRat(-1, 1),
		radialError, held.Round)
	if !ok {
		return inner, false, nil
	}
	var endpoint [3]loftFaceInterval
	var tailAtLow bool
	var sourceTail loftFaceInterval
	var opposite int
	switch {
	case heldLow.Cmp(sourceLow.lo) < 0 && sourceLow.hi.Cmp(heldHigh) < 0 &&
		heldHigh.Cmp(sourceHigh.lo) < 0 && seam.slope.lo.Cmp(heldLow) > 0:
		// The held chain extends below the source and the source extends above it.
		endpoint = lowBox
		tailAtLow = true
		sourceTail = loftFaceInterval{lo: heldHigh, hi: seam.slope.hi}
		opposite = high
	case seam.slope.hi.Cmp(heldHigh) < 0 && sourceLow.hi.Cmp(heldLow) < 0 &&
		heldLow.Cmp(sourceHigh.lo) < 0 && sourceHigh.hi.Cmp(heldHigh) < 0:
		// The source extends below the held chain and the held chain extends above it.
		endpoint = highBox
		sourceTail = loftFaceInterval{lo: seam.slope.lo, hi: heldLow}
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
		held.Verts[chain.vertices[opposite]], chain.minBeta)
	if !ok {
		return trueTail, false, nil
	}
	return max(inner, heldTail, trueTail), true, nil
}
