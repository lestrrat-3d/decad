package spiralsector

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func seamRatZero() *big.Rat             { return new(big.Rat) }
func seamRatOne() *big.Rat              { return big.NewRat(1, 1) }
func seamRatF(v float64) *big.Rat       { return proof.FloatRat(v) }
func seamRatAdd(a, b *big.Rat) *big.Rat { return proof.AddRat(new(big.Rat), a, b) }
func seamRatSub(a, b *big.Rat) *big.Rat { return proof.SubRat(new(big.Rat), a, b) }
func seamRatMul(a, b *big.Rat) *big.Rat { return proof.MulRat(new(big.Rat), a, b) }
func seamRatDiv(a, b *big.Rat) *big.Rat { return proof.QuoRat(new(big.Rat), a, b) }
func seamRatAbs(a *big.Rat) *big.Rat    { return new(big.Rat).Abs(a) }

// HeelSameRayBound applies only after another gate proves that the source
// and held seams cover the same angular interval. A held point lies within
// radialError of the blank's exact linear heel meridian at its X.
func HeelSameRayBound(held HeldSwitch, trueX loftface.Interval,
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
			d := seamRatAbs(seamRatSub(a, b))
			if d.Cmp(dx) > 0 {
				dx = d
			}
		}
	}
	absSlope := seamRatAbs(meridianSlope)
	stitchRadial := seamRatMul(seamRatAdd(big.NewRat(1, 1), absSlope), seamRatF(stitchRound))
	dr := seamRatAdd(seamRatMul(absSlope, dx), seamRatAdd(seamRatF(radialError), stitchRadial))
	distance := proofbound.RatSqrtUp(seamRatAdd(seamRatMul(dx, dx), seamRatMul(dr, dr)))
	return distance, !proofbound.IsNonFinite(distance) && distance < held.MinBeta
}

// HeelTailBoxBound encloses a whole source seam tail by one held endpoint.
// The source proof must certify that every tail point lies in box.
func HeelTailBoxBound(box [3]loftface.Interval, endpoint r3.Vec,
	curveBound float64) (float64, bool) {
	if !proofbound.FiniteVec(endpoint) || proofbound.IsNonFinite(curveBound) || curveBound <= 0 {
		return 0, false
	}
	return pointBoxDistance(box,
		[3]*big.Rat{seamRatF(endpoint.X), seamRatF(endpoint.Y), seamRatF(endpoint.Z)}, curveBound)
}

func pointBoxDistance(box [3]loftface.Interval,
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
		lo := seamRatAbs(seamRatSub(interval.Lo, coordinates[axis]))
		hi := seamRatAbs(seamRatSub(interval.Hi, coordinates[axis]))
		if hi.Cmp(lo) > 0 {
			lo = hi
		}
		squared = seamRatAdd(squared, seamRatMul(lo, lo))
	}
	distance := proofbound.RatSqrtUp(squared)
	return distance, !proofbound.IsNonFinite(distance) && distance < curveBound
}

// HeldMonotoneTailBound covers every held edge between one chain end and the
// uncertain source endpoint slope. Strict angular order and Y>0 make the
// slope cut unique. Squared distance to any fixed source endpoint is convex
// along each straight held edge, so its maximum is at a vertex or the exact
// cut where this conservative tail ends.
func HeldMonotoneTailBound(ctx context.Context, held *meshbool.StitchedMesh,
	chain HeldSwitch, sourceEndpoint [3]loftface.Interval,
	tailAtLow bool) (float64, bool, error) {
	if ctx == nil || held == nil || len(chain.Vertices) < 2 ||
		proofbound.IsNonFinite(chain.MinBeta) || chain.MinBeta <= 0 {
		return 0, false, nil
	}
	sourceSlope, ok := SeamEndpointSlope(sourceEndpoint)
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
		return [3]*big.Rat{seamRatF(v.X), seamRatF(v.Y), seamRatF(v.Z)}, true
	}
	first, ok := point(start)
	if !ok {
		return 0, false, nil
	}
	firstSlope := seamRatDiv(first[2], first[1])
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
		distance, within := pointBoxDistance(sourceEndpoint, p, chain.MinBeta)
		if !within {
			return distance, false, nil
		}
		maxDistance = max(maxDistance, distance)
		pSlope, qSlope := seamRatDiv(p[2], p[1]), seamRatDiv(q[2], q[1])
		if tailAtLow && pSlope.Cmp(qSlope) >= 0 ||
			!tailAtLow && pSlope.Cmp(qSlope) <= 0 {
			return 0, false, nil
		}
		crossed := tailAtLow && qSlope.Cmp(target) >= 0 ||
			!tailAtLow && qSlope.Cmp(target) <= 0
		if !crossed {
			continue
		}
		numerator := seamRatSub(seamRatMul(target, p[1]), p[2])
		denominator := seamRatSub(seamRatSub(q[2], p[2]),
			seamRatMul(target, seamRatSub(q[1], p[1])))
		if denominator.Sign() == 0 {
			return 0, false, nil
		}
		t := seamRatDiv(numerator, denominator)
		if t.Sign() < 0 || t.Cmp(seamRatOne()) > 0 {
			return 0, false, nil
		}
		var cut [3]*big.Rat
		for axis := range cut {
			cut[axis] = seamRatAdd(p[axis], seamRatMul(t, seamRatSub(q[axis], p[axis])))
		}
		distance, within = pointBoxDistance(sourceEndpoint, cut, chain.MinBeta)
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

// SeamEndpointSlope encloses the Z/Y slope at one source seam endpoint.
func SeamEndpointSlope(box [3]loftface.Interval) (loftface.Interval, bool) {
	if box[1].Lo == nil || box[1].Hi == nil || box[2].Lo == nil || box[2].Hi == nil ||
		box[1].Lo.Sign() <= 0 || box[1].Lo.Cmp(box[1].Hi) > 0 ||
		box[2].Lo.Cmp(box[2].Hi) > 0 {
		return loftface.Interval{}, false
	}
	var out loftface.Interval
	for _, z := range []*big.Rat{box[2].Lo, box[2].Hi} {
		for _, y := range []*big.Rat{box[1].Lo, box[1].Hi} {
			slope := seamRatDiv(z, y)
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
