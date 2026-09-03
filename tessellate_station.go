package decad

import (
	"math"
	"math/big"
)

// This file owns the proven bound on ONE interior chord station of a circular
// walk — the coordinate-construction half of docs/tessellation-design.md §2's
// sourceBound(face), which docs/tessellation-reach-design.md §3 names deltaStore.
//
// A walk's own two endpoints already state what they are worth
// (segmentWalk.startBound/endBound, extrude.go). Every sample BETWEEN them is a
// point this package computed: chordLoop evaluates math.Cos/math.Sin at an angle
// it formed itself, from a centre, a radius and a sweep the walk had already
// rounded. None of that arithmetic is a quantity the record states, so the
// station's held (u, v) is not a recorded coordinate and publishing it as one
// would claim an exactness the build never proved.

// chordStationBound is circularWalkEndBound's interior-sample twin: the proven
// per-component gap between a chord station's held (u, v) and the point the
// RECORD denotes at that station's own parameter.
//
// chordLoop divides a circular walk into n chords and emits station k at
// angle th0 + k·(th1 − th0)/n. Both circular walk kinds parameterise their
// angle affinely in the recorded parameter — a CircleSeg's angle is 2π·T, an
// ArcSeg's is a0 + T·sweep (walkOf) — so that station is the recorded
// parameter TStart + (k/n)·(TEnd − TStart), and k/n is an EXACT rational. It
// is handed to circularEndpointInterval unrounded, through circularPointBound:
// rounding it to a float first would enclose the recorded curve at a
// NEIGHBOURING parameter, and the answer would then be a proof about a point
// this chording never named.
//
// The mechanism per kind is circularEndpointInterval's own: turnSinCosInterval
// for a CircleSeg's exactly-rational turn, radSinCosSpan over atan2Interval for
// an ArcSeg's enclosed angle. Neither ever compares against π.
//
// An enclosure the recorded data cannot state, a parameter that is not
// representable as a rational, and a station index outside the walk's own
// interior all answer +Inf on both components — the underivable bound the
// tessellation refuses on (docs/tessellation-design.md §12), never a zero.
func chordStationBound(seg CurveSegment, k, n int, heldU, heldV float64) walkEndBound {
	underivable := walkEndBound{u: math.Inf(1), v: math.Inf(1)}
	if n <= 0 || k <= 0 || k >= n {
		return underivable
	}
	seg, err := normalizeSegment(seg)
	if err != nil {
		return underivable
	}
	start, span, ok := circularSegmentRange(seg)
	if !ok {
		return underivable
	}
	frac := new(big.Rat).SetFrac64(int64(k), int64(n))
	rt := new(big.Rat).Add(start, new(big.Rat).Mul(frac, span))
	return circularPointBound(seg, rt, heldU, heldV)
}
