package offset2d

import (
	"fmt"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// ArcRange is a circular segment's recorded parameter range.
func ArcRange(seg sectionrecord.CurveSegment) (float64, float64) {
	switch s := seg.(type) {
	case sectionrecord.ArcSeg:
		return s.TStart, s.TEnd
	case sectionrecord.CircleSeg:
		return s.TStart, s.TEnd
	}
	return 0, 0
}

// ArcWithRange walks seg over [t0, t1] of its own parameterisation, retaining
// the defining circle and its recorded end points.
func ArcWithRange(seg sectionrecord.CurveSegment, t0, t1 float64) sectionrecord.CurveSegment {
	switch s := seg.(type) {
	case sectionrecord.ArcSeg:
		s.TStart, s.TEnd = t0, t1
		return s
	case sectionrecord.CircleSeg:
		s.TStart, s.TEnd, s.CCW = t0, t1, t0 < t1
		return s
	}
	return seg
}

type Point2 = sectionrecord.Point2
type CurveSegment = sectionrecord.CurveSegment
type ArcSeg = sectionrecord.ArcSeg
type CircleSeg = sectionrecord.CircleSeg
type LineSeg = sectionrecord.LineSeg
type ProfileRecord = momentinput.Profile
type LoopRecord = sectionrecord.LoopRecord

// PrismRemovedRun maps the removed outer-loop segments onto the section's
// coalesced walks and checks §5's SO6 run rule: every removed walk is removed
// whole, the removed walks are one proper connected run around the loop, and
// at least one walk is kept. It returns the run's walk indices in walk order.
func PrismRemovedRun(walks []survey2d.SideWalk, segs map[int]struct{}) ([]int, error) {
	n := len(walks)
	removed := make([]bool, n)
	count := 0
	for i, w := range walks {
		hit := 0
		for _, si := range w.Segs {
			if _, ok := segs[si]; ok {
				hit++
			}
		}
		switch hit {
		case 0:
		case len(w.Segs):
			removed[i] = true
			count++
		default:
			return nil, fmt.Errorf(`%w: a removed side face covers only part of a section walk (modify-reach SX8, shell-opening SO6)`, decaderr.ErrUnsupported)
		}
	}
	if count == 0 || count == n {
		return nil, fmt.Errorf(`%w: a side opening must remove a proper run of the prism's side faces, not all of them (modify-reach SX8, shell-opening SO6)`, decaderr.ErrUnsupported)
	}
	first := -1
	runs := 0
	for i := range n {
		if removed[i] && !removed[(i+n-1)%n] {
			runs++
			first = i
		}
	}
	if runs != 1 {
		return nil, fmt.Errorf(`%w: a side opening's removed faces must be one connected run (modify-reach SX8, shell-opening SO6)`, decaderr.ErrUnsupported)
	}
	run := make([]int, 0, count)
	for k := range count {
		run = append(run, (first+k)%n)
	}
	return run, nil
}

// RequireSideOpeningWalks refuses a section walk this build does not take:
// every walk, kept or removed, must be an open line or circular arc. A
// free-form walk or a whole circle is ErrUnsupported, SO5's sentinel.
func RequireSideOpeningWalks(walks []survey2d.SideWalk) error {
	for _, w := range walks {
		if !w.Closed && (w.Kind == survey2d.WalkLine || w.IsCircular()) {
			continue
		}
		return fmt.Errorf(`%w: this evaluator builds a prism side opening only where every section walk is a line or a circular arc (shell-opening SO5)`, decaderr.ErrUnsupported)
	}
	return nil
}

// ArcStation is a point a piece on a removed circular walk's carrier starts
// or ends at (§4.2): p as the record states it, and where it lies along the
// walk. An in-span station names the walk segment seg (an index into the
// walk's Segs) and its parameter t in that segment's own parameterisation; a
// station off the walk's span (off) lies on the carrier's extension, before
// the walk's start where before is set and past its end otherwise.
type ArcStation struct {
	p      Point2
	seg    int
	t      float64
	off    bool
	before bool
}

// arcParam is the parameter at which seg's own parameterisation reaches the
// angle of p about its centre, as the segment's walk evaluates angles
// (boundarywalk's walkOf): an ArcSeg's counter-clockwise sweep from Start's
// angle to End's, a CircleSeg's whole turn from angle zero. The answer lies in
// [0, 2π/sweep) for an arc and [0, 1) for a circle; it is a float solve, and
// the caller charges the point it names (sideOpeningSection.cutGap).
func arcParam(seg CurveSegment, p Point2) (float64, bool) {
	turn := func(a float64) float64 {
		a = math.Mod(a, 2*math.Pi)
		if a < 0 {
			a += 2 * math.Pi
		}
		return a
	}
	switch s := seg.(type) {
	case ArcSeg:
		a0 := math.Atan2(s.Start.V-s.Center.V, s.Start.U-s.Center.U)
		a1 := math.Atan2(s.End.V-s.Center.V, s.End.U-s.Center.U)
		sweep := math.Mod(a1-a0, 2*math.Pi)
		if sweep <= 0 {
			sweep += 2 * math.Pi
		}
		return turn(math.Atan2(p.V-s.Center.V, p.U-s.Center.U)-a0) / sweep, true
	case CircleSeg:
		return turn(math.Atan2(p.V-s.Center.V, p.U-s.Center.U)) / (2 * math.Pi), true
	}
	return 0, false
}

// arcWalkStation and arcWalkEnd are the stations at a circular walk's own
// start and end: its first segment's TStart and its last segment's TEnd.
func arcWalkStation(w survey2d.SideWalk, segs []CurveSegment) ArcStation {
	t0, _ := ArcRange(segs[w.Segs[0]])
	return ArcStation{p: Point2{U: w.StartU, V: w.StartV}, seg: 0, t: t0}
}

func arcWalkEnd(w survey2d.SideWalk, segs []CurveSegment) ArcStation {
	last := len(w.Segs) - 1
	_, t1 := ArcRange(segs[w.Segs[last]])
	return ArcStation{p: Point2{U: w.EndU, V: w.EndV}, seg: last, t: t1}
}

// CutStation places a rim cut q on removed walk w: a forward cut strictly
// inside the parameter range of one of w's segments, a backward cut on the
// carrier's extension behind the walk's start (atStart) or past its end. A
// straight walk needs only the point. A forward cut no segment's range holds
// strictly inside — its parameter rounded onto or past a segment's end — is
// SO5, ErrUnsupported.
func CutStation(w survey2d.SideWalk, segs []CurveSegment, q Point2, forward, atStart bool) (ArcStation, error) {
	if !w.IsCircular() {
		return ArcStation{p: q}, nil
	}
	if !forward {
		return ArcStation{p: q, off: true, before: atStart}, nil
	}
	for i, si := range w.Segs {
		t, ok := arcParam(segs[si], q)
		t0, t1 := ArcRange(segs[si])
		if ok && (t-t0)*(t1-t) > 0 {
			return ArcStation{p: q, seg: i, t: t}, nil
		}
	}
	return ArcStation{}, fmt.Errorf(`%w: a side opening's rim cut on a removed arc lies at no parameter strictly inside the arc's recorded range (shell-opening SO5)`, decaderr.ErrUnsupported)
}

// removedPiece is the piece of the removed walk w's carrier from station a to
// station b (§4.2). A straight walk's piece is one LineSeg. On a circular walk
// every piece is a parameter range of w's own recorded segments, so it keys
// w's circle bit for bit: between two in-span stations, each segment the
// piece crosses over the part of its range the piece covers, verbatim where
// it covers the whole range, and walked backward where b precedes a; between
// a walk end and a station off the span, one segment (arcExtension).
func removedPiece(w survey2d.SideWalk, segs []CurveSegment, a, b ArcStation) ([]CurveSegment, error) {
	switch {
	case !w.IsCircular():
		return []CurveSegment{LineSeg{Start: a.p, End: b.p, TStart: 0, TEnd: 1}}, nil
	case a.off:
		seg, err := arcExtension(w, segs, a, false)
		return []CurveSegment{seg}, err
	case b.off:
		seg, err := arcExtension(w, segs, b, true)
		return []CurveSegment{seg}, err
	}
	precedes := func(x, y ArcStation) bool {
		if x.seg != y.seg {
			return x.seg < y.seg
		}
		t0, t1 := ArcRange(segs[w.Segs[x.seg]])
		return (y.t-x.t)*(t1-t0) > 0
	}
	if precedes(b, a) {
		fwd, err := removedPiece(w, segs, b, a)
		if err != nil {
			return nil, err
		}
		out := make([]CurveSegment, 0, len(fwd))
		for _, seg := range slices.Backward(fwd) {
			t0, t1 := ArcRange(seg)
			out = append(out, ArcWithRange(seg, t1, t0))
		}
		return out, nil
	}
	var out []CurveSegment
	for i := a.seg; i <= b.seg; i++ {
		seg := segs[w.Segs[i]]
		from, to := ArcRange(seg)
		if i == a.seg {
			from = a.t
		}
		if i == b.seg {
			to = b.t
		}
		if from == to {
			continue
		}
		out = append(out, ArcWithRange(seg, from, to))
	}
	return out, nil
}

// arcExtension is the piece of a circular walk's carrier between the walk's
// own end v (its start where q lies before it) and a cut q off its span,
// walked from v to q where fromV is set. Its segment is the one recorded at v:
// over its own parameterisation where q's parameter lies inside [0, 1] beyond
// v's, and otherwise, where v is an ArcSeg's own Start or End, over that arc's
// complement — the same Center with Start and End swapped, which sweeps every
// angle the arc does not. The complement reads its radius from the arc's End,
// so it keys and denotes the arc's own circle only where End lies exactly on
// the circle Start defines, compared in exact rational arithmetic; any other
// extension is SO5, ErrUnsupported.
func arcExtension(w survey2d.SideWalk, segs []CurveSegment, q ArcStation, fromV bool) (CurveSegment, error) {
	v := arcWalkEnd(w, segs)
	if q.before {
		v = arcWalkStation(w, segs)
	}
	seg := segs[w.Segs[v.seg]]
	t0, t1 := ArcRange(seg)
	other := t0
	if q.before {
		other = t1
	}
	base, tv := seg, v.t
	tq, ok := arcParam(seg, q.p)
	if !ok || tq < 0 || tq > 1 || (tq-v.t)*(other-v.t) >= 0 {
		arc, isArc := seg.(ArcSeg)
		if !isArc || (v.t != 0 && v.t != 1) || !arcEndOnCircle(arc) {
			return nil, fmt.Errorf(`%w: a side opening's rim cut behind a removed arc's end lies off the arc's recorded parameterisation, and the arc's complement does not read its circle (shell-opening SO5)`, decaderr.ErrUnsupported)
		}
		base = ArcSeg{Center: arc.Center, Start: arc.End, End: arc.Start}
		tv = 1 - v.t
		tq, _ = arcParam(base, q.p)
		if !(tq > 0 && tq < 1) {
			return nil, fmt.Errorf(`%w: a side opening's rim cut behind a removed arc's end lies at no parameter strictly inside the arc's complement (shell-opening SO5)`, decaderr.ErrUnsupported)
		}
	}
	if fromV {
		return ArcWithRange(base, tv, tq), nil
	}
	return ArcWithRange(base, tq, tv), nil
}

// arcEndOnCircle reports whether an ArcSeg's End lies exactly on the circle
// its Start defines about its Center, compared over exact rationals.
func arcEndOnCircle(arc ArcSeg) bool {
	sq := func(p Point2) *big.Rat {
		du := new(big.Rat).Sub(new(big.Rat).SetFloat64(p.U), new(big.Rat).SetFloat64(arc.Center.U))
		dv := new(big.Rat).Sub(new(big.Rat).SetFloat64(p.V), new(big.Rat).SetFloat64(arc.Center.V))
		return du.Add(du.Mul(du, du), dv.Mul(dv, dv))
	}
	return sq(arc.Start).Cmp(sq(arc.End)) == 0
}

// arcCutGap bounds how far the point a rim's record names at the cut q lies
// from q itself: the rim's pieces end at q's parameter (their last piece's
// TEnd where atEnd is set, the first piece's TStart otherwise), and the point
// the record denotes there is enclosed over rational intervals
// (boundarywalk.CircularWalkEndBound). A straight piece ends at q exactly.
func arcCutGap(pieces []CurveSegment, q Point2, atEnd bool) float64 {
	if len(pieces) == 0 {
		return 0
	}
	seg := pieces[0]
	if atEnd {
		seg = pieces[len(pieces)-1]
	}
	t0, t1 := ArcRange(seg)
	t := t0
	if atEnd {
		t = t1
	}
	switch seg.(type) {
	case ArcSeg, CircleSeg:
		return proofbound.WalkEndBoundAllow(boundarywalk.CircularWalkEndBound(seg, t, q.U, q.V))
	}
	return 0
}

// obliqueLine reports whether a line walk lies off both section axes. The
// record build keys an oblique line by its two recorded endpoints, so every
// region walking its carrier must state it in the same pieces (§4.2).
func obliqueLine(w survey2d.SideWalk) bool {
	return !w.IsCircular() && w.StartU != w.EndU && w.StartV != w.EndV
}

// SplitAtCuts reports whether a removed end walk is stated in pieces at its
// cut and corner (RemovedPieces): an oblique line, whose carrier the record
// build keys by its endpoints, and a circular arc, whose bulge the area
// identity pairs piece by piece (§4.7).
func SplitAtCuts(w survey2d.SideWalk) bool {
	return obliqueLine(w) || w.IsCircular()
}

// walkStations is the station at a removed walk's own start and at its end:
// on a circular walk its segments' natural range ends (arcWalkStation), on a
// straight one the points alone.
func walkStations(w survey2d.SideWalk, segs []CurveSegment) (ArcStation, ArcStation) {
	if w.IsCircular() {
		return arcWalkStation(w, segs), arcWalkEnd(w, segs)
	}
	return ArcStation{p: Point2{U: w.StartU, V: w.StartV}}, ArcStation{p: Point2{U: w.EndU, V: w.EndV}}
}

// RemovedPieces writes the removed run on carrier(r) as one region walks it
// (§3, §4.2): the receiver's own run R from vB to vA (recut false), or R',
// the run re-cut to start at the cut sB and end at sA (recut true). forwardB
// and forwardA say each end's cut runs forward into r's span. An oblique end
// walk is stated in the pieces every other region needs: R's walk split at a
// forward cut, and R' split at v where its cut runs backward along the
// carrier, so the regions' pieces on that carrier are vB → qB, qB → qA and
// qA → vA, each keyed by its own endpoints. A circular end walk is stated in
// the same pieces, each a parameter range of the walk's own recorded segments
// (removedPiece), so every piece keys the walk's circle and the area identity
// pairs each piece's bulge across the regions (§4.7). An axis-aligned end
// walk keeps its one piece: the engine reads its plane by level and splits it
// at the vertices it records. A removed oblique end walk recorded as more
// than one segment is SO5: the record cannot state its pieces alike in every
// region.
func RemovedPieces(walks []survey2d.SideWalk, run []int, segs []CurveSegment, sB, sA ArcStation, forwardB, forwardA, recut bool) ([]CurveSegment, error) {
	rFirst, rLast := walks[run[0]], walks[run[len(run)-1]]
	vB, eB := walkStations(rFirst, segs)
	sA0, vA := walkStations(rLast, segs)
	for _, w := range []survey2d.SideWalk{rFirst, rLast} {
		if obliqueLine(w) && len(w.Segs) != 1 {
			return nil, fmt.Errorf(`%w: a side opening's removed oblique face is recorded as %d collinear segments, which this record build does not state (shell-opening SO5)`, decaderr.ErrUnsupported, len(w.Segs))
		}
	}
	// splitB and splitA are the extra points each end's walk takes on an
	// oblique or circular carrier: the forward cut in R, the corner in R'.
	splitB := SplitAtCuts(rFirst) && forwardB != recut
	splitA := SplitAtCuts(rLast) && forwardA != recut
	var out []CurveSegment
	pieces := func(w survey2d.SideWalk, sts ...ArcStation) error {
		for i := 0; i+1 < len(sts); i++ {
			seg, err := removedPiece(w, segs, sts[i], sts[i+1])
			if err != nil {
				return err
			}
			out = append(out, seg...)
		}
		return nil
	}
	from, to := vB, vA
	inB, inA := sB, sA
	if recut {
		from, to = sB, sA
		inB, inA = vB, vA
	}
	if len(run) == 1 {
		if !recut && !splitB && !splitA {
			return appendWalkSegs(out, rFirst, segs), nil
		}
		sts := []ArcStation{from}
		if splitB {
			sts = append(sts, inB)
		}
		if splitA {
			sts = append(sts, inA)
		}
		if err := pieces(rFirst, append(sts, to)...); err != nil {
			return nil, err
		}
		return out, nil
	}
	var err error
	switch {
	case splitB:
		err = pieces(rFirst, from, inB, eB)
	case recut:
		err = pieces(rFirst, from, eB)
	default:
		out = appendWalkSegs(out, rFirst, segs)
	}
	if err != nil {
		return nil, err
	}
	for _, ri := range run[1 : len(run)-1] {
		out = appendWalkSegs(out, walks[ri], segs)
	}
	switch {
	case splitA:
		err = pieces(rLast, sA0, inA, to)
	case recut:
		err = pieces(rLast, sA0, to)
	default:
		out = appendWalkSegs(out, rLast, segs)
	}
	if err != nil {
		return nil, err
	}
	return out, nil
}

// appendWalkSegs appends a walk's recorded segments verbatim.
func appendWalkSegs(out []CurveSegment, w survey2d.SideWalk, segs []CurveSegment) []CurveSegment {
	for _, si := range w.Segs {
		out = append(out, segs[si])
	}
	return out
}

// SideOpeningRims writes the two rims of §3's W, each already running the way
// W walks it, and the larger cut gap (arcCutGap) of the two: at K's end
// (k = last, r = rFirst) from v to q inward, at K's start (k = first,
// r = rLast) from q to v inward, both reversed outward. A straight r's rim is
// RimSegment's LineSeg; a circular r's is the piece of r's own
// record between v and the cut station (removedPiece).
func SideOpeningRims(walks []survey2d.SideWalk, run []int, segs []CurveSegment, last, first survey2d.SideWalk, sB, sA ArcStation, ends [2]ChainEnd, s float64, inward bool) ([]CurveSegment, []CurveSegment, float64, error) {
	rFirst, rLast := walks[run[0]], walks[run[len(run)-1]]
	rim := func(k, r survey2d.SideWalk, atEnd bool, j Join, cut ArcStation, fromV bool) ([]CurveSegment, float64, error) {
		if !r.IsCircular() {
			seg, err := RimSegment(k, r, atEnd, s, j, fromV)
			return []CurveSegment{seg}, 0, err
		}
		vStart, vEnd := walkStations(r, segs)
		v := vEnd
		if atEnd {
			v = vStart
		}
		from, to := v, cut
		if !fromV {
			from, to = cut, v
		}
		pieces, err := removedPiece(r, segs, from, to)
		if err != nil {
			return nil, 0, err
		}
		return pieces, arcCutGap(pieces, cut.p, fromV), nil
	}
	atEnd, gapB, err := rim(last, rFirst, true, ends[1].Join, sB, inward)
	if err != nil {
		return nil, nil, 0, err
	}
	atStart, gapA, err := rim(first, rLast, false, ends[0].Join, sA, !inward)
	if err != nil {
		return nil, nil, 0, err
	}
	return atEnd, atStart, math.Max(gapB, gapA), nil
}

// WalkedEnds is the first and last point a recorded line or arc is walked
// through, as its walk reads them: verbatim record points at a natural bound
// of its range. It reports false for every other kind.
func WalkedEnds(seg CurveSegment) (Point2, Point2, bool) {
	switch seg.(type) {
	case LineSeg, ArcSeg:
	default:
		return Point2{}, Point2{}, false
	}
	w, err := boundarywalk.WalkOf(seg, nil)
	if err != nil {
		return Point2{}, Point2{}, false
	}
	return Point2{U: w.StartU, V: w.StartV}, Point2{U: w.EndU, V: w.EndV}, true
}

// RequireAreaIdentity is §4.7 step 3: the cap region is the wall plus the
// cavity inward, and the wall plus the receiver's section outward, each
// loop's area read exactly from its walked floats. An arc enters as its
// chord. Every arc stands on both sides of the identity piece by piece — a
// kept arc in P and W, an arc of K' in W and the offset region, a rim in W
// and, split at its cut, in P (forward) or R' (backward), every other piece
// of a removed arc in P and R' — so its bulge cancels and the chord identity
// is the exact one: the caller passes P as the record states it, split at
// each forward cut on an oblique or circular end walk (RemovedPieces). A
// region holding another segment kind, or a sum that differs, is SO5.
func RequireAreaIdentity(caps, wall, cavity ProfileRecord) error {
	area := func(p ProfileRecord) (*big.Rat, bool) {
		total := new(big.Rat)
		for _, loop := range append([]LoopRecord{p.Outer}, p.Holes...) {
			for _, seg := range loop.Segments {
				from, to, ok := WalkedEnds(seg)
				if !ok {
					return nil, false
				}
				su, sv := proofarith.FloatRat(from.U), proofarith.FloatRat(from.V)
				eu, ev := proofarith.FloatRat(to.U), proofarith.FloatRat(to.V)
				if su == nil || sv == nil || eu == nil || ev == nil {
					return nil, false
				}
				total.Add(total, new(big.Rat).Sub(new(big.Rat).Mul(su, ev), new(big.Rat).Mul(eu, sv)))
			}
		}
		return total, true
	}
	ac, okC := area(caps)
	aw, okW := area(wall)
	ax, okX := area(cavity)
	if !okC || !okW || !okX {
		return fmt.Errorf(`%w: a side opening's regions hold a segment whose area this evaluator does not read exactly (shell-opening SO5)`, decaderr.ErrUnsupported)
	}
	// Inward the caps region is P and the cavity C, so P = W + C. Outward the
	// caps region is O and the cavity P, so O = W + P: the same sum.
	if ac.Cmp(new(big.Rat).Add(aw, ax)) != 0 {
		return fmt.Errorf(`%w: a side opening's wall and cavity regions do not tile the section exactly (shell-opening SO5)`, decaderr.ErrUnsupported)
	}
	return nil
}
