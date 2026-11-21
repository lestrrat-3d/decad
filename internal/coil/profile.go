package coil

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
)

// Profile is a coil profile's held station chain (agent-docs/helix-design.md §5.3
// and §11 PR 3): every station once, loop-major (outer loop first, then each
// hole), and per loop the station indices in walk order. Every segment
// contributes its walk start and the interior stations of its chords, loft
// §5.1's chord chain, so a wall cell is chorded along the profile as well as
// along the helix: an ArcSeg or a whole CircleSeg by its sweep, a LineSeg at
// its LinePlan's cuts, one chord unless its single cell's departure passes
// the plan's keep threshold. Station v's chord runs to the next station of its loop.
type Profile struct {
	Pts     []sectionrecord.Point2
	LoopIdx [][]int
	// Round[v] bounds the distance from held station v to the point the
	// record denotes there, rounded up: zero at a line end and at an arc's
	// Start, the arc's radial residual at its End, and the enclosure's own
	// gap at a generated station.
	Round []float64
	// Chord[v] is the profile chord that starts at station v.
	Chord []Chord
	// Segments lists every recorded segment, loop-major, in walk order.
	Segments []Segment
}

// Corners returns, per loop, the walk start of every recorded segment in walk
// order: on a line-only profile its recorded vertices, without the generated
// interior stations of its chords. A profile whose every segment holds one
// chord answers its LoopIdx.
func (p Profile) Corners() [][]int {
	out := make([][]int, len(p.LoopIdx))
	for _, s := range p.Segments {
		out[s.Loop] = append(out[s.Loop], s.First)
	}
	return out
}

// Chord is one profile chord: the recorded segment it belongs to, an index
// into Profile.Segments, and for an arc chord Sag, the bound on how far the
// recorded arc departs from the chord between its two denoted ends at a
// matched parameter (r·Δφ²/8), and Arc, the bound on that arc piece's
// length (r·Δφ). Both are nil on a line.
type Chord struct {
	Segment  int
	Sag, Arc *big.Rat
}

// Segment is one recorded segment of a coil profile: its normalized record,
// its loop and position in the loop, the station its walk starts at, and the
// chords it is cut into. Closed marks a whole circle, or a
// whole-turn arc whose End is its Start, a loop on its own; Reversed marks a walk against the curve's natural sense.
type Segment struct {
	Record        sectionrecord.CurveSegment
	Loop, Index   int
	First, Chords int
	Closed        bool
	Reversed      bool
	// Radius, Sweep and Length enclose a circular segment's radius, its
	// unsigned swept angle and its length.
	Radius, Sweep, Length Iv
	// EndRound bounds how far the segment's walk end, the next segment's
	// walk start, sits from the point this segment denotes there.
	EndRound float64
}

// Next is the index of the segment after segment i in its loop's walk.
func (p Profile) Next(i int) int {
	if i+1 < len(p.Segments) && p.Segments[i+1].Loop == p.Segments[i].Loop {
		return i + 1
	}
	for i > 0 && p.Segments[i-1].Loop == p.Segments[i].Loop {
		i--
	}
	return i
}

// Prev is the index of the segment before segment i in its loop's walk.
func (p Profile) Prev(i int) int {
	if i > 0 && p.Segments[i-1].Loop == p.Segments[i].Loop {
		return i - 1
	}
	for i+1 < len(p.Segments) && p.Segments[i+1].Loop == p.Segments[i].Loop {
		i++
	}
	return i
}

// HasArcs reports whether any segment is circular.
func (p Profile) HasArcs() bool {
	for _, s := range p.Segments {
		if s.IsArc() {
			return true
		}
	}
	return false
}

// IsArc reports whether the segment is circular.
func (s Segment) IsArc() bool {
	switch s.Record.(type) {
	case sectionrecord.ArcSeg, sectionrecord.CircleSeg:
		return true
	default:
		return false
	}
}

// LineChordTarget is the per-cell departure, in millimetres, a line aims at
// (agent-docs/helix-design.md §5.3): a line whose one cell's departure
// passes it is cut into chords that meet it. A line never aims below twice
// its own helix sag, which no profile chord shortens. At 0.002 mm a chorded
// wall sits ten times inside a 0.02 mm document chain tolerance
// (WithChainTolerance), so the rims later booleans trim across it stay
// admitted; 0.004 mm was measured too coarse for that, and the §13 springs and
// every reduced-density thread coil keep one chord per line.
const LineChordTarget = 0.002

// MaxLineChords is the most chords one line is cut into. A line whose target
// needs more aims at the smallest target the ceiling fits.
const MaxLineChords = 256

// LinePlan decides where each LineSeg of a coil profile is cut into chords
// (agent-docs/helix-design.md §5.3). A line held as one chord makes every
// wall cell span its whole radial run, and §5.4's twist term grows with that
// run, the station step and pitch/ρ_min. Cutting the line gives every cell its
// own end radii. The plan holds the axis, the pitch and the station step as
// floats, which decide the cuts and nothing else: every bound reads the exact
// radii of each chord's ends. The zero plan holds every line as one chord.
type LinePlan struct {
	// AU, AV is a point of the axis and DU, DV its direction, in plane
	// coordinates; Side puts the profile on the positive side of
	// e_r = Side·(−DV, DU).
	AU, AV, DU, DV float64
	Side           int
	// Pitch is the advance per turn and Step the station step in turns.
	Pitch, Step float64
	// MaxChords caps each line's chord count below MaxLineChords; zero
	// leaves MaxLineChords.
	MaxChords int
}

// maxChords is the plan's chord ceiling per line.
func (lp LinePlan) maxChords() int {
	if lp.MaxChords > 0 && lp.MaxChords < MaxLineChords {
		return lp.MaxChords
	}
	return MaxLineChords
}

// Cuts returns the walk parameters, increasing inside (0, 1), at which the
// line from start to end is cut; nil keeps one chord. The target is
// max(LineChordTarget, 2·sag), with sag = ρ_max·h²/2 the line's helix sag. A
// line whose one cell's §5.4 departure (sag, twist and shift) meets it keeps
// one chord. Any other is cut from its walk start outward, each chord the
// longest whose radial run stays within its own inner radius and whose cell
// meets the target, so the chords grade with the radius as the twist term's
// pitch/ρ_min does. A line that would take more chords than the plan's
// ceiling (MaxChords, at most MaxLineChords) aims at the smallest target that
// fits the ceiling instead.
func (lp LinePlan) Cuts(start, end sectionrecord.Point2) []float64 {
	if !(lp.Step > 0) || lp.Side == 0 || proofbound.IsNonFinite(lp.Step) || proofbound.IsNonFinite(lp.Pitch) {
		return nil
	}
	rho := func(p sectionrecord.Point2) float64 {
		r := (p.V-lp.AV)*lp.DU - (p.U-lp.AU)*lp.DV
		if lp.Side < 0 {
			return -r
		}
		return r
	}
	r0, r1 := rho(start), rho(end)
	if !(r0 > 0) || !(r1 > 0) || proofbound.IsNonFinite(r0) || proofbound.IsNonFinite(r1) {
		return nil
	}
	h := math.Pi * lp.Step
	target := math.Max(LineChordTarget, math.Max(r0, r1)*h*h)
	if lp.departure(r0, r1) <= target || lp.maxChords() < 2 {
		return nil
	}
	if cuts, ok := lp.grade(r0, r1, target); ok {
		return cuts
	}
	// The target needs more chords than the ceiling: raise it to the
	// smallest that fits the ceiling, which one chord's departure bounds.
	lo, hi := target, lp.departure(r0, r1)
	best, _ := lp.grade(r0, r1, hi)
	for range 40 {
		mid := lo + (hi-lo)/2
		if cuts, ok := lp.grade(r0, r1, mid); ok {
			best, hi = cuts, mid
		} else {
			lo = mid
		}
	}
	return best
}

// grade cuts the line at radii r0 → r1 from its walk start outward, each
// chord the longest that fits target. ok is false when that takes more
// chords than the plan's ceiling.
func (lp LinePlan) grade(r0, r1, target float64) ([]float64, bool) {
	at := func(t float64) float64 { return r0 + t*(r1-r0) }
	var cuts []float64
	from := 0.0
	for len(cuts) < lp.maxChords()-1 {
		if lp.fits(at(from), r1, target) {
			return cuts, true
		}
		lo, hi := from, 1.0
		for range 60 {
			mid := lo + (hi-lo)/2
			if lp.fits(at(from), at(mid), target) {
				lo = mid
			} else {
				hi = mid
			}
		}
		if !(lo > from) {
			return nil, false
		}
		cuts = append(cuts, lo)
		from = lo
	}
	return nil, false
}

// fits reports whether one cell at radii a and b keeps its radial run within
// its inner radius and its departure within target, each with a relative
// margin of 1e-9 so the exact chord radii, which differ from these floats by
// rounding, meet both too.
func (lp LinePlan) fits(a, b, target float64) bool {
	const margin = 1 - 1e-9
	return math.Abs(b-a) <= margin*math.Min(a, b) && lp.departure(a, b) <= margin*target
}

// departure is CellDepartureUpper's sum in floats for one cell at radii a
// and b: sag + twist + shift with h = π·Step, k = Pitch/2π and
// c = min(1, ρ_min/|Δρ|). It decides the cuts only.
func (lp LinePlan) departure(a, b float64) float64 {
	h := math.Pi * lp.Step
	k := lp.Pitch / (2 * math.Pi)
	lo, hi := math.Min(a, b), math.Max(a, b)
	out := hi * h * h / 2
	dr := hi - lo
	if dr == 0 {
		return out
	}
	c := math.Min(1, lo/dr)
	twist := dr * h / 2 * ((1 - c) + c*(h+k/lo))
	shift := c * c * dr * dr * h * h / (8 * lo)
	return out + twist + shift
}

// Loops returns the held station chain of a coil profile whose segments
// are whole LineSegs, whole ArcSegs and whole CircleSegs (agent-docs/helix-design.md
// Table CS row CS7). chordsPerTurn is the number of chords a full turn of an
// arc is cut into; an arc of sweep Δ takes ⌈chordsPerTurn·Δ/2π⌉ of them, at
// least one, and a whole circle takes chordsPerTurn, at least three. lines
// decides where each line is cut (LinePlan.Cuts). Any
// other kind is ErrUnsupported, as is a trimmed segment, a circle sharing its
// loop, and a loop whose segment ends do not meet exactly: none of those has
// an exact recorded vertex for the held station-0 section to hold at the
// junction.
func Loops(outer sectionrecord.LoopRecord, holes []sectionrecord.LoopRecord, chordsPerTurn int, lines LinePlan) (Profile, error) {
	loops := append([]sectionrecord.LoopRecord{outer}, holes...)
	var out Profile
	for i, loop := range loops {
		n := len(loop.Segments)
		ends := make([]sectionrecord.Point2, n)
		var idx []int
		for j, raw := range loop.Segments {
			segment, err := sectionrecord.NormalizeSegment(raw)
			if err != nil {
				return Profile{}, err
			}
			seg := Segment{Record: segment, Loop: i, Index: j, First: len(out.Pts)}
			var start sectionrecord.Point2
			switch s := segment.(type) {
			case sectionrecord.LineSeg:
				switch {
				case s.TStart == 0 && s.TEnd == 1:
					start, ends[j] = s.Start, s.End
				case s.TStart == 1 && s.TEnd == 0:
					start, ends[j], seg.Reversed = s.End, s.Start, true
				default:
					return Profile{}, fmt.Errorf(`%w: a coil sweeps whole profile segments only; loop %d segment %d is a trimmed line`,
						decaderr.ErrUnsupported, i, j)
				}
				if err := out.addLine(&seg, start, ends[j], lines.Cuts(start, ends[j])); err != nil {
					return Profile{}, err
				}
			case sectionrecord.ArcSeg:
				if err := out.addArc(&seg, s, chordsPerTurn, ends, i, j); err != nil {
					return Profile{}, err
				}
				// An arc that ends where it starts, alone in its loop, sweeps
				// a whole turn and closes on its own Start: it bounds its wall
				// as a whole circle does, with no junction of its own.
				seg.Closed = n == 1 && s.Start == s.End
			case sectionrecord.CircleSeg:
				if n != 1 {
					return Profile{}, fmt.Errorf(`%w: coil profile loop %d holds a whole circle beside other segments`, decaderr.ErrUnsupported, i)
				}
				if err := out.addCircle(&seg, s, chordsPerTurn, i); err != nil {
					return Profile{}, err
				}
			default:
				return Profile{}, fmt.Errorf(`%w: a coil sweeps line, arc and circle profile segments only; loop %d segment %d is %T`,
					decaderr.ErrUnsupported, i, j, segment)
			}
			for k := seg.First; k < len(out.Pts); k++ {
				idx = append(idx, k)
			}
			out.Segments = append(out.Segments, seg)
		}
		if n == 1 && out.Segments[len(out.Segments)-1].Closed {
			out.LoopIdx = append(out.LoopIdx, idx)
			continue
		}
		first := len(out.Segments) - n
		for j := range n {
			next := out.Segments[first+(j+1)%n].First
			if ends[j] != out.Pts[next] {
				return Profile{}, fmt.Errorf(`%w: coil profile loop %d segment %d does not end exactly where the next one starts`,
					decaderr.ErrUnsupported, i, j)
			}
			out.Round[next] = math.Max(out.Round[next], out.Segments[first+j].EndRound)
		}
		out.LoopIdx = append(out.LoopIdx, idx)
	}
	return out, nil
}

// addLine appends a whole LineSeg's walk start, a recorded point, and one
// interior station per cut, at the exact walk parameter t the cut's float
// states: each station is the float nearest the exact point
// start + t·(end − start), with the distance to it as its plane round. Every
// chord of a line carries no sagitta.
func (p *Profile) addLine(seg *Segment, start, end sectionrecord.Point2, cuts []float64) error {
	segIndex := len(p.Segments)
	seg.Chords = len(cuts) + 1
	p.Pts = append(p.Pts, start)
	p.Round = append(p.Round, 0)
	p.Chord = append(p.Chord, Chord{Segment: segIndex})
	if len(cuts) == 0 {
		return nil
	}
	u0, v0 := proofarith.FloatRat(start.U), proofarith.FloatRat(start.V)
	u1, v1 := proofarith.FloatRat(end.U), proofarith.FloatRat(end.V)
	if u0 == nil || v0 == nil || u1 == nil || v1 == nil {
		return fmt.Errorf(`%w: coil profile loop %d line %d states no finite end`, decaderr.ErrUnsupported, seg.Loop, seg.Index)
	}
	du, dv := new(big.Rat).Sub(u1, u0), new(big.Rat).Sub(v1, v0)
	for k, t := range cuts {
		f := proofarith.FloatRat(t)
		if f == nil || f.Sign() <= 0 || f.Cmp(big.NewRat(1, 1)) >= 0 || (k > 0 && !(t > cuts[k-1])) {
			return fmt.Errorf(`%w: coil profile loop %d line %d states no station %d`, decaderr.ErrUnsupported, seg.Loop, seg.Index, k+1)
		}
		u := new(big.Rat).Add(u0, new(big.Rat).Mul(f, du))
		v := new(big.Rat).Add(v0, new(big.Rat).Mul(f, dv))
		pt, round, ok := heldOf(Point(u), Point(v))
		if !ok {
			return fmt.Errorf(`%w: coil profile loop %d line %d states no station %d`, decaderr.ErrUnsupported, seg.Loop, seg.Index, k+1)
		}
		p.Pts = append(p.Pts, pt)
		p.Round = append(p.Round, round)
		p.Chord = append(p.Chord, Chord{Segment: segIndex})
	}
	return nil
}

// arcChords is the chord count of a circular sweep of held angle sweep: the
// held float decides a count and nothing else.
func arcChords(sweep float64, chordsPerTurn, least int) int {
	m := int(math.Ceil(math.Abs(sweep)/(2*math.Pi)*float64(chordsPerTurn) - 1e-9))
	return max(m, least)
}

// addArc appends a whole ArcSeg's walk start and interior stations. The
// arc denotes θ(t) = a0 + t·sweep on Start's radius, so its Start is the
// denoted point at t = 0 exactly and its End sits off the denoted point at
// t = 1 by the radial residual; every interior station is the float nearest
// the midpoint of the denoted point's enclosure at its exact parameter.
func (p *Profile) addArc(seg *Segment, s sectionrecord.ArcSeg, chordsPerTurn int, ends []sectionrecord.Point2, i, j int) error {
	switch {
	case s.TStart == 0 && s.TEnd == 1:
	case s.TStart == 1 && s.TEnd == 0:
		seg.Reversed = true
	default:
		return fmt.Errorf(`%w: a coil sweeps whole profile segments only; loop %d segment %d is a trimmed arc`,
			decaderr.ErrUnsupported, i, j)
	}
	residual := circularbounds.ArcRadialResidualUpper(s)
	if proofbound.IsNonFinite(residual) {
		return fmt.Errorf(`%w: coil profile loop %d arc %d states no radius`, decaderr.ErrUnsupported, i, j)
	}
	r, sweep, ok := circularbounds.WalkEnclosures(circularbounds.RecordSegment(s))
	if !ok || r.Lo.Sign() <= 0 || sweep.Lo.Sign() <= 0 {
		return fmt.Errorf(`%w: coil profile loop %d arc %d states no radius or sweep`, decaderr.ErrUnsupported, i, j)
	}
	held := math.Atan2(s.End.V-s.Center.V, s.End.U-s.Center.U) - math.Atan2(s.Start.V-s.Center.V, s.Start.U-s.Center.U)
	if held <= 0 {
		held += 2 * math.Pi
	}
	m := arcChords(held, chordsPerTurn, 1)
	seg.Chords, seg.Radius, seg.Sweep = m, r, sweep
	seg.Length = proofbound.IntervalMul(r, sweep)
	start, end, startRound, endRound := s.Start, s.End, 0.0, residual
	if seg.Reversed {
		start, end, startRound, endRound = s.End, s.Start, residual, 0.0
	}
	ends[j] = end
	seg.EndRound = endRound
	return p.addChain(seg, s, start, startRound, m, r, sweep)
}

// addCircle appends a whole CircleSeg's stations. The circle records no
// point at all, so every station, the walk start included, is generated.
func (p *Profile) addCircle(seg *Segment, s sectionrecord.CircleSeg, chordsPerTurn, i int) error {
	t0, t1 := proofarith.FloatRat(s.TStart), proofarith.FloatRat(s.TEnd)
	if t0 == nil || t1 == nil {
		return fmt.Errorf(`%w: coil profile loop %d circle states no range`, decaderr.ErrUnsupported, i)
	}
	span := new(big.Rat).Sub(t1, t0)
	if new(big.Rat).Abs(span).Cmp(big.NewRat(1, 1)) != 0 {
		return fmt.Errorf(`%w: a coil sweeps whole profile segments only; loop %d is a trimmed circle`, decaderr.ErrUnsupported, i)
	}
	radius, err := s.Radius.In(units.Millimeter)
	rr := proofarith.FloatRat(math.Abs(radius))
	if err != nil || rr == nil || rr.Sign() == 0 {
		return fmt.Errorf(`%w: coil profile loop %d circle states no radius`, decaderr.ErrUnsupported, i)
	}
	m := max(chordsPerTurn, 3)
	r := Point(rr)
	sweep := proofbound.TwoPiInterval()
	seg.Chords, seg.Closed, seg.Reversed, seg.Radius, seg.Sweep = m, true, span.Sign() < 0, r, sweep
	seg.Length = proofbound.IntervalMul(r, sweep)
	rec := circularbounds.RecordSegment(s)
	u, v, ok := circularbounds.EndpointInterval(rec, t0)
	if !ok {
		return fmt.Errorf(`%w: coil profile loop %d circle states no start`, decaderr.ErrUnsupported, i)
	}
	start, startRound, ok := heldOf(u, v)
	if !ok {
		return fmt.Errorf(`%w: coil profile loop %d circle states no start`, decaderr.ErrUnsupported, i)
	}
	return p.addChain(seg, s, start, startRound, m, r, sweep)
}

// addChain appends a circular segment's walk start and its m − 1 interior
// stations at the exact parameters TStart + (k/m)·(TEnd − TStart), and each
// of its m chords' sagitta and arc-length bounds.
func (p *Profile) addChain(seg *Segment, s sectionrecord.CurveSegment, start sectionrecord.Point2, startRound float64, m int, r, sweep Iv) error {
	rec := circularbounds.RecordSegment(s)
	segIndex := len(p.Segments)
	step := new(big.Rat).Quo(sweep.Hi, big.NewRat(int64(m), 1))
	arc := new(big.Rat).Mul(r.Hi, step)
	sag := new(big.Rat).Mul(arc, step)
	sag.Quo(sag, big.NewRat(8, 1))
	tStart, tEnd := walkRange(s)
	span := new(big.Rat).Sub(tEnd, tStart)
	p.Pts = append(p.Pts, start)
	p.Round = append(p.Round, startRound)
	p.Chord = append(p.Chord, Chord{Segment: segIndex, Sag: sag, Arc: arc})
	for k := 1; k < m; k++ {
		rt := new(big.Rat).Add(tStart, new(big.Rat).Mul(big.NewRat(int64(k), int64(m)), span))
		u, v, ok := circularbounds.EndpointInterval(rec, rt)
		if !ok {
			return fmt.Errorf(`%w: coil profile loop %d segment %d states no station %d`, decaderr.ErrUnsupported, seg.Loop, seg.Index, k)
		}
		pt, round, ok := heldOf(u, v)
		if !ok {
			return fmt.Errorf(`%w: coil profile loop %d segment %d states no station %d`, decaderr.ErrUnsupported, seg.Loop, seg.Index, k)
		}
		p.Pts = append(p.Pts, pt)
		p.Round = append(p.Round, round)
		p.Chord = append(p.Chord, Chord{Segment: segIndex, Sag: sag, Arc: arc})
	}
	return nil
}

// walkRange is a circular segment's recorded range as exact rationals.
func walkRange(s sectionrecord.CurveSegment) (*big.Rat, *big.Rat) {
	switch s := s.(type) {
	case sectionrecord.ArcSeg:
		return proofarith.FloatRat(s.TStart), proofarith.FloatRat(s.TEnd)
	case sectionrecord.CircleSeg:
		return proofarith.FloatRat(s.TStart), proofarith.FloatRat(s.TEnd)
	default:
		return new(big.Rat), big.NewRat(1, 1)
	}
}

// heldOf is the float point nearest an enclosure's midpoint and the plane
// distance from it to the enclosure's far corner, rounded up.
func heldOf(u, v Iv) (sectionrecord.Point2, float64, bool) {
	hu, _ := Mid(u).Float64()
	hv, _ := Mid(v).Float64()
	if proofbound.IsNonFinite(hu) || proofbound.IsNonFinite(hv) {
		return sectionrecord.Point2{}, 0, false
	}
	round := proofbound.Radius2D(proofbound.IntervalFloatError(u, hu), proofbound.IntervalFloatError(v, hv))
	if proofbound.IsNonFinite(round) {
		return sectionrecord.Point2{}, 0, false
	}
	return sectionrecord.Point2{U: hu, V: hv}, round, true
}
