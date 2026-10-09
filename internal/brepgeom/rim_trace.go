package brepgeom

import (
	"context"
	"slices"

	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// RimTraceResult holds the loops left after a removed face's outer loop and
// the reversed cavity loops cancel their shared line pieces. Without any
// cancellation, the first loop is the original outer and the rest are holes.
type RimTraceResult struct {
	Loops     []sectionrecord.LoopRecord
	Cancelled bool
}

// RimTraceError is a trace that cannot chain into closed loops. The caller
// names the removed face and maps this reason to its SG7 refusal.
type RimTraceError string

func (e RimTraceError) Error() string { return string(e) }

// rimTracePiece is one directed piece in the removed face's frame. Curved
// pieces learn their ends from adjacent lines after the shared lines split.
type rimTracePiece struct {
	seg                sectionrecord.CurveSegment
	from, to           sectionrecord.Point2
	fromKnown, toKnown bool
	line, gone         bool
	src, seqIndex      int
}

// TraceRim cancels coincident line pieces of outer and the reversed cavity
// loops, then chains the survivors. Every split uses a recorded line end.
// The caller classifies the resulting loops by their certified signed areas.
func TraceRim(ctx context.Context, outer sectionrecord.LoopRecord,
	cavity []sectionrecord.LoopRecord) (RimTraceResult, error) {
	holes := make([]sectionrecord.LoopRecord, len(cavity))
	for i, q := range cavity {
		var err error
		if holes[i], err = offset2d.ReverseLoopRecordContext(ctx, q); err != nil {
			return RimTraceResult{}, err
		}
	}
	loops := append([]sectionrecord.LoopRecord{outer}, holes...)
	seqs := make([][]*rimTracePiece, len(loops))
	for li, loop := range loops {
		for _, seg := range loop.Segments {
			p := &rimTracePiece{seg: seg, src: li}
			if from, to, ok := NaturalLine(seg); ok && (from.U == to.U) != (from.V == to.V) {
				p.from, p.to, p.line, p.fromKnown, p.toKnown = from, to, true, true, true
			}
			seqs[li] = append(seqs[li], p)
		}
	}
	for li := range seqs {
		var split []*rimTracePiece
		for _, p := range seqs[li] {
			split = append(split, p.splitAt(seqs, li)...)
		}
		seqs[li] = split
	}
	cancelled := 0
	for _, p := range seqs[0] {
		for _, other := range seqs[1:] {
			if i := slices.IndexFunc(other, func(q *rimTracePiece) bool {
				return p.line && q.line && !q.gone && q.from == p.to && q.to == p.from
			}); i >= 0 {
				p.gone, other[i].gone = true, true
				cancelled++
				break
			}
		}
	}
	if cancelled == 0 {
		return RimTraceResult{Loops: loops}, nil
	}
	for _, seq := range seqs {
		n := len(seq)
		for i, p := range seq {
			p.seqIndex = i
			if p.line {
				continue
			}
			if prev := seq[(i+n-1)%n]; prev.line {
				p.from, p.fromKnown = prev.to, true
			}
			if next := seq[(i+1)%n]; next.line {
				p.to, p.toKnown = next.from, true
			}
		}
	}
	chained, err := rimTraceChain(seqs)
	if err != nil {
		return RimTraceResult{}, err
	}
	result := RimTraceResult{Cancelled: true}
	for _, loop := range chained {
		var rec sectionrecord.LoopRecord
		for _, p := range loop {
			rec.Segments = append(rec.Segments, p.seg)
		}
		result.Loops = append(result.Loops, rec)
	}
	return result, nil
}

// splitAt cuts a line at every end of another loop lying strictly within it.
func (p *rimTracePiece) splitAt(seqs [][]*rimTracePiece, li int) []*rimTracePiece {
	if !p.line {
		return []*rimTracePiece{p}
	}
	along := 0
	if p.from.U == p.to.U {
		along = 1
	}
	coord := func(c sectionrecord.Point2) (float64, float64) {
		if along == 0 {
			return c.U, c.V
		}
		return c.V, c.U
	}
	a, level := coord(p.from)
	b, _ := coord(p.to)
	lo, hi := min(a, b), max(a, b)
	var cuts []float64
	for lj, other := range seqs {
		if lj == li {
			continue
		}
		for _, q := range other {
			if !q.line {
				continue
			}
			for _, c := range []sectionrecord.Point2{q.from, q.to} {
				x, y := coord(c)
				if y == level && lo < x && x < hi && !slices.Contains(cuts, x) {
					cuts = append(cuts, x)
				}
			}
		}
	}
	if len(cuts) == 0 {
		return []*rimTracePiece{p}
	}
	slices.Sort(cuts)
	if a > b {
		slices.Reverse(cuts)
	}
	point := func(x float64) sectionrecord.Point2 {
		if along == 0 {
			return sectionrecord.Point2{U: x, V: level}
		}
		return sectionrecord.Point2{U: level, V: x}
	}
	out := make([]*rimTracePiece, 0, len(cuts)+1)
	from := p.from
	for _, x := range append(cuts, b) {
		to := point(x)
		if x == b {
			to = p.to
		}
		out = append(out, &rimTracePiece{seg: sectionrecord.LineSeg{Start: from, End: to, TStart: 0, TEnd: 1},
			from: from, to: to, fromKnown: true, toKnown: true, line: true, src: p.src})
		from = to
	}
	return out
}

// rimTraceChain follows each surviving piece, changing loops where a
// cancelled predecessor exposes exactly one continuation.
func rimTraceChain(seqs [][]*rimTracePiece) ([][]*rimTracePiece, error) {
	next := func(p *rimTracePiece) (*rimTracePiece, error) {
		seq := seqs[p.src]
		if s := seq[(p.seqIndex+1)%len(seq)]; !s.gone {
			return s, nil
		}
		var found *rimTracePiece
		for _, other := range seqs {
			for _, q := range other {
				prev := other[(q.seqIndex+len(other)-1)%len(other)]
				if q.gone || !prev.gone || !q.fromKnown || q.from != p.to {
					continue
				}
				if found != nil {
					return nil, RimTraceError("two pieces continue the cavity's trace from one vertex")
				}
				found = q
			}
		}
		if found == nil {
			return nil, RimTraceError("a piece left by the cavity's trace has no continuation")
		}
		return found, nil
	}
	visited := map[*rimTracePiece]struct{}{}
	var loops [][]*rimTracePiece
	for _, seq := range seqs {
		for _, start := range seq {
			if _, seen := visited[start]; start.gone || seen {
				continue
			}
			var loop []*rimTracePiece
			for p := start; ; {
				if _, seen := visited[p]; seen {
					return nil, RimTraceError("the cavity's trace leaves a loop that does not close")
				}
				visited[p] = struct{}{}
				loop = append(loop, p)
				q, err := next(p)
				if err != nil {
					return nil, err
				}
				if q == start {
					break
				}
				p = q
			}
			loops = append(loops, loop)
		}
	}
	return loops, nil
}
