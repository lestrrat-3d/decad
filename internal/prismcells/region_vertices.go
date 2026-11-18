package prismcells

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
)

// This file reads the arranged vertices of a one-scene arrangement of
// several recorded sections (agent-docs/prism-boolean-design.md Part K §5.3,
// "Vertex reading"). Every record is one region of the scene's operand A, so
// Origin.Region names the record.

// SourceSegment is the recorded segment an origin names, with its carrier
// entry ε and its Lemma H bound h.
func SourceSegment(src []prismrecord.CarrierRecord, o Origin) (seg sectionrecord.CurveSegment, eps, h float64, ok bool) {
	if o.Region < 0 || o.Region >= len(src) {
		return nil, 0, 0, false
	}
	r := src[o.Region]
	l := o.Hole + 1
	loops := append([]sectionrecord.LoopRecord{r.Profile.Outer}, r.Profile.Holes...)
	if l < 0 || l >= len(loops) || o.Segment < 0 || o.Segment >= len(loops[l].Segments) ||
		l >= len(r.Segment) || o.Segment >= len(r.Segment[l]) {
		return nil, 0, 0, false
	}
	h = r.SegmentH(l, o.Segment)
	if proofbound.IsNonFinite(h) {
		return nil, 0, 0, false
	}
	return loops[l].Segments[o.Segment], r.Segment[l][o.Segment], h, true
}

// Vertex is one arranged vertex where entities of two records cross.
type Vertex struct {
	Point    [2]*big.Rat // P̂_v: A6's exact junction point
	Err      float64     // err_v: a bound on |P̂_v − O| for the recorded crossing O
	CutDelta float64     // δ_cut(v): the larger CutDelta of its two edges
	L        float64     // L_v: Lemma T's distance for the denoted carriers about P̂_v
	Delta    float64     // Δ_v = up(δ_cut + err_v + L_v)
	Origins  [2]Origin
	ends     [2]vertexEnd
}

// VertexReading holds every V1 vertex keyed by the walk-end point sketch
// reports (the vertex identity ChainClosedSurvivors reads).
type VertexReading struct {
	Vertices map[[2]float64]*Vertex
}

// ReadVertices reads every arranged vertex of the cells. A vertex where
// entities of two records meet must be V1's transverse crossing: exactly two
// entities, each passing through it at a parameter the arrangement computed
// on both of its edges. Each is charged Δ_v = up(δ_cut + err_v + L_v) with
// Lemma T's L_v over the two DENOTED source carriers. ok=false (err nil) means
// V1 or a charge failed, and the caller misses.
func ReadVertices(budget *proofbound.WorkBudget, tags map[sketch.Entity]Origin, cells []*sketch.Profile,
	src []prismrecord.CarrierRecord) (VertexReading, bool, error) {
	type endKey struct {
		entity sketch.Entity
		t      float64
	}
	vertices := map[[2]float64][]vertexEnd{}
	seen := map[endKey]struct{}{}
	for _, p := range cells {
		for _, loop := range append([][]sketch.BoundaryEdge{p.Outer}, p.Holes...) {
			for _, e := range loop {
				if err := budget.Step(); err != nil {
					return VertexReading{}, false, err
				}
				if len(e.Polyline) == 0 {
					return VertexReading{}, false, nil
				}
				seg, t0, t1, ok := recordedRange(e)
				if !ok {
					return VertexReading{}, false, nil
				}
				for _, end := range []vertexEnd{{edge: e, seg: seg, t: t0, atStart: true}, {edge: e, seg: seg, t: t1}} {
					k := endKey{entity: e.Entity, t: end.t}
					if _, dup := seen[k]; dup {
						continue
					}
					seen[k] = struct{}{}
					at := e.Polyline[len(e.Polyline)-1]
					if end.atStart {
						at = e.Polyline[0]
					}
					vertices[at] = append(vertices[at], end)
				}
			}
		}
	}
	out := VertexReading{Vertices: map[[2]float64]*Vertex{}}
	for at, ends := range vertices {
		if err := budget.Step(); err != nil {
			return VertexReading{}, false, err
		}
		mixed := false
		first := -1
		for _, end := range ends {
			o, ok := tags[end.edge.Entity]
			if !ok {
				return VertexReading{}, false, nil
			}
			if first >= 0 && o.Region != first {
				mixed = true
			}
			first = o.Region
		}
		if !mixed {
			continue
		}
		// V1: exactly two entities, each passing through at a computed parameter.
		if !transversePair(ends) {
			if guardOn("V1") {
				return VertexReading{}, false, nil
			}
			continue // guard off (tests only): the vertex is read as no vertex, charged nothing
		}
		v, ok := readVertex(ends, tags, src)
		if !ok {
			return VertexReading{}, false, nil
		}
		out.Vertices[at] = v
	}
	return out, true, nil
}

// transversePair is V1's predicate for the ends meeting at one arranged vertex
// of two operands: exactly two entities, each passing through the vertex at a
// parameter the arrangement computed on both of its edges (cutAt).
func transversePair(ends []vertexEnd) bool {
	if len(ends) != 2 || ends[0].edge.Entity == ends[1].edge.Entity {
		return false
	}
	return cutAt(ends[0].edge, ends[0].seg, ends[0].t) && cutAt(ends[1].edge, ends[1].seg, ends[1].t)
}

func readVertex(ends []vertexEnd, tags map[sketch.Entity]Origin, src []prismrecord.CarrierRecord) (*Vertex, bool) {
	v := &Vertex{}
	var segs [2]sectionrecord.CurveSegment
	var eps, hs [2]float64
	for i, end := range ends {
		if !cutAt(end.edge, end.seg, end.t) {
			return nil, false
		}
		d, err := CutDelta(end.edge, end.seg)
		if err != nil {
			return nil, false
		}
		v.CutDelta = math.Max(v.CutDelta, d)
		o := tags[end.edge.Entity]
		seg, e, h, ok := SourceSegment(src, o)
		if !ok {
			return nil, false
		}
		v.Origins[i], v.ends[i], segs[i], eps[i], hs[i] = o, end, seg, e, h
	}
	pt, pointErr, err := junctionPoint(ends[0], ends[1])
	if err != nil {
		return nil, false
	}
	v.Point, v.Err = pt, pointErr
	if hs[0] == 0 && hs[1] == 0 {
		v.Delta = v.CutDelta
		return v, true
	}
	l, ok := lemmaTDistance(segs, eps, hs, pt, pointErr)
	if !ok {
		return nil, false
	}
	v.L = l
	v.Delta = proofbound.AbsSumUpper(v.CutDelta, pointErr, l)
	return v, true
}

// TangentAllowance is §6.1's θ for a denoted carrier: 4ε/ℓ for a line of
// recorded length ℓ and zero for a circle read over a ball widened by ε.
func TangentAllowance(seg sectionrecord.CurveSegment, eps float64) float64 {
	line, ok := seg.(sectionrecord.LineSeg)
	if !ok || eps == 0 {
		return 0
	}
	// ℓ is bounded below by an exact square root of the recorded differences,
	// not a float hypot less an ulp.
	l := 0.0
	ru, rv := recordedDiff(line.End.U, line.Start.U), recordedDiff(line.End.V, line.Start.V)
	if ru != nil && rv != nil {
		l = proofbound.RatSqrtDown(proofbound.RatAdd(new(big.Rat).Mul(ru, ru), new(big.Rat).Mul(rv, rv)))
	}
	if !(l > 0) {
		return math.Inf(1)
	}
	return proofbound.DivUpper(proofbound.ProductUpper(4, eps), l)
}

// recordedDiff is a − b exactly; nil where a coordinate is not finite.
func recordedDiff(a, b float64) *big.Rat {
	ra, rb := proofarith.FloatRat(a), proofarith.FloatRat(b)
	if ra == nil || rb == nil {
		return nil
	}
	return ra.Sub(ra, rb)
}

// lemmaTDistance is Lemma T's L for two denoted carriers about p with
// q_i = err + h_i: start with ρ = 4(q_1 + q_2), read s over B(p, ρ + max ε),
// set s* = s − θ_1 − θ_2, accept when s* > ContactEps and 2L ≤ ρ, else set
// ρ = 4L and retry, at most four times.
func lemmaTDistance(segs [2]sectionrecord.CurveSegment, eps, hs [2]float64, p [2]*big.Rat, err float64) (float64, bool) {
	q1, q2 := proofbound.AbsSumUpper(err, hs[0]), proofbound.AbsSumUpper(err, hs[1])
	theta := proofbound.AbsSumUpper(TangentAllowance(segs[0], eps[0]), TangentAllowance(segs[1], eps[1]))
	rho := math.Max(proofbound.ProductUpper(4, proofbound.AbsSumUpper(q1, q2)), 0x1p-60)
	for range 4 {
		s, serr := sinLower(segs[0], segs[1], p, proofbound.AbsSumUpper(rho, math.Max(eps[0], eps[1])))
		if serr != nil {
			return 0, false
		}
		sStar := math.Nextafter(s-theta, math.Inf(-1))
		if !(sStar > sectionaudit.ContactEps) {
			return 0, false
		}
		l := proofbound.AbsSumUpper(proofbound.DivUpper(proofbound.AbsSumUpper(q1, q2), sStar), math.Min(q1, q2))
		if 2*l <= rho {
			return l, true
		}
		if proofbound.IsNonFinite(l) {
			return 0, false
		}
		rho = proofbound.ProductUpper(4, l)
	}
	return 0, false
}
