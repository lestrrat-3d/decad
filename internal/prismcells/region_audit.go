package prismcells

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/sketch"
)

// This file is the arrangement stability audit (V2) of
// agent-docs/prism-boolean-design.md Part K §6.3. Every pair of arranged edges from
// different records, at least one of them displaced, keeps the arrangement's
// topology under each edge's displacement η.

// trimSlack is the 2⁻⁴⁰ factor by which a trim length exceeds its target.
const trimSlack = 0x1p-40

// Arrangement is the arranged edges of a scene's cells read against the
// carrier records of its operands.
type Arrangement struct {
	edges []arrangedEdge
	src   []prismrecord.CarrierRecord
	vr    VertexReading
}

// NewArrangement reads the distinct arranged edges of the cells with their
// displacement allowances. ok=false (err nil) is a miss.
func NewArrangement(budget *proofbound.WorkBudget, tags map[sketch.Entity]Origin, cells []*sketch.Profile,
	src []prismrecord.CarrierRecord, vr VertexReading) (*Arrangement, bool, error) {
	edges, ok, err := collectEdges(budget, tags, cells, src, vr)
	if err != nil || !ok {
		return nil, false, err
	}
	return &Arrangement{edges: edges, src: src, vr: vr}, true, nil
}

// Stable runs §6.3's audit over the arranged edges. floor is F, the scene's
// contact floor (sectionaudit.ContactFloor over every operand segment).
// ok=false (err nil) is a miss.
func (arr *Arrangement) Stable(budget *proofbound.WorkBudget, floor float64) (bool, error) {
	if !guardOn("V2") {
		return true, nil
	}
	edges, src, vr := arr.edges, arr.src, arr.vr
	worst := 0.0
	for _, e := range edges {
		worst = math.Max(worst, math.Max(e.eta, e.c))
	}
	for _, r := range src {
		worst = math.Max(worst, r.HMax())
	}
	if !(proofbound.ProductUpper(4, worst) < floor) {
		return false, nil
	}
	displaced := make([]bool, len(src))
	for i, r := range src {
		displaced[i] = r.Max() > 0
	}
	boxes := make([][4]float64, len(edges))
	boxed := make([]bool, len(edges))
	for i, e := range edges {
		boxes[i], boxed[i] = widenedBox(e)
	}
	for i := range edges {
		for j := i + 1; j < len(edges); j++ {
			if err := budget.Step(); err != nil {
				return false, err
			}
			a, b := &edges[i], &edges[j]
			if a.origin.Region == b.origin.Region || (!displaced[a.origin.Region] && !displaced[b.origin.Region]) {
				continue
			}
			shared := sharedEndpoints(a, b)
			if len(shared) == 0 {
				if boxed[i] && boxed[j] && boxGapLower(boxes[i], boxes[j]) > floor {
					continue
				}
				if !(gapAbove(a.walk, b.walk, a.c, b.c, floor)) {
					return false, nil
				}
				continue
			}
			ok, err := auditShared(budget, a, b, shared, vr, floor)
			if err != nil || !ok {
				return false, err
			}
		}
	}
	return true, nil
}

// gapAbove reports G > F for two walks with allowances ca and cb:
// SegGapLower − ca − cb.
func gapAbove(a, b walkT, ca, cb, floor float64) bool {
	g := sectionaudit.SegGapLower(a, b)
	return math.Nextafter(g-proofbound.AbsSumUpper(ca, cb), math.Inf(-1)) > floor
}

// widenedBox is the box of an edge's walk as SegGapLower reads it
// (sectionaudit.WalkGapBox: the same endpoint enclosures), widened by the
// walk's allowance c_e, so a gap between two such boxes is a lower bound on
// G(e, f) = SegGapLower(w_e, w_f) − c_e − c_f.
func widenedBox(e arrangedEdge) ([4]float64, bool) {
	u0, v0, u1, v1, ok := sectionaudit.WalkGapBox(e.walk)
	if !ok {
		return [4]float64{}, false
	}
	w := e.c
	return [4]float64{
		math.Nextafter(u0-w, math.Inf(-1)), math.Nextafter(v0-w, math.Inf(-1)),
		math.Nextafter(u1+w, math.Inf(1)), math.Nextafter(v1+w, math.Inf(1)),
	}, true
}

// boxGapLower is a lower bound on the distance between two boxes.
func boxGapLower(a, b [4]float64) float64 {
	gx := math.Max(0, math.Max(b[0]-a[2], a[0]-b[2]))
	gy := math.Max(0, math.Max(b[1]-a[3], a[1]-b[3]))
	return math.Nextafter(math.Max(gx, gy), 0)
}

func sharedEndpoints(a, b *arrangedEdge) [][2]float64 {
	var out [][2]float64
	for _, p := range [][2]float64{a.start, a.end} {
		if p != b.start && p != b.end {
			continue
		}
		if len(out) == 0 || out[0] != p {
			out = append(out, p)
		}
	}
	return out
}

func ratDist2(x, y float64, p [2]*big.Rat) *big.Rat {
	rx, ry := proofarith.FloatRat(x), proofarith.FloatRat(y)
	if rx == nil || ry == nil {
		return nil
	}
	dx, dy := new(big.Rat).Sub(rx, p[0]), new(big.Rat).Sub(ry, p[1])
	return dx.Mul(dx, dx).Add(dx, new(big.Rat).Mul(dy, dy))
}

func distUpper(x, y float64, p [2]*big.Rat) float64 {
	d2 := ratDist2(x, y, p)
	if d2 == nil {
		return math.Inf(1)
	}
	return proofbound.RatSqrtUp(d2)
}

func distLower(x, y float64, p [2]*big.Rat) float64 {
	d2 := ratDist2(x, y, p)
	if d2 == nil {
		return 0
	}
	return proofbound.RatSqrtDown(d2)
}

// ownReadEnds lists the points a source segment denotes at its two ends, with
// the radial residual an ArcSeg's recorded End owes.
func ownReadEnds(seg sectionrecord.CurveSegment) (pts [][2]float64, residual float64) {
	switch s := seg.(type) {
	case sectionrecord.LineSeg:
		return [][2]float64{{s.Start.U, s.Start.V}, {s.End.U, s.End.V}}, 0
	case sectionrecord.ArcSeg:
		return [][2]float64{{s.Start.U, s.Start.V}, {s.End.U, s.End.V}}, arcResidual(s)
	}
	return nil, 0
}

type ball struct {
	centre [2]*big.Rat
	r      float64
}

// auditShared is ASA2 for two edges that share cut vertices.
func auditShared(budget *proofbound.WorkBudget, a, b *arrangedEdge, shared [][2]float64, vr VertexReading,
	floor float64) (bool, error) {
	bothLines := a.walk.IsLine() && b.walk.IsLine()
	trimA, trimB := a.seg, b.seg
	emptyA, emptyB := false, false
	var balls []ball
	for _, at := range shared {
		vx, ok := vr.Vertices[at]
		if !ok {
			return false, nil
		}
		s0, err := sinLower(a.src, b.src, vx.Point, proofbound.AbsSumUpper(2*floor, vx.Err))
		if err != nil || !(s0 > sectionaudit.ContactEps) {
			return false, nil //nolint:nilerr // no certified crossing angle: a miss
		}
		rho1 := proofbound.DivUpper(proofbound.ProductUpper(8, floor), s0)
		target := proofbound.ProductUpper(rho1, 1+trimSlack)
		var spread float64
		for _, side := range []struct {
			e    *arrangedEdge
			trim *sectionrecord.CurveSegment
			dead *bool
		}{{a, &trimA, &emptyA}, {b, &trimB, &emptyB}} {
			e := side.e
			atStart := at == e.start
			w := e.walk
			wu, wv := w.EndU, w.EndV
			if atStart {
				wu, wv = w.StartU, w.StartV
			}
			spread = math.Max(spread, proofbound.AbsSumUpper(distUpper(wu, wv, vx.Point), e.c))
			if bothLines {
				continue
			}
			ts, kappa, empty, ok := trimSegment(*side.trim, atStart, target)
			if !ok {
				return false, nil
			}
			if empty {
				*side.dead = true
				continue
			}
			*side.trim = ts
			spread = math.Max(spread, proofbound.AbsSumUpper(distUpper(wu, wv, vx.Point), e.c, kappa))
		}
		rv := proofbound.AbsSumUpper(target, spread)
		rhoT := proofbound.ProductUpper(2, proofbound.AbsSumUpper(rv, floor))
		s1, err := sinLower(a.src, b.src, vx.Point, proofbound.AbsSumUpper(rhoT, floor))
		if err != nil || !(s1 > sectionaudit.ContactEps) {
			return false, nil //nolint:nilerr // no certified crossing angle: a miss
		}
		theta := proofbound.AbsSumUpper(TangentAllowance(a.src, a.srcEps), TangentAllowance(b.src, b.srcEps))
		if proofbound.DivUpper(proofbound.ProductUpper(5, floor), s1) > rho1 || !(math.Nextafter(s1-theta, math.Inf(-1)) >= s1/2) {
			return false, nil
		}
		far := proofbound.AbsSumUpper(rv, floor)
		for _, srcSeg := range []sectionrecord.CurveSegment{a.src, b.src} {
			pts, residual := ownReadEnds(srcSeg)
			for _, p := range pts {
				if !(math.Nextafter(distLower(p[0], p[1], vx.Point)-residual, math.Inf(-1)) > far) {
					return false, nil
				}
			}
		}
		for _, other := range balls {
			d := ratPointDistLower(vx.Point, other.centre)
			if !(math.Nextafter(d-proofbound.AbsSumUpper(rv, other.r), math.Inf(-1)) > floor) {
				return false, nil
			}
		}
		balls = append(balls, ball{centre: vx.Point, r: rv})
	}
	if bothLines {
		return len(shared) == 1, nil
	}
	if err := budget.Step(); err != nil {
		return false, err
	}
	if !emptyA {
		wa, err := boundarywalk.WalkOf(trimA, nil)
		if err != nil || !gapAbove(wa, b.walk, a.trimmedAllowance(trimA, wa), b.c, floor) {
			return false, nil //nolint:nilerr // an unreadable trimmed walk is a miss
		}
	}
	if !emptyB {
		wb, err := boundarywalk.WalkOf(trimB, nil)
		if err != nil || !gapAbove(a.walk, wb, a.c, b.trimmedAllowance(trimB, wb), floor) {
			return false, nil //nolint:nilerr // an unreadable trimmed walk is a miss
		}
	}
	return true, nil
}

func ratPointDistLower(p, q [2]*big.Rat) float64 {
	dx, dy := new(big.Rat).Sub(p[0], q[0]), new(big.Rat).Sub(p[1], q[1])
	return proofbound.RatSqrtDown(dx.Mul(dx, dx).Add(dx, new(big.Rat).Mul(dy, dy)))
}

// WalkAllowance is c for a recorded edge and its walk: Lemma H's h read with
// the walk's radius bound as ε and each end's WalkEndBoundAllow as δ (§6.2).
func WalkAllowance(seg sectionrecord.CurveSegment, w survey2d.SegmentWalk) float64 {
	return walkAllowance(seg, w)
}

// GapAbove reports G(a, b) > floor, where G is the certified gap
// sectionaudit.SegGapLower minus both walks' allowances (§6.2, ASA1).
func GapAbove(a, b survey2d.SegmentWalk, ca, cb, floor float64) bool {
	return gapAbove(a, b, ca, cb, floor)
}
