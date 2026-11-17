package brepmodify

import (
	"math"
	"math/big"
	"strings"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/retainedblend"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// recordTangentEnds proves a C3 arc end from the record the view's faces came
// from (agent-docs/boss-root-overflow-design.md §5.5.2). The view's faces are
// matched to record faces by their wall(k) origin role under the body's own
// producer, never by position. Every check here only refuses; a corner is
// admitted by the exact tests capband.JoinIsG1 and capband.NearTangentCorner
// alone.
type recordTangentEnds struct {
	body *topo.Body
	bp   breprecord.Payload
}

var _ retainedblend.TangentEnds = recordTangentEnds{}

// stackedTangentEnds is the TangentEnds of b's record bp.
func stackedTangentEnds(b *topo.Body, bp breprecord.Payload) retainedblend.TangentEnds {
	return recordTangentEnds{body: b, bp: bp}
}

// Prove implements retainedblend.TangentEnds.
func (t recordTangentEnds) Prove(bFace, gFace, kFace *topo.Face) (retainedblend.TangentProof, bool) {
	ib, okB := t.recordWall(bFace)
	ig, okG := t.recordWall(gFace)
	ik, okK := t.recordWall(kFace)
	if !okB || !okG || !okK || ib == ig || ib == ik || ig == ik {
		return retainedblend.TangentProof{}, false
	}
	fb, fg, fk := t.bp.Faces[ib], t.bp.Faces[ig], t.bp.Faces[ik]
	if fb.Frame != fg.Frame || fb.Frame != fk.Frame {
		return retainedblend.TangentProof{}, false
	}
	// The levels: G ends where B starts, and K runs across that level.
	if !(fg.Z1 == fb.Z0) || !(fk.Z0 < fg.Z1) || !(fg.Z1 < fk.Z1) {
		return retainedblend.TangentProof{}, false
	}
	wb, okB := recordArc(fb)
	wg, okG := recordArc(fg)
	wk, okK := recordArc(fk)
	if !okB || !okG || !okK {
		return retainedblend.TangentProof{}, false
	}
	vr, kFirst, ok := sharedEnd(wk, wg)
	if !ok {
		return retainedblend.TangentProof{}, false
	}
	// K and B meet at the same recorded point, exactly G1.
	prevB, curB := wk, wb
	if !kFirst {
		prevB, curB = wb, wk
	}
	if !capband.JoinIsG1(prevB, curB) {
		return retainedblend.TangentProof{}, false
	}
	prev, cur := wk, wg
	if !kFirst {
		prev, cur = wg, wk
	}
	var turn float64
	if !capband.JoinIsG1(prev, cur) {
		if _, why, _ := capband.NearTangentCorner(prev, cur, 0, 0, 0); why != "" {
			return retainedblend.TangentProof{}, false
		}
		sinSq, ok := capband.JoinTurnSinSq(prev, cur)
		if !ok {
			return retainedblend.TangentProof{}, false
		}
		if turn = turnUp(sinSq); math.IsNaN(turn) {
			return retainedblend.TangentProof{}, false
		}
	}
	at := fk.View(t.bp.Xform).Point(vr.U, vr.V, fg.Z1)
	return retainedblend.TangentProof{At: at, Turn: turn}, true
}

// recordWall returns the index of the one swept record face of the body that
// f carries a wall(k) origin of, under the record's producer.
func (t recordTangentEnds) recordWall(f *topo.Face) (int, bool) {
	if f == nil {
		return 0, false
	}
	found := -1
	for _, origin := range f.Origins {
		if origin.Producer != t.body.Origin.Producer || !strings.HasPrefix(origin.Role, "wall(") {
			continue
		}
		for i, rf := range t.bp.Faces {
			if rf.Planar() || rf.Role != origin.Role {
				continue
			}
			if found >= 0 && found != i {
				return 0, false
			}
			found = i
		}
	}
	return found, found >= 0
}

// recordArc is the normalized ArcSeg wall of a swept record face.
func recordArc(f breprecord.Face) (sectionrecord.ArcSeg, bool) {
	if f.Planar() || f.Wall == nil {
		return sectionrecord.ArcSeg{}, false
	}
	seg, err := sectionrecord.NormalizeSegment(f.Wall)
	if err != nil {
		return sectionrecord.ArcSeg{}, false
	}
	arc, ok := seg.(sectionrecord.ArcSeg)
	return arc, ok
}

// walkEnds are an arc wall's walk start and end.
func walkEnds(a sectionrecord.ArcSeg) (sectionrecord.Point2, sectionrecord.Point2) {
	if a.TStart == 1 {
		return a.End, a.Start
	}
	return a.Start, a.End
}

// sharedEnd finds the one recorded point where walls k and g meet, exactly
// equal floats, with one walk ending there and the other starting there. kFirst
// says k is the one that ends there. A pair that shares no endpoint, or more
// than one, has no such point.
func sharedEnd(k, g sectionrecord.ArcSeg) (sectionrecord.Point2, bool, bool) {
	ks, ke := walkEnds(k)
	gs, ge := walkEnds(g)
	shared := 0
	for _, p := range []sectionrecord.Point2{ks, ke} {
		for _, q := range []sectionrecord.Point2{gs, ge} {
			if p == q {
				shared++
			}
		}
	}
	switch {
	case shared != 1:
		return sectionrecord.Point2{}, false, false
	case ke == gs:
		return ke, true, true
	case ge == ks:
		return ge, false, true
	}
	return sectionrecord.Point2{}, false, false
}

// turnUp is an upper bound on asin(√sinSq): the square root rounded up, asin of
// it, and a few ulps up for asin's own rounding.
func turnUp(sinSq *big.Rat) float64 {
	s := proofbound.RatSqrtUp(sinSq)
	if !(s < 1) {
		return math.NaN()
	}
	turn := math.Asin(s)
	for range 4 {
		turn = math.Nextafter(turn, math.Inf(1))
	}
	return turn
}
