package retainedblend

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// This file holds the ends of an open tangent chain
// (agent-docs/retained-modify-reach-design.md §5.1 C2, §5.5 step 5). An end is
// the corner where the chain's first or last straight edge meets the base P,
// its wall and an end face G. G is a retained plane normal to the edge, so the
// edge's blend ends on G in blend §5.0's end curve, the quarter circle (or the
// setback segment) in G's plane.

// chainEndCut is one end of an open chain: the corner, the end face G and, once
// read, the vertices of G's loop that the blend removes or keeps.
type chainEndCut struct {
	faceIx int
	g      plane
	exact  r3.Vec // the meeting of P, the wall and G
	wallN  r3.Vec
	v      int // the held corner vertex

	fl      faceLoops
	li      int
	remP    []int // G's chain along G ∩ P that the setback removes
	remW    []int // G's chain along G ∩ wall that the setback removes
	prevP   bool  // G's loop reaches the corner from the P side
	beyondP int   // the first vertex of G ∩ P beyond the setback point
}

// readEndCorner reads the corner at which the chain edge e ends at vertex v.
func (w *work) readEndCorner(v *topo.Vertex, e *topo.Edge, base *topo.Face, bp plane,
	fp facetedrecord.Payload, faceIndex map[*topo.Face]int, slack float64) (*chainEndCut, error) {
	var wall *topo.Face
	for _, f := range e.Faces {
		if f != base {
			wall = f
		}
	}
	wp, ok := retainedPlane(wall)
	if !ok {
		return nil, fmt.Errorf("%w: an open tangent chain ends on a wall that is not a retained plane; "+
			"only a straight edge end is built", decaderr.ErrUnsupported)
	}
	held, third, err := cornerOf(v, fp, faceIndex, faceIndex[base], faceIndex[wall])
	if err != nil {
		return nil, err
	}
	g, ok := retainedPlane(third)
	if !ok {
		return nil, fmt.Errorf("%w: an open tangent chain ends on a face that is not a retained plane",
			decaderr.ErrUnsupported)
	}
	exact, ok := intersect3(bp, wp, g)
	if !ok {
		return nil, fmt.Errorf("%w: an open tangent chain meets its end face at a grazing angle",
			decaderr.ErrUnsupported)
	}
	if exact.Sub(fp.Verts[held]).Len() > fp.VertexBound[held]+slack {
		return nil, fmt.Errorf("%w: an open tangent chain's end is not where its three tags meet",
			decaderr.ErrUnsupported)
	}
	// The end face is normal to the edge: the closed forms of blend §5.0 with
	// no end-plane skew apply.
	d := bp.n.Cross(wp.n)
	if d.Cross(g.n).Len() > 1e-9*d.Len() {
		return nil, fmt.Errorf("%w: an open tangent chain's end face is not normal to its edge",
			decaderr.ErrUnsupported)
	}
	return &chainEndCut{faceIx: faceIndex[third], g: g, exact: exact, wallN: wp.n, v: held}, nil
}

// readEndCut reads the end face's loop around the corner: the chains leaving
// the corner along G ∩ P and G ∩ wall, which of them the setback removes, and
// the gates of blend §5.0 step 2 and blend SR5.
func (w *work) readEndCut(ec *chainEndCut, nP r3.Vec, tP, tW, margin, slack float64) error {
	var err error
	if ec.fl, err = w.faceLoops(ec.faceIx); err != nil {
		return err
	}
	li, ok := ec.fl.loopWith(ec.v)
	if !ok {
		return fmt.Errorf("%w: an end face does not hold the chain's corner", decaderr.ErrUnsupported)
	}
	ec.li = li
	loop := ec.fl.loops[li]
	n := len(loop)
	at := indexIn(loop, ec.v)
	dirP, okP := nP.Cross(ec.g.n).Normalize()
	dirW, okW := ec.wallN.Cross(ec.g.n).Normalize()
	if !okP || !okW {
		return fmt.Errorf("%w: an end face is parallel to a face of the chain", decaderr.ErrUnsupported)
	}
	dirs := [2]r3.Vec{dirP, dirW}
	onLine := func(x int, dir r3.Vec) bool {
		return distToLine(w.verts[x], ec.exact, dir) <= w.beta[x]+slack
	}
	var chains [2][]int
	for _, step := range []int{1, -1} {
		first := loop[((at+step)%n+n)%n]
		which := -1
		for k := range dirs {
			if onLine(first, dirs[k]) {
				which = k
			}
		}
		if which < 0 || chains[which] != nil {
			return fmt.Errorf("%w: an end face leaves the chain's corner along neither of its lines",
				decaderr.ErrUnsupported)
		}
		chains[which] = walk(loop, at, step, func(x int) bool { return onLine(x, dirs[which]) })
		if step == -1 {
			ec.prevP = which == 0
		}
	}
	// A convex chain ends on a face that is convex at the corner.
	prev, next := loop[(at+n-1)%n], loop[(at+1)%n]
	in, out := w.verts[ec.v].Sub(w.verts[prev]), w.verts[next].Sub(w.verts[ec.v])
	turn := in.Cross(out).Dot(ec.g.n)
	if math.Abs(turn) <= margin*in.Len() || turn < 0 {
		return fmt.Errorf("%w: an end face's corner does not match the chain's convexity", decaderr.ErrUnsupported)
	}
	setback := [2]float64{tP, tW}
	ec.beyondP = -1
	rem := [2][]int{}
	for k, chain := range chains {
		last := chain[len(chain)-1]
		if tau(w.verts[last], ec.exact, dirs[k]) < 0 {
			dirs[k] = dirs[k].Scale(-1)
		}
		if err := clearance(tau(w.verts[last], ec.exact, dirs[k])-setback[k], w.beta[last]+margin,
			"the end face"); err != nil {
			return err
		}
		for _, c := range chain {
			tc := tau(w.verts[c], ec.exact, dirs[k])
			if math.Abs(tc-setback[k]) <= 4*margin {
				return fmt.Errorf("%w: a setback point falls on a vertex of the end face's boundary",
					decaderr.ErrUnsupported)
			}
			if tc < setback[k] {
				rem[k] = append(rem[k], c)
			} else if k == 0 && ec.beyondP < 0 {
				ec.beyondP = c
			}
		}
	}
	ec.remP, ec.remW = rem[0], rem[1]
	// Nothing else of the end face comes near the corner.
	reach := math.Max(tP, tW)*math.Sqrt2 + margin
	skip := map[int]bool{ec.v: true}
	for _, chain := range chains {
		for _, c := range chain {
			skip[c] = true
		}
	}
	for _, lp := range ec.fl.loops {
		for k, v := range lp {
			q := lp[(k+1)%len(lp)]
			if skip[v] && skip[q] {
				continue
			}
			if distToSegment(ec.exact, w.verts[v], w.verts[q]) < reach+w.beta[v]+w.beta[q] {
				return fmt.Errorf("%w: another part of an end face comes within the blend's reach of the corner",
					decaderr.ErrUnsupported)
			}
		}
	}
	return nil
}

// findEndCut picks, among the vertices a wall's cut made, the one on the line
// through the corner normal to the base: where the cut meets the end face.
func (w *work) findEndCut(cuts map[int]bool, corner, nP r3.Vec, slack float64) (int, error) {
	found := -1
	for c := range cuts {
		if distToLine(w.verts[c], corner, nP) <= w.beta[c]+slack {
			if found >= 0 {
				return 0, fmt.Errorf("%w: a wall's cut meets the end face twice", decaderr.ErrUnsupported)
			}
			found = c
		}
	}
	if found < 0 {
		return 0, fmt.Errorf("%w: a wall's cut does not reach the end face", decaderr.ErrUnsupported)
	}
	return found, nil
}

// rebuildEnd gives the end face its new loop: the corner and the removed parts
// of the two chains give way to seq, the end column from the setback point on
// G ∩ P to the cut on G ∩ wall. It returns the exact area the face loses.
func (w *work) rebuildEnd(ec *chainEndCut, seq []int, fillet bool, r, tP, tW float64) (float64, error) {
	loop := ec.fl.loops[ec.li]
	run := map[int]bool{ec.v: true}
	for _, v := range ec.remP {
		run[v] = true
	}
	for _, v := range ec.remW {
		run[v] = true
	}
	outside, _, err := splice(loop, run)
	if err != nil {
		return 0, err
	}
	repl := append([]int(nil), seq...)
	if !ec.prevP {
		for a, b := 0, len(repl)-1; a < b; a, b = a+1, b-1 {
			repl[a], repl[b] = repl[b], repl[a]
		}
	}
	loops := make([][]int, 0, len(ec.fl.loops))
	for k, lp := range ec.fl.loops {
		if k == ec.li {
			loops = append(loops, append(outside, repl...))
			continue
		}
		loops = append(loops, lp)
	}
	if err := w.retriangulate(ec.g.n, ec.exact, loops, ec.fl.group); err != nil {
		return 0, err
	}
	w.removed = append(w.removed, ec.fl.facets...)
	if fillet {
		return r * r * (1 - math.Pi/4), nil
	}
	return tP * tW / 2, nil
}
