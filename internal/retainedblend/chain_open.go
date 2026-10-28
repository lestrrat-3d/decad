package retainedblend

import (
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// This file holds the ends of an open tangent chain
// (agent-docs/retained-modify-reach-design.md §5.1 C2, §5.5 step 5). An end is
// the corner where the chain's first or last straight edge meets the base P,
// its wall and an end face G. G is a retained plane normal to the edge, so the
// edge's blend ends on G in blend §5.0's end curve, the quarter circle (or the
// setback segment) in G's plane. The base and the wall must be convex at the
// corner; G gives the blend's end cap up where it is convex there and takes it
// where it is reflex, for a convex chain and a concave one alike.

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
	// reflex is set when G is reflex at the corner: the blend's end cap then
	// lies outside G, and G takes it in place of giving it up.
	reflex bool
	// rP and rW are the unit directions from the corner along G ∩ P and
	// G ∩ wall toward the end face's chains.
	rP, rW r3.Vec

	// arc is set when the chain's end edge is an arc: the end curve is then a
	// chain of exact points, not a quarter circle (chain_arc_end.go).
	arc *arcEnd
	// seamEdge is the held edge of G ∩ wall that the wall's setback cut
	// crosses; its cut vertex is the end column's last vertex.
	seamEdge [2]int
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
		if _, isArc := e.Curve.(surfacegeom.Arc3); isArc {
			return w.readArcEndCorner(v, e, base, wall, fp, faceIndex)
		}
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
// the corner along G ∩ P and G ∩ wall, which of them the setback removes, G's
// turn at the corner, and the gates of blend §5.0 step 2 and blend SR5. uP is
// the base loop's neighbour of the corner along G ∩ P. The held mesh is
// oriented, so G's loop runs the shared edge from the corner to uP the other
// way round: that, and not a vertex's distance from either line, tells which
// way G's loop leaves along G ∩ P.
func (w *work) readEndCut(ec *chainEndCut, nP r3.Vec, uP int, tP, tW, margin, slack float64) error {
	var err error
	if ec.fl, err = w.faceLoops(ec.faceIx, &ec.g); err != nil {
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
	if ec.arc != nil {
		// The wall is a cylinder about the base's normal: it meets G in a
		// generator.
		dirW, okW = nP, true
	}
	if !okP || !okW {
		return fmt.Errorf("%w: an end face is parallel to a face of the chain", decaderr.ErrUnsupported)
	}
	dirs := [2]r3.Vec{dirP, dirW}
	onLine := func(x int, dir r3.Vec) bool {
		return distToLine(w.verts[x], ec.exact, dir) <= w.beta[x]+slack
	}
	stepP := 0
	switch uP {
	case loop[(at+1)%n]:
		stepP = 1
	case loop[(at+n-1)%n]:
		stepP = -1
	default:
		return fmt.Errorf("%w: an end face does not share the base's edge at the chain's corner",
			decaderr.ErrUnsupported)
	}
	ec.prevP = stepP == -1
	var chains [2][]int
	for k, step := range []int{stepP, -stepP} {
		if chains[k] = walk(loop, at, step, func(x int) bool { return onLine(x, dirs[k]) }); len(chains[k]) == 0 {
			return fmt.Errorf("%w: an end face leaves the chain's corner along neither of its lines",
				decaderr.ErrUnsupported)
		}
	}
	setback := [2]float64{tP, tW}
	ec.beyondP = -1
	rem := [2][]int{}
	for k, chain := range chains {
		last := chain[len(chain)-1]
		if tau(w.verts[last], ec.exact, dirs[k]) < 0 {
			dirs[k] = dirs[k].Scale(-1)
		}
		if k == 0 && ec.arc != nil {
			// The plane's setback point is where the setback circle meets
			// G ∩ P, not a fixed distance from the corner.
			if setback[0] = tau(ec.arc.pt0, ec.exact, dirs[0]); setback[0] <= margin {
				return fmt.Errorf("%w: a setback point lies behind an arc end", decaderr.ErrUnsupported)
			}
		}
		if err := clearance(tau(w.verts[last], ec.exact, dirs[k])-setback[k], w.beta[last]+margin,
			"the end face"); err != nil {
			return err
		}
		prevC, keptC := ec.v, -1
		for _, c := range chain {
			tc := tau(w.verts[c], ec.exact, dirs[k])
			if tc < setback[k] && keptC < 0 {
				prevC = c
			} else if keptC < 0 {
				keptC = c
			}
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
		if k == 1 && keptC >= 0 {
			ec.seamEdge = [2]int{min(prevC, keptC), max(prevC, keptC)}
		}
	}
	ec.remP, ec.remW = rem[0], rem[1]
	// Each line now points from the corner to its chain's last vertex, which
	// the clearance above places beyond the setback by more than its bound:
	// the lines' senses, and the turns read from them, are certain.
	ec.rP, ec.rW = dirs[0], dirs[1]
	// G's turn at the corner picks the side of the cap: a convex G gives the
	// cap up, a reflex G takes it (§5.5 step 5). An arc end is built on a
	// convex G only.
	switch ec.endFaceTurn() {
	case 0:
		return fmt.Errorf("%w: an end face's turn at the chain's corner is undecided", decaderr.ErrUnsupported)
	case -1:
		if ec.arc != nil {
			return fmt.Errorf("%w: an arc end of a tangent chain needs an end face convex at the corner",
				decaderr.ErrUnsupported)
		}
		ec.reflex = true
	}
	// Nothing else of the end face comes near the corner.
	reach := math.Max(tP, tW)*math.Sqrt2 + margin
	if ec.arc != nil {
		reach = ec.arc.reach + margin
	}
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

// seamCut is the cut vertex the wall's setback made on the held edge of the
// generator at an arc end. Near the generator, bounds can place several cuts
// on the line, so it is read from the edge the end face's own chain crosses.
func (w *work) seamCut(ec *chainEndCut, cuts map[int]bool) (int, error) {
	id, ok := w.cuts[ec.seamEdge]
	if !ok || !cuts[id] {
		return 0, fmt.Errorf("%w: a wall's cut does not reach the end face", decaderr.ErrUnsupported)
	}
	return id, nil
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

// cornerTurn is the turn of a face's loop at a corner it enters along the
// line whose direction away from the corner is in and leaves along the line
// whose direction is out, about the face's outward normal n: +1 where the face
// is convex, −1 where it is reflex, 0 where the two lines are too near
// parallel to tell. The lines of a chain end are pairwise orthogonal, so the
// turn reads ±1 up to rounding.
func cornerTurn(in, out, n r3.Vec) int {
	turn := in.Scale(-1).Cross(out).Dot(n)
	switch {
	case math.Abs(turn) <= minSine:
		return 0
	case turn < 0:
		return -1
	}
	return 1
}

// endFaceTurn is G's cornerTurn at the corner: its loop enters along G ∩ P
// and leaves along G ∩ wall when it reaches the corner from the P side, and
// the other way round otherwise.
func (ec *chainEndCut) endFaceTurn() int {
	if ec.prevP {
		return cornerTurn(ec.rP, ec.rW, ec.g.n)
	}
	return cornerTurn(ec.rW, ec.rP, ec.g.n)
}

// checkEndTurns refuses a straight chain end whose base or wall is not convex
// at the corner (SM8): the base or the wall then wraps the corner, and the
// blend's section has no end face to lie in. rc is the unit direction from the
// corner along the chain's end edge, and leaves is set when the base loop
// leaves the corner along that edge (the chain's first end). The held mesh is
// oriented, so the base loop runs the edge to G one way and G's loop the other,
// and the wall's loop runs the chain edge the other way from the base's: the
// three loops' senses at the corner follow from the base's alone. An arc end is
// read by blend §4.4 instead.
func checkEndTurns(ec *chainEndCut, rc, nP r3.Vec, leaves bool) error {
	if ec.arc != nil {
		return nil
	}
	baseIn, baseOut, wallIn, wallOut := rc, ec.rP, ec.rW, rc
	if leaves {
		baseIn, baseOut, wallIn, wallOut = ec.rP, rc, rc, ec.rW
	}
	base, wall := cornerTurn(baseIn, baseOut, nP), cornerTurn(wallIn, wallOut, ec.wallN)
	if base == 0 || wall == 0 {
		return fmt.Errorf("%w: a chain end's base or wall turn at the corner is undecided", decaderr.ErrUnsupported)
	}
	if base < 0 || wall < 0 {
		return fmt.Errorf("%w: a chain end's base or wall is reflex at the corner", decaderr.ErrUnsupported)
	}
	return nil
}

// rebuildEnds gives each end face its new loop: at each end the corner and the
// removed parts of the two chains give way to seqs[i], the end column from the
// setback point on G ∩ P to the cut on G ∩ wall. Two ends on one face are
// spliced into one copy of its loops, which is triangulated once. Each end's
// run holds only vertices nearer its corner than the setbacks, and readEndCut
// places the last vertex of each of its chains beyond them; where two runs
// would still share a vertex, the second splice finds its run broken and
// refuses. It returns the signed exact area the end faces lose and, for each
// end face that takes a cap, the range of w.out its new facets fill.
func (w *work) rebuildEnds(ends []*chainEndCut, seqs [][]int, fillet bool, r, tP, tW float64) (float64, [][2]int,
	error) {
	total := 0.0
	var taken [][2]int
	done := make([]bool, len(ends))
	for i, first := range ends {
		if done[i] {
			continue
		}
		loops := make([][]int, len(first.fl.loops))
		for k, lp := range first.fl.loops {
			loops[k] = append([]int(nil), lp...)
		}
		takes := false
		for j := i; j < len(ends); j++ {
			ec := ends[j]
			if ec.faceIx != first.faceIx {
				continue
			}
			done[j] = true
			li := slices.IndexFunc(loops, func(lp []int) bool { return slices.Contains(lp, ec.v) })
			if li < 0 {
				return 0, nil, fmt.Errorf("%w: an end face does not hold the chain's corner", decaderr.ErrUnsupported)
			}
			takes = takes || ec.reflex
			run := map[int]bool{ec.v: true}
			for _, v := range ec.remP {
				run[v] = true
			}
			for _, v := range ec.remW {
				run[v] = true
			}
			outside, _, err := splice(loops[li], run)
			if err != nil {
				return 0, nil, err
			}
			repl := append([]int(nil), seqs[j]...)
			if !ec.prevP {
				slices.Reverse(repl)
			}
			loops[li] = append(outside, repl...)
			total += ec.capArea(fillet, r, tP, tW)
		}
		from := len(w.out)
		if err := w.retriangulate(first.g.n, first.exact, loops, first.fl.group); err != nil {
			return 0, nil, err
		}
		if takes {
			taken = append(taken, [2]int{from, len(w.out)})
		}
		w.removed = append(w.removed, first.fl.facets...)
	}
	return total, taken, nil
}

// capArea is the signed exact area the end face loses at this end: the
// blend's section corner, positive when G is convex at the corner and gives it
// up, negative when G is reflex and takes it.
func (ec *chainEndCut) capArea(fillet bool, r, tP, tW float64) float64 {
	area := tP * tW / 2
	switch {
	case ec.arc != nil:
		area = ec.arc.cornerArea
	case fillet:
		area = r * r * (1 - math.Pi/4)
	}
	if ec.reflex {
		return -area
	}
	return area
}
