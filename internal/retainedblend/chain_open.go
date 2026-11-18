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
// (agent-docs/retained-agent-docs/modify-design.md Part R §5.1 C2, §5.5 step 5). An end is
// the corner where the chain's first or last straight edge meets the base P,
// its wall and an end face G. G is a retained plane normal to the edge, or one
// that meets the wall square and so tilts only about its line on the base (a
// ramp), and the edge's blend ends on G in blend §5.0's end curve, the quarter
// circle (or the setback segment) projected along the edge into G's plane. The
// base must be convex at the corner, and the wall convex or running on past G;
// G gives the blend's end cap up where it is convex there and takes it where it
// is reflex or the wall runs on past it, for a convex chain and a concave one
// alike.

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
	// wraps is set when the base is reflex at the corner of a concave chain
	// (agent-docs/retained-agent-docs/modify-design.md Part R §5.5 step 5): the base keeps
	// the corner and its boundary gains the run from it to the setback point on
	// G ∩ P, away from G's chain along G ∩ P. G is convex there and takes the
	// cap.
	wraps bool
	// extendsW is set when the wall continues past G at the corner: the wall's
	// chain along G ∩ wall leaves the corner away from the blend's setback point
	// on G ∩ wall (a concave chain's floor running on behind a block, a convex
	// chain's side wall running on up a riser or a ramp). G is convex there and
	// takes the cap, keeps
	// the corner and the whole chain of G ∩ wall, and its loop gains the run
	// along the wall's cut in G's plane from the wall's setback corner back to
	// the corner (the wall's band, chain_band.go).
	extendsW bool
	// wallChain is G's chain along G ∩ wall that extendsW keeps whole; wallRun
	// and corner are the band's run along G's plane and the vertex where the
	// setback plane meets it, set once the wall is cut.
	wallChain []int
	wallRun   []int
	corner    int
	// rP and rW are the unit directions from the corner along G ∩ P and
	// G ∩ wall toward the end face's chains.
	rP, rW r3.Vec

	// oblique is set when the straight end's face G is not normal to the edge:
	// it meets the wall square and tilts about its line on the base, and cosine
	// is |d·n_G|, the cosine of the edge's angle to G's normal (1 for a normal
	// G). kappa is the slide along the edge, into the chain, per unit of height
	// on the chain's side at which a row meets G, negated: a row at height h runs
	// h·kappa longer than the edge (rewriteChain).
	oblique bool
	cosine  float64
	kappa   float64

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
	ec := &chainEndCut{faceIx: faceIndex[third], g: g, exact: exact, wallN: wp.n, v: held, cosine: 1}
	// An end face normal to the edge takes the closed forms of blend §5.0 with
	// no end-plane skew. Any other end face must meet the wall square, so that it
	// tilts only about its line on the base, as a ramp or a drafted end wall
	// does: the base's setback point stays on that line at tP from the corner,
	// and the rows slide along the edge onto G by an amount that grows with
	// their height (§5.5 step 5, "An oblique end face").
	d, ok := bp.n.Cross(wp.n).Normalize()
	if !ok {
		return nil, fmt.Errorf("%w: an open tangent chain's end edge has no direction", decaderr.ErrDegenerate)
	}
	if d.Cross(g.n).Len() <= 1e-9 {
		return ec, nil
	}
	if math.Abs(g.n.Dot(wp.n)) > 1e-9 {
		return nil, fmt.Errorf("%w: an open tangent chain's end face does not meet its wall square",
			decaderr.ErrUnsupported)
	}
	if ec.cosine = math.Abs(d.Dot(g.n)); ec.cosine < minSine {
		return nil, fmt.Errorf("%w: an open tangent chain meets its end face at a grazing angle",
			decaderr.ErrUnsupported)
	}
	ec.oblique = true
	return ec, nil
}

// readEndCut reads the end face's loop around the corner: the chains leaving
// the corner along G ∩ P and G ∩ wall, which of them the setback removes, G's
// turn at the corner, and the gates of blend §5.0 step 2 and blend SR5. uP is
// the base loop's neighbour of the corner along G ∩ P. The held mesh is
// oriented, so G's loop runs the shared edge from the corner to uP the other
// way round: that, and not a vertex's distance from either line, tells which
// way G's loop leaves along G ∩ P. rc is the unit direction from the corner
// along the chain's end edge and leaves is set for the chain's first end, as
// for checkEndTurns; concave is set for a concave chain, whose end may be one
// where the base wraps the corner. pW is the exact point where the blend's
// last row meets G: the wall's setback point, on the line G ∩ wall. Where G's
// chain along that line leaves the corner away from it, the wall continues past
// G (extendsW).
func (w *work) readEndCut(ec *chainEndCut, nP, rc, pW r3.Vec, uP int, leaves, concave bool,
	tP, tW, margin, slack float64) error {
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
	if concave && ec.arc == nil && okP {
		// The base's own turn at the corner decides whether it wraps it. Its
		// neighbour uP lies on G ∩ P, and the base loop leaves the corner
		// toward it.
		if tu := tau(w.verts[uP], ec.exact, dirP); math.Abs(tu) > w.beta[uP]+margin {
			toward := dirP
			if tu < 0 {
				toward = dirP.Scale(-1)
			}
			baseIn, baseOut := toward, rc
			if !leaves {
				baseIn, baseOut = rc, toward
			}
			ec.wraps = cornerTurn(baseIn, baseOut, nP) < 0
		}
	}
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
		if k == 1 && (concave || ec.arc == nil) {
			// The wall's setback point lies on G ∩ wall: along the base's normal
			// on the chain's side where G is normal to the edge (into the void
			// for a concave chain, into the material for a convex one), slid
			// along the edge onto an oblique G. G's chain along G ∩ wall leaves
			// the corner toward it (the wall trims) or away from it (the wall
			// continues past G). An arc end's wall is a generator along the
			// base's normal, and only a concave arc end's wall continues.
			toward := nP
			if ec.arc == nil {
				var ok bool
				if toward, ok = pW.Sub(ec.exact).Normalize(); !ok {
					return fmt.Errorf("%w: a blend's setback point lies on its edge's corner", decaderr.ErrUnsupported)
				}
			}
			along := dirs[1].Dot(toward)
			if math.Abs(along) <= minSine {
				return fmt.Errorf("%w: a face leaves an edge end along the edge's own line", decaderr.ErrUnsupported)
			}
			if ec.oblique {
				setback[1] = tau(pW, ec.exact, dirs[1])
			}
			if along < 0 {
				if err := clearance(tau(w.verts[last], ec.exact, dirs[1]), w.beta[last]+margin,
					"the end face"); err != nil {
					return err
				}
				ec.extendsW, ec.wallChain = true, chain
				continue
			}
		}
		if k == 0 && ec.arc != nil {
			// The plane's setback point is where the setback circle meets
			// G ∩ P, not a fixed distance from the corner.
			if setback[0] = tau(ec.arc.pt0, ec.exact, dirs[0]); setback[0] <= margin {
				return fmt.Errorf("%w: a setback point lies behind an arc end", decaderr.ErrUnsupported)
			}
		}
		if k == 0 && ec.wraps {
			// The base extends along G ∩ P away from G's chain: nothing of
			// that chain is removed, and the setback point lies on no vertex
			// of G.
			continue
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
	// cap up, a reflex G takes it (§5.5 step 5). A concave chain's arc end
	// takes either; a convex chain's arc end is built on a convex G only.
	if err := ec.readEndFaceKind(); err != nil {
		return err
	}
	// Nothing else of the end face comes near the corner. The cap lies within the
	// triangle of the corner and the two setback points, so within the farther of
	// them; an oblique G puts the wall's farther than tW.
	far := tW
	if ec.oblique {
		far = pW.Sub(ec.exact).Len()
	}
	reach := math.Max(tP, far)*math.Sqrt2 + margin
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

// readEndFaceKind reads G's turn at the corner (§5.5 step 5): a convex G gives
// the cap up, a reflex G takes it. A convex chain's arc end is built on a convex
// G only; a concave chain's arc end takes either.
// Where the base wraps the corner G must be convex: it then takes the cap, and
// a reflex G beside a wrapping base has no end section to take.
func (ec *chainEndCut) readEndFaceKind() error {
	switch ec.endFaceTurn() {
	case 0:
		return fmt.Errorf("%w: an end face's turn at the chain's corner is undecided", decaderr.ErrUnsupported)
	case -1:
		if ec.arc != nil && ec.arc.a.convex {
			return fmt.Errorf("%w: an arc end of a tangent chain needs an end face convex at the corner",
				decaderr.ErrUnsupported)
		}
		if ec.wraps {
			return fmt.Errorf("%w: a chain end's base and end face are both reflex at the corner",
				decaderr.ErrUnsupported)
		}
		if ec.extendsW {
			return fmt.Errorf("%w: a chain end's wall extends past an end face that is reflex at the corner",
				decaderr.ErrUnsupported)
		}
		ec.reflex = true
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
// G ∩ wall through the end's corner: where the cut meets the end face. That line
// is the base's normal where G is normal to the edge.
func (w *work) findEndCut(cuts map[int]bool, ec *chainEndCut, nP r3.Vec, slack float64) (int, error) {
	line := nP
	if ec.oblique {
		var ok bool
		if line, ok = ec.wallN.Cross(ec.g.n).Normalize(); !ok {
			return 0, fmt.Errorf("%w: an end face is parallel to a face of the chain", decaderr.ErrUnsupported)
		}
	}
	found := -1
	for c := range cuts {
		if distToLine(w.verts[c], ec.exact, line) <= w.beta[c]+slack {
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
	case !(math.Abs(turn) > minSine):
		return 0
	case turn < 0:
		return -1
	}
	return 1
}

// checkEndSense refuses an end whose face loop runs its edge with the base
// the same way as the base's loop. The turns of §5.5 step 5 read each loop's
// sense at the corner from the base's alone, which holds only on an oriented
// mesh: there G's loop enters the corner from the base's side at the chain's
// last end, where the base loop leaves along G ∩ P, and leaves toward it at
// the first. last is set for the chain's last end.
func checkEndSense(ec *chainEndCut, last bool) error {
	if ec.prevP != last {
		return fmt.Errorf("%w: an end face's loop runs its edge with the base the same way as the base's loop",
			decaderr.ErrUnsupported)
	}
	return nil
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
// blend's section has no end face to lie in. The exceptions are a concave
// chain's base, which readEndCut marks wraps, and a wall of either chain that
// runs on past G, which it marks extendsW: the section then lies in G's plane
// outside G's held face, which G takes. rc is the unit direction from the
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
	// A wall that continues past G is reflex at the corner; readEndCut read
	// that from G's chain, and the loop's own turn must agree.
	if (base < 0 && !ec.wraps) || (wall < 0 && !ec.extendsW) {
		return fmt.Errorf("%w: a chain end's base or wall is reflex at the corner", decaderr.ErrUnsupported)
	}
	if ec.extendsW && ec.wraps {
		return fmt.Errorf("%w: a chain end's base and wall both extend past the end face", decaderr.ErrUnsupported)
	}
	if ec.extendsW && wall > 0 {
		return fmt.Errorf("%w: a chain end's wall turn at the corner is undecided", decaderr.ErrUnsupported)
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
	var err error
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
			takes = takes || ec.takesCap()
			if ec.wraps {
				if loops[li], err = ec.wrapLoop(loops[li], seqs[j]); err != nil {
					return 0, nil, err
				}
				total += ec.capArea(fillet, r, tP, tW)
				continue
			}
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
			if ec.extendsW {
				// The wall continues past G: G keeps the corner, which follows
				// the wall's cut along G's plane back from the setback corner.
				repl = append(append(repl, ec.wallRun...), ec.v)
			}
			if !ec.prevP {
				slices.Reverse(repl)
			}
			loops[li] = append(outside, repl...)
			total += ec.capArea(fillet, r, tP, tW)
		}
		from := len(w.out)
		if err := w.retriangulate(first.g.n, loops, first.fl.group); err != nil {
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
// up, negative when G is reflex and takes it. A straight end's cap is the
// section's corner projected along the edge onto G, so an oblique G's is the
// section's over the cosine |d·n_G| (blend §5.0, "Exact areas").
func (ec *chainEndCut) capArea(fillet bool, r, tP, tW float64) float64 {
	area := tP * tW / 2
	switch {
	case ec.arc != nil:
		area = ec.arc.cornerArea
	case fillet:
		area = r * r * (1 - math.Pi/4)
	}
	if ec.arc == nil && ec.oblique {
		area /= ec.cosine
	}
	if ec.takesCap() {
		return -area
	}
	return area
}

// takesCap reports whether G takes the end cap: where G is reflex at the
// corner, and where the base wraps the corner or the wall continues past it,
// so that the section lies outside G's held face.
func (ec *chainEndCut) takesCap() bool { return ec.reflex || ec.wraps || ec.extendsW }

// wrapLoop is G's loop with the end column of a wrapping end put in: the
// chain's wall removes the run of G ∩ wall below its setback, G keeps the
// corner, and the column from the setback point on G ∩ P to the wall's cut
// follows it. The loop reaches the corner from the P side when prevP is set,
// and the column is then inserted after the corner; otherwise it is reversed
// and inserted before it.
func (ec *chainEndCut) wrapLoop(loop, column []int) ([]int, error) {
	gone := map[int]bool{}
	for _, v := range ec.remW {
		gone[v] = true
	}
	kept := make([]int, 0, len(loop)+len(column))
	for _, v := range loop {
		if !gone[v] {
			kept = append(kept, v)
		}
	}
	at := indexIn(kept, ec.v)
	if at < 0 || len(kept)+len(gone) != len(loop) {
		return nil, fmt.Errorf("%w: an end face does not hold the chain's corner", decaderr.ErrUnsupported)
	}
	repl := append([]int(nil), column...)
	if !ec.prevP {
		slices.Reverse(repl)
		at--
	}
	out := append([]int(nil), kept[:at+1]...)
	out = append(out, repl...)
	return append(out, kept[at+1:]...), nil
}
