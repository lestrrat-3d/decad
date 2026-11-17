package retainedblend

import (
	"fmt"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// This file holds what rewriteArc does differently at a C3 arc end
// (agent-docs/boss-root-overflow-design.md §5.5.5): the wall's seam cut, G's
// loop at the corner and the cap's wall run along K.

// tangentAt is end i's C3 reading, nil for any other end.
func (a arcRim) tangentAt(i int) *tangentEnd {
	if c := a.cyl[i]; c != nil {
		return c.tan
	}
	return nil
}

// eTan is the end's error e that every test against G's circle adds to its
// allowance (§5.5.3); zero for an end that is not C3.
func (c *cylEnd) eTan() float64 {
	if c == nil || c.tan == nil {
		return 0
	}
	return c.tan.e
}

// reachTan is e·(1 + 2R/d') + e_d, the allowance of a held point compared
// with the tangent point T; zero for an end that is not C3.
func (c *cylEnd) reachTan() float64 {
	if c == nil || c.tan == nil {
		return 0
	}
	return c.tan.reach
}

// marginTan is margin widened by end i's e.
func (a arcRim) marginTan(i int, margin float64) float64 {
	return proofbound.AbsSumUpper(margin, a.cyl[i].eTan())
}

// seamCutOf is the held edge of the wall's boundary with K that the wall's
// setback cut crossed at end i: the edge whose cut vertex is pb.
func (w *work) seamCutOf(pb int) ([2]int, bool) {
	for key, id := range w.cuts {
		if id == pb {
			return key, true
		}
	}
	return [2]int{}, false
}

// noteSeamCut enters the wall's cut vertex pb in w.seam for the held edge it
// cuts, so the facets of K that keep the edge are split there (splitSeams).
func (w *work) noteSeamCut(pb int) error {
	key, ok := w.seamCutOf(pb)
	if !ok {
		return fmt.Errorf("%w: the wall's cut on its seam with K was not found", decaderr.ErrUnsupported)
	}
	if w.seam == nil {
		w.seam = map[[2]int][]int{}
	}
	if !slices.Contains(w.seam[key], pb) {
		w.seam[key] = append(w.seam[key], pb)
	}
	return nil
}

// tangentWallRun is the cap's wall run at C3 end i: the held vertices of the
// B–K seam strictly between V and the wall's cut vertex, listed from the cut
// down to V. rem is the seam's vertices the wall's cut removed, listed from V
// up (clipArcWall).
func tangentWallRun(rem []int) []int {
	run := slices.Clone(rem)
	slices.Reverse(run)
	return run
}

// seamBelow is the vertex of G's loop at V that G shares with K below A, the
// loop's neighbour of V other than A's chain, which takes the place of B's
// chain in G's loop (§5.5.5). It lies below A on the generator through V.
func (a arcRim) seamBelow(w *work, i int, g faceLoops, chainA []int, margin float64) (int, error) {
	end := a.ends[i]
	li, ok := g.loopWith(end.v)
	if !ok {
		return 0, fmt.Errorf("%w: an end face does not hold the edge's corner", decaderr.ErrUnsupported)
	}
	loop := g.loops[li]
	n := len(loop)
	at := indexIn(loop, end.v)
	prev, next := loop[(at+n-1)%n], loop[(at+1)%n]
	var s int
	switch chainA[0] {
	case next:
		s = prev
	case prev:
		s = next
	default:
		return 0, fmt.Errorf("%w: an end face's chains disagree with the faces beside the edge", decaderr.ErrUnsupported)
	}
	p := w.verts[s]
	below := -a.side * a.height(p)
	if !(below > margin+w.beta[s]) ||
		!(distToLine(p, end.exact, a.n) <= w.beta[s]+a.slack()+a.cyl[i].reachTan()) {
		return 0, fmt.Errorf("%w: a corner round's seam with the full wall does not run below the plane along the arc's generator",
			decaderr.ErrUnsupported)
	}
	return s, nil
}

// tangentKinds refuses a C3 arc whose wall is not trimmed at both ends, and
// reports whether the arc is C3.
func (a arcRim) tangentKinds(kindB [2]sideKind) (bool, error) {
	tan := a.tangentAt(0) != nil
	if !tan {
		return false, nil
	}
	if a.twoSided || kindB[0] != trims || kindB[1] != trims {
		return true, fmt.Errorf("%w: the wall's generator at a tangent arc end does not climb the wall",
			decaderr.ErrUnsupported)
	}
	return true, nil
}
