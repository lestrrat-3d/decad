package retainedblend

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/r3"
)

// The removed band of a chain wall whose face is not one-sided
// (agent-docs/retained-agent-docs/modify-design.md Part R §5.5 step 5, blend §5.0 "A wall
// that extends past the corner" and §6.2). Where the wall is a notched round,
// or the floor beside a block that stands on it, the wall's facets lie behind
// the base plane P and past the end plane G of a chain end, and a cut by the
// height above P alone would remove them. The band is instead the prism over
// the chain edge between P and the setback plane P′ (at height tW on the
// chain's side), inside the end plane of each end where the wall continues
// past it. A junction needs no plane: the wall face ends at the seam it shares
// with its neighbour.

// chainWallNeedsBand reports whether the entry's wall is cut by the band: the
// wall is a two-sided rim's, continues past an end plane, or holds a vertex
// behind P that the whole-face height cut would refuse. Every other wall keeps
// the height cut and so its exact vertices and triangles.
func (w *work) chainWallNeedsBand(en chainEntry, facets []int, up, oP r3.Vec, topSet map[int]bool,
	ends [2]*chainEndCut, slack float64) bool {
	if en.arc && en.rim.twoSided {
		return true
	}
	for _, ec := range ends {
		if ec != nil && ec.extendsW {
			return true
		}
	}
	for _, f := range facets {
		for _, v := range w.fp.Tris[f] {
			if !topSet[v] && up.Dot(w.verts[v].Sub(oP)) < -(w.beta[v]+slack) {
				return true
			}
		}
	}
	return false
}

// newChainBand is the band of the entry's wall. ends holds, for the entry's
// start and end, the chain end whose wall extends there, else nil. own is the
// entry's own boundary on the base: its junction corners and the base loop's
// vertices between them.
func (w *work) newChainBand(en chainEntry, facets []int, up, oP r3.Vec, tW float64, own map[int]bool,
	ends [2]*chainEndCut, slack float64) (*wallBand, error) {
	band := &wallBand{slack: slack, noteCuts: true}
	band.main = &bandEnd{g: plane{n: up, o: oP.Add(up.Scale(tW))}, sign: -1, what: "the blend's setback plane"}
	band.lean, band.leanOK = w.wallTilt(en.wallIx, up)
	onP := map[int]bool{}
	for _, f := range facets {
		for _, v := range w.fp.Tris[f] {
			if own[v] && up.Dot(w.verts[v].Sub(oP)) <= w.beta[v]+slack {
				onP[v] = true
			}
		}
	}
	band.others = []*bandEnd{{g: plane{n: up, o: oP}, sign: 1, on: onP, what: "the base's plane"}}
	for i, ec := range ends {
		if ec == nil || !ec.extendsW {
			continue
		}
		// The band lies on the side of G the entry's other end is on.
		far := en.p1
		if i == 1 {
			far = en.p0
		}
		d := ec.g.offset(far)
		if !(d > w.tau+slack || d < -(w.tau+slack)) {
			return nil, fmt.Errorf("%w: a chain edge's two ends lie on one end plane", decaderr.ErrUnsupported)
		}
		sign := 1.0
		if d < 0 {
			sign = -1
		}
		on := map[int]bool{ec.v: true}
		for _, v := range ec.wallChain {
			on[v] = true
		}
		for v := range w.onTagPlane(w.faceAt(ec.faceIx)) {
			on[v] = true
		}
		sine, planar := w.wallSine(en.wallIx, ec.g)
		end := &bandEnd{g: ec.g, sign: sign, on: on, sine: sine, planarWall: planar}
		// A straight end's plane is normal to the chain edge or tilts about its line
		// on the base, and meets the wall's plane square either way
		// (readEndCorner); an arc end's wall meets it at a slant.
		if ec.arc != nil {
			end.slanted, end.slant = true, w.wallSlant(en.wallIx, ec.g)
		}
		band.ends[i] = end
	}
	return band, nil
}

// clipChainWallBand is clipChainWall for a wall cut by the band. It emits the
// part of the wall's facets outside the band and returns the vertices the
// setback plane cut, with the band's own record of the corners and the runs
// along an extending end plane. Nothing of the wall but its boundary along the
// chain edge, its seams and its chain along an end plane may lie within the
// blend's reach of the band: the wall's boundary chords must miss the band
// inflated by the held bounds, or the blend may consume a feature of the wall
// it cannot see (SR4, SR5).
func (w *work) clipChainWallBand(en chainEntry, up, oP r3.Vec, tW float64, own, near map[int]bool,
	faceVerts map[int]map[int]bool, ends [2]*chainEndCut, slack float64) (map[int]bool, int, bandCuts, error) {
	var none bandCuts
	f, err := w.faceLoops(en.wallIx, wallPlane(en))
	if err != nil {
		return nil, 0, none, err
	}
	band, err := w.newChainBand(en, f.facets, up, oP, tW, own, ends, slack)
	if err != nil {
		return nil, 0, none, err
	}
	cuts, err := w.cutBand(f, band)
	if err != nil {
		return nil, 0, none, err
	}
	w.removed = append(w.removed, f.facets...)
	margin := w.tau + slack

	// The seam lines of the entry's two junctions, normal to the base.
	onSeam := func(v int) bool {
		p := w.verts[v]
		b := w.beta[v] + slack
		return distToLine(p, en.p0, up) <= b || distToLine(p, en.p1, up) <= b
	}
	exempt := map[int]bool{}
	for v := range own {
		exempt[v] = true
	}
	for _, ec := range ends {
		if ec == nil || !ec.extendsW {
			continue
		}
		exempt[ec.v] = true
		for _, v := range ec.wallChain {
			exempt[v] = true
		}
	}
	// Map order must not choose the refusal: a vertex of another part of the
	// wall refuses before a seam vertex that borders a face off the chain.
	var seamBad bool
	for v, gone := range cuts.removed {
		if !gone || exempt[v] {
			continue
		}
		// A seam vertex inside the band borders only the neighbouring wall.
		if !onSeam(v) {
			return nil, 0, none, fmt.Errorf("%w: another part of the wall lies within the blend's reach of the edge",
				decaderr.ErrUnsupported)
		}
		for fi := range faceVerts[v] {
			seamBad = seamBad || !near[fi]
		}
	}
	if seamBad {
		return nil, 0, none, fmt.Errorf("%w: a seam vertex inside the band borders another face",
			decaderr.ErrUnsupported)
	}
	seamChord := func(v, q int) bool {
		for _, at := range []r3.Vec{en.p0, en.p1} {
			b := max(w.beta[v], w.beta[q]) + slack
			if distToLine(w.verts[v], at, up) <= b && distToLine(w.verts[q], at, up) <= b {
				return true
			}
		}
		return false
	}
	for _, loop := range f.loops {
		for k, v := range loop {
			q := loop[(k+1)%len(loop)]
			if (exempt[v] && exempt[q]) || seamChord(v, q) {
				continue
			}
			if band.chordMeets(w.verts[v], w.verts[q], max(w.beta[v], w.beta[q])+margin) {
				return nil, 0, none, fmt.Errorf("%w: another part of the wall lies within the blend's reach of the edge",
					decaderr.ErrUnsupported)
			}
		}
	}

	// The vertices cut on the setback plane: those on the wall's own held
	// edges, and the band's corners at an extending end.
	mine := map[int]bool{}
	for _, fi := range f.facets {
		t := w.fp.Tris[fi]
		for k := range 3 {
			a, b := t[k], t[(k+1)%3]
			if c, ok := w.cuts[[2]int{min(a, b), max(a, b)}]; ok {
				mine[c] = true
			}
		}
	}
	for i, ec := range ends {
		if ec != nil && ec.extendsW {
			if cuts.corner[i] < 0 {
				return nil, 0, none, fmt.Errorf("%w: the blend's setback plane does not meet an end face on the wall",
					decaderr.ErrUnsupported)
			}
			mine[cuts.corner[i]] = true
		}
	}
	if len(mine) < 2 {
		return nil, 0, none, fmt.Errorf("%w: a wall's cut leaves fewer than two vertices", decaderr.ErrUnsupported)
	}
	return mine, f.group, cuts, nil
}

// wallPlane is the retained plane of a planar chain wall, which its face's
// facets may hold vertices on without their lying on its boundary, or nil.
func wallPlane(en chainEntry) *plane {
	if en.arc {
		return nil
	}
	p, ok := chainPlane(en.wall)
	if !ok {
		return nil
	}
	return &p
}
