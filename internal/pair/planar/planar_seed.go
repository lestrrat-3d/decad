package planar

import "math"

// seedVertices and seedFacets bound the guess seedCap makes: this many
// vertices of each solid, nearest to a vertex of the other, each against this
// many facets nearest to it.
const (
	seedVertices = 16
	seedFacets   = 4
)

// seedCap lowers the cap to the least exact distance of a guessed set of
// candidate pairs. A guess is a candidate whatever its distance, so its exact
// squared distance is at or above the exact minimum and prunes as a hinted
// pair does (planar_prune.go): the answer is the same for every guess. The
// guess walks float distances to a nearby vertex pair (nearVertices), then
// reads the vertices of each solid nearest the other's vertex against the
// facets nearest to it, and keeps the least exact distance among those whose
// projection falls inside the facet. A cap the caller's hint already set stays
// when it is the lesser.
func (k *planarKernel) seedCap() {
	scratch := &planarKernel{a: k.a, b: k.b, poll: k.poll}
	scratch.seedSide(k.a, k.b)
	scratch.seedSide(k.b, k.a)
	if scratch.hasBest && (!k.hasCap || fracCmp(scratch.best, k.cap) < 0) {
		k.cap, k.capUp, k.hasCap = scratch.best, scratch.bestUp, true
	}
}

// seedSide offers the guessed vertex-facet candidates of verts against tris:
// the vertices of verts nearest to the vertex of tris that the nearest-vertex
// walk (nearVertices) settles on, each against the facets nearest to it.
func (k *planarKernel) seedSide(verts, tris *planarPrep) {
	if len(verts.s.Verts) == 0 || len(tris.s.Tris) == 0 || len(tris.s.Verts) == 0 {
		return
	}
	near, ok := nearVertices(verts, tris)
	if !ok {
		return
	}
	var facets []seedPick
	for _, vertex := range near {
		facets = facets[:0]
		at := verts.vertBox[vertex.index].lo
		for t := range tris.triBox {
			box := tris.triBox[t]
			// Facets whose boxes all meet the vertex's tie on the gap, so the
			// distance to the box center ranks the facet the vertex lies over.
			centre := 0.0
			for axis := range 3 {
				d := at[axis] - (box.lo[axis]+box.hi[axis])/2
				centre += d * d
			}
			facets = seedKeep(facets, seedFacets, seedPick{t, gapSquaredBelow(verts.vertBox[vertex.index], box), centre})
		}
		for _, facet := range facets {
			k.vertexFacet(verts, vertex.index, tris, facet.index)
		}
	}
}

// seedSamples bounds the vertices of each solid the nearest-vertex walk starts
// from, and seedWalk the alternations after it.
const (
	seedSamples = 128
	seedWalk    = 4
)

// nearVertices finds the seedVertices vertices of verts nearest, by float
// distance, to a vertex w of tris. It starts from the nearest pair among
// seedSamples vertices of each solid, then alternates: w becomes the vertex of
// tris nearest the current vertex of verts, and that vertex the one of verts
// nearest w. The walk only chooses where to look; every candidate it leads to
// is read exactly. ok is false when a coordinate is not finite.
func nearVertices(verts, tris *planarPrep) ([]seedPick, bool) {
	point := func(p *planarPrep, i int) [3]float64 { return p.vertBox[i].lo }
	dist := func(a, b [3]float64) float64 {
		s := 0.0
		for axis := range 3 {
			d := a[axis] - b[axis]
			s += d * d
		}
		return s
	}
	best, bv, bw := math.Inf(1), -1, -1
	stepA := max(1, len(verts.s.Verts)/seedSamples)
	stepB := max(1, len(tris.s.Verts)/seedSamples)
	for v := 0; v < len(verts.s.Verts); v += stepA {
		for w := 0; w < len(tris.s.Verts); w += stepB {
			if d := dist(point(verts, v), point(tris, w)); d < best {
				best, bv, bw = d, v, w
			}
		}
	}
	if bv < 0 {
		return nil, false
	}
	for range seedWalk {
		w, wd := bw, math.Inf(1)
		for i := range tris.s.Verts {
			if d := dist(point(verts, bv), point(tris, i)); d < wd {
				w, wd = i, d
			}
		}
		v, vd := bv, math.Inf(1)
		for i := range verts.s.Verts {
			if d := dist(point(verts, i), point(tris, w)); d < vd {
				v, vd = i, d
			}
		}
		if v == bv && w == bw {
			break
		}
		bv, bw = v, w
	}
	picks := make([]seedPick, 0, seedVertices)
	for i := range verts.s.Verts {
		picks = seedKeep(picks, seedVertices, seedPick{index: i, gap: dist(point(verts, i), point(tris, bw))})
	}
	return picks, true
}

// seedPick is an index with its float gap, the key seedKeep ranks by.
type seedPick struct {
	index int
	gap   float64
	// tie orders picks whose gaps are equal.
	tie float64
}

// seedKeep inserts p into the ascending list picks of at most limit entries,
// dropping the greatest when the list is full. Equal gaps keep index order.
func seedKeep(picks []seedPick, limit int, p seedPick) []seedPick {
	at := len(picks)
	for at > 0 && (picks[at-1].gap > p.gap || picks[at-1].gap == p.gap && picks[at-1].tie > p.tie) {
		at--
	}
	if at >= limit {
		return picks
	}
	if len(picks) < limit {
		picks = append(picks, seedPick{})
	}
	copy(picks[at+1:], picks[at:])
	picks[at] = p
	return picks
}
