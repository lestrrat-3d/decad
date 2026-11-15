package retainedblend

import (
	"fmt"
	"math"
	"sort"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// The removed band of an arc's wall when the wall is not one-sided or extends
// past an end plane (agent-docs/retained-edge-blend-design.md §5.0, "A wall that
// extends past the corner", R4). The band is the part of the wall that lies
// between the rim's plane P and the setback plane P′ (at height tb on the
// wall's side), inside the end plane of each end where the wall extends, and on
// the arc's side of the guard, the plane through the axis normal to the arc's
// middle direction. The guard keeps the band to the half of the cylinder the arc
// lies on: the end planes alone also cut the far side of the axis.

// guardPlane is the index of the guard in wallBand.others; the rim's plane is
// the first.
const (
	arcBasePlane  = bandOther
	arcGuardPlane = bandOther + 1
)

// newWallBand is the band of the wall face f cut at height tb. An end whose
// wall extends past its end plane (kind) bounds the band by that plane; the wall
// of any other end stops there and needs none.
func (a arcRim) newWallBand(w *work, f faceLoops, tb float64, run edgeRun, kind [2]sideKind) *wallBand {
	band := &wallBand{slack: a.slack(), noteCuts: true}
	band.main = &bandEnd{g: plane{n: a.n, o: a.center.Add(a.n.Scale(a.side * tb))}, sign: -a.side,
		what: "the blend's setback plane"}
	onRim := map[int]bool{}
	for _, facet := range f.facets {
		for _, v := range w.fp.Tris[facet] {
			if a.onRim(w.verts[v], w.beta[v]) {
				onRim[v] = true
			}
		}
	}
	mid := a.theta0 + a.span/2
	dir := a.atTheta(1, mid, 0).Sub(a.center)
	band.others = []*bandEnd{
		{g: plane{n: a.n, o: a.center}, sign: a.side, on: onRim, what: "the rim's plane"},
		{g: plane{n: dir, o: a.center}, sign: 1, what: "the guard plane through the arc's axis"},
	}
	for i := range 2 {
		if kind[i] != extends {
			continue
		}
		g := a.ends[i].g
		sign := 1.0
		if g.offset(a.ends[1-i].exact) < 0 {
			sign = -1
		}
		on := map[int]bool{a.ends[i].v: true}
		for _, v := range run.chain[i] {
			on[v] = true
		}
		for v := range w.onTagPlane(a.ends[i].face) {
			on[v] = true
		}
		sine, planar := w.wallSine(f.index, g)
		band.ends[i] = &bandEnd{g: g, sign: sign, on: on, sine: sine, planarWall: planar}
	}
	return band
}

// clipArcWallBand is clipArcWall for a wall cut by the band: it emits the part
// of the wall's facets outside the band and returns the cut vertices in order of
// angle, the removed part of each trimming end's generator chain, the cut vertex
// on the chain edge that crosses the setback plane (an extending end has the
// band's corner in the end plane instead) and the cuts of the band.
func (w *work) clipArcWallBand(f faceLoops, a arcRim, tb float64, run edgeRun, kind [2]sideKind,
	margin float64) ([]int, [2][]int, [2]int, bandCuts, error) {
	var rem [2][]int
	var pb [2]int
	fail := func(err error) ([]int, [2][]int, [2]int, bandCuts, error) { return nil, rem, pb, bandCuts{}, err }
	if !(a.span < math.Pi-minSine) {
		return fail(fmt.Errorf("%w: an arc this wide reaches across half the cylinder", decaderr.ErrUnsupported))
	}
	band := a.newWallBand(w, f, tb, run, kind)
	cuts, err := w.cutBand(f, band)
	if err != nil {
		return fail(err)
	}
	w.removed = append(w.removed, f.facets...)

	// The band's corners lie inside the guard, farther than their bounds.
	for i := range 2 {
		if kind[i] != extends {
			continue
		}
		if cuts.corner[i] < 0 {
			return fail(fmt.Errorf("%w: the blend's setback plane does not meet an end face on the wall",
				decaderr.ErrUnsupported))
		}
		if s, err := band.sideOf(w.heldPt(cuts.corner[i]), arcGuardPlane); err != nil || s < 0 {
			return fail(fmt.Errorf("%w: the held bounds cannot show that the band's corner lies on the arc's side of the axis",
				decaderr.ErrUnsupported))
		}
	}

	// Nothing of the wall but its own boundary along the edge lies in the band.
	exempt := map[int]bool{}
	for i := range 2 {
		exempt[a.ends[i].v] = true
		for _, v := range run.chain[i] {
			exempt[v] = true
		}
	}
	for _, v := range run.inner {
		exempt[v] = true
	}
	for v, gone := range cuts.removed {
		if gone && !exempt[v] {
			return fail(fmt.Errorf("%w: another part of the wall lies within the blend's reach of the edge",
				decaderr.ErrUnsupported))
		}
	}
	for _, loop := range f.loops {
		for k, v := range loop {
			q := loop[(k+1)%len(loop)]
			if exempt[v] && exempt[q] {
				continue
			}
			if band.chordMeets(w.verts[v], w.verts[q], max(w.beta[v], w.beta[q])+margin) {
				return fail(fmt.Errorf("%w: another part of the wall lies within the blend's reach of the edge",
					decaderr.ErrUnsupported))
			}
		}
	}

	// A trimming end's chain is cut where it crosses the setback plane; an
	// extending end's chain stays whole and the strip ends at the band's corner.
	inMain := func(v int) bool {
		s, _ := band.sideOf(w.heldPt(v), bandGen)
		return s > 0
	}
	for i := range 2 {
		if kind[i] == extends {
			pb[i] = cuts.corner[i]
			continue
		}
		seq := append([]int{a.ends[i].v}, run.chain[i]...)
		if !inMain(seq[0]) {
			return fail(fmt.Errorf("%w: an arc end is not on the removed side of the wall's setback plane",
				decaderr.ErrUnsupported))
		}
		crossed := false
		for k := 1; k < len(seq); k++ {
			if inMain(seq[k-1]) && !inMain(seq[k]) {
				id, ok := w.cuts[[2]int{min(seq[k-1], seq[k]), max(seq[k-1], seq[k])}]
				if !ok {
					return fail(fmt.Errorf("%w: the wall's generator edge at the setback height was not cut",
						decaderr.ErrUnsupported))
				}
				pb[i], crossed = id, true
				break
			}
			if !inMain(seq[k]) {
				break
			}
			rem[i] = append(rem[i], seq[k])
		}
		if !crossed {
			return fail(fmt.Errorf("%w: the wall's generator ends before the blend's setback height",
				decaderr.ErrDegenerate))
		}
	}

	ids := make([]int, 0, len(w.cuts)+2)
	for _, id := range w.cuts {
		ids = append(ids, id)
	}
	for i := range 2 {
		if kind[i] == extends {
			ids = append(ids, cuts.corner[i])
		}
	}
	mid := a.theta0 + a.span/2
	sort.Slice(ids, func(x, y int) bool {
		return a.theta(w.verts[ids[x]], mid) < a.theta(w.verts[ids[y]], mid)
	})
	if len(ids) < 2 || ids[0] != pb[0] || ids[len(ids)-1] != pb[1] {
		return fail(fmt.Errorf("%w: the wall's cut does not run from one end face to the other", decaderr.ErrUnsupported))
	}
	return ids, rem, pb, cuts, nil
}

// chordMeets reports whether the segment pq comes within infl of the band: a
// point of it lies inside every plane of the band widened by infl. The wall's
// own boundary must stay clear of the band by the held bounds, or the blend may
// consume a feature of the wall it cannot see (SR4, SR5).
func (b *wallBand) chordMeets(p, q r3.Vec, infl float64) bool {
	lo, hi := 0.0, 1.0
	for qi := bandGen; qi < bandOther+len(b.others); qi++ {
		if qi != bandGen && b.exact(qi) == nil {
			continue
		}
		v0, v1 := b.value(qi, p)+infl, b.value(qi, q)+infl
		switch {
		case v0 < 0 && v1 < 0:
			return false
		case v0 >= 0 && v1 >= 0:
		case v0 < 0:
			lo = math.Max(lo, v0/(v0-v1))
		default:
			hi = math.Min(hi, v0/(v0-v1))
		}
		if lo > hi {
			return false
		}
	}
	return true
}

// cuspParam is the profile parameter s at which a fillet's end curve on end
// plane i lies need from the wall's generator there, in the plane G: the end
// curve is (X(s), z(s)), X = sqrt(ρ² − h²), and the wall's cut along G runs at
// X_R = sqrt(R² − h²). The curve nears that line without touching it, tangent at
// the wall's tangent point, so the cap's loop (the rows, the wall's cut back to
// the corner and the plane's line) closes in a cusp there (cyl_rewrite.go,
// clearCusp). It returns the s at which the curve is need from the line, which
// the last row before the wall's row must not pass.
func (a arcRim) cuspParam(spec blendSpec, e endSection, need float64) (float64, error) {
	r := spec.r
	xr := math.Sqrt(a.radius*a.radius - e.h*e.h)
	x := xr + a.kappa*need
	if !(x > 0) {
		return 0, fmt.Errorf("%w: the fillet's arc is too near the wall's generator at an end face", decaderr.ErrUnsupported)
	}
	rho := math.Sqrt(x*x + e.h*e.h)
	sin := 1 - (rho-a.radius)/(a.kappa*r)
	alpha := math.Asin(math.Max(-1, math.Min(1, sin)))
	if !(sin < 1) || !(alpha > minSine) {
		return 0, fmt.Errorf("%w: the fillet's end curve lies within %g mm of the wall's generator at its plane's end, "+
			"and the wall's held facets may lie that far from it at the end cap", decaderr.ErrUnsupported, need)
	}
	return alpha * 2 / math.Pi, nil
}

// widenRows spaces the rows ss again so the last row before the wall's lies at
// parameter at most sStar: the rows below sStar keep their certified spacing,
// and the last gap, from sStar to 1, is no longer certified against the end
// curves. endChord bounds how far its chord on each end plane lies from the
// curve.
func widenRows(spec blendSpec, tau float64, ends []endSection, ss []float64, sStar float64) ([]float64, error) {
	grid := []float64{}
	for _, s := range ss {
		if s < sStar {
			grid = append(grid, s)
		}
	}
	if len(grid) == 0 || grid[len(grid)-1] != sStar {
		grid = append(grid, sStar)
	}
	out, err := sectionGrid(grid, tau, ends, arcApart(spec, tau, ends))
	if err != nil {
		return nil, err
	}
	return append(out, 1), nil
}

// endChord is the smallest tau·2^k within which the end curve of e between s0
// and s1 lies of its chord, by certify.
func endChord(e endSection, s0, s1, tau float64) (float64, error) {
	t := tau
	for range 40 {
		if e.certify(s0, s1, t) {
			return t, nil
		}
		t = proofbound.ProductUpper(2, t)
	}
	return 0, fmt.Errorf("%w: the blend's end curve cannot be bounded over its last row's gap", decaderr.ErrUnsupported)
}
