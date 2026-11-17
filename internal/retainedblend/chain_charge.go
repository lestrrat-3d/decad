package retainedblend

import (
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// This file reads the charged walls of a closed chamfer chain and charges them
// (agent-docs/charged-window-chamfer-design.md §3). A face that states a
// displacement is read only on the chain path, through tagChain. Every other
// route keeps refusing it by its own gate.

// tagMode is how a chain reader treats a face that states a displacement.
type tagMode uint8

const (
	// tagExact reads a face as an exact tag and declines any stated displacement or departure.
	tagExact tagMode = iota
	// tagRefuse also reads a cylinder that states a displacement, for refusal only
	// (refuseChargedJunctions).
	tagRefuse
	// tagChain reads every face of charged-window-chamfer-design.md §3.1, with its charge.
	tagChain
)

func tagModeOf(charged bool) tagMode {
	if charged {
		return tagRefuse
	}
	return tagExact
}

// readChainFace reads a retained plane or cylinder face in the given mode.
func readChainFace(f *topo.Face, mode tagMode) (chainWall, bool) {
	switch mode {
	case tagExact:
		return readChainTagMode(f, false)
	case tagRefuse:
		return readChainTagMode(f, true)
	}
	return readChargedChainFace(f)
}

// readChargedChainFace reads a face a closed chamfer chain admits
// (charged-window-chamfer-design.md §3.1): any face readChainTagMode reads, a cylinder that
// states a finite AxialDelta >= 0, a plane that states one, an oblique plane that states its
// ExtentDelta. The wall's delta is the charge each states. A face with a negative, infinite or
// NaN charge is not read.
func readChargedChainFace(f *topo.Face) (chainWall, bool) {
	if f == nil || !f.RetainedTag || f.ExtentDelta != 0 && f.TagDeparture == 0 {
		return chainWall{}, false
	}
	if w, ok := readChainTagMode(f, true); ok {
		if f.HasAxialDelta {
			w.delta = f.AxialDelta
		}
		return w, true
	}
	if _, plane := f.Surface.(surfacegeom.Plane); !plane {
		return chainWall{}, false
	}
	delta, ok := faceDelta(f)
	if !ok || !recordTag(f) {
		return chainWall{}, false
	}
	if f.TagDeparture == 0 {
		w, ok := chainGeometry(f)
		w.delta = delta
		return w, ok
	}
	// An oblique plane: a stated departure that equals its normal bound, no denoted patch, no
	// AxialDelta, and the extent bound the view states (stacked_view_tilt.go).
	ext := f.ExtentDelta
	if !(f.TagDeparture > 0) || math.IsInf(f.TagDeparture, 0) || f.NormalBound != f.TagDeparture || f.Denoted != nil ||
		delta != 0 || !(ext > 0) || math.IsInf(ext, 0) {
		return chainWall{}, false
	}
	w, ok := chainGeometry(f)
	w.delta, w.departure = ext, f.TagDeparture
	return w, ok
}

// chainPlane is retainedPlane for a wall of a chain: it also reads an oblique plane, whose delta
// is its ExtentDelta. A plane retainedPlane reads is read the same way.
func chainPlane(f *topo.Face) (plane, bool) {
	if p, ok := retainedPlane(f); ok {
		return p, true
	}
	w, ok := readChargedChainFace(f)
	if !ok || !w.isPlane || w.departure == 0 {
		return plane{}, false
	}
	n, ok := w.fn.Normalize()
	if !ok {
		return plane{}, false
	}
	if f.Reversed {
		n = n.Scale(-1)
	}
	return plane{n: n, o: w.fo, delta: w.delta}, true
}

// chainCandidate reports whether e is an edge a charged chain may hold: a Line3 or Arc3 between
// two faces that tagChain reads. The entry gate of blend admits it only so that planChains
// decides it; it never routes the edge to a per-edge route.
func chainCandidate(e *topo.Edge) bool { return chainShape(e) && chainableIn(e, tagChain) }

// chainFaceCharged reports whether f states a charge tagChain reads: a displacement or an
// extent bound.
func chainFaceCharged(f *topo.Face) bool {
	w, ok := readChargedChainFace(f)
	return ok && (w.delta != 0 || w.departure != 0)
}

// chainObliqueWall reports whether f is an oblique plane wall tagChain reads.
func chainObliqueWall(f *topo.Face) bool {
	w, ok := readChargedChainFace(f)
	return ok && w.departure != 0
}

// chainCharged reports whether any face of the edges states a charge that tagChain reads.
func chainCharged(edges []*topo.Edge) bool {
	return slices.ContainsFunc(edges, func(e *topo.Edge) bool {
		return slices.ContainsFunc(e.Faces, chainFaceCharged)
	})
}

// refuseChargedChain refuses a chain that reads charged walls where only a closed chamfer chain
// builds them (charged-window-chamfer-design.md §3.5).
func refuseChargedChain(cyc chainCycle, fillet bool) error {
	if !chainCharged(cyc.edges) {
		return nil
	}
	if cyc.open {
		return fmt.Errorf("%w: an open tangent chain on walls that state a displacement is not built",
			decaderr.ErrUnsupported)
	}
	if fillet {
		return fmt.Errorf("%w: a tangent chain fillet on walls that state a displacement is not built",
			decaderr.ErrUnsupported)
	}
	return nil
}

// heldCharge widens charge by the displacement a held face states: a plane's or a cylinder's
// AxialDelta and an oblique plane's ExtentDelta, each amplified by planeAmp (cylCharge). It
// refuses a face whose AxialDelta is unusable. An ExtentDelta that is not finite and positive
// states nothing a chain reads.
func heldCharge(charge float64, f *topo.Face) (float64, error) {
	d, ok := faceDelta(f)
	if !ok {
		return 0, fmt.Errorf("%w: a face states no usable plane displacement", decaderr.ErrUnsupported)
	}
	_, plane := f.Surface.(surfacegeom.Plane)
	_, cyl := f.Surface.(surfacegeom.Cylinder)
	if plane || cyl {
		charge = proofbound.AbsSumUpper(charge, proofbound.ProductUpper(planeAmp, d))
	}
	if plane && f.ExtentDelta > 0 && !math.IsInf(f.ExtentDelta, 0) {
		charge = proofbound.AbsSumUpper(charge, proofbound.ProductUpper(planeAmp, f.ExtentDelta))
	}
	return charge, nil
}

// chainCharges is what a charged closed chamfer chain adds to its bounds (§3.2, §3.3). It is
// the zero value for a chain that reads no charged face, which adds nothing.
type chainCharges struct {
	charged bool
	// delta is the largest wall charge and deltaP the base's.
	delta, deltaP float64
	// eJ is, per junction (indexed as the chain's joins), e_J = β(v) + |v − V|.
	eJ []float64
	// crease is the largest c_J over the junctions.
	crease float64
}

// wallScale is the coordinate scale of a wall's tag.
func wallScale(w chainWall) float64 {
	a := w.fo
	if !w.isPlane {
		a = w.fc
	}
	return tagScale(a, r3.Vec{}, w.fr)
}

// psiOf is the turn a wall's true curve may have against its tag's at a junction whose e_J is
// eJ (§3.2, "The crease"): (e_J + 2δ)/R for a cylinder, the departure for an oblique plane, zero
// for a displaced plane.
func psiOf(w chainWall, eJ float64) float64 {
	switch {
	case !w.isPlane:
		return proofbound.DivUpper(proofbound.AbsSumUpper(eJ, proofbound.ProductUpper(2, w.delta)), down(w.fr))
	case w.departure != 0:
		return w.departure
	}
	return 0
}

// newChainCharges reads the charges of the closed chain cyc on base. hv are the held vertices
// of its junctions and jp the tags' junction points, both indexed as cyc.joins. size is the
// larger setback.
func newChainCharges(cyc chainCycle, base *topo.Face, fp facetedrecord.Payload, hv []int, jp []r3.Vec,
	size float64) (chainCharges, error) {
	var cc chainCharges
	if !chainCharged(cyc.edges) {
		return cc, nil
	}
	if cyc.open {
		return cc, fmt.Errorf("%w: an open tangent chain on walls that state a displacement is not built",
			decaderr.ErrUnsupported)
	}
	bw, ok := readChargedChainFace(base)
	if !ok || bw.departure != 0 {
		return cc, fmt.Errorf("%w: the chain's base is not a retained plane", decaderr.ErrUnsupported)
	}
	cc.charged, cc.deltaP = true, bw.delta
	cc.eJ = make([]float64, len(cyc.joins))
	for i, l := range cyc.joins {
		walls := make([]chainWall, 0, 2)
		for _, f := range []*topo.Face{l.j.wall1, l.j.wall2} {
			w, ok := readChargedChainFace(f)
			if !ok {
				return cc, fmt.Errorf("%w: a chain wall is not a retained plane or cylinder", decaderr.ErrUnsupported)
			}
			cc.delta = max(cc.delta, w.delta)
			walls = append(walls, w)
		}
		dist := fp.Verts[hv[i]].Sub(jp[i]).Len()
		eJ := proofbound.AbsSumUpper(fp.VertexBound[hv[i]], up(dist))
		cc.eJ[i] = eJ
		// ψ_max = Σ_k ψ_k + ψ_0, where ψ_0 is the angle the tangentRoundoff gap between the tags
		// allows: a plane or cylinder wall that lies off a cylinder by g meets it at a turn of at
		// most √(2g/R).
		scale, rMin := 1.0, math.Inf(1)
		psi := 0.0
		for _, w := range walls {
			psi = proofbound.AbsSumUpper(psi, psiOf(w, eJ))
			scale = max(scale, wallScale(w))
			if !w.isPlane {
				rMin = min(rMin, w.fr)
			}
		}
		gap := up(tangentRoundoff * ulp * scale)
		psi0 := up(math.Sqrt(up(proofbound.DivUpper(proofbound.ProductUpper(2, gap), down(rMin)))))
		psi = proofbound.AbsSumUpper(psi, psi0)
		if !(psi <= 1) {
			return cc, fmt.Errorf("%w: the true walls of a chain junction may turn there by more than a radian",
				decaderr.ErrUnsupported)
		}
		c := up(proofbound.ProductUpper(size, proofbound.ProductUpper(psi, psi)) / 4)
		cc.crease = max(cc.crease, c)
	}
	return cc, nil
}

// raise widens the bound of every vertex of a junction's row by that junction's e_J (§3.3,
// "Junction rows").
func (cc chainCharges) raise(w *work, j int, rows ...int) {
	if !cc.charged {
		return
	}
	for _, v := range rows {
		w.beta[v] = proofbound.AbsSumUpper(w.beta[v], cc.eJ[j])
	}
}

// sigma is the shell half-width of §3.2: 3δ + δ_P + max c_J, rounded up.
func (cc chainCharges) sigma() float64 {
	return proofbound.AbsSumUpper(proofbound.ProductUpper(3, cc.delta), cc.deltaP, cc.crease)
}

// chainPiece is one group a closed chain rewrites, with the junctions it meets and its width
// there.
type chainPiece struct {
	geo       chargePiece
	junctions []int
	width     float64
}

// areaCharge is the area bound a charged chain adds (§3.3, "Area"): per changed group
// 2·P·σ + (π + Φ)·σ² + A·σ/r + w·max(1, ρ_max/r_min)·Σ e_J. The chain's blend groups, the base
// and each wall are the changed groups. ρ_max and r_min are the largest and smallest radius over
// all the chain's arcs and their setback circles, which bound each piece's own ratio. entries
// hold the chain in walk order and the junctions of entry t are entries[t].ja and entries[t].jb.
func (cc chainCharges) areaCharge(entries []chainEntry, tP, tW float64, spec blendSpec) float64 {
	if !cc.charged {
		return 0
	}
	sigma := cc.sigma()
	hyp := up(math.Hypot(tP, tW))
	rhoMax, rMin := 0.0, math.Inf(1)
	for _, en := range entries {
		if !en.arc {
			continue
		}
		rho := en.rim.radius + en.rim.kappa*tP
		rhoMax = max(rhoMax, en.rim.radius, rho)
		rMin = min(rMin, en.rim.radius, rho)
	}
	factor := 1.0
	if !math.IsInf(rMin, 1) {
		factor = max(1, proofbound.DivUpper(rhoMax, down(rMin)))
	}
	all := make([]int, 0, len(entries))
	for _, en := range entries {
		all = append(all, en.ja)
	}
	base := chainPiece{geo: chargePiece{turning: 2 * proofbound.TwoPiUpper()}, junctions: all, width: tP}
	var pieces []chainPiece
	for _, en := range entries {
		pa, wa, ba := en.areas(tP, tW, spec)
		outer, inner, span := en.length, en.length, 0.0
		wall := chainPiece{junctions: []int{en.ja, en.jb}, width: tW}
		blend := chainPiece{junctions: []int{en.ja, en.jb}, width: hyp}
		if en.arc {
			rho := en.rim.radius + en.rim.kappa*tP
			span = en.span
			outer = proofbound.ProductUpper(en.rim.radius, span)
			inner = proofbound.ProductUpper(math.Abs(rho), span)
			wall.geo.rMin = down(en.rim.radius)
			blend.geo.rMin = down(min(en.rim.radius, rho))
		}
		base.geo.perimeter = proofbound.AbsSumUpper(base.geo.perimeter, outer, inner)
		base.geo.turning = proofbound.AbsSumUpper(base.geo.turning, 2*span)
		base.geo.area = proofbound.AbsSumUpper(base.geo.area, pa)
		wall.geo.perimeter = proofbound.AbsSumUpper(proofbound.ProductUpper(2, outer), proofbound.ProductUpper(2, tW))
		wall.geo.turning = proofbound.AbsSumUpper(proofbound.TwoPiUpper(), 2*span)
		wall.geo.area = wa
		blend.geo.perimeter = proofbound.AbsSumUpper(outer, inner, proofbound.ProductUpper(2, hyp))
		blend.geo.turning = proofbound.AbsSumUpper(proofbound.TwoPiUpper(), 2*span)
		blend.geo.area = ba
		pieces = append(pieces, wall, blend)
	}
	pieces = append(pieces, base)
	cps := make([]chargePiece, len(pieces))
	total := 0.0
	for i, p := range pieces {
		cps[i] = p.geo
		sum := 0.0
		for _, j := range p.junctions {
			sum = proofbound.AbsSumUpper(sum, cc.eJ[j])
		}
		total = proofbound.AbsSumUpper(total,
			proofbound.ProductUpper(proofbound.ProductUpper(p.width, factor), sum))
	}
	return proofbound.AbsSumUpper(total, arcAreaCharge(cps, sigma, sigma))
}
