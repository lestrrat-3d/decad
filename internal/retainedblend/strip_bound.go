package retainedblend

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// This file holds the bounds of a blend strip's facets (agent-docs/
// retained-edge-blend-design.md §6.3, "Strip triangles"). A strip triangle
// whose vertices lie on the exact blend's rows stays within the profile's
// sagitta over its row gap plus rhoHi·(1 − cos(Δφ/2)) of the surface, with Δφ
// the spread of its vertices' azimuths. A triangle of the band beside the
// wall's cut adds how far its cut vertices lie from the exact setback curve.

// azimuthShare is the part of tau a revolve strip's columns may spend on the
// azimuth term: half of it on a fillet, whose rows spend the other half on
// the profile's sagitta (profileShare), and all of it on a chamfer, whose
// profile is straight. The two terms add, so a strip triangle between exact
// rows stays within tau, which every row vertex states.
func azimuthShare(spec blendSpec, tau float64) float64 {
	if spec.fillet {
		return tau / 2
	}
	return tau
}

// profileShare is the sagitta a fillet's rows keep over each gap of the
// profile's arc: half of tau, the azimuth term taking the rest (azimuthShare).
func profileShare(tau float64) float64 { return tau / 2 }

// filletSag is the sagitta of a fillet's profile arc of radius r over the
// tube angle span: r·(1 − cos(span/2)). A chamfer's profile is straight.
func filletSag(spec blendSpec, span float64) float64 {
	if !spec.fillet {
		return 0
	}
	return spec.r * (1 - math.Cos(span/2))
}

// facetBound is the facet bound of held triangle t: the largest bound of its
// corners, within which every point of the facet lies of the true piece it
// stands for (agent-docs/faceted-vertex-bounds-design.md §2).
func (w *work) facetBound(t [3]int) float64 {
	return max(w.beta[t[0]], w.beta[t[1]], w.beta[t[2]])
}

// noteCut records that vertex c was cut from a held wall facet whose facet
// bound is delta. A vertex two facets cut keeps the first one's: either
// facet's bound places the vertex within it of the true wall.
func (w *work) noteCut(c int, delta float64) {
	if w.cutDelta == nil {
		w.cutDelta = map[int]float64{}
	}
	if _, ok := w.cutDelta[c]; !ok {
		w.cutDelta[c] = delta
	}
}

// cutOffset is how far wall cut vertex v, cut from a held facet of bound
// delta, lies from the point of the exact setback curve at its own azimuth
// (or, on a planar wall, its own place along the setback line): its surrogate.
// e is the rounding of the cut's interpolation. On a wall whose surface the
// cut plane meets square, a cylinder about the axis or a plane along the
// edge, the distance from the wall is the offset in the cut plane, so
// |v − ṽ| ≤ δ + 2e. On a sphere the cut plane meets the wall at a slant, and
// the offset within the plane is |ρ_v − ρ_B| ≤ (δ'·(2R + δ') +
// e·(2|z_B − z_S| + e)) / (ρ_v + ρ_B), δ' = δ + e (sphereWall).
func (w *work) cutOffset(v int, delta, e float64) float64 {
	if w.sphereWall == nil {
		return proofbound.AbsSumUpper(delta, e, e)
	}
	s := w.sphereWall
	d := proofbound.AbsSumUpper(delta, e)
	num := proofbound.AbsSumUpper(proofbound.ProductUpper(d, proofbound.AbsSumUpper(2*s.radius, d)),
		proofbound.ProductUpper(e, proofbound.AbsSumUpper(2*math.Abs(s.zB-s.zS), e)))
	rho := w.r.radial(w.verts[v])
	den := (rho + s.rhoB) * (1 - 4*ulp)
	return proofbound.AbsSumUpper(proofbound.DivUpper(num, den), e)
}

// sphereWall is the sphere an R6 rim's wall lies on, in the rim's meridian
// coordinates: its radius, its centre's height above the rim's plane, and the
// setback circle's radius and height.
type sphereWall struct {
	radius, zS, rhoB, zB float64
}

// azimuthTerm is rhoHi·(1 − cos(Δφ/2)), Δφ the largest angle between the
// directions of the points ps about the axis through o along n, which the
// points' spread stays under. A strip triangle spans less than a half turn,
// so the largest pairwise angle is the spread. The rounding of the
// directions and of the half-angle formula is a few units in the last place
// of rhoHi, which the caller's slack holds.
func azimuthTerm(o, n r3.Vec, rhoHi float64, ps ...r3.Vec) float64 {
	dirs := make([]r3.Vec, 0, len(ps))
	for _, p := range ps {
		q := p.Sub(o)
		u, ok := q.Sub(n.Scale(q.Dot(n))).Normalize()
		if !ok {
			return rhoHi
		}
		dirs = append(dirs, u)
	}
	lowest := 1.0
	for i := range dirs {
		for j := i + 1; j < len(dirs); j++ {
			lowest = math.Min(lowest, dirs[i].Dot(dirs[j]))
		}
	}
	half := math.Sqrt(math.Max(0, (1+lowest)/2))
	return proofbound.ProductUpper(rhoHi, proofbound.AbsSumUpper(1-half, 4*ulp))
}

// bandAxis is the axis a revolve strip's rows turn about; a straight strip
// has none.
type bandAxis struct {
	o, n r3.Vec
	ok   bool
}

// chargeBand raises the bound of every wall cut vertex of the strip
// triangles out[from:] to what its band triangles need: the larger cut
// offset of the triangle's cut vertices, plus the profile's sagitta sag over
// the band's row gap, plus, about axis, (1 + 1/cos(Δα/2))·rhoHi·(1 − cos(Δφ/2))
// of the triangle's own azimuth spread, plus slack for the rounding (§6.3).
// The azimuth term counts once for the triangle's distance from the blend and
// once more, over cos(Δα/2), for a point of the blend beside a row's polyline,
// Δα the gap's tube angle. A triangle with no cut vertex lies between exact
// rows and needs nothing more than tau.
func (w *work) chargeBand(from int, axis bandAxis, rhoHi, sag, slack float64) {
	grow := 2.0
	if w.spec.fillet && sag > 0 {
		grow = 1 + 1/((1-sag/w.spec.r)*(1-4*ulp))
	}
	for _, t := range w.out[from:] {
		off, cut := 0.0, false
		for _, v := range t {
			if d, ok := w.cutDelta[v]; ok {
				cut = true
				off = max(off, w.cutOffset(v, d, slack))
			}
		}
		if !cut {
			continue
		}
		e := 0.0
		if axis.ok {
			e = azimuthTerm(axis.o, axis.n, rhoHi, w.verts[t[0]], w.verts[t[1]], w.verts[t[2]])
		}
		charge := proofbound.AbsSumUpper(off, sag, proofbound.ProductUpper(grow, e), slack)
		for _, v := range t {
			if _, ok := w.cutDelta[v]; ok {
				w.beta[v] = max(w.beta[v], charge)
			}
		}
	}
}

// chargeSpherePatch raises the bound of every vertex of the sphere patch's
// triangles out[from:] to the triangle's own departure from the sphere of
// centre c and radius r. Every point x of a triangle whose corners lie on the
// sphere lies inside it, at least the distance of c from the triangle's plane
// from c, so r − |x − c| ≤ r − dist(c, plane); a point of the sphere and the
// point of the triangle on the ray from c to it are as far apart as that,
// which bounds both directions. The corners' own rounding and that of the
// formula are within slack.
func (w *work) chargeSpherePatch(from int, c r3.Vec, r, slack float64) {
	for _, t := range w.out[from:] {
		a, b, d := w.verts[t[0]], w.verts[t[1]], w.verts[t[2]]
		nrm := b.Sub(a).Cross(d.Sub(a))
		l := nrm.Len()
		dep := r
		if l > 0 {
			dep = math.Max(0, r-math.Abs(a.Sub(c).Dot(nrm))/l)
		}
		charge := proofbound.AbsSumUpper(dep, slack, slack)
		for _, v := range t {
			w.beta[v] = max(w.beta[v], charge)
		}
	}
}
