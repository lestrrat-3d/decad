package retainedblend

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// This file holds the end of an open tangent chain on an arc edge
// (agent-docs/retained-modify-reach-design.md §5.5 step 5, blend §4.4). The
// arc's free end lies on a retained plane G parallel to the axis. Each row of
// the end column is the point where the row's circle of the torus (or cone)
// meets G, so the end curve is a chain of exact points and not a conic.

// arcEnd is the arc-specific data of one chain end.
type arcEnd struct {
	a     arcRim // sign +1: theta is the counter-clockwise angle about the axis
	g     plane
	ang   []float64 // the angle of each end row's point
	pt0   r3.Vec    // the plane's setback point on G ∩ P
	reach float64   // the largest distance of the blend's end curve from the corner
	// cornerArea is the area the blend takes from G.
	cornerArea float64
}

// readArcEndCorner reads the corner at which the chain's arc e ends at vertex
// v: the arc's own end reading (blend §4.4), which checks that G is a retained
// plane parallel to the axis that cuts the rim at a real angle, and that the
// three tags meet at the held vertex.
func (w *work) readArcEndCorner(v *topo.Vertex, e *topo.Edge, base, wall *topo.Face,
	fp facetedrecord.Payload, faceIndex map[*topo.Face]int) (*chainEndCut, error) {
	r, err := readRim(e, fp, faceIndex)
	if err != nil {
		return nil, err
	}
	if r.planeFace != base || r.wallFace != wall {
		return nil, fmt.Errorf("%w: a chain arc does not border the base and its wall", decaderr.ErrUnsupported)
	}
	// Blend §4.4's end reads a convex rim only (SM9).
	if !r.convex {
		return nil, fmt.Errorf("%w: a concave open tangent chain ends on an arc; only a straight end is built",
			decaderr.ErrUnsupported)
	}
	a := arcRim{rim: r, sign: 1}
	end, err := a.readEnd(v, fp, faceIndex, faceIndex[base], faceIndex[wall])
	if err != nil {
		return nil, err
	}
	return &chainEndCut{faceIx: faceIndex[end.face], g: end.g, exact: end.exact, v: end.v,
		arc: &arcEnd{a: a, g: end.g}}, nil
}

// endAngles are the angles at which the circles of the profile's rows meet G,
// taken in turn as the nearest root to the previous row's.
func (ae *arcEnd) endAngles(p arcProfile, ss []float64, exact r3.Vec) ([]float64, error) {
	ref := ae.a.angle(exact)
	out := make([]float64, len(ss))
	for j, s := range ss {
		rho, _ := p(s)
		roots, ok := ae.a.ringAzimuths(rho, ae.g, ref)
		if !ok {
			return nil, fmt.Errorf("%w: a blend circle does not reach an end face", decaderr.ErrUnsupported)
		}
		t := roots[0]
		if math.Abs(roots[1]-ref) < math.Abs(roots[0]-ref) {
			t = roots[1]
		}
		out[j], ref = t, t
	}
	return out, nil
}

// arcEndColumn builds the end column of an arc end: the vertices of rows
// 0 … rowsM−1 of the blend's profile on G. It records the plane's setback
// point, the reach of the end curve from the corner, and the area the blend
// takes from G.
func (w *work) arcEndColumn(ec *chainEndCut, step blendSpec, rowsM int, bound float64) ([]int, error) {
	ae := ec.arc
	p, _ := ae.a.profile(step)
	ss := make([]float64, rowsM)
	for j := range ss {
		ss[j] = float64(j) / float64(rowsM)
	}
	var err error
	if ae.ang, err = ae.endAngles(p, ss, ec.exact); err != nil {
		return nil, err
	}
	rows := make([]int, rowsM)
	ae.reach = step.wallSetback()
	for j, s := range ss {
		rho, z := p(s)
		pt := ae.a.atTheta(rho, ae.ang[j], z)
		rows[j] = w.vertex(pt, bound)
		ae.reach = max(ae.reach, pt.Sub(ec.exact).Len())
		if j == 0 {
			ae.pt0 = pt
		}
	}
	ae.cornerArea = ae.cornerAreaOf(p, ec.exact)
	return rows, nil
}

// cornerAreaOf is the area of the corner region of G the blend removes: the
// region between the end curve and the two lines of G through the corner. The
// end curve is sampled densely, as arcAreas does for a lone arc.
func (ae *arcEnd) cornerAreaOf(p arcProfile, exact r3.Vec) float64 {
	const samples = 4000
	pts := []r3.Vec{{}}
	ref := ae.a.angle(exact)
	for s := 0; s <= samples; s++ {
		rho, z := p(float64(s) / samples)
		roots, ok := ae.a.ringAzimuths(rho, ae.g, ref)
		if !ok {
			continue
		}
		t := roots[0]
		if math.Abs(roots[1]-ref) < math.Abs(roots[0]-ref) {
			t = roots[1]
		}
		ref = t
		pts = append(pts, ae.a.atTheta(rho, t, z).Sub(exact))
	}
	return polyArea(pts...)
}

// chamferEndRows is the number of rows a chamfer's cone needs for its chords to
// follow the section G cuts in it to within tau. A chain of straight edges has
// one row; an arc end needs more (arc.chamferRows, for one end).
func (ae *arcEnd) chamferEndRows(p arcProfile, exact r3.Vec, tauMax float64) int {
	for m := 2; m < 64; m *= 2 {
		ss := make([]float64, 0, 2*m+1)
		for j := 0; j <= 2*m; j++ {
			ss = append(ss, float64(j)/float64(2*m))
		}
		angs, err := ae.endAngles(p, ss, exact)
		if err != nil {
			return m
		}
		worst := 0.0
		pt := func(k int) r3.Vec {
			rho, z := p(ss[k])
			return ae.a.atTheta(rho, angs[k], z)
		}
		for j := range m {
			worst = max(worst, distToSegment(pt(2*j+1), pt(2*j), pt(2*j+2)))
		}
		if worst <= tauMax {
			return m
		}
	}
	return 64
}

// fanBounds are, for each row, the angle at which the entry's blend starts and
// the angle at which it ends. A junction end is a meridian, the same angle for
// every row; a free end is the row's own meeting with G.
func (en *chainEntry) fanBounds(rowsM int) (start, end []float64) {
	start, end = make([]float64, rowsM), make([]float64, rowsM)
	phi1 := en.phi0 + en.sigma*en.span
	for j := range rowsM {
		start[j], end[j] = en.phi0, phi1
		if en.freeStart != nil {
			start[j] = en.freeStart.ang[j]
		}
		if en.freeEnd != nil {
			ae := en.freeEnd
			shift := 2 * math.Pi * math.Round((phi1-ae.a.angle(en.p1))/(2*math.Pi))
			end[j] = ae.ang[j] + shift
		}
	}
	return start, end
}

// checkFan refuses an entry whose end curve on G crosses its other end: every
// row must run forward along the arc.
func (en *chainEntry) checkFan(rowsM int) error {
	start, end := en.fanBounds(rowsM)
	for j := range rowsM {
		if !(en.sigma*(end[j]-start[j]) > minSine) {
			return fmt.Errorf("%w: the blend's end curve on its end face crosses the chain arc", decaderr.ErrUnsupported)
		}
	}
	return nil
}

// fanRows builds the rows of the arc entry's column at fraction f of its
// span. Row j lies on the circle of the profile at the angle that is f of the
// way from the row's start angle to its end angle, so a free end's columns fan
// out from the end curve on G.
func (w *work) fanRows(en *chainEntry, f float64, rowsM int, bound float64) []int {
	start, end := en.fanBounds(rowsM)
	a := arcRim{rim: en.rim, sign: 1}
	p, _ := a.profile(en.step)
	rows := make([]int, rowsM)
	for j := range rows {
		rho, z := p(float64(j) / float64(rowsM))
		rows[j] = w.vertex(en.rim.at(rho, start[j]+(end[j]-start[j])*f, z), bound)
	}
	return rows
}

// areasFree is chainEntry.areas for an arc with a free end on G. The wall's
// band keeps its generators at the corner angles. The base loses the region
// between the rim and the setback arc, bounded by the radial line at a
// junction and by G's line at a free end (Green's theorem over the two arcs and
// the two lines); the blend face is the torus (or cone) patch between the two
// angles of each row.
func (en chainEntry) areasFree(tP, tW float64) (base, wall, blend float64) {
	const samples = 4000
	R, k := en.rim.radius, en.rim.kappa
	rhoA := R + k*tP
	rowsM := 1
	start, end := en.fanBounds(rowsM)
	// The base: rows' angles at the plane's row.
	th0, th1 := en.phi0, en.phi0+en.sigma*en.span
	xy := func(rho, ang float64) (float64, float64) { return rho * math.Cos(ang), rho * math.Sin(ang) }
	cross := func(x0, y0, x1, y1 float64) float64 { return x0*y1 - x1*y0 }
	// The plane's row is row 0: its angles are the first row's.
	p0x, p0y := xy(R, th0)
	p1x, p1y := xy(R, th1)
	q0x, q0y := xy(rhoA, start[0])
	q1x, q1y := xy(rhoA, end[0])
	area := R*R*(th1-th0)/2 + cross(p1x, p1y, q1x, q1y)/2 + rhoA*rhoA*(start[0]-end[0])/2 + cross(q0x, q0y, p0x, p0y)/2
	base = math.Abs(area)
	wall = en.span * R * tW
	// The blend face, by midpoint sums over the profile.
	a := arcRim{rim: en.rim, sign: 1}
	prof, _ := a.profile(en.step)
	rowAngle := func(ae *arcEnd, s, ref float64) float64 {
		rho, _ := prof(s)
		roots, ok := ae.a.ringAzimuths(rho, ae.g, ref)
		if !ok {
			return ref
		}
		if math.Abs(roots[1]-ref) < math.Abs(roots[0]-ref) {
			return roots[1]
		}
		return roots[0]
	}
	refS, refE := th0, th1
	prev := 0.0
	for i := 1; i <= samples; i++ {
		s := float64(i) / samples
		sm := (prev + s) / 2
		rho0, z0 := prof(prev)
		rho1, z1 := prof(s)
		rhoM, _ := prof(sm)
		prev = s
		fromAngle, toAngle := th0, th1
		if en.freeStart != nil {
			refS = rowAngle(en.freeStart, sm, refS)
			fromAngle = refS
		}
		if en.freeEnd != nil {
			refE = rowAngle(en.freeEnd, sm, refE)
			toAngle = refE
		}
		blend += rhoM * en.sigma * (toAngle - fromAngle) * math.Hypot(rho1-rho0, z1-z0)
	}
	return base, wall, blend
}
