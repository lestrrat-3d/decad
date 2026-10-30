package retainedblend

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// sphereRim is a closed circular edge between a retained plane and a retained
// sphere (agent-docs/retained-edge-blend-design.md §4.5, R6). Its rim frame is
// the plane's: center is the foot of the sphere's centre on the plane, n the
// plane's outward normal, radius the rim's radius. matOutside is the sphere
// face's Reversed flag.
type sphereRim struct {
	rim
	sphere       r3.Vec
	sphereRadius float64
}

// looksLikeSphereRim reports whether edge is a candidate for the plane-sphere
// rim route: a closed retained Circle3 between exactly one retained Plane, read
// as its record, and one retained Sphere, neither with a stated tag departure.
// Whether the sphere's tag is its record is readSphereRim's to refuse, with a
// message that names the failing leaf. It reads no mesh and refuses nothing.
func looksLikeSphereRim(edge *topo.Edge) bool {
	if edge == nil || edge.Start == nil || edge.Start != edge.End || !edge.RetainedCurve || len(edge.Faces) != 2 {
		return false
	}
	if _, ok := edge.Curve.(surfacegeom.Circle3); !ok {
		return false
	}
	planes, spheres := 0, 0
	for _, f := range edge.Faces {
		if f == nil || !f.RetainedTag || f.TagDeparture != 0 {
			return false
		}
		switch f.Surface.(type) {
		case surfacegeom.Plane:
			if !tagIsRecord(f) {
				return false
			}
			planes++
		case surfacegeom.Sphere:
			spheres++
		}
	}
	return planes == 1 && spheres == 1
}

// readSphereRim reads the rim of edge from its two tags and the held wall. It
// refuses a sphere whose tag is not its record (SR1), a plane that meets the
// sphere at a grazing angle (SR5), and an edge whose tags and held mesh do not
// describe one rim (SR6).
func readSphereRim(edge *topo.Edge, fp facetedrecord.Payload, faceIndex map[*topo.Face]int) (sphereRim, error) {
	var s sphereRim
	for _, f := range edge.Faces {
		if _, ok := f.Surface.(surfacegeom.Plane); ok {
			s.planeFace = f
		} else {
			s.wallFace = f
		}
	}
	g, planeOK := retainedPlane(s.planeFace)
	sph, sphereOK := s.wallFace.Surface.(surfacegeom.Sphere)
	circleCentre, circleRadiusValue, circleOK := circleOf(edge.Curve)
	if !planeOK || !sphereOK || !circleOK {
		return sphereRim{}, fmt.Errorf("%w: the rim's faces or curve changed kind", decaderr.ErrUnsupported)
	}
	if err := sphereIsRecord(s.wallFace, sph); err != nil {
		return sphereRim{}, err
	}
	radius, err := sph.Radius.In(units.Millimeter)
	if err != nil || !(radius > 0) || math.IsInf(radius, 0) {
		return sphereRim{}, fmt.Errorf("%w: the rim's sphere has no finite radius", decaderr.ErrUnsupported)
	}
	s.charge = planeCharge(g)
	s.scale = 1
	for _, v := range fp.Verts {
		s.scale = max(s.scale, math.Abs(v.X), math.Abs(v.Y), math.Abs(v.Z))
	}
	// The sphere's centre stands zs above the plane. The rim's radius over the
	// sphere's is the sine of the angle at which the plane crosses the sphere.
	zs := g.offset(sph.Center)
	if err := crossing(radius*radius-zs*zs, radius); err != nil {
		return sphereRim{}, err
	}
	s.n, s.center, s.radius = g.n, sph.Center.Sub(g.n.Scale(zs)), math.Sqrt(radius*radius-zs*zs)
	s.sphere, s.sphereRadius = sph.Center, radius
	circleRadius, err := circleRadiusValue.In(units.Millimeter)
	if err != nil {
		return sphereRim{}, fmt.Errorf("%w: the rim's circle has no radius", decaderr.ErrUnsupported)
	}
	slack := s.slack()
	if s.center.Sub(circleCentre).Len() > slack || math.Abs(circleRadius-s.radius) > slack {
		return sphereRim{}, fmt.Errorf("%w: the rim's circle is not the meeting of its two tags", decaderr.ErrUnsupported)
	}
	var ok bool
	if s.u, ok = perpendicular(s.n); !ok {
		return sphereRim{}, fmt.Errorf("%w: the rim's plane has no frame", decaderr.ErrUnsupported)
	}
	s.w = s.n.Cross(s.u)
	s.matOutside = s.wallFace.Reversed
	if err := s.readSide(edge, fp, faceIndex[s.wallFace]); err != nil {
		return sphereRim{}, err
	}
	return s, nil
}

// crossing refuses a plane that misses the sphere, touches it, or crosses it
// at an angle whose sine, the rim's radius over the sphere's, is below
// minSine: rr is the rim's squared radius and radius the sphere's. A nearly
// tangent rim is no edge the section solve can resolve.
func crossing(rr, radius float64) error {
	if !(rr > 0) || !(math.Sqrt(rr) >= minSine*radius) {
		return fmt.Errorf("%w: the plane meets the sphere at a grazing angle", decaderr.ErrUnsupported)
	}
	return nil
}

// checkSize refuses a blend size that is not positive and finite (SR3). The
// size's fit is the section's to decide.
func (sphereRim) checkSize(spec blendSpec) error {
	ta, tb := spec.planeSetback(), spec.wallSetback()
	if !(ta > 0) || !(tb > 0) || math.IsInf(ta, 0) || math.IsInf(tb, 0) {
		return fmt.Errorf("%w: a blend needs a positive finite size", decaderr.ErrDegenerate)
	}
	return nil
}

// meridian is the blend's section in the rim's meridian half-plane: ρ from the
// axis through the sphere's centre along n, z along n from the rim's plane.
type meridian struct {
	rhoA      float64      // the plane's setback circle, at z = 0
	rhoB, zB  float64      // the wall's setback circle
	rows      [][2]float64 // the fillet's rows between them, as (ρ, z)
	phi       float64      // the fillet arc's angle, 0 for a chamfer
	rhoHi     float64      // the largest ρ of the blend's rows
	zA        float64      // the plane's setback circle's height, 0 to rounding
	rhoC, zC  float64      // the fillet ball's centre
	r         float64      // the fillet's radius, 0 for a chamfer
	trueArea  float64      // the exact area of the blend face
	outwardMR [2]float64   // a chamfer's outward normal, as (ρ, z)
}

// section solves the blend in the meridian at azimuth 0 (§4.5). The plane is
// the line z = 0 and the sphere the circle about its centre: R3's
// line-circle section, with the plane's face along ±u from the rim's point K
// and the wall along the circle's tangent at K toward the wall's side.
func (s sphereRim) section(spec blendSpec, tau float64) (meridian, error) {
	var m meridian
	k := s.center.Add(s.u.Scale(s.radius))
	wA := s.u.Scale(s.kappa)
	u1 := k.Sub(s.sphere).Scale(1 / s.sphereRadius)
	a, b := u1.Dot(s.u), u1.Dot(s.n)
	wB := s.u.Scale(-b).Add(s.n.Scale(a)).Scale(s.side)
	margin := tau + s.slack()
	line := cylLine{a: plane{n: s.n, o: s.center}, radius: s.sphereRadius, matOutside: s.matOutside,
		charge: s.charge}
	sec, err := line.solveSection(spec, k, s.sphere, wA, wB, s.convex, tau, margin)
	if err != nil {
		return m, err
	}
	at := func(p r3.Vec) (float64, float64) {
		q := p.Sub(s.center)
		return q.Dot(s.u), q.Dot(s.n)
	}
	m.rhoA, m.zA = at(sec.ta)
	m.rhoB, m.zB = at(sec.tb)
	if !(m.rhoA > margin) {
		return m, fmt.Errorf("%w: the blend reaches the rim's axis: setback %g mm against rim radius %g mm",
			decaderr.ErrDegenerate, sec.setback, s.radius)
	}
	if !(m.rhoB > margin) {
		return m, fmt.Errorf("%w: the blend reaches past the sphere's pole on the rim's axis", decaderr.ErrDegenerate)
	}
	if !spec.fillet {
		// A chamfer's outward normal leans toward the sum of the two faces'
		// outward normals at K, in the meridian.
		chordRho, chordZ := m.rhoB-m.rhoA, m.zB
		normRho, normZ := -chordZ, chordRho
		nB := [2]float64{a, b}
		if s.matOutside {
			nB = [2]float64{-a, -b}
		}
		if normRho*nB[0]+normZ*(nB[1]+1) < 0 {
			normRho, normZ = -normRho, -normZ
		}
		m.outwardMR = [2]float64{normRho, normZ}
		m.trueArea = math.Pi * (m.rhoA + m.rhoB) * math.Hypot(chordRho, chordZ)
		m.rhoHi = max(m.rhoA, m.rhoB)
		return m, nil
	}
	r := spec.r
	m.r = r
	m.rhoC, m.zC = at(sec.centre)
	m.phi = sec.phi
	m.rhoHi = max(m.rhoA, m.rhoB)
	for _, p := range sec.rows[1 : len(sec.rows)-1] {
		rho, z := at(p)
		m.rows = append(m.rows, [2]float64{rho, z})
		m.rhoHi = max(m.rhoHi, rho)
	}
	// The torus patch's area by Pappus: the arc runs from a0 to a1 about the
	// ball's centre through phi, and its points stand ρc + r·(a(α)·u) from the
	// axis, a(α) = cos α·a0 + sin α·e2.
	a0 := sec.ta.Sub(sec.centre).Scale(1 / r).Dot(s.u)
	a1 := sec.tb.Sub(sec.centre).Scale(1 / r).Dot(s.u)
	sin, cos := math.Sincos(sec.phi)
	e2 := (a1 - cos*a0) / sin
	m.trueArea = 2 * math.Pi * r * (m.rhoC*sec.phi + r*(a0*sin+e2*(1-cos)))
	return m, nil
}

// tag is the blend's exact surface: the Torus that the fillet ball sweeps about
// the rim's axis, or the Cone through the chamfer's two setback circles.
func (s sphereRim) tag(spec blendSpec, m meridian) blendTag {
	r := s.rim
	if spec.fillet {
		rr := spec.r
		tag := blendTag{
			surface: surfacegeom.Torus{
				Center: r.center.Add(r.n.Scale(m.zC)), Axis: r.n,
				Major: units.Millimeters(m.rhoC), Minor: units.Millimeters(rr),
			},
			reversed: !r.convex,
			trueArea: m.trueArea,
			rhoHi:    m.rhoHi,
			lastSag:  filletSag(spec, m.phi/float64(len(m.rows)+1)),
		}
		tag.outward = func(p r3.Vec) r3.Vec {
			d := p.Sub(r.at(m.rhoC, r.angle(p), m.zC))
			if !r.convex {
				d = d.Scale(-1)
			}
			return d
		}
		for _, row := range m.rows {
			tag.rows = append(tag.rows, func(phi float64) r3.Vec { return r.at(row[0], phi, row[1]) })
		}
		return tag
	}
	// The cone's Origin is its narrower circle and its radius grows along Axis
	// toward the wider one.
	half := math.Atan(math.Abs(m.rhoB-m.rhoA) / math.Abs(m.zB))
	cone := surfacegeom.Cone{HalfAngle: units.Radians(half)}
	if m.rhoA < m.rhoB {
		cone.Origin, cone.Axis, cone.Radius = r.center, r.n.Scale(r.side), units.Millimeters(m.rhoA)
	} else {
		cone.Origin, cone.Axis, cone.Radius = r.center.Add(r.n.Scale(m.zB)), r.n.Scale(-r.side), units.Millimeters(m.rhoB)
	}
	outward := func(p r3.Vec) r3.Vec {
		radial := p.Sub(r.center)
		radial = radial.Sub(r.n.Scale(radial.Dot(r.n)))
		unit, ok := radial.Normalize()
		if !ok {
			unit = r.u
		}
		return unit.Scale(m.outwardMR[0]).Add(r.n.Scale(m.outwardMR[1]))
	}
	natural := r.u.Scale(math.Cos(half)).Sub(cone.Axis.Scale(math.Sin(half)))
	return blendTag{
		surface:  cone,
		reversed: outward(r.at(m.rhoA, 0, 0)).Dot(natural) < 0,
		outward:  outward,
		trueArea: m.trueArea,
		rhoHi:    m.rhoHi,
	}
}

// rewriteSphereRim blends one plane-sphere rim and builds the body over the
// rewritten mesh: the plane loses the annulus out to its setback circle, the
// sphere the zone down to its setback circle, and the strip of rows joins
// them (§4.5, §5).
func rewriteSphereRim(ctx context.Context, env topo.Env, fp facetedrecord.Payload, s sphereRim,
	spec blendSpec, role string, ord int, faceIndex map[*topo.Face]int) (*topo.Body, error) {
	if err := s.checkSize(spec); err != nil {
		return nil, err
	}
	w := newWork(fp, s.rim, spec)
	// The rows and the plane's polygon each keep their term within all of
	// tau, so a fillet's rows state twice it (§6.3, Strip triangles): split as
	// the other rims are, the grid folds the band on a slice's lower rim.
	w.share = w.tau
	if spec.fillet {
		w.rowTau = 2 * w.tau
	}
	m, err := s.section(spec, w.tau)
	if err != nil {
		return nil, err
	}
	planeTag, _ := retainedPlane(s.planeFace)
	ringA, groupA, err := w.rewritePlane(faceIndex[s.planeFace], planeTag, math.Abs(m.rhoA-s.radius), m.rhoHi)
	if err != nil {
		return nil, err
	}
	// The wall's cut vertices lie on the sphere's held facets, which the cut
	// plane meets at a slant (cutOffset).
	w.sphereWall = &sphereWall{radius: s.sphereRadius, zS: s.height(s.sphere), rhoB: m.rhoB, zB: m.zB}
	ringB, groupB, err := w.rewriteWall(faceIndex[s.wallFace], s.side*m.zB, s.sphereRadius,
		func(p r3.Vec) float64 { return m.dist(s.meridianOf(p)) })
	if err != nil {
		return nil, err
	}
	return w.finishRim(ctx, env, s.tag(spec, m), ringA, ringB, [2]int{groupA, groupB}, role, ord, faceIndex)
}

// meridianOf is p in the rim's meridian coordinates: its distance from the
// axis and its height above the plane.
func (s sphereRim) meridianOf(p r3.Vec) (float64, float64) {
	return s.radial(p), s.height(p)
}

// dist is the distance from the meridian point (rho, z) to the blend's
// profile: the fillet's arc from T_A to T_B about the ball's centre, or the
// chamfer's segment. A surface of revolution is as far from a point as its
// profile is from the point's meridian image.
func (m meridian) dist(rho, z float64) float64 {
	ta, tb := [2]float64{m.rhoA, m.zA}, [2]float64{m.rhoB, m.zB}
	ends := math.Min(math.Hypot(rho-ta[0], z-ta[1]), math.Hypot(rho-tb[0], z-tb[1]))
	if m.r == 0 {
		return segmentDist2(rho, z, ta, tb)
	}
	// The arc spans less than π, so a direction from the centre lies on it
	// when it turns from a0 the same way as a1 and not past a1.
	c := [2]float64{m.rhoC, m.zC}
	a0 := [2]float64{ta[0] - c[0], ta[1] - c[1]}
	a1 := [2]float64{tb[0] - c[0], tb[1] - c[1]}
	d := [2]float64{rho - c[0], z - c[1]}
	cross := func(u, v [2]float64) float64 { return u[0]*v[1] - u[1]*v[0] }
	turn := cross(a0, a1)
	if cross(a0, d)*turn >= 0 && cross(d, a1)*turn >= 0 && d[0]*(a0[0]+a1[0])+d[1]*(a0[1]+a1[1]) > 0 {
		return math.Abs(math.Hypot(d[0], d[1]) - m.r)
	}
	return ends
}

// segmentDist2 is the distance from (x, y) to the segment ab of the plane.
func segmentDist2(x, y float64, a, b [2]float64) float64 {
	dx, dy := b[0]-a[0], b[1]-a[1]
	t := 0.0
	if l := dx*dx + dy*dy; l > 0 {
		t = math.Min(1, math.Max(0, ((x-a[0])*dx+(y-a[1])*dy)/l))
	}
	return math.Hypot(x-a[0]-t*dx, y-a[1]-t*dy)
}
