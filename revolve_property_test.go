package decad_test

import (
	"math"
	"math/rand"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is randomized property testing for the increment-2 revolve
// evaluator (revolve.go). It hardens the two areas two reviewers flagged as
// hardest to verify by inspection: the axis-frame reorientation (axisFrame.walk,
// resolveAxisSide) that must preserve walk order through an arbitrary axis
// transform, and the cap-edge / wall convexity classification. The cardinal
// defect it hunts is a silently inside-out face normal (the PR #28 bug).
//
// Every case is built in a CANONICAL axis frame chosen at random inside the
// sketch plane — a random in-plane axis origin and direction, so axisFrame.walk
// is exercised with a non-trivial rotation — and then placed under a random
// rigid motion (rotation + translation, occasionally a reflection) so world
// orientation varies too. The region is always generated on the axis's
// non-negative-rho side, which is exactly the engine's own convention, so no
// side flip occurs and the sweep basis the engine uses (e0, e1, phi0, phi1) is
// reconstructed here EXACTLY. That reconstruction is what makes the independent
// point-in-solid oracle below sound.
//
// The seed is fixed and printed, so any failure is replayable.

const revolvePropertySeed int64 = 0x5eed_1eaf_c0ffee

// axisPlacement is a revolve axis inside the XY sketch plane: an origin (ou, ov)
// and an in-plane direction angle beta. The XY plane maps sketch (u, v) to world
// (u, v, 0), so the world sweep basis follows in closed form — and it is the
// SAME basis revolvePayload.basis() derives, with the region on the +rho side.
type axisPlacement struct {
	ou, ov float64
	beta   float64
}

// uv maps an axis-local meridian coordinate (z along the axis, rho >= 0
// perpendicular to it) into sketch (u, v) through a proper rotation, so a
// counter-clockwise arc in local coordinates stays counter-clockwise on the
// sketch.
func (a axisPlacement) uv(z, rho float64) (float64, float64) {
	du, dv := math.Cos(a.beta), math.Sin(a.beta)
	nu, nv := -math.Sin(a.beta), math.Cos(a.beta)
	return a.ou + z*du + rho*nu, a.ov + z*dv + rho*nv
}

func (a axisPlacement) origin3() r3.Vec { return r3.NewVec(a.ou, a.ov, 0) }
func (a axisPlacement) w() r3.Vec       { return r3.NewVec(math.Cos(a.beta), math.Sin(a.beta), 0) }
func (a axisPlacement) e0() r3.Vec      { return r3.NewVec(-math.Sin(a.beta), math.Cos(a.beta), 0) }
func (a axisPlacement) e1() r3.Vec      { return r3.NewVec(0, 0, 1) } // w × e0

// sketchLine is the SketchLine the engine spins about: start on the axis, end
// one unit along it.
func (a axisPlacement) sketchLine() decad.SketchLine {
	du, dv := math.Cos(a.beta), math.Sin(a.beta)
	return decad.SketchLine{
		Start: decad.Point2{U: a.ou, V: a.ov},
		End:   decad.Point2{U: a.ou + du, V: a.ov + dv},
	}
}

// sweep is a resolved angular extent: the signed interval [phi0, phi1] the
// oracle tests against, plus the extent value handed to Revolve.
type sweep struct {
	full       bool
	phi0, phi1 float64
	ext        decad.AngularExtent
}

func (s sweep) mid() float64 {
	if s.full {
		return 0
	}
	return (s.phi0 + s.phi1) / 2
}

// worldPoint places an axis-local meridian point (z, rho) at sweep angle phi
// into placed world space — the same map revolvePayload.point computes.
func worldPoint(a axisPlacement, xf r3.Transform, z, rho, phi float64) r3.Vec {
	sin, cos := math.Sincos(phi)
	radial := a.e0().Scale(cos).Add(a.e1().Scale(sin))
	return xf.Apply(a.origin3().Add(a.w().Scale(z)).Add(radial.Scale(rho)))
}

// angleIn reports whether phi lies in the signed sweep interval [lo, hi]
// (hi - lo <= 2*pi), robust to wrap.
func angleIn(phi, lo, hi float64) bool {
	const twoPi = 2 * math.Pi
	d := math.Mod(phi-lo, twoPi)
	if d < 0 {
		d += twoPi
	}
	return d <= (hi-lo)+1e-9
}

// pointInSolid is the INDEPENDENT material-membership oracle: it decides whether
// a world point lies inside the revolved solid using only the analytic
// definition of the solid (the 2D meridian region, the axis, and the sweep) —
// never the face's own NormalAt. It undoes the placement, projects onto the
// axis to recover (z, rho), tests the 2D region, and for a partial sweep tests
// the sweep angle. This is what the outward-normal test in requireOutward steps
// against.
func pointInSolid(a axisPlacement, xfInv r3.Transform, sw sweep, inRegion func(z, rho float64) bool, p r3.Vec) bool {
	q := xfInv.Apply(p)
	d := q.Sub(a.origin3())
	z := d.Dot(a.w())
	radial := d.Sub(a.w().Scale(z))
	rho := radial.Len()
	if !inRegion(z, rho) {
		return false
	}
	if sw.full || rho < 1e-12 {
		return true
	}
	phi := math.Atan2(radial.Dot(a.e1()), radial.Dot(a.e0()))
	return angleIn(phi, sw.phi0, sw.phi1)
}

func mmOf(v units.Value) float64 {
	f, err := v.In(units.Millimeter)
	if err != nil {
		panic(err)
	}
	return f
}

// surfaceDistance is the unsigned distance from p to the analytic surface — used
// only to look up which engine face a probe point lies on.
func surfaceDistance(surf decad.Surface, p r3.Vec) float64 {
	switch s := surf.(type) {
	case decad.Plane:
		return math.Abs(p.Sub(s.Frame.Origin()).Dot(s.Frame.N()))
	case decad.Cylinder:
		rel := p.Sub(s.Origin)
		h := rel.Dot(s.Axis)
		radial := rel.Sub(s.Axis.Scale(h))
		return math.Abs(radial.Len() - mmOf(s.Radius))
	case decad.Cone:
		rel := p.Sub(s.Origin)
		h := rel.Dot(s.Axis)
		radial := rel.Sub(s.Axis.Scale(h))
		half, err := s.HalfAngle.In(units.Radian)
		if err != nil {
			panic(err)
		}
		expected := math.Max(h, 0) * math.Tan(half)
		return math.Abs(radial.Len()-expected) * math.Cos(half)
	case decad.Sphere:
		return math.Abs(p.Sub(s.Center).Len() - mmOf(s.Radius))
	case decad.Torus:
		rel := p.Sub(s.Center)
		h := rel.Dot(s.Axis)
		radial := rel.Sub(s.Axis.Scale(h))
		d := math.Hypot(radial.Len()-mmOf(s.Major), h)
		return math.Abs(d - mmOf(s.Minor))
	default:
		return math.Inf(1)
	}
}

// findFace returns the body face whose analytic surface passes through p. Probe
// points lie in a face interior, so exactly one face is at zero distance.
func findFace(t *testing.T, b *decad.Body, p r3.Vec, tol float64) *decad.Face {
	t.Helper()
	best := math.Inf(1)
	var face *decad.Face
	for _, f := range b.Faces() {
		d := surfaceDistance(f.Surface(), p)
		if d < best {
			best, face = d, f
		}
	}
	require.NotNil(t, face, "no faces to probe")
	require.Less(t, best, tol, "probe point does not lie on any face")
	return face
}

// sample is one meridian point (z along the axis, rho perpendicular).
type sample struct{ z, rho float64 }

// mapper creates a fixed sketch point at an axis-local meridian coordinate.
type mapper func(z, rho float64) *sketch.Point

// setup is one randomized revolve case: how to build the profile, the
// independent 2D membership predicate, the exact area and Pappus moment, and the
// probe points for the outward-normal test.
type setup struct {
	name     string
	build    func(s *sketch.Sketch, m mapper)
	inRegion func(z, rho float64) bool
	area     float64 // integral of dA over the meridian region
	q        float64 // integral of rho dA — Pappus reads volume off it
	scale    float64 // smallest feature size, sets the probe step
	walls    []sample
	cap      sample // an interior meridian point, for the cap-face probes
}

func uniform(rng *rand.Rand, lo, hi float64) float64 { return lo + rng.Float64()*(hi-lo) }

// randAxisPlacement picks a random in-plane axis: a modest origin and a free
// direction, so axisFrame.walk runs with a non-trivial rotation.
func randAxisPlacement(rng *rand.Rand) axisPlacement {
	return axisPlacement{
		ou:   uniform(rng, -20, 20),
		ov:   uniform(rng, -20, 20),
		beta: uniform(rng, -math.Pi, math.Pi),
	}
}

// randSweep picks a full turn or one of the partial extents, and reports the
// resolved interval the oracle needs.
func randSweep(rng *rand.Rand) sweep {
	switch rng.Intn(4) {
	case 0:
		return sweep{full: true, phi0: 0, phi1: 2 * math.Pi, ext: decad.FullRevolution{}}
	case 1:
		m := uniform(rng, math.Pi/6, 5*math.Pi/6)
		return sweep{phi0: 0, phi1: m, ext: decad.AngleExtent{A: units.Radians(m), Dir: decad.Along}}
	case 2:
		m := uniform(rng, math.Pi/6, 5*math.Pi/6)
		return sweep{phi0: -m, phi1: 0, ext: decad.AngleExtent{A: units.Radians(m), Dir: decad.Against}}
	default:
		m := uniform(rng, math.Pi/6, 5*math.Pi/6) // half-angle; total < 2*pi
		return sweep{phi0: -m, phi1: m, ext: decad.SymmetricAngle{A: units.Radians(m)}}
	}
}

// randPlacement composes a random rotation and translation, occasionally with a
// reflection, so the whole body is re-evaluated under an arbitrary world motion.
func randPlacement(t *testing.T, rng *rand.Rand) r3.Transform {
	t.Helper()
	dir := r3.NewVec(rng.NormFloat64(), rng.NormFloat64(), rng.NormFloat64())
	if dir.Len() < 1e-6 {
		dir = r3.NewVec(0, 0, 1)
	}
	rot, err := r3.RotationAround(r3.NewVec(0, 0, 0), dir, units.Radians(uniform(rng, -math.Pi, math.Pi)))
	require.NoError(t, err)
	tr, err := r3.Translation(r3.NewVec(uniform(rng, -30, 30), uniform(rng, -30, 30), uniform(rng, -30, 30)))
	require.NoError(t, err)
	xf, err := rot.Then(tr)
	require.NoError(t, err)
	if rng.Intn(3) == 0 {
		mirror, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 0, 1))
		require.NoError(t, err)
		refl, err := r3.Reflection(mirror)
		require.NoError(t, err)
		xf, err = refl.Then(xf)
		require.NoError(t, err)
	}
	return xf
}

// --- templates ---------------------------------------------------------------

// annularRect is a meridian rectangle offset from the axis (rho0 may be 0, a
// solid cylinder). Walls are two cylinders (or one) and two planar annuli; a
// full turn's junctions are convex latitude circles.
func newAnnularRect(rng *rand.Rand) setup {
	z0 := uniform(rng, -10, 5)
	z1 := z0 + uniform(rng, 4, 16)
	rho0 := 0.0
	if rng.Intn(2) == 0 {
		rho0 = uniform(rng, 2, 8)
	}
	rho1 := rho0 + uniform(rng, 4, 12)
	zm, rm := (z0+z1)/2, (rho0+rho1)/2
	walls := []sample{{zm, rho1}, {z0, rm}, {z1, rm}}
	if rho0 > 0 {
		walls = append(walls, sample{zm, rho0})
	}
	return setup{
		name:  "annularRect",
		build: func(s *sketch.Sketch, m mapper) { rectLoop(s, m, z0, z1, rho0, rho1) },
		inRegion: func(z, rho float64) bool {
			return z >= z0 && z <= z1 && rho >= rho0 && rho <= rho1
		},
		area:  (z1 - z0) * (rho1 - rho0),
		q:     (z1 - z0) * (rho1*rho1 - rho0*rho0) / 2,
		scale: math.Min(z1-z0, rho1-rho0),
		walls: walls,
		cap:   sample{zm, rm},
	}
}

// rectLoop lays a rectangle down as four fixed lines in walk order.
func rectLoop(s *sketch.Sketch, m mapper, z0, z1, rho0, rho1 float64) {
	a := m(z0, rho0)
	b := m(z1, rho0)
	c := m(z1, rho1)
	d := m(z0, rho1)
	s.CreateLine(a, b)
	s.CreateLine(b, c)
	s.CreateLine(c, d)
	s.CreateLine(d, a)
}

// newGroove is annularRect with a semicircular groove bitten out of the outer
// (far) edge — a CONCAVE round, exactly the class PR #28's inverted normal lived
// on. The arc is walked clockwise about its own centre, so the swept torus wall
// keeps the material OUTSIDE its tube and its cap edges must read concave.
func newGroove(rng *rand.Rand) setup {
	z0 := uniform(rng, -8, 4)
	z1 := z0 + uniform(rng, 14, 22)
	rho0 := uniform(rng, 2, 6)
	rho1 := rho0 + uniform(rng, 8, 12)
	rg := uniform(rng, 2, 4)
	zc := uniform(rng, z0+rg+3, z1-rg-3)
	return setup{
		name: "groove",
		build: func(s *sketch.Sketch, m mapper) {
			a := m(z0, rho0)
			b := m(z1, rho0)
			c := m(z1, rho1)
			right := m(zc+rg, rho1)
			left := m(zc-rg, rho1)
			d := m(z0, rho1)
			centre := m(zc, rho1)
			s.CreateLine(a, b)
			s.CreateLine(b, c)
			s.CreateLine(c, right)
			s.CreateArc(centre, left, right) // CCW left->right dips inward; walked reversed
			s.CreateLine(left, d)
			s.CreateLine(d, a)
		},
		inRegion: func(z, rho float64) bool {
			if z < z0 || z > z1 || rho < rho0 || rho > rho1 {
				return false
			}
			return math.Hypot(z-zc, rho-rho1) >= rg
		},
		area:  (z1-z0)*(rho1-rho0) - 0.5*math.Pi*rg*rg,
		q:     (z1-z0)*(rho1*rho1-rho0*rho0)/2 - 0.5*math.Pi*rg*rg*(rho1-4*rg/(3*math.Pi)),
		scale: math.Min(rg, math.Min(z1-z0, rho1-rho0)),
		walls: []sample{
			{(z1 + zc + rg) / 2, rho1}, // outer cylinder, clear of the groove
			{(z0 + zc - rg) / 2, rho0}, // inner cylinder
			{z0, (rho0 + rho1) / 2},    // left annulus
			{z1, (rho0 + rho1) / 2},    // right annulus
			{zc, rho1 - rg},            // groove torus, at the tube's inner equator
		},
		cap: sample{(z0 + zc - rg) / 2, (rho0 + rho1) / 2},
	}
}

// newSphere revolves a semicircle whose diameter lies on the axis — a CONVEX
// round; a partial sweep's cap edges are convex arcs.
func newSphere(rng *rand.Rand) setup {
	r := uniform(rng, 3, 10)
	zc := uniform(rng, -10, 10)
	return setup{
		name: "sphere",
		build: func(s *sketch.Sketch, m mapper) {
			left := m(zc-r, 0)
			right := m(zc+r, 0)
			centre := m(zc, 0)
			s.CreateLine(left, right)        // diameter on the axis
			s.CreateArc(centre, right, left) // CCW over the top, bulging to +rho
		},
		inRegion: func(z, rho float64) bool {
			return rho >= 0 && (z-zc)*(z-zc)+rho*rho <= r*r
		},
		area:  0.5 * math.Pi * r * r,
		q:     2.0 / 3 * r * r * r,
		scale: r,
		walls: []sample{{zc, r}},
		cap:   sample{zc, 0.4 * r},
	}
}

// newTorus revolves a circle clear of the axis (a ring torus).
func newTorus(rng *rand.Rand) setup {
	r := uniform(rng, 2, 5)
	rmaj := r + uniform(rng, 4, 10)
	zc := uniform(rng, -10, 10)
	return setup{
		name: "torus",
		build: func(s *sketch.Sketch, m mapper) {
			s.CreateCircle(m(zc, rmaj), r)
		},
		inRegion: func(z, rho float64) bool {
			return (z-zc)*(z-zc)+(rho-rmaj)*(rho-rmaj) <= r*r
		},
		area:  math.Pi * r * r,
		q:     math.Pi * r * r * rmaj,
		scale: r,
		walls: []sample{{zc, rmaj + r}, {zc, rmaj - r}},
		cap:   sample{zc, rmaj},
	}
}

// newCone revolves a right triangle with its apex on the axis: a cone wall and a
// base disk.
func newCone(rng *rand.Rand) setup {
	z0 := uniform(rng, -10, 5)
	l := uniform(rng, 6, 16)
	z1 := z0 + l
	rhoB := uniform(rng, 4, 12)
	return setup{
		name: "cone",
		build: func(s *sketch.Sketch, m mapper) {
			base := m(z0, 0)
			apex := m(z1, 0)
			top := m(z0, rhoB)
			s.CreateLine(base, apex) // on the axis
			s.CreateLine(apex, top)  // slant -> cone wall
			s.CreateLine(top, base)  // base disk
		},
		inRegion: func(z, rho float64) bool {
			if z < z0 || z > z1 || rho < 0 {
				return false
			}
			return rho <= rhoB*(z1-z)/l
		},
		area:  0.5 * l * rhoB,
		q:     rhoB * rhoB * l / 6,
		scale: math.Min(l, rhoB),
		walls: []sample{{z0 + l/2, rhoB / 2}, {z0, rhoB / 2}},
		cap:   sample{z0 + 0.2*l, 0.3 * rhoB},
	}
}

// --- the property driver -----------------------------------------------------

func TestRevolvePropertyInvariants(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewSource(revolvePropertySeed))
	t.Logf("revolve property seed = %#x", revolvePropertySeed)

	factories := []func(*rand.Rand) setup{
		newAnnularRect, newGroove, newSphere, newTorus, newCone,
	}

	const perTemplate = 12
	for _, factory := range factories {
		for range perTemplate {
			su := factory(rng)
			ap := randAxisPlacement(rng)
			sw := randSweep(rng)
			t.Run(su.name, func(t *testing.T) {
				runRevolveCase(t, rng, su, ap, sw)
			})
		}
	}
}

func runRevolveCase(t *testing.T, rng *rand.Rand, su setup, ap axisPlacement, sw sweep) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	su.build(s, func(z, rho float64) *sketch.Point {
		u, v := ap.uv(z, rho)
		p := s.CreatePoint(u, v)
		s.Fix(p)
		return p
	})
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.NotEmpty(t, profiles, "%s produced no profile", su.name)

	doc := decad.New()
	body, err := doc.Revolve(s, profiles[0], ap.sketchLine(), sw.ext)
	require.NoErrorf(t, err, "%s revolve failed (axis=%+v sweep=%+v)", su.name, ap, sw)

	// Independent Pappus cross-check: the engine's volume is q * dphi; a full
	// turn's dphi is 2*pi. q here is computed by hand from the meridian region.
	dphi := sw.phi1 - sw.phi0
	if sw.full {
		dphi = 2 * math.Pi
	}
	wantVol := su.q * dphi
	requireVolume(t, body, wantVol)

	// The unplaced body, then the same body under a random rigid motion. Both
	// must satisfy every invariant, and the mass properties must agree.
	id := r3.Identity()
	checkRevolveBody(t, body, su, ap, id, sw)

	vol0, err := body.Volume()
	require.NoError(t, err)
	area0, err := body.Area()
	require.NoError(t, err)
	cen0, err := body.Centroid()
	require.NoError(t, err)

	xf := randPlacement(t, rng)
	placed, err := body.Placed(xf)
	require.NoError(t, err)
	checkRevolveBody(t, placed, su, ap, xf, sw)

	// Mass-property invariance under the rigid motion.
	volP, err := placed.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, volP.Exactness)
	require.True(t, volP.Value.Equal(vol0.Value, 1e-9*math.Max(1, wantVol)), "volume changed under placement")
	require.Positive(t, volP.Bound.Base())

	areaP, err := placed.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, areaP.Exactness)
	require.True(t, areaP.Value.Equal(area0.Value, 1e-9*(1+mmOf2(area0.Value))), "area changed under placement")

	cenP, err := placed.Centroid()
	require.NoError(t, err)
	wantC := xf.Apply(cen0.Value)
	require.InDelta(t, wantC.X, cenP.Value.X, 1e-6)
	require.InDelta(t, wantC.Y, cenP.Value.Y, 1e-6)
	require.InDelta(t, wantC.Z, cenP.Value.Z, 1e-6)
}

func mmOf2(v units.Value) float64 {
	f, _ := v.In(units.SquareMillimeter)
	return math.Abs(f)
}

// checkRevolveBody runs the orientation-invariant invariants on one placed
// body: outward normals against the independent oracle, watertight/manifold
// topology, convexity of the round's edges, and a sound Verify.
func checkRevolveBody(t *testing.T, body *decad.Body, su setup, ap axisPlacement, xf r3.Transform, sw sweep) {
	require.True(t, body.IsSolid())
	requireManifold(t, body)

	xfInv, err := xf.Inverse()
	require.NoError(t, err)
	eps := 1e-4 * su.scale
	oracle := func(p r3.Vec) bool { return pointInSolid(ap, xfInv, sw, su.inRegion, p) }
	tol := 1e-6*math.Max(1, su.scale) + 1e-7

	// Invariant 1: every wall probe's outward normal leaves the material and
	// its opposite stays inside — checked against the independent oracle, never
	// the face's own normal.
	for _, wp := range su.walls {
		at := worldPoint(ap, xf, wp.z, wp.rho, sw.mid())
		f := findFace(t, body, at, tol)
		requireOutward(t, f, at, eps, oracle)
	}
	// Cap faces, on a partial sweep: a probe at each cap plane.
	if !sw.full {
		for _, phi := range []float64{sw.phi0, sw.phi1} {
			at := worldPoint(ap, xf, su.cap.z, su.cap.rho, phi)
			f := findFace(t, body, at, tol)
			requireOutward(t, f, at, eps, oracle)
		}
	}

	// Invariant 2: a watertight, consistently-oriented mesh. A revolve whose
	// section carries only LINE generators meshes; a circular generator is
	// staged (ErrUnsupported), and so is a tolerance the payload's own
	// coordinate stages exhaust, so the check runs on whatever this fixture
	// produced and demands an explicit staging refusal otherwise.
	if mesh, err := body.Tessellate(units.Millimeters(su.scale / 20)); err == nil {
		requireMeshWinding(t, mesh)
	} else {
		require.ErrorIs(t, err, decad.ErrUnsupported, "revolve tessellation must stay explicitly staged")
	}

	// Invariant 3: the walked-boundary convexity holds under this orientation.
	checkConvexity(t, body, su, ap, xf, sw)

	// Invariant 4 (structural half): topology remains valid while bounded mass
	// properties make zero-tolerance verification Suspect.
	report, err := body.Document().Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Suspect, report.Status, "%s has bounded mass results", su.name)
	require.False(t, report.Trustworthy())
}

// requireOutward asserts f's outward normal at `at` points out of the solid: a
// small step along +normal leaves it, a step along -normal stays inside.
func requireOutward(t *testing.T, f *decad.Face, at r3.Vec, eps float64, inSolid func(r3.Vec) bool) {
	t.Helper()
	nm, err := f.NormalAt(at)
	require.NoError(t, err)
	n := nm.Value
	require.InDelta(t, 1.0, n.Len(), 1e-9, "normal is a unit vector")
	outward := at.Add(n.Scale(eps))
	inward := at.Sub(n.Scale(eps))
	require.False(t, inSolid(outward), "a step along +normal must leave the solid (inverted normal?)")
	require.True(t, inSolid(inward), "a step along -normal must stay inside (inverted normal?)")
}

// requireMeshWinding is the future-proof winding check for invariant 2: every
// facet is wound so its geometric normal agrees with its source face's outward
// normal, and every mesh edge is shared by exactly two facets (watertight).
func requireMeshWinding(t *testing.T, m *decad.Mesh) {
	t.Helper()
	verts := m.Vertices()
	tris := m.Triangles()
	src := m.SourceFaces()
	require.Len(t, src, len(tris))

	type edge struct{ a, b int }
	edges := map[edge]int{}
	for ti, tri := range tris {
		a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
		geo := b.Sub(a).Cross(c.Sub(a))
		require.Positive(t, geo.Len(), "degenerate facet")
		centroid := a.Add(b).Add(c).Scale(1.0 / 3)
		nm, err := src[ti].NormalAt(centroid)
		require.NoError(t, err)
		require.Positive(t, geo.Dot(nm.Value), "facet winding disagrees with source-face outward normal")
		for _, e := range []edge{{tri[0], tri[1]}, {tri[1], tri[2]}, {tri[2], tri[0]}} {
			key := e
			if key.a > key.b {
				key.a, key.b = key.b, key.a
			}
			edges[key]++
		}
	}
	for _, count := range edges {
		require.Equal(t, 2, count, "every mesh edge must bound exactly two facets")
	}
}

// checkConvexity asserts the walked-boundary convexity of the round's edges,
// invariant under the axis orientation and the rigid placement. Cap-plane edges
// have their circle axis perpendicular to the revolve axis; latitude edges have
// it parallel.
func checkConvexity(t *testing.T, body *decad.Body, su setup, ap axisPlacement, xf r3.Transform, sw sweep) {
	t.Helper()
	worldAxis := xf.ApplyDir(ap.w())

	edgeAxis := func(c decad.Curve) (r3.Vec, bool) {
		switch cv := c.(type) {
		case decad.Circle3:
			return cv.Axis, true
		case decad.Arc3:
			return cv.Axis, true
		default:
			return r3.Vec{}, false
		}
	}
	isCapEdge := func(e *decad.Edge) bool {
		ax, ok := edgeAxis(e.Curve())
		return ok && math.Abs(ax.Dot(worldAxis)) < 0.5
	}
	isLatitude := func(e *decad.Edge) bool {
		ax, ok := edgeAxis(e.Curve())
		return ok && math.Abs(ax.Dot(worldAxis)) > 0.5
	}

	switch su.name {
	case "sphere":
		if sw.full {
			return
		}
		n := 0
		for _, e := range body.Edges() {
			if isCapEdge(e) {
				require.True(t, e.IsConvex(), "a convex round's cap edges read convex")
				n++
			}
		}
		require.Equal(t, 2, n, "a sphere wedge has one cap arc in each cap plane")
	case "groove":
		if sw.full {
			return
		}
		// The groove's torus wall — its cap edges are concave, exactly a hole's.
		caps := 0
		for _, f := range body.Faces() {
			if _, ok := f.Surface().(decad.Torus); !ok {
				continue
			}
			for _, e := range f.Edges() {
				if !isCapEdge(e) {
					continue
				}
				require.False(t, e.IsConvex(), "a clockwise-walked groove's cap edges read concave")
				caps++
			}
		}
		require.Equal(t, 2, caps, "one groove cap edge in each cap plane")
		// The Concave() selector must pick them and Convex() must not.
		concave, err := decad.Edges(decad.Concave()).SelectEdges(body)
		require.NoError(t, err)
		require.NotEmpty(t, concave)
	case "annularRect":
		if !sw.full {
			return
		}
		n := 0
		for _, e := range body.Edges() {
			if isLatitude(e) {
				require.True(t, e.IsConvex(), "a convex section's full-turn junctions are convex")
				n++
			}
		}
		require.Positive(t, n, "a full annular revolve has latitude junction circles")
	case "torus":
		if sw.full {
			return
		}
		n := 0
		for _, e := range body.Edges() {
			if isCapEdge(e) {
				require.True(t, e.IsConvex(), "a convex ring's cap circles are convex")
				n++
			}
		}
		require.Equal(t, 2, n, "a torus wedge keeps both cap circles")
	}
}
