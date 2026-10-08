package apitest_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

// This file pins docs/evaluator-design.md §6's wall-normal rule: a revolve
// wall's Face.NormalAt result, grown by its own published bound, encloses the
// normal of the surface the record DENOTES — the recorded meridian segment
// rotated about the recorded axis — at the queried point. The face's Surface
// tag is a float re-expression of that surface: its axis coordinates are
// rounded, a near-axis centre is snapped onto the axis, a near-parallel or
// near-perpendicular segment is tagged Cylinder or Plane, and a cone's half
// angle is a float atan2. A bound proven against the tag alone covers none of
// that.
//
// The oracle below reads only the recorded sketch coordinates and evaluates
// the denoted normal in 512-bit big.Float arithmetic, so its own error is
// some 140 orders below every bound it is compared with.

const normalOraclePrec = 512

// normalOracleAxis is a recorded sketch-line axis on the world XY plane.
type normalOracleAxis struct{ start, end [2]float64 }

// normalOracleWall is one recorded meridian segment: the line p0→p1, or the
// circle about centre c.
type normalOracleWall struct {
	circular bool
	p0, p1   [2]float64
	c        [2]float64
}

type onVec [3]*big.Float

func onF(x float64) *big.Float { return new(big.Float).SetPrec(normalOraclePrec).SetFloat64(x) }

func onOf(v r3.Vec) onVec { return onVec{onF(v.X), onF(v.Y), onF(v.Z)} }

func onPlane(p [2]float64) onVec { return onVec{onF(p[0]), onF(p[1]), onF(0)} }

func (a onVec) sub(b onVec) onVec {
	var out onVec
	for i := range out {
		out[i] = new(big.Float).SetPrec(normalOraclePrec).Sub(a[i], b[i])
	}
	return out
}

func (a onVec) add(b onVec) onVec {
	var out onVec
	for i := range out {
		out[i] = new(big.Float).SetPrec(normalOraclePrec).Add(a[i], b[i])
	}
	return out
}

func (a onVec) scale(s *big.Float) onVec {
	var out onVec
	for i := range out {
		out[i] = new(big.Float).SetPrec(normalOraclePrec).Mul(a[i], s)
	}
	return out
}

func (a onVec) dot(b onVec) *big.Float {
	sum := new(big.Float).SetPrec(normalOraclePrec)
	for i := range a {
		sum.Add(sum, new(big.Float).SetPrec(normalOraclePrec).Mul(a[i], b[i]))
	}
	return sum
}

func (a onVec) norm() *big.Float { return new(big.Float).SetPrec(normalOraclePrec).Sqrt(a.dot(a)) }

func (a onVec) unit() onVec {
	inv := new(big.Float).SetPrec(normalOraclePrec).Quo(onF(1), a.norm())
	return a.scale(inv)
}

// denotedNormal is the unit normal, up to sign, of the recorded wall rotated
// about the recorded axis, at p: a straight wall's normal lies in p's own
// meridian half-plane perpendicular to the rotated segment, and a circular
// wall's runs from the rotated circle's centre at p's azimuth to p.
func denotedNormal(ax normalOracleAxis, wall normalOracleWall, p r3.Vec) onVec {
	return denotedNormalShifted(ax, wall, r3.Vec{}, p)
}

// denotedNormalShifted is denotedNormal for the record translated by shift,
// added exactly.
func denotedNormalShifted(ax normalOracleAxis, wall normalOracleWall, shift r3.Vec, p r3.Vec) onVec {
	sh := onOf(shift)
	at := func(q [2]float64) onVec { return onPlane(q).add(sh) }
	a := at(ax.start)
	w := at(ax.end).sub(a).unit()
	bp := onOf(p)
	rel := bp.sub(a)
	rhat := rel.sub(w.scale(rel.dot(w))).unit()
	if wall.circular {
		c := at(wall.c)
		foot := a.add(w.scale(c.sub(a).dot(w)))
		tube := foot.add(rhat.scale(c.sub(foot).norm()))
		return bp.sub(tube).unit()
	}
	// The in-plane perpendicular toward the profile's own side carries the
	// segment's radial run, so the rotated segment's tangent in p's
	// half-plane is dz·W + dr·r̂.
	e := onVec{new(big.Float).Neg(w[1]), w[0], onF(0)}
	side := at(wall.p0).sub(a)
	if side.dot(e).Sign() == 0 {
		side = at(wall.p1).sub(a)
	}
	if side.dot(e).Sign() < 0 {
		e = e.scale(onF(-1))
	}
	d := at(wall.p1).sub(at(wall.p0))
	dz, dr := d.dot(w), d.dot(e)
	return w.scale(dr).sub(rhat.scale(dz)).unit()
}

// normalResidual is the Euclidean distance from held to whichever sign of
// truth it points along, rounded up into a float64.
func normalResidual(held r3.Vec, truth onVec) float64 {
	h := onOf(held)
	if h.dot(truth).Sign() < 0 {
		truth = truth.scale(onF(-1))
	}
	f, acc := h.sub(truth).norm().Float64()
	if acc == big.Below {
		f = math.Nextafter(f, math.Inf(1))
	}
	return f
}

// axisSample places the axis-frame point (z, ρ) at sweep angle φ: along the
// unit axis from its start, out along the in-plane perpendicular toward
// side, swept toward world +Z. It is a float sample point only; the oracle
// never reads it as geometry.
func axisSample(ax normalOracleAxis, side [2]float64, z, rho, phi float64) r3.Vec {
	a := r3.NewVec(ax.start[0], ax.start[1], 0)
	w, _ := r3.NewVec(ax.end[0]-ax.start[0], ax.end[1]-ax.start[1], 0).Normalize()
	e := r3.NewVec(-w.Y, w.X, 0)
	if r3.NewVec(side[0], side[1], 0).Sub(a).Dot(e) < 0 {
		e = e.Scale(-1)
	}
	out := w.Cross(e)
	return a.Add(w.Scale(z)).Add(e.Scale(rho * math.Cos(phi)).Add(out.Scale(rho * math.Sin(phi))))
}

// axisCoords re-expresses a recorded plane point in float axis coordinates,
// for building sample points.
func axisCoords(ax normalOracleAxis, side, p [2]float64) (float64, float64) {
	a := r3.NewVec(ax.start[0], ax.start[1], 0)
	w, _ := r3.NewVec(ax.end[0]-ax.start[0], ax.end[1]-ax.start[1], 0).Normalize()
	e := r3.NewVec(-w.Y, w.X, 0)
	if r3.NewVec(side[0], side[1], 0).Sub(a).Dot(e) < 0 {
		e = e.Scale(-1)
	}
	rel := r3.NewVec(p[0], p[1], 0).Sub(a)
	return rel.Dot(w), rel.Dot(e)
}

// wallSamples are points spread over the rotated wall: three stations along
// a straight wall, or three along a circular one's half-turn, each at four
// sweep angles.
func wallSamples(ax normalOracleAxis, side [2]float64, wall normalOracleWall, radius float64) []r3.Vec {
	var out []r3.Vec
	for _, t := range []float64{0.2, 0.5, 0.8} {
		var z, rho float64
		if wall.circular {
			cz, crho := axisCoords(ax, side, wall.c)
			th := math.Pi * t
			z, rho = cz+radius*math.Cos(th), crho+radius*math.Sin(th)
		} else {
			z0, r0 := axisCoords(ax, side, wall.p0)
			z1, r1 := axisCoords(ax, side, wall.p1)
			z, rho = z0+(z1-z0)*t, r0+(r1-r0)*t
		}
		for _, phi := range []float64{0, 0.7, 2.1, 4} {
			out = append(out, axisSample(ax, side, z, rho, phi))
		}
	}
	return out
}

// requireWallNormalsEnclose finds the one face of b whose surface kind is
// kind and passes pick, and requires every sample's NormalAt result to sit
// within its own published bound of the denoted normal. It returns the worst
// residual, so a caller can show the fixture genuinely departs from its tag.
func requireWallNormalsEnclose(t *testing.T, b *decad.Body, kind decad.SurfaceKind, pick func(decad.Surface) bool, ax normalOracleAxis, side [2]float64, wall normalOracleWall, radius float64) float64 {
	t.Helper()
	var face *decad.Face
	for _, f := range b.Faces() {
		if f.Surface().Kind() == kind && (pick == nil || pick(f.Surface())) {
			require.Nil(t, face, "the fixture must hold one matching wall")
			face = f
		}
	}
	require.NotNil(t, face, "no %v wall in the body", kind)
	worst := 0.0
	for _, p := range wallSamples(ax, side, wall, radius) {
		n, err := face.NormalAt(p)
		require.NoError(t, err)
		res := normalResidual(n.Value, denotedNormal(ax, wall, p))
		require.Less(t, res, 1e-6, "the reading must point along the denoted normal at %v", p)
		require.Less(t, n.Bound.Base(), 1e-6, "the bound must stay a rounding-scale figure")
		require.LessOrEqualf(t, res, n.Bound.Base(),
			"the %v wall's normal at %v sits %g off the denoted normal, outside its bound %g", kind, p, res, n.Bound.Base())
		worst = math.Max(worst, res)
	}
	return worst
}

// circleSketch draws a half-disk: the chord from (cu−r, cv) to (cu+r, cv)
// and the arc bulging to +v about (cu, cv).
func circleSketch(t *testing.T, cu, cv, r float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(cu-r, cv)
	s.Fix(o)
	end := s.CreatePoint(cu+r, cv)
	c := s.CreatePoint(cu, cv)
	s.CreateLine(o, end)
	s.CreateArc(c, end, o)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	return s, s.Profiles()[0]
}

func revolveAbout(t *testing.T, s *sketch.Sketch, p *sketch.Profile, ax normalOracleAxis) *decad.Body {
	t.Helper()
	b, err := decad.New().Revolve(s, p,
		decad.SketchLine{Start: decad.Point2{U: ax.start[0], V: ax.start[1]}, End: decad.Point2{U: ax.end[0], V: ax.end[1]}},
		decad.FullRevolution{})
	require.NoError(t, err)
	return b
}

func radiusNear(want float64) func(decad.Surface) bool {
	return func(s decad.Surface) bool {
		c, ok := s.(decad.Cylinder)
		return ok && math.Abs(c.Radius.Base()-want) < 1e-3
	}
}

// TestRevolveWallNormalCoversNearParallelSegment revolves a quadrilateral
// whose top side climbs 5e-10 over a unit run: inside the 1e-9 slope the
// classifier reads as parallel, so the wall is tagged a Cylinder, while the
// record denotes a cone of half angle 5e-10.
//
// Shown to fail: before the denoted-normal comparison the tagged Cylinder's
// normal published Exact (bound 0) 5e-10 off the denoted normal.
func TestRevolveWallNormalCoversNearParallelSegment(t *testing.T) {
	t.Parallel()
	pts := [][2]float64{{0, 1}, {1, 1}, {1, 2}, {0, 2 + 5e-10}}
	s, p := polygonSketch(t, pts)
	ax := normalOracleAxis{end: [2]float64{1, 0}}
	b := revolveAbout(t, s, p, ax)
	worst := requireWallNormalsEnclose(t, b, decad.KindCylinder, radiusNear(2), ax, pts[1],
		normalOracleWall{p0: pts[2], p1: pts[3]}, 0)
	require.Greater(t, worst, 1e-10, "the fixture must genuinely depart from its tag")
}

// TestRevolveWallNormalCoversNearPerpendicularSegment is the same departure
// on a side leaning 5e-10 off perpendicular, tagged a Plane.
//
// Shown to fail: before the denoted-normal comparison the tagged Plane's
// normal published Exact 5e-10 off the denoted normal.
func TestRevolveWallNormalCoversNearPerpendicularSegment(t *testing.T) {
	t.Parallel()
	pts := [][2]float64{{0, 1}, {1, 1}, {1 + 5e-10, 2}, {0, 2}}
	s, p := polygonSketch(t, pts)
	ax := normalOracleAxis{end: [2]float64{1, 0}}
	b := revolveAbout(t, s, p, ax)
	pick := func(sf decad.Surface) bool { pl, ok := sf.(decad.Plane); return ok && pl.Frame.Origin().X > 0.5 }
	worst := requireWallNormalsEnclose(t, b, decad.KindPlane, pick, ax, pts[1],
		normalOracleWall{p0: pts[1], p1: pts[2]}, 0)
	require.Greater(t, worst, 1e-10, "the fixture must genuinely depart from its tag")
}

// TestRevolveWallNormalCoversSnappedSphereCentre revolves a half-disk whose
// chord and centre sit 5e-10 off the axis, inside the contact tolerance: the
// chord snaps onto the axis and the arc is tagged a Sphere about an on-axis
// centre, while the record denotes the circle rotated about an axis 5e-10
// from its centre.
//
// Shown to fail: before the denoted-normal comparison the Sphere's normal
// published a 2.3e-16 bound 8.1e-11 off the denoted normal. Reading the
// centre as on the axis in the denoted surface (dropping
// surfacenormal.Revolved's off-axis centre term) turns it red again.
func TestRevolveWallNormalCoversSnappedSphereCentre(t *testing.T) {
	t.Parallel()
	const near = 5e-10
	s, p := circleSketch(t, 5, near, 5)
	ax := normalOracleAxis{end: [2]float64{1, 0}}
	b := revolveAbout(t, s, p, ax)
	worst := requireWallNormalsEnclose(t, b, decad.KindSphere, nil, ax, [2]float64{5, 1},
		normalOracleWall{circular: true, c: [2]float64{5, near}}, 5)
	require.Greater(t, worst, 1e-11, "the fixture must genuinely depart from its tag")
}

// farAxis runs along world X with its recorded start a million millimetres
// back along it, so every axial coordinate the build reads is a difference
// rounded at ulp(10⁶) and every centre it places is rounded again there.
var farAxis = normalOracleAxis{start: [2]float64{-1e6 - 0.3, 0}, end: [2]float64{-1e6 + 0.7, 0}}

// TestRevolveWallNormalCoversFarAnchoredSphere revolves a half-disk about
// farAxis: the Sphere tag's centre is placed from the rounded axial
// coordinate, about 1e-10 mm off the recorded centre.
//
// Shown to fail: before the denoted-normal comparison the Sphere's normal
// published a 1.2e-16 bound 2.7e-12 off the denoted normal.
func TestRevolveWallNormalCoversFarAnchoredSphere(t *testing.T) {
	t.Parallel()
	s, p := circleSketch(t, 5.1, 0, 5)
	b := revolveAbout(t, s, p, farAxis)
	worst := requireWallNormalsEnclose(t, b, decad.KindSphere, nil, farAxis, [2]float64{5, 1},
		normalOracleWall{circular: true, c: [2]float64{5.1, 0}}, 5)
	require.Greater(t, worst, 1e-13, "the fixture must genuinely depart from its tag")
}

// TestRevolveWallNormalCoversFarAnchoredTorus is the same far axis under an
// off-axis half-disk's Torus wall.
//
// Shown to fail: before the denoted-normal comparison the Torus's normal
// published a 1.1e-16 bound 2.7e-12 off the denoted normal. Dropping the
// off-axis centre term from the denoted surface turns it red again.
func TestRevolveWallNormalCoversFarAnchoredTorus(t *testing.T) {
	t.Parallel()
	s, p := circleSketch(t, 5.1, 10, 5)
	b := revolveAbout(t, s, p, farAxis)
	worst := requireWallNormalsEnclose(t, b, decad.KindTorus, nil, farAxis, [2]float64{5, 10},
		normalOracleWall{circular: true, c: [2]float64{5.1, 10}}, 5)
	require.Greater(t, worst, 1e-13, "the fixture must genuinely depart from its tag")
}

// TestRevolveWallNormalCoversFarTiltedAxis revolves a rectangle about an
// axis along (3, 4) whose recorded anchor sits 10⁵ mm back along it from the
// profile. The axis's rounded unit direction (0.6, 0.8), which the axis
// resolution bounds, swings the held axis line some 1e-11 mm off the recorded
// one at the profile, and every wall tag is placed from axial coordinates
// rounded at ulp(10⁵) in both world coordinates, so a Cylinder's own axis line
// sits off the recorded one sideways. The profile itself stays near the
// origin: sketch's own area check cannot reproduce a profile recorded at 10⁶.
//
// Shown to fail: before the denoted-normal comparison the inner Cylinder
// published a 2.2e-16 bound 5.6e-12 off the denoted normal. Dropping the
// axis's own anchor and direction bounds from the denoted surface (a zero
// revolvemesh.AxisBound in wallDenotation) turns it red again, a 2.8e-12
// bound against the same 5.6e-12.
func TestRevolveWallNormalCoversFarTiltedAxis(t *testing.T) {
	t.Parallel()
	ax := normalOracleAxis{start: [2]float64{-6e4, -8e4}, end: [2]float64{-6e4 + 3, -8e4 + 4}}
	// The axis coordinates (z, ρ) of (10⁵+1, 1), (10⁵+3, 1), (10⁵+3, 2),
	// (10⁵+1, 2) along d = (0.6, 0.8) and e = (0.8, −0.6).
	rect := [][2]float64{{1.4, 0.2}, {2.6, 1.8}, {3.4, 1.2}, {2.2, -0.4}}
	s, p := polygonSketch(t, rect)
	b := revolveAbout(t, s, p, ax)
	worst := requireWallNormalsEnclose(t, b, decad.KindCylinder, radiusNear(1), ax, rect[0],
		normalOracleWall{p0: rect[0], p1: rect[1]}, 0)
	require.Greater(t, worst, 1e-13, "the fixture must genuinely depart from its tag")
}

// requireStraightWallsEnclose samples each recorded straight wall of walls on
// the face of b, of the given kind, whose seam vertices sit at the wall's two
// recorded ends, and requires each NormalAt result to sit within its own
// bound of the denoted normal.
func requireStraightWallsEnclose(t *testing.T, b *decad.Body, kind decad.SurfaceKind, ax normalOracleAxis, side [2]float64, walls []normalOracleWall) {
	t.Helper()
	near := func(f *decad.Face, q [2]float64) bool {
		for _, e := range f.Edges() {
			for _, v := range []*decad.Vertex{e.Start(), e.End()} {
				if v.Position().Value.Sub(r3.NewVec(q[0], q[1], 0)).Len() < 1e-6 {
					return true
				}
			}
		}
		return false
	}
	for _, wall := range walls {
		var face *decad.Face
		for _, f := range b.Faces() {
			if f.Surface().Kind() == kind && near(f, wall.p0) && near(f, wall.p1) {
				require.Nil(t, face, "one face per wall")
				face = f
			}
		}
		require.NotNil(t, face, "no %v face spans %v → %v", kind, wall.p0, wall.p1)
		for _, pt := range wallSamples(ax, side, wall, 0) {
			n, err := face.NormalAt(pt)
			require.NoError(t, err)
			res := normalResidual(n.Value, denotedNormal(ax, wall, pt))
			require.Less(t, res, 1e-6)
			require.Less(t, n.Bound.Base(), 1e-6)
			require.LessOrEqualf(t, res, n.Bound.Base(), "the %v wall's normal at %v sits %g off the denoted normal, outside its bound %g", kind, pt, res, n.Bound.Base())
		}
	}
}

// TestRevolveWallNormalCoversConeAngle revolves frustums whose leaning sides
// are tagged Cones with a float atan2 half angle of their re-expressed runs.
// Neither the angle's own rounding nor the decimal runs' was seen to break the
// tag's own bound — a scan of 240 decimal frustums peaked at 0.81 of it — so
// this pins only that the denoted-surface bound still covers every Cone.
func TestRevolveWallNormalCoversConeAngle(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name string
		pts  [][2]float64
	}{
		{"integer", [][2]float64{{0, 1}, {4, 1}, {3, 2}, {1, 2}}},
		{"decimal", [][2]float64{{0.1, 0.2}, {1.1, 0.2}, {0.7, 0.9}, {0.3, 0.9}}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			s, p := polygonSketch(t, tc.pts)
			ax := normalOracleAxis{end: [2]float64{1, 0}}
			b := revolveAbout(t, s, p, ax)
			requireStraightWallsEnclose(t, b, decad.KindCone, ax, tc.pts[0],
				[]normalOracleWall{{p0: tc.pts[1], p1: tc.pts[2]}, {p0: tc.pts[3], p1: tc.pts[0]}})
		})
	}
}

// TestRevolveWallNormalCoversTiltedAxis revolves a rectangle about the
// diagonal through the origin, the fixture TestRevolveVertexBoundCoversTiltedAxis
// reads: the axis direction (√½, √½) rounds, so every side sweeps a 45° Cone
// read through a rounded direction and a float atan2 of rounded runs.
func TestRevolveWallNormalCoversTiltedAxis(t *testing.T) {
	t.Parallel()
	s := rectSketch(t, 2, 0, 3, 1)
	ax := normalOracleAxis{end: [2]float64{1, 1}}
	b := revolveAbout(t, s, s.Profiles()[0], ax)
	corners := [][2]float64{{2, 0}, {3, 0}, {3, 1}, {2, 1}}
	var walls []normalOracleWall
	for i := range corners {
		walls = append(walls, normalOracleWall{p0: corners[i], p1: corners[(i+1)%4]})
	}
	requireStraightWallsEnclose(t, b, decad.KindCone, ax, corners[0], walls)
}

// TestRevolveWallNormalSurvivesUnstitch unstitches the near-parallel wall of
// TestRevolveWallNormalCoversNearParallelSegment and places the sheet a
// million millimetres away: the copy carries the source wall's denoted
// surface under the exact placement, so its normal still covers the record's
// own departure from the Cylinder tag.
//
// Shown to fail: copying the face without its denoted surface (a nil
// Face.denotedUnder) turns both subtests red, the copy publishing the tag's
// 1.1e-16 bound 5e-10 off the denoted normal.
func TestRevolveWallNormalSurvivesUnstitch(t *testing.T) {
	t.Parallel()
	pts := [][2]float64{{0, 1}, {1, 1}, {1, 2}, {0, 2 + 5e-10}}
	s, p := polygonSketch(t, pts)
	ax := normalOracleAxis{end: [2]float64{1, 0}}
	b := revolveAbout(t, s, p, ax)
	sheets, err := b.Unstitch(t.Context())
	require.NoError(t, err)
	var sheet *decad.Body
	for _, sh := range sheets {
		fs := sh.Faces()
		require.Len(t, fs, 1)
		if radiusNear(2)(fs[0].Surface()) {
			sheet = sh
		}
	}
	require.NotNil(t, sheet)
	wall := normalOracleWall{p0: pts[2], p1: pts[3]}
	shift := r3.NewVec(1e6+0.3, 0.7, -2.5)
	tr, err := r3.Translation(shift)
	require.NoError(t, err)
	placed, err := sheet.Placed(t.Context(), tr)
	require.NoError(t, err)
	for _, tc := range []struct {
		name  string
		body  *decad.Body
		shift r3.Vec
	}{
		{"unstitched", sheet, r3.Vec{}},
		{"far placement", placed, shift},
	} {
		t.Run(tc.name, func(t *testing.T) {
			face := tc.body.Faces()[0]
			worst := 0.0
			for _, q := range wallSamples(ax, pts[1], wall, 0) {
				q = q.Add(tc.shift)
				n, err := face.NormalAt(q)
				require.NoError(t, err)
				res := normalResidual(n.Value, denotedNormalShifted(ax, wall, tc.shift, q))
				require.Less(t, n.Bound.Base(), 1e-6)
				require.LessOrEqualf(t, res, n.Bound.Base(), "the copy's normal at %v sits %g off, outside its bound %g", q, res, n.Bound.Base())
				worst = math.Max(worst, res)
			}
			require.Greater(t, worst, 1e-10)
		})
	}
}

// TestRevolveChainWallNormalCoversNearParallelSegment is the RevolveChain
// build's reading of the near-parallel departure: an open chain whose second
// segment climbs 5e-10 over a unit run sweeps a wall tagged a Cylinder.
//
// Shown to fail: before the denoted-normal comparison the chain's tagged
// Cylinder published Exact 5e-10 off the denoted normal, and building the
// chain's walls without their denoted surface turns it red again.
func TestRevolveChainWallNormalCoversNearParallelSegment(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	pts := [][2]float64{{0, 1}, {1, 1}, {2, 1 + 5e-10}}
	sp := make([]*sketch.Point, len(pts))
	for i, p := range pts {
		sp[i] = s.CreatePoint(p[0], p[1])
	}
	s.Fix(sp[0])
	for i := range len(sp) - 1 {
		s.CreateLine(sp[i], sp[i+1])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Chains(), 1)
	ax := normalOracleAxis{end: [2]float64{1, 0}}
	b, err := decad.New().RevolveChain(s, s.Chains()[0], uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	requireStraightWallsEnclose(t, b, decad.KindCylinder, ax, pts[0],
		[]normalOracleWall{{p0: pts[0], p1: pts[1]}, {p0: pts[1], p1: pts[2]}})
}

// TestRevolveWallNormalIntegerStaysExact is the exact half: an integer
// annulus and an integer half-disk about the world X axis re-express, place
// and sweep exactly, so the denoted surface IS the tag, and every normal
// whose exact direction is a coordinate axis publishes Exact with a zero
// bound — plane walls anywhere, the cylinder and the sphere where their
// radial direction is axis-aligned.
//
// Shown to fail: flooring the denoted comparison's answer at one ulp above
// zero (surfacenormal.Revolved.Allow) turns every subtest red.
func TestRevolveWallNormalIntegerStaysExact(t *testing.T) {
	t.Parallel()
	exact := func(t *testing.T, f *decad.Face, p, want r3.Vec) {
		t.Helper()
		n, err := f.NormalAt(p)
		require.NoError(t, err)
		require.Equal(t, decad.Exact, n.Exactness, "the normal at %v", p)
		require.Zero(t, n.Bound.Base())
		require.Equal(t, want, n.Value)
	}
	t.Run("annulus", func(t *testing.T) {
		t.Parallel()
		s, p := annularSketch(t)
		b, err := decad.New().Revolve(s, p, uAxis, decad.FullRevolution{})
		require.NoError(t, err)
		for _, f := range b.Faces() {
			switch sf := f.Surface().(type) {
			case decad.Plane:
				x := sf.Frame.Origin().X
				want := r3.NewVec(1, 0, 0)
				if x == 0 {
					want = r3.NewVec(-1, 0, 0)
				}
				exact(t, f, r3.NewVec(x, 7, 3), want)
			case decad.Cylinder:
				r := sf.Radius.Base()
				want := r3.NewVec(0, 1, 0)
				if r == 5 {
					want = r3.NewVec(0, -1, 0)
				}
				exact(t, f, r3.NewVec(4, r, 0), want)
				exact(t, f, r3.NewVec(4, 0, -r), r3.NewVec(0, 0, -want.Y))
			}
		}
	})
	t.Run("sphere", func(t *testing.T) {
		t.Parallel()
		s, p := semicircleSketch(t)
		b, err := decad.New().Revolve(s, p, uAxis, decad.FullRevolution{})
		require.NoError(t, err)
		f := b.Faces()[0]
		require.Equal(t, decad.KindSphere, f.Surface().Kind())
		exact(t, f, r3.NewVec(5, 5, 0), r3.NewVec(0, 1, 0))
		exact(t, f, r3.NewVec(10, 0, 0), r3.NewVec(1, 0, 0))
		exact(t, f, r3.NewVec(5, 0, -5), r3.NewVec(0, 0, -1))
	})
}
