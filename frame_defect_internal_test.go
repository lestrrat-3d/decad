package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These fixtures pin docs/evaluator-design.md §5.1's frame charge for every
// prism-family build and face copy, §6's revolve box basis term and
// docs/multibody-dynamics-design.md §8.6's affine revolve mass. Each builds a
// body on the exact XY frame under the identity as its reference, then the
// same record on a tilted plane, under a rotation, or both. The second body
// denotes the first carried through L = B·[U V N], every leaf read exactly,
// so a reading x1 ± b1 provably misses when |x1 − f·x0| − f·b0 > b1, with x0
// ± b0 the reference reading and f L's exact factor for it: |det L| for a
// volume, |L·a × L·b| for a planar face spanned by a, b, |L·d| for a line
// along d. Legs shown to fail by deleting them and watching this test go red,
// then restoring them (on amd64):
//
//   - chargePrismMap in evalPrismContext: the prism, mirror, edge fillet and
//     line sweep went red in every variant, publishing their volume, areas
//     and lengths Exact, and so did the tilted unstitch and body patch whose
//     source sheet is a prism;
//   - in measureDraftBody, evalCapBlendContext, evalPatchContext and
//     evalChainExtrudeContext: the draft, the cap chamfer, the patch and the
//     chain extrude went red in every variant;
//   - in evalStackedPlanContext: the union, the blind cut and the cup went
//     red in every variant;
//   - in evalBrepContext: the rotated drilled block went red;
//   - chargePlacement in evalUnstitchFaceContext and the copy charge in
//     copyPatchFacesUnder: the rotated unstitch and body patch went red;
//   - massmoment.AffineInertia in revolveMassProperties, replaced by the
//     rigid path: TestRevolveMassCoversFrameDefect went red;
//   - the basis term of revolveaxis.FrameRoundAllow:
//     TestRevolveBoxChargesTheBasisRounding went red;
//   - buildPatchFace's fitted-frame charge: the tilted and the rotated body
//     patch went red;
//   - patchPolygonAreaBound in buildPatchFace: the thin body patch's fitted
//     face missed in every variant, by up to 870× its bound;
//   - patchCurvedAreaCharge in buildPatchFace: the thin slot body patch's
//     fitted face missed by 74× its bound on the tilted plane, and its
//     straight-edge leg alone by 12×. Its circular legs (the curve bound,
//     the lifted circle's gap, the end matching) were each deleted without
//     any fixture here going red: a circle's area does not move with its
//     centre, and the π enclosure's own width already covers what its radius
//     terms charge on these rims;
//   - the chain revolve's latitude curve bound and the revolve's cap-arc
//     curve bound: the revolve latitude and cap circle patches refused with
//     R46. A revolve junction arc and a cap blend's arcs carry vertex bounds
//     Body.Patch refuses first (R6), so TestCurveBoundsCoverMappedReference
//     checks their curve bounds directly;
//   - the endpoint-support arm of auditAdjacentSweepSpans: the rotated
//     composite sweep refused as "not certified on opposite sides".
//
// Every expected factor reads the frame the payload records
// (fdRecordedFrame), not the sketch plane's held frame: Extrude and its
// siblings normalize the plane's axes once more, and the two differ by a few
// ulps, which reads as a miss of up to 1.23× on a fitted patch face.

const frameDefectPrec = 512

// fdTilted names the variant built on the tilted plane under the identity.
const fdTilted = "tilted"

func fdf(x float64) *big.Float { return new(big.Float).SetPrec(frameDefectPrec).SetFloat64(x) }

type fdVec [3]*big.Float

func fdVecOf(v r3.Vec) fdVec { return fdVec{fdf(v.X), fdf(v.Y), fdf(v.Z)} }

func (a fdVec) dot(b fdVec) *big.Float {
	s := new(big.Float).SetPrec(frameDefectPrec)
	for i := range 3 {
		s.Add(s, new(big.Float).SetPrec(frameDefectPrec).Mul(a[i], b[i]))
	}
	return s
}

func (a fdVec) cross(b fdVec) fdVec {
	c := func(i, j int) *big.Float {
		x := new(big.Float).SetPrec(frameDefectPrec).Mul(a[i], b[j])
		return x.Sub(x, new(big.Float).SetPrec(frameDefectPrec).Mul(a[j], b[i]))
	}
	return fdVec{c(1, 2), c(2, 0), c(0, 1)}
}

func (a fdVec) norm() *big.Float { n := a.dot(a); return n.Sqrt(n) }

func (a fdVec) scale(s *big.Float) fdVec {
	m := func(x *big.Float) *big.Float { return new(big.Float).SetPrec(frameDefectPrec).Mul(x, s) }
	return fdVec{m(a[0]), m(a[1]), m(a[2])}
}

func (a fdVec) add(b fdVec) fdVec {
	m := func(x, y *big.Float) *big.Float { return new(big.Float).SetPrec(frameDefectPrec).Add(x, y) }
	return fdVec{m(a[0], b[0]), m(a[1], b[1]), m(a[2], b[2])}
}

func (a fdVec) unit() fdVec {
	return a.scale(new(big.Float).SetPrec(frameDefectPrec).Quo(fdf(1), a.norm()))
}

// fdMap is L = B·[U V N], N the held frame.N() the prism family lifts through.
type fdMap struct{ cols [3]fdVec }

func fdMapOf(frame r3.Frame, place r3.Transform) fdMap {
	return fdMapThrough(frame, place, false)
}

// fdMapThrough is fdMapOf with the third column the exact U×V where
// exactCross is set: the map a revolve denotes through
// (docs/evaluator-design.md §6).
func fdMapThrough(frame r3.Frame, place r3.Transform, exactCross bool) fdMap {
	b := place.Basis()
	ex, ey, ez := fdVecOf(b.EX), fdVecOf(b.EY), fdVecOf(b.EZ)
	lin := func(v fdVec) fdVec { return ex.scale(v[0]).add(ey.scale(v[1])).add(ez.scale(v[2])) }
	u, v := fdVecOf(frame.U()), fdVecOf(frame.V())
	n := fdVecOf(frame.N())
	if exactCross {
		n = u.cross(v)
	}
	return fdMap{cols: [3]fdVec{lin(u), lin(v), lin(n)}}
}

func (m fdMap) apply(x fdVec) fdVec {
	return m.cols[0].scale(x[0]).add(m.cols[1].scale(x[1])).add(m.cols[2].scale(x[2]))
}

func (m fdMap) volumeFactor() *big.Float {
	return new(big.Float).Abs(m.cols[0].dot(m.cols[1].cross(m.cols[2])))
}

// areaFactor is |L·a × L·b| for an orthonormal pair spanning the plane with
// normal n.
func (m fdMap) areaFactor(n fdVec) *big.Float {
	nn := n.unit()
	e := fdVec{fdf(1), fdf(0), fdf(0)}
	if x, _ := new(big.Float).Abs(nn[0]).Float64(); x > 0.5 {
		e = fdVec{fdf(0), fdf(1), fdf(0)}
	}
	a := nn.cross(e).unit()
	return m.apply(a).cross(m.apply(nn.cross(a))).norm()
}

func (m fdMap) lengthFactor(d fdVec) *big.Float { return m.apply(d.unit()).norm() }

// fdMissRatio is (|x1 − f·x0| − f·b0) / b1: above 1 proves x1 ± b1 misses.
func fdMissRatio(x1, b1, x0, b0 float64, f *big.Float) float64 {
	d := new(big.Float).SetPrec(frameDefectPrec).Sub(fdf(x1), new(big.Float).SetPrec(frameDefectPrec).Mul(f, fdf(x0)))
	d.Abs(d)
	d.Sub(d, new(big.Float).SetPrec(frameDefectPrec).Mul(f, fdf(b0)))
	r, _ := d.Float64()
	if r <= 0 {
		return 0
	}
	if b1 == 0 {
		return math.Inf(1)
	}
	return r / b1
}

func fdSketch(t *testing.T, w *sketch.World, pl *sketch.Plane, loop [][2]float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s, err := w.CreateSketch(pl)
	require.NoError(t, err)
	pts := make([]*sketch.Point, len(loop))
	for i, p := range loop {
		pts[i] = s.CreatePoint(p[0], p[1])
		s.Fix(pts[i])
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	return s, s.Profiles()[0]
}

var fdRect = [][2]float64{{2, 1}, {5, 1}, {5, 3}, {2, 3}}

func fdExtrude(t *testing.T, frame r3.Frame, opts ...ExtrudeOption) *Body {
	t.Helper()
	return fdExtrudeLoop(t, frame, fdRect, opts...)
}

func fdExtrudeLoop(t *testing.T, frame r3.Frame, loop [][2]float64, opts ...ExtrudeOption) *Body {
	t.Helper()
	w := sketch.NewWorld()
	pl, err := w.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, p := fdSketch(t, w, pl, loop)
	b, err := New().Extrude(s, p, Distance{D: units.Millimeters(2), Dir: Along}, opts...)
	require.NoError(t, err)
	return b
}

func fdTwoPrisms(t *testing.T, f r3.Frame, h0, h1 float64) (*Body, *Body) {
	t.Helper()
	w := sketch.NewWorld()
	pl, err := w.CreatePlaneFromFrame(f)
	require.NoError(t, err)
	s0, p0 := fdSketch(t, w, pl, fdRect)
	s1, p1 := fdSketch(t, w, pl, [][2]float64{{3, 1.5}, {4, 1.5}, {4, 2.5}, {3, 2.5}})
	doc := New()
	a, err := doc.Extrude(s0, p0, Distance{D: units.Millimeters(h0), Dir: Along})
	require.NoError(t, err)
	b, err := doc.Extrude(s1, p1, Distance{D: units.Millimeters(h1), Dir: Along})
	require.NoError(t, err)
	return a, b
}

// requireCoversMappedReference asserts every volume, planar face area and
// line length of b covers the reference's reading carried through m.
func requireCoversMappedReference(t *testing.T, ref, b *Body, m fdMap) {
	t.Helper()
	const slack = 1 + 1e-9
	checked := 0
	if b.IsSolid() {
		v1, err := b.Volume()
		require.NoError(t, err)
		v0, err := ref.Volume()
		require.NoError(t, err)
		r := fdMissRatio(v1.Value.Base(), v1.Bound.Base(), v0.Value.Base(), v0.Bound.Base(), m.volumeFactor())
		require.LessOrEqual(t, r, slack, "volume misses by %g× its bound", r)
		checked++
	}
	rf, bf := ref.Faces(), b.Faces()
	require.Len(t, bf, len(rf))
	for i := range rf {
		pl, ok := rf[i].surface.(Plane)
		if !ok {
			continue
		}
		require.Equal(t, rf[i].origins[0].Role, bf[i].origins[0].Role)
		a0, err := rf[i].Area()
		require.NoError(t, err)
		a1, err := bf[i].Area()
		require.NoError(t, err)
		r := fdMissRatio(a1.Value.Base(), a1.Bound.Base(), a0.Value.Base(), a0.Bound.Base(), m.areaFactor(fdVecOf(pl.Frame.N())))
		require.LessOrEqual(t, r, slack, "face %s area misses by %g× its bound", rf[i].origins[0].Role, r)
		checked++
	}
	re, be := ref.Edges(), b.Edges()
	require.Len(t, be, len(re))
	for i := range re {
		if _, ok := re[i].curve.(Line3); !ok || re[i].start == nil || re[i].lengthUnbounded {
			continue
		}
		l0, err := re[i].Length()
		require.NoError(t, err)
		l1, err := be[i].Length()
		require.NoError(t, err)
		d := fdVecOf(re[i].end.position.Sub(re[i].start.position))
		r := fdMissRatio(l1.Value.Base(), l1.Bound.Base(), l0.Value.Base(), l0.Bound.Base(), m.lengthFactor(d))
		require.LessOrEqual(t, r, slack, "edge length misses by %g× its bound", r)
		checked++
	}
	require.Positive(t, checked)
}

// fdRecordedFrame is the frame a feature records for a sketch on a plane
// built from f: the plane's held frame, normalized once more by r3.NewFrame
// as Extrude and its siblings do, so its floats are the payload's own.
func fdRecordedFrame(t *testing.T, f r3.Frame) r3.Frame {
	t.Helper()
	pl, err := sketch.NewWorld().CreatePlaneFromFrame(f)
	require.NoError(t, err)
	plane, err := pl.Frame()
	require.NoError(t, err)
	held, err := r3.NewFrame(plane.Origin(), plane.U(), plane.V())
	require.NoError(t, err)
	return held
}

// fdSlotSheet extrudes, as a sheet, the slot whose caps of radius r sit at
// (0, 0) and (length, 0), every point fixed at an exact dyadic coordinate.
func fdSlotSheet(t *testing.T, f r3.Frame, length, r float64) *Body {
	t.Helper()
	w := sketch.NewWorld()
	pl, err := w.CreatePlaneFromFrame(f)
	require.NoError(t, err)
	s, err := w.CreateSketch(pl)
	require.NoError(t, err)
	pt := func(x, y float64) *sketch.Point {
		p := s.CreatePoint(x, y)
		s.Fix(p)
		return p
	}
	c1, c2 := pt(0, 0), pt(length, 0)
	p1l, p1r, p2l, p2r := pt(0, r), pt(0, -r), pt(length, r), pt(length, -r)
	s.CreateArc(c1, p1l, p1r)
	s.CreateArc(c2, p2r, p2l)
	s.CreateLine(p1r, p2r)
	s.CreateLine(p2l, p1l)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	b, err := New().Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(2), Dir: Along}, WithSurfaceResult())
	require.NoError(t, err)
	return b
}

// fdAxisU is the sketch's own U axis.
var fdAxisU = SketchLine{Start: Point2{U: 0, V: 0}, End: Point2{U: 1, V: 0}}

// fdRadialChainSheet revolves the line from (0, 0) to (0, 3), perpendicular
// to the sketch's U axis, by ext into a disk or a sector of one.
func fdRadialChainSheet(t *testing.T, f r3.Frame, ext AngularExtent) *Body {
	t.Helper()
	w := sketch.NewWorld()
	pl, err := w.CreatePlaneFromFrame(f)
	require.NoError(t, err)
	s, err := w.CreateSketch(pl)
	require.NoError(t, err)
	a, b := s.CreatePoint(0, 0), s.CreatePoint(0, 3)
	s.Fix(a)
	s.Fix(b)
	s.CreateLine(a, b)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := New().RevolveChain(s, s.Chains()[0], fdAxisU, ext)
	require.NoError(t, err)
	return body
}

// fdSlotSolid extrudes the slot whose caps of radius 2 sit at (0, 0) and
// (5, 0).
func fdSlotSolid(t *testing.T, f r3.Frame) *Body {
	t.Helper()
	w := sketch.NewWorld()
	pl, err := w.CreatePlaneFromFrame(f)
	require.NoError(t, err)
	s, err := w.CreateSketch(pl)
	require.NoError(t, err)
	pt := func(x, y float64) *sketch.Point {
		p := s.CreatePoint(x, y)
		s.Fix(p)
		return p
	}
	c1, c2 := pt(0, 0), pt(5, 0)
	p1l, p1r, p2l, p2r := pt(0, 2), pt(0, -2), pt(5, 2), pt(5, -2)
	s.CreateArc(c1, p1l, p1r)
	s.CreateArc(c2, p2r, p2l)
	s.CreateLine(p1r, p2r)
	s.CreateLine(p2l, p1l)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	b, err := New().Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(2), Dir: Along})
	require.NoError(t, err)
	return b
}

// fdCompositeSweep sweeps a unit square on XY along an arc and a line:
// docs/sweep-design.md's composite path.
func fdCompositeSweep(t *testing.T) (*Body, error) {
	t.Helper()
	w := sketch.NewWorld()
	s, p := fdSketch(t, w, w.XY(), [][2]float64{{-0.5, -0.5}, {0.5, -0.5}, {0.5, 0.5}, {-0.5, 0.5}})
	path, err := NewPath(r3.NewVec(0, 0, 0),
		ArcThrough{Through: r3.NewVec(2, 0, 4), End: r3.NewVec(5, 0, 5)},
		LineTo{End: r3.NewVec(8, 0, 5)})
	require.NoError(t, err)
	return New().Sweep(t.Context(), s, p, path)
}

func TestPlaneMapReadingsCoverFrameDefect(t *testing.T) {
	t.Parallel()
	tilted, err := r3.NewFrame(r3.NewVec(1, 2, 3), r3.NewVec(1, 1, 0), r3.NewVec(-1, 1, 1))
	require.NoError(t, err)
	xy, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	rot, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
	require.NoError(t, err)

	type kind struct {
		name string
		// xyOnly builds on the XY frame alone, so only the rotated variant
		// runs.
		xyOnly bool
		// exactCross reads the revolve's map, whose third column is U×V.
		exactCross bool
		build      func(t *testing.T, f r3.Frame) (*Body, error)
	}
	kinds := []kind{
		{name: "prism", build: func(t *testing.T, f r3.Frame) (*Body, error) { return fdExtrude(t, f), nil }},
		{name: "draft", build: func(t *testing.T, f r3.Frame) (*Body, error) {
			return fdExtrude(t, f, WithTaper(units.Degrees(5))), nil
		}},
		{name: "cap chamfer", build: func(t *testing.T, f r3.Frame) (*Body, error) {
			b := fdExtrude(t, f)
			return b.Chamfer(t.Context(), Edges(CreatedBy(CapEnd(b))), units.Millimeters(0.25))
		}},
		{name: "edge fillet", build: func(t *testing.T, f r3.Frame) (*Body, error) {
			b := fdExtrude(t, f)
			return b.Fillet(t.Context(), Edges(ParallelTo(f.N())), units.Millimeters(0.25))
		}},
		{name: "stacked union", build: func(t *testing.T, f r3.Frame) (*Body, error) {
			a, b := fdTwoPrisms(t, f, 2, 3)
			return Union(t.Context(), a, b)
		}},
		{name: "blind cut", build: func(t *testing.T, f r3.Frame) (*Body, error) {
			a, b := fdTwoPrisms(t, f, 2, 1)
			return Cut(t.Context(), a, b)
		}},
		{name: "cup shell", build: func(t *testing.T, f r3.Frame) (*Body, error) {
			b := fdExtrude(t, f)
			return b.Shell(t.Context(), Faces(FaceCreatedBy(CapEnd(b))), units.Millimeters(0.25))
		}},
		{name: "patch", build: func(t *testing.T, f r3.Frame) (*Body, error) {
			w := sketch.NewWorld()
			pl, err := w.CreatePlaneFromFrame(f)
			require.NoError(t, err)
			s, p := fdSketch(t, w, pl, fdRect)
			return New().Patch(t.Context(), s, p)
		}},
		{name: "chain extrude", build: func(t *testing.T, f r3.Frame) (*Body, error) {
			w := sketch.NewWorld()
			pl, err := w.CreatePlaneFromFrame(f)
			require.NoError(t, err)
			s, err := w.CreateSketch(pl)
			require.NoError(t, err)
			a, b, c := s.CreatePoint(2, 1), s.CreatePoint(5, 1), s.CreatePoint(5, 3)
			s.Fix(a)
			s.Fix(b)
			s.Fix(c)
			s.CreateLine(a, b)
			s.CreateLine(b, c)
			_, err = s.Solve(t.Context())
			require.NoError(t, err)
			return New().ExtrudeChain(s, s.Chains()[0], Distance{D: units.Millimeters(2), Dir: Along})
		}},
		{name: "mirror", build: func(t *testing.T, f r3.Frame) (*Body, error) {
			mf, err := r3.NewFrame(r3.NewVec(0.3, -2, 1), r3.NewVec(1, 0.2, 0.1), r3.NewVec(0, 1, -0.3))
			require.NoError(t, err)
			return fdExtrude(t, f).Mirrored(t.Context(), MirrorFrame{Frame: mf})
		}},
		{name: "unstitch", build: func(t *testing.T, f r3.Frame) (*Body, error) {
			parts, err := fdExtrude(t, f, WithSurfaceResult()).Unstitch(t.Context())
			require.NoError(t, err)
			return parts[0], nil
		}},
		{name: "body patch", build: func(t *testing.T, f r3.Frame) (*Body, error) {
			return fdExtrude(t, f, WithSurfaceResult()).Patch(t.Context(), Edges(Free()))
		}},
		{name: "line sweep", xyOnly: true, build: func(t *testing.T, f r3.Frame) (*Body, error) {
			w := sketch.NewWorld()
			pl, err := w.CreatePlaneFromFrame(f)
			require.NoError(t, err)
			s, p := fdSketch(t, w, pl, fdRect)
			path, err := NewPath(r3.Vec{}, LineTo{End: r3.NewVec(0, 0, 2)})
			require.NoError(t, err)
			return New().Sweep(t.Context(), s, p, path)
		}},
		{name: "composite sweep", xyOnly: true, build: func(t *testing.T, _ r3.Frame) (*Body, error) {
			return fdCompositeSweep(t)
		}},
		// A long thin rim: its corners' own rounding and their float
		// re-expression into the fitted frame scale with the perimeter, far
		// past the fitted frame's own stretch of the area.
		{name: "thin body patch", build: func(t *testing.T, f r3.Frame) (*Body, error) {
			thin := [][2]float64{{0, 0}, {100, 0}, {100, 1.0 / 128}, {0, 1.0 / 128}}
			return fdExtrudeLoop(t, f, thin, WithSurfaceResult()).Patch(t.Context(), Edges(Free()))
		}},
		// A long thin slot: its rim mixes straight flanks with semicircular
		// caps, so its fitted face's area is bounded edge by edge against
		// the curves its rim denotes.
		{name: "thin slot body patch", build: func(t *testing.T, f r3.Frame) (*Body, error) {
			return fdSlotSheet(t, f, 100, 1.0/128).Patch(t.Context(), Edges(Free()))
		}},
		// A revolve's latitude circle: a radial line swept a full turn about
		// the sketch's U axis into a disk whose one free edge it is. On a
		// tilted plane the rim's vertices carry bounds, which Body.Patch
		// refuses (R6), so only the placed copy runs.
		{name: "revolve latitude patch", xyOnly: true, exactCross: true, build: func(t *testing.T, f r3.Frame) (*Body, error) {
			return fdRadialChainSheet(t, f, FullRevolution{}).Patch(t.Context(), Edges(Free()))
		}},
		// A revolve's cap arc: a circle swept a quarter turn as a sheet; its
		// start cap copy, at the exact angle zero, is patched.
		{name: "revolve cap circle patch", xyOnly: true, exactCross: true, build: func(t *testing.T, f r3.Frame) (*Body, error) {
			w := sketch.NewWorld()
			pl, err := w.CreatePlaneFromFrame(f)
			require.NoError(t, err)
			s, err := w.CreateSketch(pl)
			require.NoError(t, err)
			c := s.CreatePoint(0, 3)
			s.Fix(c)
			s.CreateCircle(c, 1)
			_, err = s.Solve(t.Context())
			require.NoError(t, err)
			b, err := New().Revolve(s, s.Profiles()[0], fdAxisU, AngleExtent{A: units.Degrees(90), Dir: Along}, WithSurfaceResult())
			require.NoError(t, err)
			return b.Patch(t.Context(), Edges(Free(), EndpointAt(r3.NewVec(1, 3, 0))))
		}},
		{name: "class B drill", xyOnly: true, build: func(t *testing.T, _ r3.Frame) (*Body, error) {
			w := sketch.NewWorld()
			s0, p0 := fdSketch(t, w, w.XY(), [][2]float64{{0, 0}, {40, 0}, {40, 20}, {0, 20}})
			doc := New()
			box, err := doc.Extrude(s0, p0, Distance{D: units.Millimeters(20), Dir: Along})
			require.NoError(t, err)
			xz, err := w.CreateOffsetPlane(w.XZ(), -10)
			require.NoError(t, err)
			ts, err := w.CreateSketch(xz)
			require.NoError(t, err)
			c := ts.CreatePoint(20, 10)
			ts.Fix(c)
			ts.CreateCircle(c, 3)
			_, err = ts.Solve(t.Context())
			require.NoError(t, err)
			drill, err := doc.Extrude(ts, ts.Profiles()[0], Symmetric{D: units.Millimeters(11)})
			require.NoError(t, err)
			got, err := Cut(t.Context(), box, drill)
			require.NoError(t, err)
			_, ok := got.payload.(brepPayload)
			require.True(t, ok, "premise: the drilled block is a brep")
			return got, nil
		}},
	}
	type variant struct {
		name  string
		frame r3.Frame
		place r3.Transform
	}
	for _, k := range kinds {
		t.Run(k.name, func(t *testing.T) {
			t.Parallel()
			ref, err := k.build(t, xy)
			require.NoError(t, err)
			variants := []variant{{"rotated", xy, rot}}
			if !k.xyOnly {
				variants = append(variants, variant{fdTilted, tilted, r3.Identity()}, variant{"tilted and rotated", tilted, rot})
			}
			for _, v := range variants {
				t.Run(v.name, func(t *testing.T) {
					b, err := k.build(t, v.frame)
					require.NoError(t, err)
					if v.place != r3.Identity() {
						b, err = b.Placed(t.Context(), v.place)
						require.NoError(t, err)
					}
					requireCoversMappedReference(t, ref, b, fdMapThrough(fdRecordedFrame(t, v.frame), v.place, k.exactCross))
				})
			}
		})
	}
}

// TestRevolveMassCoversFrameDefect requires the revolve's published mass to
// enclose ρ·|det L|·V, the mass of the solid its readings and vertices
// denote, for the square ρ ∈ [2, 3], z ∈ [0, 1] about the sketch's V axis
// under a rotation: V = 5π, L = B·[U V U×V].
func TestRevolveMassCoversFrameDefect(t *testing.T) {
	t.Parallel()
	xy, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	rot, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
	require.NoError(t, err)
	w := sketch.NewWorld()
	pl, err := w.CreatePlaneFromFrame(xy)
	require.NoError(t, err)
	s, p := fdSketch(t, w, pl, [][2]float64{{2, 0}, {3, 0}, {3, 1}, {2, 1}})
	b, err := New().Revolve(s, p, coilAxisV, FullRevolution{})
	require.NoError(t, err)
	b, err = b.Placed(t.Context(), rot)
	require.NoError(t, err)
	rp := b.payload.(revolvePayload)
	l, err := massmoment.PlaneMap(rp.frame, rp.xform)
	require.NoError(t, err)
	det := massmoment.Determinant(l)
	det.Abs(det)
	require.NotZero(t, det.Cmp(big.NewRat(1, 1)), "the fixture's map must not be exactly orthonormal")
	const rho = 0.001
	mp, err := b.MassProperties(t.Context(), units.KilogramsPerCubicMillimeter(rho))
	require.NoError(t, err)
	want := new(big.Float).SetPrec(512).Mul(bigPi(), big.NewFloat(5))
	want.Mul(want, new(big.Float).SetPrec(512).SetRat(det))
	want.Mul(want, new(big.Float).SetPrec(512).SetRat(new(big.Rat).SetFloat64(rho)))
	requireEnclosesBig(t, mp.Mass.Value.Base(), mp.Mass.Bound.Base(), want, "mass")
}

// TestRevolveBoxChargesTheBasisRounding requires the revolve box's frame
// allowance to cover the float sweep basis's exact gap from the basis its
// record denotes. On a tilted plane about the sketch's V axis, E1 = W × E0 is
// a rounded cross product while the record denotes the exact one.
func TestRevolveBoxChargesTheBasisRounding(t *testing.T) {
	t.Parallel()
	tilted, err := r3.NewFrame(r3.NewVec(1, 2, 3), r3.NewVec(1, 1, 0), r3.NewVec(-1, 1, 1))
	require.NoError(t, err)
	w := sketch.NewWorld()
	pl, err := w.CreatePlaneFromFrame(tilted)
	require.NoError(t, err)
	s, p := fdSketch(t, w, pl, [][2]float64{{2, 0}, {3, 0}, {3, 1}, {2, 1}})
	b, err := New().Revolve(s, p, coilAxisV, FullRevolution{})
	require.NoError(t, err)
	rp := b.payload.(revolvePayload)
	basis := rp.basis()
	gaps := rp.lift().BasisRound(basis)
	require.Positive(t, gaps[3], "premise: the float E1 is not the exact cross product")

	// The exact E1 gap, read independently: |b.E1 − W × E0| over rationals.
	rat := func(v r3.Vec) [3]*big.Rat {
		return [3]*big.Rat{new(big.Rat).SetFloat64(v.X), new(big.Rat).SetFloat64(v.Y), new(big.Rat).SetFloat64(v.Z)}
	}
	wv, e0 := rat(basis.W), rat(basis.E0)
	cross := func(i, j int) *big.Rat {
		return new(big.Rat).Sub(new(big.Rat).Mul(wv[i], e0[j]), new(big.Rat).Mul(wv[j], e0[i]))
	}
	exactE1 := [3]*big.Rat{cross(1, 2), cross(2, 0), cross(0, 1)}
	gap := new(big.Rat)
	for i, held := range rat(basis.E1) {
		gap.Add(gap, new(big.Rat).Abs(new(big.Rat).Sub(held, exactE1[i])))
	}
	gapF, _ := gap.Float64()
	require.Positive(t, gapF)

	for _, g := range []r3.Vec{r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)} {
		c1 := rp.xform.ApplyDir(basis.E1).Dot(g)
		allow, err := rp.frameRoundAllow(g, basis, rp.xform.Apply(basis.A3).Dot(g), rp.xform.ApplyDir(basis.W).Dot(g),
			rp.xform.ApplyDir(basis.E0).Dot(g), c1, freeform.NewFreeformWork(), nil)
		require.NoError(t, err)
		// E1's coefficient multiplies ρ ≤ 3, so its gap moves the extreme by
		// at most 3·gap; the allowance must cover at least that.
		require.GreaterOrEqual(t, allow, 3*gapF, "axis %v", g)
	}
}

// TestPlaneMapMassCoversFrameDefect requires the mass of a rotated prism, a
// cup and a composite sweep to cover ρ·|det L|·V, the mass of the solid their
// volume and vertices denote (docs/multibody-dynamics-design.md §8.1, §8.7,
// §8.8), as the revolve's does. Shown to fail: on the rigid paths
// (massmoment.RigidInertia, massmoment.Rotate) the prism's and the cup's
// masses missed in every variant.
func TestPlaneMapMassCoversFrameDefect(t *testing.T) {
	t.Parallel()
	tilted, err := r3.NewFrame(r3.NewVec(1, 2, 3), r3.NewVec(1, 1, 0), r3.NewVec(-1, 1, 1))
	require.NoError(t, err)
	xy, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	rot, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
	require.NoError(t, err)
	// A dyadic density keeps ρ·V exact, so the rigid reading's mass
	// publishes a zero bound and any |det L| ≠ 1 shows.
	density := units.KilogramsPerCubicMillimeter(1.0 / 1024)
	for _, c := range []struct {
		name   string
		xyOnly bool
		// volume is the plane-coordinate volume where it is rational; nil
		// compares against the reference body's own reading instead.
		volume *big.Rat
		build  func(t *testing.T, f r3.Frame) (*Body, error)
	}{
		{name: "prism", volume: big.NewRat(12, 1), build: func(t *testing.T, f r3.Frame) (*Body, error) { return fdExtrude(t, f), nil }},
		// The cavity is 2.5 × 1.5 × 1.75 inside the 3 × 2 × 2 block.
		{name: "cup", volume: big.NewRat(87, 16), build: func(t *testing.T, f r3.Frame) (*Body, error) {
			b := fdExtrude(t, f)
			return b.Shell(t.Context(), Faces(FaceCreatedBy(CapEnd(b))), units.Millimeters(0.25))
		}},
		{name: "composite sweep", xyOnly: true, build: func(t *testing.T, _ r3.Frame) (*Body, error) { return fdCompositeSweep(t) }},
	} {
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			ref, err := c.build(t, xy)
			require.NoError(t, err)
			m0, err := ref.MassProperties(t.Context(), density)
			require.NoError(t, err)
			type variant struct {
				name  string
				frame r3.Frame
				place r3.Transform
			}
			variants := []variant{{"rotated", xy, rot}}
			if !c.xyOnly {
				variants = append(variants, variant{fdTilted, tilted, r3.Identity()}, variant{"tilted and rotated", tilted, rot})
			}
			for _, v := range variants {
				t.Run(v.name, func(t *testing.T) {
					b, err := c.build(t, v.frame)
					require.NoError(t, err)
					if v.place != r3.Identity() {
						b, err = b.Placed(t.Context(), v.place)
						require.NoError(t, err)
					}
					m1, err := b.MassProperties(t.Context(), density)
					require.NoError(t, err)
					factor := fdMapOf(fdRecordedFrame(t, v.frame), v.place).volumeFactor()
					if c.volume == nil {
						r := fdMissRatio(m1.Mass.Value.Base(), m1.Mass.Bound.Base(), m0.Mass.Value.Base(), m0.Mass.Bound.Base(), factor)
						require.LessOrEqual(t, r, 1+1e-9, "mass misses by %g× its bound", r)
						return
					}
					want := new(big.Float).SetPrec(frameDefectPrec).SetRat(new(big.Rat).Mul(c.volume, new(big.Rat).SetFloat64(density.Mag())))
					want.Mul(want, factor)
					requireEnclosesBig(t, m1.Mass.Value.Base(), m1.Mass.Bound.Base(), want, "mass")
				})
			}
		})
	}
}

// fdCirclePoints is sixteen rational points of the unit circle, from the
// triples (1, 0), (4/5, 3/5) and (3/5, 4/5) in every quadrant.
func fdCirclePoints() [][2]*big.Float {
	var out [][2]*big.Float
	for _, q := range [][2]int64{{5, 0}, {4, 3}, {3, 4}, {0, 5}} {
		for _, sgn := range [][2]int64{{1, 1}, {-1, 1}, {-1, -1}, {1, -1}} {
			c := new(big.Float).SetPrec(frameDefectPrec).SetRat(big.NewRat(q[0]*sgn[0], 5))
			sn := new(big.Float).SetPrec(frameDefectPrec).SetRat(big.NewRat(q[1]*sgn[1], 5))
			out = append(out, [2]*big.Float{c, sn})
		}
	}
	return out
}

// fdCircleGap is the distance from p to the circle about center normal to
// axis whose radius is radius: √(h² + (r − radius)²), h the height off its plane
// and r the in-plane distance from its centre.
func fdCircleGap(p, center, axis fdVec, radius *big.Float) *big.Float {
	d := p.add(center.scale(fdf(-1)))
	h := d.dot(axis.unit())
	r2 := new(big.Float).SetPrec(frameDefectPrec).Sub(d.dot(d), new(big.Float).SetPrec(frameDefectPrec).Mul(h, h))
	if r2.Sign() < 0 {
		r2.SetInt64(0)
	}
	r := new(big.Float).SetPrec(frameDefectPrec).Sqrt(r2)
	r.Sub(r, radius)
	sum := new(big.Float).SetPrec(frameDefectPrec).Mul(h, h)
	sum.Add(sum, new(big.Float).SetPrec(frameDefectPrec).Mul(r, r))
	return sum.Sqrt(sum)
}

// TestCurveBoundsCoverMappedReference builds each body on the exact XY
// frame as its reference, then on a tilted plane, under a rotation, or both,
// and requires every circular edge of the second to carry a curve bound
// that covers the reference's held circle carried through the denoted map
// Φ(x) = B·(O + L·x) + t: a point q of the reference circle sits within the
// reference's own bound κ₀ of a point the reference denotes, whose image Φ
// denotes, so Φ(q) lies within κ₁ + (1 + e)·κ₀ of the held circle, e the
// orthonormality defect of B·L. Most of these rims carry vertex bounds that
// Body.Patch refuses (R6), so the bound is checked here directly.
//
// Shown to fail: before the revolve and cap-blend builds stamped a curve
// bound, every revolve latitude circle, junction arc and cap arc and every
// cap blend trimmed and apex arc here carried none.
func TestCurveBoundsCoverMappedReference(t *testing.T) {
	t.Parallel()
	tilted, err := r3.NewFrame(r3.NewVec(1, 2, 3), r3.NewVec(1, 1, 0), r3.NewVec(-1, 1, 1))
	require.NoError(t, err)
	xy, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	rot, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
	require.NoError(t, err)
	revolveBlock := func(ext AngularExtent) func(t *testing.T, f r3.Frame) *Body {
		return func(t *testing.T, f r3.Frame) *Body {
			w := sketch.NewWorld()
			pl, err := w.CreatePlaneFromFrame(f)
			require.NoError(t, err)
			s, p := fdSketch(t, w, pl, [][2]float64{{1, 0}, {3, 0}, {3, 1}, {1, 1}})
			b, err := New().Revolve(s, p, coilAxisV, ext)
			require.NoError(t, err)
			return b
		}
	}
	for _, c := range []struct {
		name       string
		exactCross bool
		build      func(t *testing.T, f r3.Frame) *Body
	}{
		{"revolve latitude circles", true, revolveBlock(FullRevolution{})},
		{"revolve junction arcs", true, revolveBlock(AngleExtent{A: units.Degrees(90), Dir: Along})},
		{"revolve cap circles", true, func(t *testing.T, f r3.Frame) *Body {
			w := sketch.NewWorld()
			pl, err := w.CreatePlaneFromFrame(f)
			require.NoError(t, err)
			s, err := w.CreateSketch(pl)
			require.NoError(t, err)
			c := s.CreatePoint(0, 3)
			s.Fix(c)
			s.CreateCircle(c, 1)
			_, err = s.Solve(t.Context())
			require.NoError(t, err)
			b, err := New().Revolve(s, s.Profiles()[0], fdAxisU, AngleExtent{A: units.Degrees(90), Dir: Along})
			require.NoError(t, err)
			return b
		}},
		{"chain revolve arcs", true, func(t *testing.T, f r3.Frame) *Body {
			return fdRadialChainSheet(t, f, AngleExtent{A: units.Degrees(90), Dir: Along})
		}},
		{"cap chamfer trimmed arcs", false, func(t *testing.T, f r3.Frame) *Body {
			b := fdSlotSolid(t, f)
			chamfered, err := b.Chamfer(t.Context(), Edges(CreatedBy(CapEnd(b))), units.Millimeters(0.25))
			require.NoError(t, err)
			return chamfered
		}},
		{"cap chamfer apex arc", false, func(t *testing.T, f r3.Frame) *Body {
			b := fdExtrudeLoop(t, f, [][2]float64{{0, 0}, {4, 0}, {4, 2}, {2, 2}, {2, 4}, {0, 4}})
			chamfered, err := b.Chamfer(t.Context(), Edges(CreatedBy(CapEnd(b))), units.Millimeters(0.25))
			require.NoError(t, err)
			return chamfered
		}},
	} {
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			ref := c.build(t, xy)
			for _, v := range []struct {
				name  string
				frame r3.Frame
				place r3.Transform
			}{{fdTilted, tilted, r3.Identity()}, {"rotated", xy, rot}, {"tilted and rotated", tilted, rot}} {
				t.Run(v.name, func(t *testing.T) {
					b := c.build(t, v.frame)
					if v.place != r3.Identity() {
						var err error
						b, err = b.Placed(t.Context(), v.place)
						require.NoError(t, err)
					}
					frame := fdRecordedFrame(t, v.frame)
					m := fdMapThrough(frame, v.place, c.exactCross)
					origin := fdVecOf(v.place.Apply(r3.Vec{}))
					o := fdVecOf(frame.Origin())
					originImage := fdMapThrough(xy, v.place, false).apply(o).add(origin)
					onePlus := fdf(1 + 1e-12)
					re, be := ref.Edges(), b.Edges()
					require.Len(t, be, len(re))
					circles := 0
					for i := range re {
						c0, a0, r0, ok := circleOf(re[i].curve)
						if !ok {
							continue
						}
						circles++
						require.True(t, re[i].curveBounded, "reference edge %d carries no curve bound", i)
						require.True(t, be[i].curveBounded, "edge %d carries no curve bound", i)
						c1, a1, r1, ok := circleOf(be[i].curve)
						require.True(t, ok)
						an := fdVecOf(a0).unit()
						e1 := fdVecOf(r3.NewVec(1, 0, 0)).cross(an)
						if n, _ := e1.norm().Float64(); n < 0.5 {
							e1 = fdVecOf(r3.NewVec(0, 1, 0)).cross(an)
						}
						e1 = e1.unit()
						e2 := an.cross(e1)
						allow := new(big.Float).SetPrec(frameDefectPrec).Mul(onePlus, fdf(re[i].curveBound))
						allow.Add(allow, fdf(be[i].curveBound))
						// Rational points of the unit circle, so every sample
						// lies on the reference circle to 512 bits.
						for k, cs := range fdCirclePoints() {
							phi := k
							q := fdVecOf(c0).add(e1.scale(new(big.Float).SetPrec(frameDefectPrec).Mul(fdf(r0), cs[0]))).add(e2.scale(new(big.Float).SetPrec(frameDefectPrec).Mul(fdf(r0), cs[1])))
							p := m.apply(q).add(originImage)
							gap := fdCircleGap(p, fdVecOf(c1), fdVecOf(a1), fdf(r1))
							require.LessOrEqual(t, gap.Cmp(allow), 0, "edge %d at φ %v sits %s off, past %s", i, phi, gap.Text('g', 6), allow.Text('g', 6))
						}
					}
					require.Positive(t, circles)
				})
			}
		})
	}
}
