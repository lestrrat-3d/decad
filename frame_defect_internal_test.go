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
//   - the endpoint-support arm of auditAdjacentSweepSpans: the rotated
//     composite sweep refused as "not certified on opposite sides".
//
// Every expected factor reads the frame the payload records
// (fdRecordedFrame), not the sketch plane's held frame: Extrude and its
// siblings normalize the plane's axes once more, and the two differ by a few
// ulps, which reads as a miss of up to 1.23× on a fitted patch face.

const frameDefectPrec = 512

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
	b := place.Basis()
	ex, ey, ez := fdVecOf(b.EX), fdVecOf(b.EY), fdVecOf(b.EZ)
	lin := func(v fdVec) fdVec { return ex.scale(v[0]).add(ey.scale(v[1])).add(ez.scale(v[2])) }
	return fdMap{cols: [3]fdVec{lin(fdVecOf(frame.U())), lin(fdVecOf(frame.V())), lin(fdVecOf(frame.N()))}}
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
		build  func(t *testing.T, f r3.Frame) (*Body, error)
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
				variants = append(variants, variant{"tilted", tilted, r3.Identity()}, variant{"tilted and rotated", tilted, rot})
			}
			for _, v := range variants {
				t.Run(v.name, func(t *testing.T) {
					b, err := k.build(t, v.frame)
					require.NoError(t, err)
					if v.place != r3.Identity() {
						b, err = b.Placed(t.Context(), v.place)
						require.NoError(t, err)
					}
					requireCoversMappedReference(t, ref, b, fdMapOf(fdRecordedFrame(t, v.frame), v.place))
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
				variants = append(variants, variant{"tilted", tilted, r3.Identity()}, variant{"tilted and rotated", tilted, rot})
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
