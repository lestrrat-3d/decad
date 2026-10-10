package apitest_test

import (
	"fmt"
	"math"
	"math/big"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is the tapered extrude's fixture set (docs/draft-design.md §11,
// F1–F9 and the Extrude refusals of Table SD). Every closed form is written
// here from §8's polynomials in 256-bit arithmetic, with d = h·tan α evaluated
// from the exact angle the stated taper denotes — the stated magnitude times
// the float degree factor, the value decad's own unit conversion is measured
// against — and never read back from the body. A bound is asserted to cover
// its closed form and to stay below a relative ceiling; no bound literal is
// pinned.

const taperPrec = 256

// bigPi is π to 76 digits.
var bigPi, _ = new(big.Float).SetPrec(taperPrec).SetString("3.141592653589793238462643383279502884197169399375105820974944592307816406286")

func bf(x float64) *big.Float { return new(big.Float).SetPrec(taperPrec).SetFloat64(x) }

func bfAdd(a, b *big.Float) *big.Float { return new(big.Float).SetPrec(taperPrec).Add(a, b) }
func bfSub(a, b *big.Float) *big.Float { return new(big.Float).SetPrec(taperPrec).Sub(a, b) }
func bfMul(xs ...*big.Float) *big.Float {
	out := new(big.Float).SetPrec(taperPrec).SetInt64(1)
	for _, x := range xs {
		out.Mul(out, x)
	}
	return out
}
func bfQuo(a, b *big.Float) *big.Float { return new(big.Float).SetPrec(taperPrec).Quo(a, b) }
func bfSqrt(a *big.Float) *big.Float   { return new(big.Float).SetPrec(taperPrec).Sqrt(a) }

// taperTan is tan of the exact angle units.Degrees(deg) denotes, to about 75
// digits: the Taylor series of sin and cos at deg times the float degree
// factor, both taken exactly.
func taperTan(deg float64) *big.Float {
	x := new(big.Float).SetPrec(taperPrec).Mul(bf(deg), bf(units.Degree.Factor()))
	x2 := bfMul(x, x)
	series := func(term *big.Float, first int64) *big.Float {
		total := new(big.Float).SetPrec(taperPrec).Set(term)
		eps := new(big.Float).SetMantExp(big.NewFloat(1), -300)
		for k := first; ; k += 2 {
			term = bfQuo(bfMul(term, x2), new(big.Float).SetInt64(-k*(k+1)))
			total.Add(total, term)
			if new(big.Float).Abs(term).Cmp(eps) < 0 {
				return total
			}
		}
	}
	return bfQuo(series(x, 2), series(bf(1), 1))
}

// requireCovers asserts m is Approximate with a nonzero bound that covers
// want and stays below ceiling·|want|.
func requireCovers(t *testing.T, what string, value, bound float64, want *big.Float, ceiling float64) {
	t.Helper()
	gap := new(big.Float).Abs(bfSub(bf(value), want))
	g, _ := gap.Float64()
	w, _ := want.Float64()
	require.Greaterf(t, bound, 0.0, "%s: a drafted body's reading carries a nonzero bound", what)
	require.LessOrEqualf(t, g, bound, "%s: %v sits %g from its closed form %v, outside its bound %g", what, value, g, w, bound)
	require.LessOrEqualf(t, bound, ceiling*math.Abs(w), "%s: bound %g above the ceiling %g·|%v|", what, bound, ceiling, w)
}

func requireMeasurementCovers(t *testing.T, what string, m decad.Measurement, want *big.Float, ceiling float64) {
	t.Helper()
	require.Equalf(t, decad.Approximate, m.Exactness, "%s: a drafted body's reading is never Exact", what)
	requireCovers(t, what, m.Value.Base(), m.Bound.Base(), want, ceiling)
}

// taperSketch draws on an XY-parallel sketch plane at origin o and returns
// the sketch's one region with the given hole count.
func taperSketch(t *testing.T, o r3.Vec, holes int, draw func(*sketch.Sketch)) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	f, err := r3.NewFrame(o, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	w := sketch.NewWorld()
	plane, err := w.CreatePlaneFromFrame(f)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	draw(s)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	for _, p := range s.Profiles() {
		if len(p.Holes) == holes {
			return s, p
		}
	}
	require.FailNow(t, "the sketch holds no region with the requested hole count")
	return nil, nil
}

// drawSquare draws an axis-aligned square of side a centred on (0, cv).
func drawSquare(cv, a float64) func(*sketch.Sketch) {
	return func(s *sketch.Sketch) {
		r := s.CreateRectangle(-a/2, cv-a/2, a/2, cv+a/2)
		s.Fix(r.A)
	}
}

func drawDisk(r float64) func(*sketch.Sketch) {
	return func(s *sketch.Sketch) {
		c := s.CreatePoint(0, 0)
		s.CreateCircle(c, r)
		s.Fix(c)
	}
}

// slotSpan is the distance between F3's two semicircle centres, and
// slotRadius their radius.
const slotSpan, slotRadius = 30.0, 5.0

// drawSlot draws F3's slot about (cu, cv), its semicircles centred along u.
func drawSlot(cu, cv float64) func(*sketch.Sketch) {
	return func(s *sketch.Sketch) {
		slot, err := s.CreateSlot(cu-slotSpan/2, cv, cu+slotSpan/2, cv, slotRadius)
		if err != nil {
			panic(err)
		}
		s.Fix(slot.C1)
		s.Fix(slot.C2)
	}
}

func drawPolygon(pts ...[2]float64) func(*sketch.Sketch) {
	return func(s *sketch.Sketch) {
		ps := make([]*sketch.Point, len(pts))
		for i, p := range pts {
			ps[i] = s.CreatePoint(p[0], p[1])
			s.Fix(ps[i])
		}
		for i := range ps {
			s.CreateLine(ps[i], ps[(i+1)%len(ps)])
		}
	}
}

func drawRing(r, hole float64) func(*sketch.Sketch) {
	return func(s *sketch.Sketch) {
		c := s.CreatePoint(0, 0)
		s.CreateCircle(c, r)
		s.CreateCircle(c, hole)
		s.Fix(c)
	}
}

func taperExtrude(t *testing.T, doc *decad.Document, s *sketch.Sketch, p *sketch.Profile, h, deg float64, dir decad.Direction) *decad.Body {
	t.Helper()
	b, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(h), Dir: dir}, decad.WithTaper(units.Degrees(deg)))
	require.NoError(t, err)
	return b
}

// boxSide is F1's square side and boxHeight its sweep.
const boxSide, boxHeight = 20.0, 10.0

// boxVolume is F1's h(a² − 2ad + 4d²/3) for a = boxSide, h = boxHeight.
func boxVolume(d *big.Float) *big.Float {
	return taperBoxVolume(boxHeight, d)
}

func taperBoxVolume(h float64, d *big.Float) *big.Float {
	return taperBoxVolumeExact(bf(h), d)
}

func taperBoxVolumeExact(h, d *big.Float) *big.Float {
	A := bf(boxSide)
	inner := bfAdd(bfSub(bfMul(A, A), bfMul(bf(2), A, d)), bfQuo(bfMul(bf(4), d, d), bf(3)))
	return bfMul(h, inner)
}

// TestTaperStopExtents uses a real stop body to resolve the far level. The
// tapered body must carry that level into its cap and volume readings.
func TestTaperStopExtents(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name   string
		extent func(*decad.Body) decad.Extent
		stopZ  float64
		farZ   float64
	}{
		{"to face", func(stop *decad.Body) decad.Extent {
			return decad.ToFace{Body: stop, Face: decad.Faces(decad.FaceCreatedBy(decad.CapStart(stop)))}
		}, 30, 30},
		{"through all", func(*decad.Body) decad.Extent {
			return decad.ThroughAll{Dir: decad.Along}
		}, 30, 35},
		{"to face against", func(stop *decad.Body) decad.Extent {
			return decad.ToFace{Body: stop, Face: decad.Faces(decad.FaceCreatedBy(decad.CapEnd(stop)))}
		}, -35, -30},
		{"through all against", func(*decad.Body) decad.Extent {
			return decad.ThroughAll{Dir: decad.Against}
		}, -35, -35},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			stopSketch, stopProfile := taperSketch(t, r3.NewVec(0, 0, tc.stopZ), 0, drawSquare(0, 60))
			stop, err := doc.Extrude(stopSketch, stopProfile,
				decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
			require.NoError(t, err)
			s, p := taperSketch(t, r3.NewVec(0, 0, 0), 0, drawSquare(0, boxSide))
			body, err := doc.Extrude(s, p, tc.extent(stop), decad.WithTaper(units.Degrees(5)))
			require.NoError(t, err)
			requireManifold(t, body)
			height := math.Abs(tc.farZ)
			d := bfMul(bf(height), taperTan(5))
			volume, err := body.Volume()
			require.NoError(t, err)
			requireMeasurementCovers(t, "stop draft volume", volume,
				taperBoxVolume(height, d), taperCeiling)
			mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1))
			require.NoError(t, err)
			requireWatertight(t, mesh)
			require.True(t, mesh.VolumeVerified())
			want, _ := taperBoxVolume(height, d).Float64()
			require.InDelta(t, want, meshVolume(mesh), 1e-6)
			x := bfSub(bf(boxSide/2), d)
			frame, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
			require.NoError(t, err)
			requireVertexAt(t, body, liftExact(frame, r3.Identity(), x, x, bf(tc.farZ)))
		})
	}
}

// The stop offset is converted from inches before the draft computes h·tan α.
// The volume and far vertex must enclose the stated inch value, not merely the
// rounded millimetre level used to build the cap.
func TestTaperToFaceOffsetBound(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	stopSketch, stopProfile := taperSketch(t, r3.NewVec(0, 0, 30), 0, drawSquare(0, 60))
	stop, err := doc.Extrude(stopSketch, stopProfile,
		decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)
	s, p := taperSketch(t, r3.NewVec(0, 0, 0), 0, drawSquare(0, boxSide))
	end := decad.ToFace{
		Body:   stop,
		Face:   decad.Faces(decad.FaceCreatedBy(decad.CapStart(stop))),
		Offset: units.Inches(0.1),
	}
	body, err := doc.Extrude(s, p, end, decad.WithTaper(units.Degrees(5)))
	require.NoError(t, err)
	h := bfAdd(bf(30), bfMul(bf(0.1), bf(units.Inch.Factor())))
	d := bfMul(h, taperTan(5))
	volume, err := body.Volume()
	require.NoError(t, err)
	requireMeasurementCovers(t, "offset stop draft volume", volume,
		taperBoxVolumeExact(h, d), taperCeiling)
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	x := bfSub(bf(boxSide/2), d)
	requireVertexAt(t, body, liftExact(frame, r3.Identity(), x, x, h))
}

const taperCeiling = 1e-9

// TestTaperTwoSidedBuild measures the two drafted halves against the sketch
// section. Their shared section must remain a wall join, not an internal cap.
func TestTaperTwoSidedBuild(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name         string
		extent       decad.Extent
		below, above float64
	}{
		{"symmetric", decad.Symmetric{D: units.Millimeters(10), FullLength: true}, 5, 5},
		{"two distances", decad.TwoSided{
			One: decad.DistanceSide{D: units.Millimeters(6)},
			Two: decad.DistanceSide{D: units.Millimeters(4)},
		}, 4, 6},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			s, p := taperSketch(t, r3.NewVec(0, 0, 0), 0, drawSquare(0, boxSide))
			doc := decad.New()
			body, err := doc.Extrude(s, p, tc.extent, decad.WithTaper(units.Degrees(5)))
			require.NoError(t, err)
			requireManifold(t, body)
			require.Len(t, body.Faces(), 10)
			for _, face := range body.Faces() {
				role := face.Origins()[0].Role
				if !strings.HasPrefix(role, "side(") {
					continue
				}
				point := face.Loops()[0].Edges()[0].Start().Position().Value
				normal, err := face.NormalAt(point)
				require.NoError(t, err)
				if strings.HasPrefix(role, "side(0,") {
					require.Less(t, normal.Value.Z, 0.0)
				} else {
					require.Greater(t, normal.Value.Z, 0.0)
				}
			}

			volume, err := body.Volume()
			require.NoError(t, err)
			want := bf(0)
			for _, h := range []float64{tc.below, tc.above} {
				d := bfMul(bf(h), taperTan(5))
				part := bfMul(bf(h), bfAdd(
					bfSub(bfMul(bf(boxSide), bf(boxSide)), bfMul(bf(2*boxSide), d)),
					bfQuo(bfMul(bf(4), d, d), bf(3)),
				))
				want = bfAdd(want, part)
			}
			requireMeasurementCovers(t, "two-sided volume", volume, want, taperCeiling)
			mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1))
			require.NoError(t, err)
			requireWatertight(t, mesh)
			require.True(t, mesh.VolumeVerified())
			wantMesh, _ := want.Float64()
			require.InDelta(t, wantMesh, meshVolume(mesh), 1e-6)
			mass, err := body.MassProperties(t.Context(), units.KilogramsPerCubicMillimeter(1.0/1024))
			require.NoError(t, err)
			requireMeasurementCovers(t, "two-sided mass", mass.Mass, bfQuo(want, bf(1024)), 1e-6)
		})
	}
}

func TestTaperTwoSidedCurvedMesh(t *testing.T) {
	t.Parallel()
	for _, degrees := range []float64{3, -3} {
		t.Run(fmt.Sprint(degrees), func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			s, p := taperSketch(t, r3.NewVec(0, 0, 0), 0, drawSlot(0, 0))
			body, err := doc.Extrude(s, p, decad.TwoSided{
				One: decad.DistanceSide{D: units.Millimeters(6)},
				Two: decad.DistanceSide{D: units.Millimeters(4)},
			}, decad.WithTaper(units.Degrees(degrees)))
			require.NoError(t, err)
			requireManifold(t, body)
			mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.05))
			require.NoError(t, err)
			requireWatertight(t, mesh)
			require.True(t, mesh.VolumeVerified())

			placed, err := body.Placed(t.Context(), generalMotion(t, 37, 3.25))
			require.NoError(t, err)
			requireManifold(t, placed)
			placedMesh, err := placed.Tessellate(t.Context(), units.Millimeters(0.05))
			require.NoError(t, err)
			requireWatertight(t, placedMesh)
			require.True(t, placedMesh.VolumeVerified())
		})
	}
}

func TestTaperTwoSidedCut(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	s, p := taperSketch(t, r3.NewVec(0, 0, 0), 0, drawSquare(0, boxSide))
	drafted, err := doc.Extrude(s, p, decad.TwoSided{
		One: decad.DistanceSide{D: units.Millimeters(6)},
		Two: decad.DistanceSide{D: units.Millimeters(4)},
	}, decad.WithTaper(units.Degrees(5)))
	require.NoError(t, err)
	post := boxBodyAtZ(t, doc, -2, -2, 2, 2, -5, 20)
	cut, err := decad.Cut(t.Context(), drafted, post)
	require.NoError(t, err)
	want := bfSub(bfAdd(
		taperBoxVolume(4, bfMul(bf(4), taperTan(5))),
		taperBoxVolume(6, bfMul(bf(6), taperTan(5))),
	), bf(160))
	requireVolumeCovers(t, "two-sided cut", cut, want, 1e-9)
	requireBodyWatertight(t, cut)
}

func TestTaperTwoSidedToFaces(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	stopBody := func(z float64) *decad.Body {
		s, p := taperSketch(t, r3.NewVec(0, 0, z), 0, drawSquare(0, 60))
		body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
		require.NoError(t, err)
		return body
	}
	top, bottom := stopBody(6), stopBody(-9)
	s, p := taperSketch(t, r3.NewVec(0, 0, 0), 0, drawSquare(0, boxSide))
	body, err := doc.Extrude(s, p, decad.TwoSided{
		One: decad.ToFace{Body: top, Face: decad.Faces(decad.FaceCreatedBy(decad.CapStart(top)))},
		Two: decad.ToFace{Body: bottom, Face: decad.Faces(decad.FaceCreatedBy(decad.CapEnd(bottom)))},
	}, decad.WithTaper(units.Degrees(5)))
	require.NoError(t, err)
	requireManifold(t, body)
	want := bfAdd(
		taperBoxVolume(4, bfMul(bf(4), taperTan(5))),
		taperBoxVolume(6, bfMul(bf(6), taperTan(5))),
	)
	volume, err := body.Volume()
	require.NoError(t, err)
	requireMeasurementCovers(t, "two-sided stops", volume, want, taperCeiling)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	require.True(t, mesh.VolumeVerified())
}

// bfRat converts a 256-bit value to the exact rational it holds.
func bfRat(x *big.Float) *big.Rat {
	r, _ := x.Rat(nil)
	return r
}

// liftExact is the world point a plane-local (u, v, z) denotes under frame then
// xform, every step over big.Rat (exactFramePoint's construction for
// coordinates the test states to 256 bits).
func liftExact(frame r3.Frame, xform r3.Transform, u, v, z *big.Float) [3]*big.Rat {
	origin, fu, fv, fn := ratVecOf(frame.Origin()), ratVecOf(frame.U()), ratVecOf(frame.V()), ratVecOf(frame.N())
	ru, rv, rz := bfRat(u), bfRat(v), bfRat(z)
	local := [3]*big.Rat{}
	for i := range local {
		local[i] = new(big.Rat).Add(
			new(big.Rat).Add(origin[i], new(big.Rat).Mul(fu[i], ru)),
			new(big.Rat).Add(new(big.Rat).Mul(fv[i], rv), new(big.Rat).Mul(fn[i], rz)),
		)
	}
	return exactApplyRat(xform, local)
}

// requireVertexAt requires a vertex of b within its own published bound of
// the exact point truth.
func requireVertexAt(t *testing.T, b *decad.Body, truth [3]*big.Rat) {
	t.Helper()
	v := closestVertex(t, b.Vertices(), truth)
	res := vertexResidual(v.Position().Value, truth)
	require.Lessf(t, res, 1e-6, "no vertex of the body sits at the expected corner")
	require.LessOrEqualf(t, res, v.Position().Bound.Base(), "vertex %v sits %g from its exact point, outside its bound", v.Position().Value, res)
}

// requireWallNormal requires the face's outward normal, read at its first
// vertex, within its own bound of want.
func requireWallNormal(t *testing.T, f *decad.Face, want [3]*big.Float) {
	t.Helper()
	n, err := f.NormalAt(f.Loops()[0].Edges()[0].Start().Position().Value)
	require.NoError(t, err)
	got := [3]float64{n.Value.X, n.Value.Y, n.Value.Z}
	gap := new(big.Float).SetPrec(taperPrec)
	for i := range got {
		c := bfSub(bf(got[i]), want[i])
		gap.Add(gap, bfMul(c, c))
	}
	g, _ := bfSqrt(gap).Float64()
	require.LessOrEqualf(t, g, n.Bound.Base(), "normal %v sits %g from the drafted wall's own normal, outside its bound %g", n.Value, g, n.Bound.Base())
}

// cosSinOf returns cos α and sin α from tan α.
func cosSinOf(tan *big.Float) (*big.Float, *big.Float) {
	cos := bfQuo(bf(1), bfSqrt(bfAdd(bf(1), bfMul(tan, tan))))
	return cos, bfMul(tan, cos)
}

func facesByKind(b *decad.Body) (planes, cones []*decad.Face) {
	for _, f := range b.Faces() {
		if !isWall(f) {
			continue
		}
		switch f.Surface().(type) {
		case decad.Plane:
			planes = append(planes, f)
		case decad.Cone:
			cones = append(cones, f)
		}
	}
	return planes, cones
}

// isWall reports whether a face carries a side(i, j) role.
func isWall(f *decad.Face) bool {
	for _, o := range f.Origins() {
		if len(o.Role) > 5 && o.Role[:5] == "side(" {
			return true
		}
	}
	return false
}

// TestTaperBox is F1: a 20 mm square swept 10 mm at a 5 degree taper, the
// positive taper narrowing the body away from the sketch plane. Shown to fail
// first: without the far contour's proofbound.SweptVolumeAllow term in the
// band volume, the volume bound fell to 1.0e-12 against a 1.08e-12 gap.
func TestTaperBox(t *testing.T) {
	t.Parallel()
	const a, h, deg = 20.0, 10.0, 5.0
	doc := decad.New()
	s, p := taperSketch(t, r3.NewVec(0, 0, 0), 0, drawSquare(0, a))
	b := taperExtrude(t, doc, s, p, h, deg, decad.Along)
	requireManifold(t, b)
	require.Len(t, b.Faces(), 6)
	require.Len(t, b.Edges(), 12)
	require.Len(t, b.Vertices(), 8)

	tan := taperTan(deg)
	d := bfMul(bf(h), tan)
	A, H := bf(a), bf(h)
	want := boxVolume(d)
	vol, err := b.Volume()
	require.NoError(t, err)
	requireMeasurementCovers(t, "volume", vol, want, taperCeiling)

	// Axial centroid h(a²/2 − 4ad/3 + d²)/(a² − 2ad + 4d²/3).
	moment := bfMul(H, H, bfAdd(bfSub(bfQuo(bfMul(A, A), bf(2)), bfQuo(bfMul(bf(4), A, d), bf(3))), bfMul(d, d)))
	c, err := b.Centroid()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, c.Exactness)
	requireCovers(t, "centroid z", c.Value.Z, c.Bound.Base(), bfQuo(moment, want), taperCeiling)
	requireCovers(t, "centroid x", c.Value.X+1, c.Bound.Base(), bf(1), 1e-9)

	// Area a² + (a − 2d)² + 4(a − d)√(h² + d²).
	top := bfSub(A, bfMul(bf(2), d))
	slant := bfSqrt(bfAdd(bfMul(H, H), bfMul(d, d)))
	area := bfAdd(bfAdd(bfMul(A, A), bfMul(top, top)), bfMul(bf(4), bfSub(A, d), slant))
	ar, err := b.Area()
	require.NoError(t, err)
	requireMeasurementCovers(t, "area", ar, area, taperCeiling)

	// The far cap's corners sit at (±(a/2 − d), ±(a/2 − d), h).
	half := bfSub(bf(a/2), d)
	for _, su := range []float64{-1, 1} {
		for _, sv := range []float64{-1, 1} {
			requireVertexAt(t, b, liftExact(xyFrame(t), r3.Identity(), bfMul(bf(su), half), bfMul(bf(sv), half), H))
		}
	}

	// Each wall is a Plane leaning α: outward normal n̂·cos α + e·sin α.
	planes, cones := facesByKind(b)
	require.Len(t, planes, 4)
	require.Empty(t, cones)
	cos, sin := cosSinOf(tan)
	zero := bf(0)
	for _, f := range planes {
		n, err := f.NormalAt(f.Loops()[0].Edges()[0].Start().Position().Value)
		require.NoError(t, err)
		nu, nv := zero, zero
		switch {
		case n.Value.X > 0.5:
			nu = cos
		case n.Value.X < -0.5:
			nu = bfMul(bf(-1), cos)
		case n.Value.Y > 0.5:
			nv = cos
		default:
			nv = bfMul(bf(-1), cos)
		}
		requireWallNormal(t, f, [3]*big.Float{nu, nv, sin})
		sel, err := decad.Faces(decad.Planar()).SelectFaces(b)
		require.NoError(t, err)
		require.Len(t, sel, 6)
	}
}

// TestTaperFrustum is F2: a disk of radius 10 swept 10 mm at a 10 degree taper
// builds the frustum, one seamless Cone wall between two circular rims.
func TestTaperFrustum(t *testing.T) {
	t.Parallel()
	const r0, h, deg = 10.0, 10.0, 10.0
	s, p := taperSketch(t, r3.NewVec(0, 0, 0), 0, drawDisk(r0))
	b := taperExtrude(t, decad.New(), s, p, h, deg, decad.Along)
	requireManifold(t, b)
	require.Len(t, b.Faces(), 3)
	require.Len(t, b.Edges(), 2, "a whole circle's wall has no seam edge")
	planes, cones := facesByKind(b)
	require.Empty(t, planes)
	require.Len(t, cones, 1)

	tan := taperTan(deg)
	d := bfMul(bf(h), tan)
	R, H := bf(r0), bf(h)
	r := bfSub(R, d)
	sum := bfAdd(bfAdd(bfMul(R, R), bfMul(R, r)), bfMul(r, r))
	want := bfQuo(bfMul(bigPi, H, sum), bf(3))
	vol, err := b.Volume()
	require.NoError(t, err)
	requireMeasurementCovers(t, "volume", vol, want, taperCeiling)

	cone := cones[0].Surface().(decad.Cone)
	alpha, _ := bfMul(bf(deg), bf(units.Degree.Factor())).Float64()
	require.InDelta(t, alpha, cone.HalfAngle.Base(), 1e-12, "the wall's half angle is the taper")

	lateral := bfMul(bigPi, bfAdd(R, r), bfSqrt(bfAdd(bfMul(H, H), bfMul(d, d))))
	wall, err := cones[0].Area()
	require.NoError(t, err)
	requireMeasurementCovers(t, "lateral area", wall, lateral, taperCeiling)

	moment := bfMul(H, bfAdd(bfAdd(bfMul(R, R), bfMul(bf(2), R, r)), bfMul(bf(3), r, r)))
	c, err := b.Centroid()
	require.NoError(t, err)
	requireCovers(t, "centroid z", c.Value.Z, c.Bound.Base(), bfQuo(moment, bfMul(bf(4), sum)), taperCeiling)
}

// slotVolume is F3's h[2ℓ(R − d/2) + π(R² − Rd + d²/3)].
func slotVolume(span, r, h float64, d *big.Float) *big.Float {
	L, R := bf(span), bf(r)
	straight := bfMul(bf(2), L, bfSub(R, bfQuo(d, bf(2))))
	round := bfMul(bigPi, bfAdd(bfSub(bfMul(R, R), bfMul(R, d)), bfQuo(bfMul(d, d), bf(3))))
	return bfMul(bf(h), bfAdd(straight, round))
}

// TestTaperSlot is F3: a slot whose lines meet its semicircles at G1 joins
// builds two Plane and two Cone walls, the four rulings between them Line3.
func TestTaperSlot(t *testing.T) {
	t.Parallel()
	const span, r, h, deg = 30.0, 5.0, 8.0, 3.0
	s, p := taperSketch(t, r3.NewVec(0, 0, 0), 0, drawSlot(0, 0))
	b := taperExtrude(t, decad.New(), s, p, h, deg, decad.Along)
	requireManifold(t, b)
	planes, cones := facesByKind(b)
	require.Len(t, planes, 2)
	require.Len(t, cones, 2)
	rulings := 0
	for _, e := range b.Edges() {
		if len(e.Faces()) == 2 && isWall(e.Faces()[0]) && isWall(e.Faces()[1]) {
			require.IsType(t, decad.Line3{}, e.Curve())
			rulings++
		}
	}
	require.Equal(t, 4, rulings)

	d := bfMul(bf(h), taperTan(deg))
	vol, err := b.Volume()
	require.NoError(t, err)
	requireMeasurementCovers(t, "volume", vol, slotVolume(span, r, h, d), taperCeiling)
}

// TestTaperLSection is F4: the reflex corner of an L section stays sharp, its
// far corner the miter of the two moved walls, so the notch square keeps its
// side and A(z) = 300 − 80·z·tan α + 4·(z·tan α)².
func TestTaperLSection(t *testing.T) {
	t.Parallel()
	const h, deg = 10.0, 5.0
	s, p := taperSketch(t, r3.NewVec(0, 0, 0), 0, drawPolygon(
		[2]float64{0, 0}, [2]float64{20, 0}, [2]float64{20, 10},
		[2]float64{10, 10}, [2]float64{10, 20}, [2]float64{0, 20}))
	b := taperExtrude(t, decad.New(), s, p, h, deg, decad.Along)
	requireManifold(t, b)
	require.Len(t, b.Faces(), 8)

	d := bfMul(bf(h), taperTan(deg))
	want := bfMul(bf(h), bfAdd(bfSub(bf(300), bfMul(bf(40), d)), bfQuo(bfMul(bf(4), d, d), bf(3))))
	vol, err := b.Volume()
	require.NoError(t, err)
	requireMeasurementCovers(t, "volume", vol, want, taperCeiling)
	reflex := bfSub(bf(10), d)
	requireVertexAt(t, b, liftExact(xyFrame(t), r3.Identity(), reflex, reflex, bf(h)))
}

// TestTaperFlare is F5: a negative taper widens the body away from the sketch
// plane.
func TestTaperFlare(t *testing.T) {
	t.Parallel()
	const a, h, deg = 20.0, 10.0, -5.0
	s, p := taperSketch(t, r3.NewVec(0, 0, 0), 0, drawSquare(0, a))
	b := taperExtrude(t, decad.New(), s, p, h, deg, decad.Along)
	requireManifold(t, b)
	d := bfMul(bf(h), taperTan(deg))
	vol, err := b.Volume()
	require.NoError(t, err)
	requireMeasurementCovers(t, "volume", vol, boxVolume(d), taperCeiling)
	half := bfSub(bf(a/2), d)
	for _, su := range []float64{-1, 1} {
		for _, sv := range []float64{-1, 1} {
			requireVertexAt(t, b, liftExact(xyFrame(t), r3.Identity(), bfMul(bf(su), half), bfMul(bf(sv), half), bf(h)))
		}
	}
	planes, _ := facesByKind(b)
	for _, f := range planes {
		n, err := f.NormalAt(f.Loops()[0].Edges()[0].Start().Position().Value)
		require.NoError(t, err)
		require.Less(t, n.Value.Z, 0.0, "a flared wall faces away from the far end")
	}
}

// TestTaperAgainst is F6: an Against sweep puts the far section at z = −h on
// the start cap, with F1's volume. Shown to fail first: without the far cap's
// proofbound.SectionDisplacementArea term its area bound fell to 4.3e-14
// against a 7.2e-14 gap, and without the band volume's SweptVolumeAllow term
// the volume failed as F1's does.
func TestTaperAgainst(t *testing.T) {
	t.Parallel()
	const a, h, deg = 20.0, 10.0, 5.0
	s, p := taperSketch(t, r3.NewVec(0, 0, 0), 0, drawSquare(0, a))
	b := taperExtrude(t, decad.New(), s, p, h, deg, decad.Against)
	requireManifold(t, b)
	d := bfMul(bf(h), taperTan(deg))
	vol, err := b.Volume()
	require.NoError(t, err)
	requireMeasurementCovers(t, "volume", vol, boxVolume(d), taperCeiling)
	start, err := decad.Faces(decad.FaceCreatedBy(decad.CapStart(b))).Exactly(1).SelectFaces(b)
	require.NoError(t, err)
	for _, e := range start[0].Edges() {
		require.Equal(t, -h, e.Start().Position().Value.Z)
	}
	startArea, err := start[0].Area()
	require.NoError(t, err)
	top := bfMul(bfSub(bf(a), bfMul(bf(2), d)), bfSub(bf(a), bfMul(bf(2), d)))
	requireMeasurementCovers(t, "far cap area", startArea, top, taperCeiling)
	end, err := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(b))).Exactly(1).SelectFaces(b)
	require.NoError(t, err)
	endArea, err := end[0].Area()
	require.NoError(t, err)
	require.Equal(t, a*a, endArea.Value.Base())
}

// TestTaperRing is F7: an annulus tapers with its hole widening toward the far
// end, the hole wall a Cone whose rims are concave.
func TestTaperRing(t *testing.T) {
	t.Parallel()
	const ro, ri, h, deg = 10.0, 4.0, 10.0, 5.0
	s, p := taperSketch(t, r3.NewVec(0, 0, 0), 1, drawRing(ro, ri))
	b := taperExtrude(t, decad.New(), s, p, h, deg, decad.Along)
	requireManifold(t, b)
	d := bfMul(bf(h), taperTan(deg))
	R, r := bf(ro), bf(ri)
	want := bfMul(bigPi, bf(h), bfSub(bfSub(bfMul(R, R), bfMul(r, r)), bfMul(d, bfAdd(R, r))))
	vol, err := b.Volume()
	require.NoError(t, err)
	requireMeasurementCovers(t, "volume", vol, want, taperCeiling)

	_, cones := facesByKind(b)
	require.Len(t, cones, 2)
	var hole *decad.Face
	for _, f := range cones {
		for _, o := range f.Origins() {
			if o.Role == "side(1,0)" {
				hole = f
			}
		}
	}
	require.NotNil(t, hole)
	for _, e := range hole.Edges() {
		require.False(t, e.IsConvex(), "a hole wall's rims are concave")
	}
	// The hole opens toward the far end: its far rim is the wider one.
	var near, far float64
	for _, e := range hole.Edges() {
		c := e.Curve().(decad.Circle3)
		if e.Start().Position().Value.Z == 0 {
			near = c.Radius.Base()
		} else {
			far = c.Radius.Base()
		}
	}
	require.Equal(t, ri, near)
	require.Greater(t, far, near)
}

// farOrigin is F8's sketch-plane origin: one ulp of its X is about 1.2e-10,
// so every lift through it rounds.
var farOrigin = r3.NewVec(1e6+0.1, 0.1, 0)

func farFrame(t *testing.T) r3.Frame {
	t.Helper()
	f, err := r3.NewFrame(farOrigin, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	return f
}

// TestTaperFarOrigin is F8: F1 and F3 on a sketch plane far from the world
// origin, each then placed under a rotation. Every vertex's published bound
// covers the exact lift of the point it denotes, near rim and far rim alike,
// and the placed slot's volume bound covers its closed form. A third body, F3
// drawn at v = 10⁶ in the plane itself, holds its far section at coordinates
// whose own rounding is that large, which the far contour's displacement must
// carry into the volume. Shown to fail first: without SweptVolumeAllow the
// placed box's volume failed; without capband.DenotedNormalAllow the slot's
// walls failed (below). Omitting the closure slivers (capband.ClosureOf's
// charge) turned no fixture here red: the axis-aligned slot holds no sliver,
// and a slanted one's slivers sit four orders below the far contour's term.
func TestTaperFarOrigin(t *testing.T) {
	t.Parallel()
	const h, deg = 10.0, 5.0
	motion := generalMotion(t, 37, 3.25)
	frame := farFrame(t)

	t.Run("box", func(t *testing.T) {
		t.Parallel()
		const a = 20.0
		doc := decad.New()
		s, p := taperSketch(t, farOrigin, 0, drawSquare(0, a))
		b := taperExtrude(t, doc, s, p, h, deg, decad.Along)
		placed, err := b.Placed(t.Context(), motion)
		require.NoError(t, err)
		d := bfMul(bf(h), taperTan(deg))
		for _, body := range []struct {
			b     *decad.Body
			xform r3.Transform
		}{{b, r3.Identity()}, {placed, motion}} {
			for _, z := range []*big.Float{bf(0), bf(h)} {
				half := bf(a / 2)
				if z.Sign() != 0 {
					half = bfSub(half, d)
				}
				for _, su := range []float64{-1, 1} {
					for _, sv := range []float64{-1, 1} {
						requireVertexAt(t, body.b, liftExact(frame, body.xform, bfMul(bf(su), half), bfMul(bf(sv), half), z))
					}
				}
			}
		}
		vol, err := placed.Volume()
		require.NoError(t, err)
		requireMeasurementCovers(t, "placed volume", vol, boxVolume(d), taperCeiling)
		ar, err := placed.Area()
		require.NoError(t, err)
		A, H := bf(a), bf(h)
		top := bfSub(A, bfMul(bf(2), d))
		area := bfAdd(bfAdd(bfMul(A, A), bfMul(top, top)), bfMul(bf(4), bfSub(A, d), bfSqrt(bfAdd(bfMul(H, H), bfMul(d, d)))))
		requireMeasurementCovers(t, "placed area", ar, area, taperCeiling)
	})

	t.Run("slot", func(t *testing.T) {
		t.Parallel()
		const span, r = 30.0, 5.0
		doc := decad.New()
		s, p := taperSketch(t, farOrigin, 0, drawSlot(0, 0))
		b := taperExtrude(t, doc, s, p, 8, 3, decad.Along)
		placed, err := b.Placed(t.Context(), motion)
		require.NoError(t, err)
		d := bfMul(bf(8), taperTan(3))
		for _, su := range []float64{-1, 1} {
			for _, sv := range []float64{-1, 1} {
				requireVertexAt(t, placed, liftExact(frame, motion, bf(su*span/2), bf(sv*r), bf(0)))
				requireVertexAt(t, placed, liftExact(frame, motion, bf(su*span/2), bfMul(bf(sv), bfSub(bf(r), d)), bf(8)))
			}
		}
		vol, err := placed.Volume()
		require.NoError(t, err)
		requireMeasurementCovers(t, "placed volume", vol, slotVolume(span, r, 8, d), taperCeiling)
	})

	t.Run("slot at v=1e6", func(t *testing.T) {
		t.Parallel()
		const span, r = 30.0, 5.0
		s, p := taperSketch(t, r3.NewVec(0, 0, 0), 0, drawSlot(0.1, 1e6))
		b := taperExtrude(t, decad.New(), s, p, 8, 3, decad.Along)
		d := bfMul(bf(8), taperTan(3))
		vol, err := b.Volume()
		require.NoError(t, err)
		requireMeasurementCovers(t, "volume", vol, slotVolume(span, r, 8, d), 1e-6)
		c, err := b.Centroid()
		require.NoError(t, err)
		requireCovers(t, "centroid v", c.Value.Y, c.Bound.Base(), bf(1e6), 1e-6)

		// The far rim's corners hold coordinates near 10⁶, rounded by about
		// 1e-10, so each straight wall's tag turns from the wall the taper
		// denotes by far more than its own arithmetic. Shown to fail first:
		// without capband.DenotedNormalAllow each wall published a 2.2e-16
		// bound while sitting 3.3e-12 from its denoted normal.
		cos, sin := cosSinOf(taperTan(3))
		planes, _ := facesByKind(b)
		require.Len(t, planes, 2)
		for _, f := range planes {
			n, err := f.NormalAt(f.Loops()[0].Edges()[0].Start().Position().Value)
			require.NoError(t, err)
			sign := 1.0
			if n.Value.Y < 0 {
				sign = -1
			}
			requireWallNormal(t, f, [3]*big.Float{bf(0), bfMul(bf(sign), cos), sin})
		}
	})
}

// TestTaperPlacedEquivalence is F9: F1 placed under a translation and a
// rotation measures as F1 does, and its Bounds covers the placed box.
func TestTaperPlacedEquivalence(t *testing.T) {
	t.Parallel()
	const a, h, deg = 20.0, 10.0, 5.0
	s, p := taperSketch(t, r3.NewVec(0, 0, 0), 0, drawSquare(0, a))
	doc := decad.New()
	b := taperExtrude(t, doc, s, p, h, deg, decad.Along)
	vol0, err := b.Volume()
	require.NoError(t, err)
	area0, err := b.Area()
	require.NoError(t, err)
	c0, err := b.Centroid()
	require.NoError(t, err)

	motion := generalMotion(t, 23, -7.5)
	placed, err := b.Placed(t.Context(), motion)
	require.NoError(t, err)
	requireManifold(t, placed)
	vol, err := placed.Volume()
	require.NoError(t, err)
	area, err := placed.Area()
	require.NoError(t, err)
	c, err := placed.Centroid()
	require.NoError(t, err)
	require.InDelta(t, vol0.Value.Base(), vol.Value.Base(), vol0.Bound.Base()+vol.Bound.Base())
	require.InDelta(t, area0.Value.Base(), area.Value.Base(), area0.Bound.Base()+area.Bound.Base())
	want := motion.Apply(c0.Value)
	require.LessOrEqual(t, want.Sub(c.Value).Len(), c0.Bound.Base()+c.Bound.Base()+1e-12)

	box, err := placed.Bounds()
	require.NoError(t, err)
	for _, v := range placed.Vertices() {
		p := v.Position()
		slack := box.Bound.Base() + p.Bound.Base()
		require.GreaterOrEqual(t, p.Value.X, box.Min.X-slack)
		require.GreaterOrEqual(t, p.Value.Y, box.Min.Y-slack)
		require.GreaterOrEqual(t, p.Value.Z, box.Min.Z-slack)
		require.LessOrEqual(t, p.Value.X, box.Max.X+slack)
		require.LessOrEqual(t, p.Value.Y, box.Max.Y+slack)
		require.LessOrEqual(t, p.Value.Z, box.Max.Z+slack)
	}
}

// TestTaperFarCornerChargesTheTangentSpan pins the offset amount's span in
// the far corners (docs/draft-design.md §8.1). A 2 mm square's far corner
// a/2 − d is a Sterbenz subtraction, so the held corner is the exact corner of
// the held d and the corner's whole displacement is the gap between the held d
// and the d the stated taper denotes. Shown to fail first: with the span
// replaced by the point d (the band's dcDelta zeroed), every far corner read
// Exact with a zero bound while sitting about 1.1e-16 from its exact point.
func TestTaperFarCornerChargesTheTangentSpan(t *testing.T) {
	t.Parallel()
	const a, h, deg = 2.0, 10.0, 5.0
	s, p := taperSketch(t, r3.NewVec(0, 0, 0), 0, drawSquare(0, a))
	b := taperExtrude(t, decad.New(), s, p, h, deg, decad.Along)
	half := bfSub(bf(a/2), bfMul(bf(h), taperTan(deg)))
	for _, su := range []float64{-1, 1} {
		for _, sv := range []float64{-1, 1} {
			requireVertexAt(t, b, liftExact(xyFrame(t), r3.Identity(), bfMul(bf(su), half), bfMul(bf(sv), half), bf(h)))
		}
	}
}

// TestTaperVerify is DD5 and DD6: every drafted fixture verifies valid, built
// and solid, and the tolerance gate reads a diameter for each. Shown to fail
// first: without the gate's draft arm (verify_gate.go) every body read
// DiagToleranceReferenceUnavailable.
func TestTaperVerify(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	origin := r3.NewVec(0, 0, 0)
	var bodies []*decad.Body
	add := func(holes int, draw func(*sketch.Sketch), h, deg float64, dir decad.Direction) {
		s, p := taperSketch(t, origin, holes, draw)
		bodies = append(bodies, taperExtrude(t, doc, s, p, h, deg, dir))
	}
	add(0, drawSquare(0, 20), 10, 5, decad.Along)
	add(0, drawDisk(10), 10, 10, decad.Along)
	add(0, drawSlot(0, 60), 8, 3, decad.Along)
	add(0, drawPolygon([2]float64{100, 0}, [2]float64{120, 0}, [2]float64{120, 10}, [2]float64{110, 10}, [2]float64{110, 20}, [2]float64{100, 20}), 10, 5, decad.Along)
	add(0, drawSquare(200, 20), 10, -5, decad.Along)
	add(0, drawSquare(300, 20), 10, 5, decad.Against)
	add(1, drawRing(10, 4), 10, 5, decad.Against)
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	for _, diag := range report.Diagnostics {
		require.NotEqual(t, decad.DiagToleranceReferenceUnavailable, diag.Code, "%+v", diag)
	}
	for _, b := range bodies {
		br, err := report.ForBody(b)
		require.NoError(t, err)
		require.Equal(t, decad.ValidityValid, br.Validity.Outcome)
	}
}

// TestTaperRefusals is every Table SD row Extrude can reach (docs/draft-design.md
// §11): each returns its sentinel and leaves the document unchanged.
func TestTaperRefusals(t *testing.T) {
	t.Parallel()
	origin := r3.NewVec(0, 0, 0)
	cases := []struct {
		name  string
		holes int
		draw  func(*sketch.Sketch)
		e     decad.Extent
		opts  []decad.ExtrudeOption
		want  error
	}{
		{"SD2 right angle", 0, drawSquare(0, 20), decad.Distance{D: units.Millimeters(10), Dir: decad.Along},
			[]decad.ExtrudeOption{decad.WithTaper(units.Degrees(90))}, decad.ErrDegenerate},
		{"SD2 past a right angle", 0, drawSquare(0, 20), decad.Distance{D: units.Millimeters(10), Dir: decad.Along},
			[]decad.ExtrudeOption{decad.WithTaper(units.Degrees(-120))}, decad.ErrDegenerate},
		{"SD3 spline wall", 0, func(s *sketch.Sketch) {
			p0, p1 := s.CreatePoint(0, 0), s.CreatePoint(20, 0)
			s.Fix(p0)
			s.Fix(p1)
			s.CreateLine(p0, p1)
			if _, err := s.CreateSpline(p1, s.CreatePoint(20, 10), s.CreatePoint(10, 20), p0); err != nil {
				panic(err)
			}
		}, decad.Distance{D: units.Millimeters(10), Dir: decad.Along},
			[]decad.ExtrudeOption{decad.WithTaper(units.Degrees(5))}, decad.ErrUnsupported},
		{"SD4 bite meeting its lines at right angles", 0, func(s *sketch.Sketch) {
			pts := []*sketch.Point{s.CreatePoint(0, 0), s.CreatePoint(40, 0), s.CreatePoint(40, 20), s.CreatePoint(25, 20), s.CreatePoint(15, 20), s.CreatePoint(0, 20)}
			for _, p := range pts {
				s.Fix(p)
			}
			s.CreateLine(pts[0], pts[1])
			s.CreateLine(pts[1], pts[2])
			s.CreateLine(pts[2], pts[3])
			s.CreateArc(s.CreatePoint(20, 20), pts[4], pts[3])
			s.CreateLine(pts[4], pts[5])
			s.CreateLine(pts[5], pts[0])
		}, decad.Distance{D: units.Millimeters(10), Dir: decad.Along},
			[]decad.ExtrudeOption{decad.WithTaper(units.Degrees(5))}, decad.ErrUnsupported},
		{"SD5 cone apex inside the sweep", 0, drawDisk(5), decad.Distance{D: units.Millimeters(10), Dir: decad.Along},
			[]decad.ExtrudeOption{decad.WithTaper(units.Degrees(30))}, decad.ErrUnsupported},
		{"SD6 square consumed", 0, drawSquare(0, 20), decad.Distance{D: units.Millimeters(10), Dir: decad.Along},
			[]decad.ExtrudeOption{decad.WithTaper(units.Degrees(50))}, decad.ErrDegenerate},
		{"SD7 short walls consumed", 0, func(s *sketch.Sketch) {
			r := s.CreateRectangle(0, 0, 40, 4)
			s.Fix(r.A)
		}, decad.Distance{D: units.Millimeters(10), Dir: decad.Along},
			[]decad.ExtrudeOption{decad.WithTaper(units.Degrees(15))}, decad.ErrUnsupported},
		{"SD8 widened hole crossing the outer wall", 1, func(s *sketch.Sketch) {
			r := s.CreateRectangle(-20, -20, 20, 20)
			s.Fix(r.A)
			c := s.CreatePoint(14, 0)
			s.CreateCircle(c, 4)
			s.Fix(c)
		}, decad.Distance{D: units.Millimeters(10), Dir: decad.Along},
			[]decad.ExtrudeOption{decad.WithTaper(units.Degrees(9))}, decad.ErrUnsupported},
		// F7 with d closing the ring: the two concentric circles never cross,
		// so the far hole ends outside the far outer loop and the audit's
		// decided nesting row reads the region consumed.
		{"F7 ring closed", 1, drawRing(10, 4), decad.Distance{D: units.Millimeters(10), Dir: decad.Along},
			[]decad.ExtrudeOption{decad.WithTaper(units.Degrees(18))}, decad.ErrDegenerate},
		{"SD12 surface result", 0, drawSquare(0, 20), decad.Distance{D: units.Millimeters(10), Dir: decad.Along},
			[]decad.ExtrudeOption{decad.WithTaper(units.Degrees(5)), decad.WithSurfaceResult()}, decad.ErrUnsupported},
		{"SD13 amount rounds away", 0, drawSquare(0, 20), decad.Distance{D: units.Millimeters(1), Dir: decad.Along},
			[]decad.ExtrudeOption{decad.WithTaper(units.Degrees(1e-14))}, decad.ErrUnsupported},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			s, p := taperSketch(t, origin, tc.holes, tc.draw)
			doc := decad.New()
			_, err := doc.Extrude(s, p, tc.e, tc.opts...)
			require.ErrorIs(t, err, tc.want)
			require.Empty(t, doc.Bodies())
		})
	}
}

// TestTaperDownstreamRefusals pins what stays refused on a draft body
// (docs/draft-design.md Table DD): the modify ops refuse it by name (DD14),
// each leaving the body live. Placement and mirroring build (DD12);
// tessellation and the readings on it are draft_mesh_test.go's.
func TestTaperDownstreamRefusals(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	s, p := taperSketch(t, r3.NewVec(0, 0, 0), 0, drawSquare(0, 20))
	b := taperExtrude(t, doc, s, p, 10, 5, decad.Along)

	_, err := b.Fillet(t.Context(), decad.Edges(decad.CreatedBy(decad.CapEnd(b))), units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Contains(t, err.Error(), "draft body")
	_, err = b.Chamfer(t.Context(), decad.Edges(decad.CreatedBy(decad.CapEnd(b))), units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Contains(t, err.Error(), "draft body")
	_, err = b.Shell(t.Context(), decad.Faces(decad.FaceCreatedBy(decad.CapEnd(b))), units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Contains(t, err.Error(), "draft body")
	require.Equal(t, []*decad.Body{b}, doc.Bodies())

	vol, err := b.Volume()
	require.NoError(t, err)
	mirror, err := r3.NewFrame(r3.NewVec(30, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	mirrored, err := b.MirroredCopy(t.Context(), decad.MirrorFrame{Frame: mirror})
	require.NoError(t, err)
	requireManifold(t, mirrored)
	mv, err := mirrored.Volume()
	require.NoError(t, err)
	require.InDelta(t, vol.Value.Base(), mv.Value.Base(), vol.Bound.Base()+mv.Bound.Base())
	planes, _ := facesByKind(mirrored)
	for _, f := range planes {
		n, err := f.NormalAt(f.Loops()[0].Edges()[0].Start().Position().Value)
		require.NoError(t, err)
		require.Greater(t, n.Value.Z, 0.0, "a mirrored drafted wall still faces toward the far end")
	}
}
