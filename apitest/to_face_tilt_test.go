package apitest_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestToFaceTiltedStopIsChargedOrRefused pins the rule a ToFace stop obeys when
// the stop face's plane is not exactly perpendicular to the sweep. The prism
// cap a ToFace stop builds is perpendicular to the sweep at one level, so a
// stop plane tilted by any nonzero angle meets the footprint at a level that
// varies across it. The stop must therefore either refuse the face
// (ErrUnsupported, document unchanged) or publish bounds that cover that
// variation. Measuring it needs no tolerance: the stop plate is extruded on a
// sketch plane whose frame is U = (1, 0, -t), V = (0, 1, 0), so its far cap
// denotes the exact plane through 10·N spanned by U and V, and the gap
// between that plane and every built reading is computed in big.Rat.
//
// The exactly parallel control (t = 0) must build with zero gap and zero
// bound, which is what keeps the rule keyed to the tilt and not to ToFace.
func TestToFaceTiltedStopIsChargedOrRefused(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name string
		tilt float64
		// plate is the stop plate's extent along U; the pin footprint is the
		// 20×20 square under the plate's far edge, x in [plate-20, plate].
		plate float64
	}{
		{name: "exactly parallel", tilt: 0, plate: 100},
		{name: "tilt 1e-12", tilt: 1e-12, plate: 100},
		{name: "tilt 1e-10", tilt: 1e-10, plate: 100},
		{name: "tilt 9e-10 wide plate", tilt: 9e-10, plate: 1e4},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			w := sketch.NewWorld()
			frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, -tc.tilt), r3.NewVec(0, 1, 0))
			require.NoError(t, err)
			tilted, err := w.CreatePlaneFromFrame(frame)
			require.NoError(t, err)
			plateSketch, err := w.CreateSketch(tilted)
			require.NoError(t, err)
			plateRect := plateSketch.CreateRectangle(0, 0, tc.plate, 60)
			plateSketch.Fix(plateRect.A)
			_, err = plateSketch.Solve(t.Context())
			require.NoError(t, err)
			require.Len(t, plateSketch.Profiles(), 1)

			pinSketch, err := w.CreateSketch(w.XY())
			require.NoError(t, err)
			x0, x1 := tc.plate-20, tc.plate
			pinRect := pinSketch.CreateRectangle(x0, 0, x1, 20)
			pinSketch.Fix(pinRect.A)
			_, err = pinSketch.Solve(t.Context())
			require.NoError(t, err)
			require.Len(t, pinSketch.Profiles(), 1)

			doc := decad.New()
			plate, err := decad.Extrude(t.Context(), doc, plateSketch, plateSketch.Profiles()[0],
				decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
			require.NoError(t, err)
			stopFace := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(plate)))
			faces, err := stopFace.SelectFaces(plate)
			require.NoError(t, err)
			require.Len(t, faces, 1)
			stopPlane, ok := faces[0].Surface().(decad.Plane)
			require.True(t, ok)
			require.Equal(t, frame.N(), stopPlane.Frame.N(), `the stop cap's tag carries the plate frame's normal`)
			t.Logf("stop tag origin %v normal %v", stopPlane.Frame.Origin(), stopPlane.Frame.N())

			// The far cap denotes {u·U + v·V + 10·N}: normal n = U×V, exact.
			u, v, nHeld := tiltRatVec(frame.U()), tiltRatVec(frame.V()), tiltRatVec(frame.N())
			n := tiltRatCross(u, v)
			level := tiltRatScale(nHeld, big.NewRat(10, 1))
			// zTrue is the true stop plane's height over (x, y):
			// n·((x, y, z) − 10·N) = 0.
			nDotP := tiltRatDot(n, level)
			zTrue := func(x, y *big.Rat) *big.Rat {
				num := new(big.Rat).Sub(nDotP, new(big.Rat).Mul(n[0], x))
				num.Sub(num, new(big.Rat).Mul(n[1], y))
				return num.Quo(num, n[2])
			}

			before := len(doc.Bodies())
			pin, err := decad.Extrude(t.Context(), doc, pinSketch, pinSketch.Profiles()[0],
				decad.ToFace{Body: plate, Face: stopFace})
			if err != nil {
				require.NotZero(t, tc.tilt, `the exactly parallel stop must build`)
				require.ErrorIs(t, err, decad.ErrUnsupported)
				require.Len(t, doc.Bodies(), before, `a refused stop leaves the document unchanged`)
				t.Logf("refused: %v", err)
				return
			}

			// Top vertices: exact gap along the sweep against the vertex bound.
			top, uncovered := 0, 0
			maxGap := new(big.Rat)
			for _, vertex := range pin.Vertices() {
				p := vertex.Position()
				if p.Value.Z == 0 {
					continue
				}
				top++
				gap := tiltRatAbs(new(big.Rat).Sub(tiltRat(p.Value.Z), zTrue(tiltRat(p.Value.X), tiltRat(p.Value.Y))))
				if gap.Cmp(maxGap) > 0 {
					maxGap = gap
				}
				bound := tiltLengthRat(t, p.Bound)
				t.Logf("vertex %v: gap %s mm, published bound %s mm (%s)",
					p.Value, tiltRatE(gap), tiltRatE(bound), p.Exactness)
				if gap.Cmp(bound) > 0 {
					uncovered++
				}
			}
			require.Equal(t, 4, top)

			// Volume: the true solid is the footprint swept to the true plane,
			// area × zTrue(centroid) because zTrue is linear.
			area := new(big.Rat).Mul(tiltRat(x1-x0), big.NewRat(20, 1))
			cx := new(big.Rat).Quo(new(big.Rat).Add(tiltRat(x0), tiltRat(x1)), big.NewRat(2, 1))
			trueVolume := new(big.Rat).Mul(area, zTrue(cx, big.NewRat(10, 1)))
			vol, err := pin.Volume()
			require.NoError(t, err)
			volValue, err := vol.Value.In(units.CubicMillimeter)
			require.NoError(t, err)
			volBound, err := vol.Bound.In(units.CubicMillimeter)
			require.NoError(t, err)
			volGap := tiltRatAbs(new(big.Rat).Sub(tiltRat(volValue), trueVolume))
			t.Logf("volume: gap %s mm^3, published bound %s mm^3 (%s)", tiltRatE(volGap), tiltRatE(tiltRat(volBound)), vol.Exactness)

			// Bounds: the true solid's top reaches max zTrue over the footprint.
			box, err := pin.Bounds()
			require.NoError(t, err)
			boxBound := tiltLengthRat(t, box.Bound)
			trueTop := zTrue(tiltRat(x0), new(big.Rat))
			for _, x := range []float64{x0, x1} {
				for _, y := range []float64{0, 20} {
					if z := zTrue(tiltRat(x), tiltRat(y)); z.Cmp(trueTop) > 0 {
						trueTop = z
					}
				}
			}
			boxGap := tiltRatAbs(new(big.Rat).Sub(tiltRat(box.Max.Z), trueTop))
			t.Logf("bounds max z: gap %s mm, published bound %s mm (%s)", tiltRatE(boxGap), tiltRatE(boxBound), box.Exactness)

			// The stopped cap face: its normal against the true plane's unit
			// normal, compared exactly through |n̂ − m|² = 2 − 2·(n·m)/|n| for
			// unit m, and its area against the true tilted cap's area·|n|/n_z.
			capFaces, err := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(pin))).SelectFaces(pin)
			require.NoError(t, err)
			require.Len(t, capFaces, 1)
			normal, err := capFaces[0].NormalAt(r3.NewVec(x0+10, 10, box.Max.Z))
			require.NoError(t, err)
			normalBound, err := normal.Bound.In(units.One)
			require.NoError(t, err)
			normalCovered := tiltNormalWithin(n, tiltRatVec(normal.Value), tiltRat(normalBound))
			t.Logf("cap normal %v: true unit normal ∝ %v, published bound %g (%s), covered %v",
				normal.Value, tiltNFloat(n), normalBound, normal.Exactness, normalCovered)

			capArea, err := capFaces[0].Area()
			require.NoError(t, err)
			capValue, err := capArea.Value.In(units.SquareMillimeter)
			require.NoError(t, err)
			capBound, err := capArea.Bound.In(units.SquareMillimeter)
			require.NoError(t, err)
			areaGap := tiltAreaGap(area, n, tiltRat(capValue))
			t.Logf("cap area %g: gap ≈ %.3e mm^2, published bound %g mm^2 (%s)", capValue, areaGap, capBound, capArea.Exactness)

			t.Logf("max vertex gap %s mm", tiltRatE(maxGap))
			// Every reading is measured and logged before the first assertion,
			// so one run reports each channel the stop feeds.
			require.Zero(t, uncovered, `each vertex's published bound must cover its distance from the true stop plane`)
			require.LessOrEqual(t, volGap.Cmp(tiltRat(volBound)), 0,
				`the published volume bound must cover the true stopped solid`)
			require.LessOrEqual(t, boxGap.Cmp(boxBound), 0, `the published box bound must cover the true top`)
			require.True(t, normalCovered, `the published normal bound must cover the true stop plane's normal`)
			require.LessOrEqual(t, areaGap, capBound, `the published cap area bound must cover the true tilted cap`)
		})
	}
}

// TestToFaceRefusesTiltPastOldTolerance pins that a stop tilted by 1.1e-9, past
// the cross-product ratio the gate once accepted, is refused.
func TestToFaceRefusesTiltPastOldTolerance(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, -1.1e-9), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	tilted, err := w.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	plateSketch, err := w.CreateSketch(tilted)
	require.NoError(t, err)
	plateRect := plateSketch.CreateRectangle(0, 0, 100, 60)
	plateSketch.Fix(plateRect.A)
	_, err = plateSketch.Solve(t.Context())
	require.NoError(t, err)
	pinSketch, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	pinRect := pinSketch.CreateRectangle(80, 0, 100, 20)
	pinSketch.Fix(pinRect.A)
	_, err = pinSketch.Solve(t.Context())
	require.NoError(t, err)

	doc := decad.New()
	plate, err := decad.Extrude(t.Context(), doc, plateSketch, plateSketch.Profiles()[0],
		decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	_, err = decad.Extrude(t.Context(), doc, pinSketch, pinSketch.Profiles()[0],
		decad.ToFace{Body: plate, Face: decad.Faces(decad.FaceCreatedBy(decad.CapEnd(plate)))})
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

// TestToFacePlacedStopNormal pins the exact-parallel rule on a stop body moved
// by a placement. A 180 degree turn about X rounds the cap's float normal off
// the sweep axis by about 1.2e-16, so the stop is refused. A 90 degree turn
// about Z keeps the normal exact, so the stop still builds with no bound.
func TestToFacePlacedStopNormal(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name  string
		axis  r3.Vec
		angle float64
		build bool
	}{
		{name: "flipped stop", axis: r3.NewVec(1, 0, 0), angle: 180},
		{name: "quarter turn stop", axis: r3.NewVec(0, 0, 1), angle: 90, build: true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			s, plateProf, pinProf := plateAndPin(t)
			doc := decad.New()
			plate, err := decad.Extrude(t.Context(), doc, s, plateProf,
				decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
			require.NoError(t, err)
			turn, err := r3.Rotation(tc.axis, units.Degrees(tc.angle))
			require.NoError(t, err)
			placed, err := decad.Transform(t.Context(), plate, turn)
			require.NoError(t, err)

			before := len(doc.Bodies())
			pin, err := decad.Extrude(t.Context(), doc, s, pinProf,
				decad.ToFace{Body: placed, Face: capEndFace(placed)})
			if !tc.build {
				require.ErrorIs(t, err, decad.ErrUnsupported)
				require.Len(t, doc.Bodies(), before, `a refused stop leaves the document unchanged`)
				return
			}
			require.NoError(t, err)
			box, err := pin.Bounds()
			require.NoError(t, err)
			require.Zero(t, box.Bound.Mag())
			require.Equal(t, decad.Exact, box.Exactness)
			require.Equal(t, 10.0, box.Max.Z)
		})
	}
}

func tiltRat(x float64) *big.Rat { return new(big.Rat).SetFloat64(x) }

func tiltRatAbs(x *big.Rat) *big.Rat { return x.Abs(x) }

func tiltRatVec(v r3.Vec) [3]*big.Rat { return [3]*big.Rat{tiltRat(v.X), tiltRat(v.Y), tiltRat(v.Z)} }

func tiltRatCross(a, b [3]*big.Rat) [3]*big.Rat {
	c := func(i, j int) *big.Rat {
		return new(big.Rat).Sub(new(big.Rat).Mul(a[i], b[j]), new(big.Rat).Mul(a[j], b[i]))
	}
	return [3]*big.Rat{c(1, 2), c(2, 0), c(0, 1)}
}

func tiltRatDot(a, b [3]*big.Rat) *big.Rat {
	s := new(big.Rat)
	for i := range a {
		s.Add(s, new(big.Rat).Mul(a[i], b[i]))
	}
	return s
}

func tiltRatScale(a [3]*big.Rat, k *big.Rat) [3]*big.Rat {
	return [3]*big.Rat{new(big.Rat).Mul(a[0], k), new(big.Rat).Mul(a[1], k), new(big.Rat).Mul(a[2], k)}
}

func tiltRatE(x *big.Rat) string {
	f, _ := x.Float64()
	return big.NewFloat(f).Text('e', 4)
}

func tiltNFloat(n [3]*big.Rat) r3.Vec {
	x, _ := n[0].Float64()
	y, _ := n[1].Float64()
	z, _ := n[2].Float64()
	return r3.NewVec(x, y, z)
}

func tiltLengthRat(t *testing.T, v units.Value) *big.Rat {
	t.Helper()
	if v.Mag() == 0 {
		return new(big.Rat)
	}
	mm, err := v.In(units.Millimeter)
	require.NoError(t, err)
	return tiltRat(mm)
}

// normalWithin reports whether |n/|n| − m| ≤ b exactly, for a held reading m
// and bound b. With s = n·m, |n̂ − m|² = 1 + |m|² − 2·s/|n|, so the reading is
// covered iff 2·s/|n| ≥ 1 + |m|² − b², which squares exactly when both sides
// are nonnegative.
func tiltNormalWithin(n, m [3]*big.Rat, b *big.Rat) bool {
	s := tiltRatDot(n, m)
	rhs := new(big.Rat).Add(big.NewRat(1, 1), tiltRatDot(m, m))
	rhs.Sub(rhs, new(big.Rat).Mul(b, b))
	if rhs.Sign() <= 0 {
		return true
	}
	if s.Sign() <= 0 {
		return false
	}
	// 2s/|n| ≥ rhs  ⇔  4s² ≥ rhs²·|n|².
	lhs := new(big.Rat).Mul(big.NewRat(4, 1), new(big.Rat).Mul(s, s))
	return lhs.Cmp(new(big.Rat).Mul(new(big.Rat).Mul(rhs, rhs), tiltRatDot(n, n))) >= 0
}

// tiltedAreaGap returns |area·|n|/n_z − held|, evaluated at 400 bits: the true
// cap over the footprint is the footprint area divided by cos θ.
func tiltAreaGap(area *big.Rat, n [3]*big.Rat, held *big.Rat) float64 {
	const prec = 400
	nn := new(big.Float).SetPrec(prec).SetRat(tiltRatDot(n, n))
	norm := new(big.Float).SetPrec(prec).Sqrt(nn)
	trueArea := new(big.Float).SetPrec(prec).SetRat(area)
	trueArea.Mul(trueArea, norm)
	trueArea.Quo(trueArea, new(big.Float).SetPrec(prec).SetRat(n[2]))
	gap := trueArea.Sub(trueArea, new(big.Float).SetPrec(prec).SetRat(held))
	f, _ := gap.Abs(gap).Float64()
	return f
}
