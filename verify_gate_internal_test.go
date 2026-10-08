package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// gateFarOffset is how far gateFarPlacement translates a body from the
// world origin. Every coordinate a placed lift rounds is about this large.
const gateFarOffset = 1e6

// gateFarPlacement turns a body about a skew axis and moves it about
// gateFarOffset away from the world origin, so every lifted point rounds at
// that magnitude. i varies the turn.
func gateFarPlacement(t *testing.T, i int) r3.Transform {
	t.Helper()
	rotation, err := r3.Rotation(r3.NewVec(1, 2, 3+0.01*float64(i)), units.Radians(0.3+0.05*float64(i)))
	require.NoError(t, err)
	translation, err := r3.Translation(r3.NewVec(gateFarOffset+0.3, -gateFarOffset/1.5, 12345.678))
	require.NoError(t, err)
	placement, err := rotation.Then(translation)
	require.NoError(t, err)
	return placement
}

// gateFarSlack is how far below the exact diameter a placed reading may sit:
// a generous multiple of the float spacing at gateFarOffset, which is the
// size of every lift's rounding and so of every gap the reading charges.
func gateFarSlack() float64 {
	return 64 * (math.Nextafter(2*gateFarOffset, math.Inf(1)) - 2*gateFarOffset)
}

// exactPrismLift is the point pp denotes at plane-local (u, v) and level z:
// the frame and placement applied over exact rationals, each held float read
// as the exact value it is.
func exactPrismLift(pp prismPayload, u, v, z float64) [3]*big.Rat {
	rat := func(x float64) *big.Rat { return new(big.Rat).SetFloat64(x) }
	return exactLiftRat(pp, rat(u), rat(v), rat(z))
}

// exactLiftRat is exactPrismLift at rational plane-local coordinates.
func exactLiftRat(pp prismPayload, u, v, z *big.Rat) [3]*big.Rat {
	rat := func(x float64) *big.Rat { return new(big.Rat).SetFloat64(x) }
	coords := func(w r3.Vec) [3]float64 { return [3]float64{w.X, w.Y, w.Z} }
	o, fu, fv, fn := coords(pp.frame.Origin()), coords(pp.frame.U()), coords(pp.frame.V()), coords(pp.frame.N())
	var local [3]*big.Rat
	for i := range local {
		s := rat(o[i])
		s.Add(s, new(big.Rat).Mul(rat(fu[i]), u))
		s.Add(s, new(big.Rat).Mul(rat(fv[i]), v))
		local[i] = s.Add(s, new(big.Rat).Mul(rat(fn[i]), z))
	}
	basis := pp.xform.Basis()
	ex, ey, ez, tr := coords(basis.EX), coords(basis.EY), coords(basis.EZ), coords(pp.xform.Translation())
	var out [3]*big.Rat
	for i := range out {
		s := rat(tr[i])
		s.Add(s, new(big.Rat).Mul(rat(ex[i]), local[0]))
		s.Add(s, new(big.Rat).Mul(rat(ey[i]), local[1]))
		out[i] = s.Add(s, new(big.Rat).Mul(rat(ez[i]), local[2]))
	}
	return out
}

// exactPrismDiameterSquare is the exact square of the diameter of the prism pp
// denotes over a convex polygonal section with the given vertices: the image
// of a convex polytope under the placement is one, so its diameter is
// realized between two of its vertices.
func exactPrismDiameterSquare(pp prismPayload, section [][2]float64) *big.Rat {
	var corners [][3]*big.Rat
	for _, uv := range section {
		corners = append(corners, exactPrismLift(pp, uv[0], uv[1], pp.z0), exactPrismLift(pp, uv[0], uv[1], pp.z1))
	}
	best := new(big.Rat)
	for i := range corners {
		for j := i + 1; j < len(corners); j++ {
			s := new(big.Rat)
			for k := range 3 {
				d := new(big.Rat).Sub(corners[i][k], corners[j][k])
				s.Add(s, d.Mul(d, d))
			}
			if s.Cmp(best) > 0 {
				best = s
			}
		}
	}
	return best
}

// placedStretchSquare is an exact upper bound on how much x's held basis B
// stretches a squared length: |Bw|^2 = w'(B'B)w <= (1 + ||B'B - I||) |w|^2, and
// the Frobenius norm bounds the spectral one. B is orthonormal only up to
// rounding, so the bound sits just above one.
func placedStretchSquare(x r3.Transform) *big.Rat {
	basis := x.Basis()
	cols := [3]r3.Vec{basis.EX, basis.EY, basis.EZ}
	frobenius := new(big.Rat)
	for i := range cols {
		for j := range cols {
			e := new(big.Rat)
			for _, pair := range [3][2]float64{{cols[i].X, cols[j].X}, {cols[i].Y, cols[j].Y}, {cols[i].Z, cols[j].Z}} {
				e.Add(e, new(big.Rat).Mul(new(big.Rat).SetFloat64(pair[0]), new(big.Rat).SetFloat64(pair[1])))
			}
			if i == j {
				e.Sub(e, big.NewRat(1, 1))
			}
			frobenius.Add(frobenius, e.Mul(e, e))
		}
	}
	f, _ := frobenius.Float64()
	root := math.Sqrt(f)
	for {
		r := new(big.Rat).SetFloat64(root)
		if new(big.Rat).Mul(r, r).Cmp(frobenius) >= 0 {
			return r.Add(r, big.NewRat(1, 1))
		}
		root = math.Nextafter(root, math.Inf(1))
	}
}

// requireGateDiameterWithin asserts lower <= d and d^2 <= upperSquare,
// the latter decided exactly.
func requireGateDiameterWithin(t *testing.T, d, lower float64, upperSquare *big.Rat) {
	t.Helper()
	square := new(big.Rat).SetFloat64(d)
	square.Mul(square, square)
	require.LessOrEqual(t, square.Cmp(upperSquare), 0,
		`the gate diameter %.17g must stay at or below the body's own diameter`, d)
	require.GreaterOrEqual(t, d, lower, `the gate diameter must reach the floor its stations guarantee`)
}

func ratSquare(x float64) *big.Rat {
	r := new(big.Rat).SetFloat64(x)
	return r.Mul(r, r)
}

func floatSqrtOf(r *big.Rat) float64 {
	f, _ := r.Float64()
	return math.Sqrt(f)
}

// A cap-loop chamfer cuts the receiver's walls back from each chamfered cap,
// so the receiver's own corners at the cap level are not points of the body.
// The gate reads each loop's wall only between its bands' side levels.
//
// The 10 mm cube chamfered 2 mm around its end cap keeps its bottom corners
// and its walls' corners at z = 8; its diameter is sqrt(10^2 + 10^2 + 8^2) =
// sqrt(264), which the side-level stations read exactly. The disc of radius
// 5, extruded 20 and chamfered 2 around its end cap, spans the bottom rim,
// the rim at the side level z = 18 and the cap contour of radius 3 at z = 20.
// Its diameter is the largest of (r1 + r2)^2 + dz^2 over those three circles,
// (5 + 3)^2 + 20^2 = 464, and the stations at most 15 degrees apart at z = 0
// and z = 18 reach at least sqrt((2*5*cos(7.5 degrees))^2 + 18^2).
//
// Shown to fail first: reading the receiver's section over its whole
// interval (the earlier carrier reading) gives the cube 17.320508 against its
// diameter 16.248077, and the disc 22.360680 against 21.540659. Reading the
// chamfered end loop's wall up to z1, rather than to its side level, sends
// both red.
func TestCapBlendGateDiameterReadsSideLevels(t *testing.T) {
	t.Parallel()
	t.Run("cube", func(t *testing.T) {
		t.Parallel()
		doc := New()
		body := axisBoxBody(t, doc, 0, 0, 10, 10, 10)
		chamfered, err := body.Chamfer(t.Context(), Edges(CreatedBy(CapEnd(body))), units.Millimeters(2))
		require.NoError(t, err)
		_, isCapBlend := chamfered.payload.(capBlendPayload)
		require.True(t, isCapBlend)

		d, ok, err := bodyGateDiameter(t.Context(), chamfered)
		require.NoError(t, err)
		require.True(t, ok)
		requireGateDiameterWithin(t, d, math.Nextafter(math.Sqrt(264), 0), big.NewRat(264, 1))
	})
	t.Run("disc", func(t *testing.T) {
		t.Parallel()
		const r, h, setback = 5.0, 20.0, 2.0
		chamfered, _ := chamferedSectionBody(t, func(s *sketch.Sketch) {
			c := s.CreatePoint(0, 0)
			s.Fix(c)
			s.CreateCircle(c, r)
		}, setback)
		require.Equal(t, h, capBlendMeshHeight)

		d, ok, err := bodyGateDiameter(t.Context(), chamfered)
		require.NoError(t, err)
		require.True(t, ok)
		upper := new(big.Rat).Add(ratSquare(2*r-setback), ratSquare(h))
		requireGateDiameterWithin(t, d, math.Hypot(2*r*math.Cos(gateStationStep/2), h-setback), upper)
	})
}

// Every point a gate diameter reads must carry a proven gap from a point of
// the body, and the reading must be shrunk by it. A placement far from the
// origin rounds every lifted point at that magnitude, about 1e-10 here, so a
// reading over uncharged points lands above the placed body's own diameter.
// Each case places a body there and asserts the reading stays at or below
// the exact diameter of the body the record denotes, decided over rationals,
// and within gateFarSlack of it.
//
// Shown to fail first, with the reading each case took before every point
// was charged (amd64): the first placed box reads 9.1104335792652247 through
// the exact carrier path, 9.1104335792376112 through its brep and
// 9.1104335792634039 through the displaced fallback, all above its exact
// diameter 9.1104335791442992. The first placed free-form triangle reads
// 14.142135623751139 against 14.142135623730951, and the placed full revolve
// 5.0000000000078026 against 5. The unplaced 90 degree revolve reads
// 4.527692569094917 against sqrt(20.5) = 4.527692569068709, and the 270
// degree one 4.8664135, below the floor its half-turn stations reach. The
// draft arm's earlier vertex reading stays at or below the placed square
// frustum's diameter, but reads the placed tapered disc's seam alone:
// 8.0109788 against a floor of 12.4816079.
//
// Legs shown to fail: dropping the stations' gap from stationGateDiameter's
// shrink sends the prism and the displaced prism red; dropping
// revolveGateDiameter's shrink sends every revolve red; dropping the vertex
// bounds and station gaps from brepGateDiameter's shrink sends the brep red;
// and charging the free-form arm each witness's walk-end bound in place of
// its lift's gap sends the free-form triangle red. The draft's farDelta charge
// is not separately observable here: the upper bound reads the least offset
// the record denotes, which leaves more room than farDelta takes.
func TestPlacedGateDiameterChargesEveryPoint(t *testing.T) {
	t.Parallel()
	const x0, y0, w, l, h = 1.0, 2.0, 3.0, 7.0, 5.0
	box := [][2]float64{{x0, y0}, {x0 + w, y0}, {x0 + w, y0 + l}, {x0, y0 + l}}
	placements := 8

	placedBox := func(t *testing.T, i int) *Body {
		t.Helper()
		doc := New()
		body := axisBoxBody(t, doc, x0, y0, x0+w, y0+l, h)
		placed, err := body.Placed(t.Context(), gateFarPlacement(t, i))
		require.NoError(t, err)
		return placed
	}
	read := func(t *testing.T, body *Body) float64 {
		t.Helper()
		d, ok, err := bodyGateDiameter(t.Context(), body)
		require.NoError(t, err)
		require.True(t, ok)
		return d
	}

	t.Run("prism", func(t *testing.T) {
		t.Parallel()
		for i := range placements {
			body := placedBox(t, i)
			pp, isPrism := body.payload.(prismPayload)
			require.True(t, isPrism)
			_, modelled := newBodyGeom(body)
			require.True(t, modelled, `the exact carrier model reads this prism`)
			upper := exactPrismDiameterSquare(pp, box)
			requireGateDiameterWithin(t, read(t, body), floatSqrtOf(upper)-gateFarSlack(), upper)
		}
	})
	t.Run("displaced prism", func(t *testing.T) {
		t.Parallel()
		for i := range placements {
			pp := placedBox(t, i).payload.(prismPayload)
			pp.sectionDelta = math.Ldexp(1, -40)
			// The held section is the denoted one moved by at most
			// sectionDelta, so the reading is shrunk below the held
			// section's own diameter, which bounds it from above.
			upper := exactPrismDiameterSquare(pp, box)
			requireGateDiameterWithin(t, read(t, &Body{payload: pp}), floatSqrtOf(upper)-gateFarSlack(), upper)
		}
	})
	t.Run("brep", func(t *testing.T) {
		t.Parallel()
		for i := range placements {
			body := placedBox(t, i)
			pp := body.payload.(prismPayload)
			brep := internalBrepBody(t, body)
			upper := exactPrismDiameterSquare(pp, box)
			requireGateDiameterWithin(t, read(t, brep), floatSqrtOf(upper)-gateFarSlack(), upper)
		}
	})
	t.Run("free-form", func(t *testing.T) {
		t.Parallel()
		for i := range placements {
			pp := freeformTrianglePayload(t, 1, 11)
			pp.xform = gateFarPlacement(t, i)
			upper := exactPrismDiameterSquare(pp, [][2]float64{{0, 0}, {5, 4}, {10, 0}})
			requireGateDiameterWithin(t, read(t, &Body{payload: pp}), floatSqrtOf(upper)-gateFarSlack(), upper)
		}
	})
	t.Run("draft", func(t *testing.T) {
		t.Parallel()
		// A 10 mm square swept 8 mm at a 3 degree taper is a frustum whose far
		// square is the near one offset inward by d = 8*tan(3 degrees). Every
		// distance between its eight corners shrinks as d grows, so the
		// corners offset by d - dDelta, the least offset the record denotes,
		// bound its diameter from above.
		for i := range placements {
			body := draftSquare(t)
			dp, isDraft := body.payload.(draftPayload)
			require.True(t, isDraft)
			placed, err := body.Placed(t.Context(), gateFarPlacement(t, i))
			require.NoError(t, err)
			pdp := placed.payload.(draftPayload)
			nearZ, _, farZ, _ := pdp.levels()
			view := prismPayload{frame: pdp.frame, xform: pdp.xform, z0: nearZ, z1: farZ}
			least := new(big.Rat).Sub(new(big.Rat).SetFloat64(dp.d), new(big.Rat).SetFloat64(dp.dDelta))
			upper := exactDraftSquareDiameterSquare(view, 5, least)
			requireGateDiameterWithin(t, read(t, placed), floatSqrtOf(upper)-gateFarSlack(), upper)
		}
	})
	t.Run("draft disc", func(t *testing.T) {
		t.Parallel()
		// A disc of radius 5 swept 8 mm at a 3 degree taper is a frustum
		// whose far rim has radius 5 - d. Its diameter is the larger of the
		// near rim's 10 and the antipodal rim pair's sqrt((10 - d)^2 + 8^2),
		// widest at the least offset the record denotes. Stations antipodal
		// on both rims reach the second at the greatest offset.
		for i := range placements {
			w := sketch.NewWorld()
			s, err := w.CreateSketch(w.XY())
			require.NoError(t, err)
			c := s.CreatePoint(0, 0)
			s.Fix(c)
			s.CreateCircle(c, 5)
			_, err = s.Solve(t.Context())
			require.NoError(t, err)
			body, err := New().Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(8), Dir: Along}, WithTaper(units.Degrees(3)))
			require.NoError(t, err)
			dp := body.payload.(draftPayload)
			placement := gateFarPlacement(t, i)
			placed, err := body.Placed(t.Context(), placement)
			require.NoError(t, err)
			least := new(big.Rat).Sub(new(big.Rat).SetFloat64(dp.d), new(big.Rat).SetFloat64(dp.dDelta))
			across := new(big.Rat).Sub(big.NewRat(10, 1), least)
			local := new(big.Rat).Add(across.Mul(across, across), big.NewRat(64, 1))
			upper := new(big.Rat).Mul(local, placedStretchSquare(placement))
			lower := math.Hypot(10-(dp.d+dp.dDelta), 8) - gateFarSlack()
			requireGateDiameterWithin(t, read(t, placed), lower, upper)
		}
	})
	t.Run("revolve", func(t *testing.T) {
		t.Parallel()
		// A rectangle 4 mm along the axis and r out from it, swept by
		// sweepDeg: two rim points at angle dphi apart are
		// sqrt(16 + 2r^2(1 - cos(dphi))) apart, widest at dphi =
		// min(sweep, 180 degrees). The placed body's diameter is at most
		// that times the placement's stretch.
		for _, tc := range []struct {
			name     string
			extent   AngularExtent
			cosWidth int64
		}{
			{"full", FullRevolution{}, -1},
			{"270 degrees", AngleExtent{A: units.Degrees(270), Dir: Along}, -1},
			{"90 degrees", AngleExtent{A: units.Degrees(90), Dir: Along}, 0},
		} {
			t.Run(tc.name, func(t *testing.T) {
				t.Parallel()
				for i := range placements {
					r := 1.5 + 0.1*float64(i)
					body := sectionSketchBody(t, func(s *sketch.Sketch) {
						s.Fix(s.CreateRectangle(0, 0, 4, r).A)
					}, tc.extent)
					local := new(big.Rat).Mul(ratSquare(r), big.NewRat(2*(1-tc.cosWidth), 1))
					local.Add(local, big.NewRat(16, 1))
					requireGateDiameterWithin(t, read(t, body), floatSqrtOf(local)-gateFarSlack(), local)

					placement := gateFarPlacement(t, i)
					placed, err := body.Placed(t.Context(), placement)
					require.NoError(t, err)
					upper := new(big.Rat).Mul(local, placedStretchSquare(placement))
					requireGateDiameterWithin(t, read(t, placed), floatSqrtOf(upper)-gateFarSlack(), upper)
				}
			})
		}
	})
}

// draftSquare extrudes a 10 mm square about the origin 8 mm along +z at a 3
// degree taper.
func draftSquare(t *testing.T) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	s.Fix(s.CreateRectangle(-5, -5, 5, 5).A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := New().Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(8), Dir: Along}, WithTaper(units.Degrees(3)))
	require.NoError(t, err)
	return body
}

// exactDraftSquareDiameterSquare is the exact square of the diameter of the
// frustum view denotes between the square of half-width half at view.z0 and
// the one offset inward by offset at view.z1, placed by view's frame and
// placement over exact rationals.
func exactDraftSquareDiameterSquare(view prismPayload, half float64, offset *big.Rat) *big.Rat {
	var corners [][3]*big.Rat
	for _, su := range []float64{-1, 1} {
		for _, sv := range []float64{-1, 1} {
			corners = append(corners, exactPrismLift(view, su*half, sv*half, view.z0))
		}
	}
	far := new(big.Rat).Sub(new(big.Rat).SetFloat64(half), offset)
	for _, su := range []int64{-1, 1} {
		for _, sv := range []int64{-1, 1} {
			corners = append(corners, exactLiftRat(view, new(big.Rat).Mul(far, big.NewRat(su, 1)),
				new(big.Rat).Mul(far, big.NewRat(sv, 1)), new(big.Rat).SetFloat64(view.z1)))
		}
	}
	best := new(big.Rat)
	for i := range corners {
		for j := i + 1; j < len(corners); j++ {
			s := new(big.Rat)
			for k := range 3 {
				d := new(big.Rat).Sub(corners[i][k], corners[j][k])
				s.Add(s, d.Mul(d, d))
			}
			if s.Cmp(best) > 0 {
				best = s
			}
		}
	}
	return best
}
