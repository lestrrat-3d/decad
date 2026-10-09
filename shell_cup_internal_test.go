package decad

import (
	"math"
	"math/big"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/radiussurvey"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestCupPayloadForTracksEachSourceEndDisplacement covers both opening ends and
// shell senses. The top end is computed while the bottom is stated exactly, so
// a floor derived from the latter must not inherit the former's displacement.
func TestCupPayloadForTracksEachSourceEndDisplacement(t *testing.T) {
	t.Parallel()
	const (
		thickness      = 0.1
		thicknessDelta = 0.03125
	)
	pp := prismPayload{
		z0:      0,
		z1:      1e12,
		z1Delta: 0.25,
	}
	floorDelta := func(from, sourceDelta, by float64) float64 {
		to := from + by
		return proofbound.AbsSumUpper(sourceDelta, thicknessDelta, proofarith.AddRoundError(from, by, to))
	}
	topFloorDelta := floorDelta(pp.z0, pp.z0Delta, thickness)
	bottomInFloorDelta := floorDelta(pp.z1, pp.z1Delta, -thickness)
	bottomOutFloorDelta := floorDelta(pp.z1, pp.z1Delta, thickness)

	tests := []struct {
		name                            string
		sense                           float64
		removedEnd                      bool
		openDelta, outerDelta, cavDelta float64
	}{
		{
			name: "top inward", sense: 1, removedEnd: true,
			openDelta: pp.z1Delta, outerDelta: pp.z0Delta, cavDelta: topFloorDelta,
		},
		{
			name: "top outward", sense: -1, removedEnd: true,
			openDelta: pp.z1Delta, outerDelta: topFloorDelta, cavDelta: pp.z0Delta,
		},
		{
			name: "bottom inward", sense: 1,
			openDelta: pp.z0Delta, outerDelta: pp.z1Delta, cavDelta: bottomInFloorDelta,
		},
		{
			name: "bottom outward", sense: -1,
			openDelta: pp.z0Delta, outerDelta: bottomOutFloorDelta, cavDelta: pp.z1Delta,
		},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			cp := cupPayloadFor(pp, ProfileRecord{}, tt.sense, thickness, thicknessDelta, 0, tt.removedEnd)
			require.Equal(t, tt.openDelta, cp.openScalar().Bound)
			require.Equal(t, tt.outerDelta, cp.outerScalar().Bound)
			require.Equal(t, tt.cavDelta, cp.cavityScalar().Bound)

			requireCupPrismLevelBounds(t, cp.outerPrism(), cp.outerScalar(), cp.openScalar())
			requireCupPrismLevelBounds(t, cp.cavityPrism(), cp.cavityScalar(), cp.openScalar())
		})
	}
}

func requireCupPrismLevelBounds(t *testing.T, prism prismPayload, a, b proofbound.BoundedScalar) {
	t.Helper()
	if a.Value <= b.Value {
		require.Equal(t, a.Bound, prism.z0Delta)
		require.Equal(t, b.Bound, prism.z1Delta)
		return
	}
	require.Equal(t, b.Bound, prism.z0Delta)
	require.Equal(t, a.Bound, prism.z1Delta)
}

// The displaced-cup fixtures give a recorded cup an offset displacement of
// 1/1024 mm, large enough that every leg it feeds dominates rounding, and
// check each reading against every cup the record can denote: each wall of
// the offset region moved independently by −δ/2, 0 or +δ/2, so no corner
// moves farther than δ. The outer region is
// 16×8 and the cavity 12×4, both centred on the frame origin; inward the
// cavity is the offset region on z ∈ [2, 10] under an outer prism on
// [0, 10], outward the outer region is on [−2, 10] over a cavity on [0, 10].
//
// Legs shown to fail (each deleted, the fixture watched go red, then
// restored):
//   - stackedRegionPart's column-displacement area term: the Volume and the
//     rim area miss, inward and outward.
//   - stackedRegionPart's column-displacement moment term: the wide
//     cavity's Centroid misses (TestDisplacedCupCentroidMoment).
//   - stackedPatchArea's per-loop column term: the inward shellCap area
//     misses.
//   - The column's own section displacement in the stacked wall build
//     (cupPlan.columnDelta into buildLoopSidesAs): each offset wall face's
//     area misses, inward and outward.
//   - stackedBoundsContext's outer-run displacement: the outward Bounds
//     miss.
//   - cupView.extentAlong's offset term: the outward extent misses.
//   - The cup gate witness's offset term (verify_gate.go): the outward gate
//     diameter exceeds the smallest denoted cup's.
//   - The interference guard (interference.go): two equal displaced records
//     read as one set.
	//   - radiussurvey.Cup's offset term: the cavity radius misses.
//   - tessellateCup's face, area-slack and occupied-volume terms: the cavity
//     wall's face bound, the mesh area and the mesh volume each miss.
//
// The centroid's envelope widening is not exhibited: the envelope is the
// body's whole coordinate scale and never the smaller of the two proofs on a
// buildable cup, so the formula bound is what publishes.

const cupOffsetDelta = 1.0 / 1024

func displacedCup(t *testing.T, sense ShellSense) cupView {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	cp := cupView{
		outer:  rectangleRecord(-8, -4, 8, 4),
		cavity: rectangleRecord(-6, -2, 6, 2),
		frame:  frame,
		zOpen:  10, zOuter: 0, zCav: 2,
		thickness: 2, offsetDelta: cupOffsetDelta,
		sense: sense,
		xform: r3.Identity(),
	}
	if sense == Outward {
		cp.zOuter, cp.zCav = -2, 0
	}
	return cp
}

// evalDisplacedCup builds a fixture cup in a document of its own.
func evalDisplacedCup(cp cupView) (*Body, error) {
	d := New()
	return evalCup(d, d.nextProducerID(), cp)
}

// cupFamily calls check with every denoted cup the displaced fixture allows:
// the offset region's four walls (u0, u1, v0, v1) each moved by −δ/2, 0 or
// +δ/2.
func cupFamily(base [4]float64, check func(walls [4]*big.Rat)) {
	d := new(big.Rat).SetFloat64(cupOffsetDelta / 2)
	moves := []*big.Rat{new(big.Rat).Neg(d), new(big.Rat), d}
	for _, a := range moves {
		for _, b := range moves {
			for _, c := range moves {
				for _, e := range moves {
					check([4]*big.Rat{
						new(big.Rat).Add(new(big.Rat).SetFloat64(base[0]), a),
						new(big.Rat).Add(new(big.Rat).SetFloat64(base[1]), b),
						new(big.Rat).Add(new(big.Rat).SetFloat64(base[2]), c),
						new(big.Rat).Add(new(big.Rat).SetFloat64(base[3]), e),
					})
				}
			}
		}
	}
}

// rectPrism is the area, ∫u dA, ∫v dA and height-weighted z moment pieces of
// the box walls[0..1] × walls[2..3] × [z0, z1].
type rectPrism struct{ volume, mu, mv, mz *big.Rat }

func rectPrismOf(walls [4]*big.Rat, z0, z1 float64) rectPrism {
	w := new(big.Rat).Sub(walls[1], walls[0])
	d := new(big.Rat).Sub(walls[3], walls[2])
	h := new(big.Rat).SetFloat64(z1 - z0)
	area := new(big.Rat).Mul(w, d)
	volume := new(big.Rat).Mul(area, h)
	mid := func(a, b *big.Rat) *big.Rat { return new(big.Rat).Quo(new(big.Rat).Add(a, b), big.NewRat(2, 1)) }
	zMid := new(big.Rat).SetFloat64((z0 + z1) / 2)
	return rectPrism{
		volume: volume,
		mu:     new(big.Rat).Mul(volume, mid(walls[0], walls[1])),
		mv:     new(big.Rat).Mul(volume, mid(walls[2], walls[3])),
		mz:     new(big.Rat).Mul(volume, zMid),
	}
}

func rectArea(walls [4]*big.Rat) *big.Rat {
	return new(big.Rat).Mul(new(big.Rat).Sub(walls[1], walls[0]), new(big.Rat).Sub(walls[3], walls[2]))
}

func requireVecCovered(t *testing.T, reading VecMeasurement, exact [3]*big.Rat) {
	t.Helper()
	held := [3]float64{reading.Value.X, reading.Value.Y, reading.Value.Z}
	for i := range held {
		requireRatCovered(t, Measurement{Value: units.Millimeters(held[i]), Bound: reading.Bound}, exact[i])
	}
}

func faceWithRole(t *testing.T, b *Body, role string) *Face {
	t.Helper()
	for _, f := range b.Faces() {
		for _, o := range f.Origins() {
			if o.Role == role {
				return f
			}
		}
	}
	t.Fatalf("no face carries role %q", role)
	return nil
}

func TestEvalCupChargesOffsetDisplacement(t *testing.T) {
	for _, sense := range []ShellSense{Inward, Outward} {
		t.Run(sense.String(), func(t *testing.T) {
			cp := displacedCup(t, sense)
			body, err := evalDisplacedCup(cp)
			require.NoError(t, err)
			volume, err := body.Volume()
			require.NoError(t, err)
			centroid, err := body.Centroid()
			require.NoError(t, err)
			area, err := body.Area()
			require.NoError(t, err)
			rim, err := faceWithRole(t, body, "rim(0)").Area()
			require.NoError(t, err)
			offsetCap := "shellCap"
			base := [4]float64{-6, 6, -2, 2}
			if sense == Outward {
				offsetCap, base = roleCapStart, [4]float64{-8, 8, -4, 4}
			}
			capArea, err := faceWithRole(t, body, offsetCap).Area()
			require.NoError(t, err)
			bounds, err := body.Bounds()
			require.NoError(t, err)
			lo, hi, extentDelta, err := cp.extentAlong(r3.NewVec(1, 0, 0))
			require.NoError(t, err)
			// The offset region's own wall faces: cavity walls inward, outer walls
			// outward, each over that region's height.
			wallPrefix, wallHeight := "shellSide(", cp.zOpen-cp.zCav
			if sense == Outward {
				wallPrefix, wallHeight = "side(", cp.zOpen-cp.zOuter
			}
			type offsetWall struct {
				area   Measurement
				alongU bool
			}
			var offsetWalls []offsetWall
			for _, f := range body.Faces() {
				if !strings.HasPrefix(f.origins[0].Role, wallPrefix) {
					continue
				}
				a, err := f.Area()
				require.NoError(t, err)
				n := f.surface.(Plane).Frame.N()
				offsetWalls = append(offsetWalls, offsetWall{area: a, alongU: math.Abs(n.Y) > 0.5})
			}
			require.Len(t, offsetWalls, 4)

			cupFamily(base, func(walls [4]*big.Rat) {
				outer := [4]*big.Rat{big.NewRat(-8, 1), big.NewRat(8, 1), big.NewRat(-4, 1), big.NewRat(4, 1)}
				cavity := [4]*big.Rat{big.NewRat(-6, 1), big.NewRat(6, 1), big.NewRat(-2, 1), big.NewRat(2, 1)}
				if sense == Outward {
					outer = walls
				} else {
					cavity = walls
				}
				o := rectPrismOf(outer, cp.zOuter, cp.zOpen)
				c := rectPrismOf(cavity, cp.zCav, cp.zOpen)
				v := new(big.Rat).Sub(o.volume, c.volume)
				requireRatCovered(t, volume, v)
				requireVecCovered(t, centroid, [3]*big.Rat{
					new(big.Rat).Quo(new(big.Rat).Sub(o.mu, c.mu), v),
					new(big.Rat).Quo(new(big.Rat).Sub(o.mv, c.mv), v),
					new(big.Rat).Quo(new(big.Rat).Sub(o.mz, c.mz), v),
				})
				requireRatCovered(t, rim, new(big.Rat).Sub(rectArea(outer), rectArea(cavity)))
				requireRatCovered(t, capArea, rectArea(walls))
				// A wall normal to v runs along u, so its length is u1 − u0.
				for _, w := range offsetWalls {
					length := new(big.Rat).Sub(walls[3], walls[2])
					if w.alongU {
						length = new(big.Rat).Sub(walls[1], walls[0])
					}
					requireRatCovered(t, w.area, new(big.Rat).Mul(length, new(big.Rat).SetFloat64(wallHeight)))
				}
				// Two caps' worth of the outer area (kept cap, plus rims and
				// pocket floor together), then each region's walls over its height.
				perimeter := func(w [4]*big.Rat) *big.Rat {
					return new(big.Rat).Mul(big.NewRat(2, 1), new(big.Rat).Add(
						new(big.Rat).Sub(w[1], w[0]), new(big.Rat).Sub(w[3], w[2])))
				}
				total := new(big.Rat).Mul(big.NewRat(2, 1), rectArea(outer))
				total.Add(total, new(big.Rat).Mul(perimeter(outer), new(big.Rat).SetFloat64(cp.zOpen-cp.zOuter)))
				total.Add(total, new(big.Rat).Mul(perimeter(cavity), new(big.Rat).SetFloat64(cp.zOpen-cp.zCav)))
				requireRatCovered(t, area, total)
				// The box and the extent read the outer region.
				requireRatCovered(t, Measurement{Value: units.Millimeters(bounds.Max.X), Bound: bounds.Bound}, outer[1])
				requireRatCovered(t, Measurement{Value: units.Millimeters(bounds.Min.X), Bound: bounds.Bound}, outer[0])
				requireRatCovered(t, Measurement{Value: units.Millimeters(hi), Bound: units.Millimeters(extentDelta)}, outer[1])
				requireRatCovered(t, Measurement{Value: units.Millimeters(lo), Bound: units.Millimeters(extentDelta)}, outer[0])
			})
		})
	}
}

// TestDisplacedCupGateAndIdentity covers the two readers that consume an
// outward cup's displaced outer region without composing a measurement: the
// verification gate's witness diameter, a lower bound that must not exceed the
// smallest denoted cup's own diameter, and the coincidence certificate, which
// must not read two equal displaced records as one set.
func TestDisplacedCupGateAndIdentity(t *testing.T) {
	cp := displacedCup(t, Outward)
	body, err := evalDisplacedCup(cp)
	require.NoError(t, err)
	gate, ok, err := fallbackGateDiameter(proofbound.NewWorkBudget(t.Context()), body)
	require.NoError(t, err)
	require.True(t, ok)
	// The smallest denoted cup's farthest pair is its outer box's diagonal,
	// each outer wall pulled in by δ/2 so its corners move by less than δ.
	d := new(big.Rat).SetFloat64(cupOffsetDelta / 2)
	two := big.NewRat(2, 1)
	du := new(big.Rat).Sub(big.NewRat(16, 1), new(big.Rat).Mul(two, d))
	dv := new(big.Rat).Sub(big.NewRat(8, 1), new(big.Rat).Mul(two, d))
	dz := big.NewRat(12, 1)
	smallest := new(big.Rat).Add(new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv)), new(big.Rat).Mul(dz, dz))
	g := new(big.Rat).SetFloat64(gate)
	require.LessOrEqual(t, new(big.Rat).Mul(g, g).Cmp(smallest), 0, "gate diameter %v exceeds the smallest denoted cup's", gate)

	twin, err := evalDisplacedCup(cp)
	require.NoError(t, err)
	same, err := analyticBodiesEqual(proofbound.NewWorkBudget(t.Context()), body, twin)
	require.NoError(t, err)
	require.False(t, same)
}

// TestDisplacedCupMinRadius reads the concave radius of an inward cup whose
// cavity corners are arcs of radius 1: every denoted cavity arc's radius is
// within the offset displacement of that, so the reading must cover 1 − δ and
// 1 + δ.
func TestDisplacedCupMinRadius(t *testing.T) {
	budget := proofbound.NewWorkBudget(t.Context())
	outer, err := offsetProfile(budget, rectangleRecord(-6, -2, 6, 2), -1, 2)
	require.NoError(t, err)
	cavity, err := offsetProfile(budget, rectangleRecord(-6, -2, 6, 2), -1, 1)
	require.NoError(t, err)
	cp := displacedCup(t, Inward)
	cp.outer, cp.cavity, cp.thickness = outer, cavity, 1
	out := radiussurvey.Cup(cp.outer, cp.cavity, cp.offsetDelta)
	require.True(t, out.OK)
	require.NotNil(t, out.Reading)
	reading := Measurement{Value: units.Millimeters(*out.Reading), Bound: units.Millimeters(out.Bound)}
	d := new(big.Rat).SetFloat64(cupOffsetDelta)
	requireRatCovered(t, reading, new(big.Rat).Sub(big.NewRat(1, 1), d))
	requireRatCovered(t, reading, new(big.Rat).Add(big.NewRat(1, 1), d))
}

// TestDisplacedCupMesh tessellates the inward displaced cup and checks the
// mesh's published proofs against every denoted cup: the occupied-volume
// bound against the exact volume difference, the area slack against the
// exact area difference, and the cavity wall's face bound against how far a
// denoted cavity wall sits from the mesh vertices on the recorded one.
func TestDisplacedCupMesh(t *testing.T) {
	cp := displacedCup(t, Inward)
	body, err := evalDisplacedCup(cp)
	require.NoError(t, err)
	mesh, err := tessellateCup(t.Context(), body, cp, 0.1, VerifyAll)
	require.NoError(t, err)
	require.True(t, mesh.symDiffOK)

	held := new(big.Rat)
	heldArea := 0.0
	for _, tri := range mesh.triangles {
		a, b, c := mesh.vertices[tri[0]], mesh.vertices[tri[1]], mesh.vertices[tri[2]]
		r := func(v r3.Vec) [3]*big.Rat {
			return [3]*big.Rat{new(big.Rat).SetFloat64(v.X), new(big.Rat).SetFloat64(v.Y), new(big.Rat).SetFloat64(v.Z)}
		}
		ra, rb, rc := r(a), r(b), r(c)
		det := new(big.Rat)
		for i := range 3 {
			j, k := (i+1)%3, (i+2)%3
			term := new(big.Rat).Sub(new(big.Rat).Mul(rb[j], rc[k]), new(big.Rat).Mul(rb[k], rc[j]))
			det.Add(det, term.Mul(term, ra[i]))
		}
		held.Add(held, det.Quo(det, big.NewRat(6, 1)))
		heldArea += b.Sub(a).Cross(c.Sub(a)).Len() / 2
	}
	wall := faceWithRole(t, body, "shellSide(0,0)")
	wallBound, ok := mesh.sourceBound(wall)
	require.True(t, ok)
	d := new(big.Rat).SetFloat64(cupOffsetDelta)

	cupFamily([4]float64{-6, 6, -2, 2}, func(walls [4]*big.Rat) {
		outerVolume := big.NewRat(16*8*10, 1)
		cavity := new(big.Rat).Mul(rectArea(walls), big.NewRat(8, 1))
		v := new(big.Rat).Sub(outerVolume, cavity)
		requireRatCovered(t, Measurement{Value: units.CubicMillimeters(0), Bound: units.CubicMillimeters(mesh.volSymDiff)},
			new(big.Rat).Sub(v, held))

		perimeter := new(big.Rat).Mul(big.NewRat(2, 1), new(big.Rat).Add(
			new(big.Rat).Sub(walls[1], walls[0]), new(big.Rat).Sub(walls[3], walls[2])))
		area := new(big.Rat).Add(big.NewRat(2*128+48*10, 1), new(big.Rat).Mul(perimeter, big.NewRat(8, 1)))
		denoted, _ := area.Float64()
		// 1e-9 covers this test's own float sum of facet areas, far below
		// the displacement's square-millimetre scale.
		require.InDelta(t, denoted, heldArea, mesh.areaSlack+1e-9)
	})
	// The recorded wall shellSide(0,0) lies on one cavity side; a denoted
	// wall may sit δ from a mesh vertex on it.
	require.GreaterOrEqual(t, new(big.Rat).SetFloat64(wallBound).Cmp(d), 0)
}

// TestDisplacedCupCentroidMoment isolates the offset region's moment term. In
// the small fixtures the weight terms of the centroid's composition already
// span a shifted cavity, because the cavity's area moves a large fraction of
// the cup's volume. Here the cavity is wide against its wall, its mid level
// sits on the frame origin, and the outer prism's mid level sits one
// millimetre below it, so the weight terms stay far below the shift a
// displaced cavity gives the centroid and only the moment term covers it.
func TestDisplacedCupCentroidMoment(t *testing.T) {
	cp := displacedCup(t, Inward)
	cp.outer = rectangleRecord(-500, -500, 500, 500)
	cp.cavity = rectangleRecord(-300, -300, 300, 300)
	cp.zOuter, cp.zCav, cp.zOpen = -6, -4, 4
	body, err := evalDisplacedCup(cp)
	require.NoError(t, err)
	centroid, err := body.Centroid()
	require.NoError(t, err)
	outer := [4]*big.Rat{big.NewRat(-500, 1), big.NewRat(500, 1), big.NewRat(-500, 1), big.NewRat(500, 1)}
	o := rectPrismOf(outer, cp.zOuter, cp.zOpen)
	cupFamily([4]float64{-300, 300, -300, 300}, func(walls [4]*big.Rat) {
		c := rectPrismOf(walls, cp.zCav, cp.zOpen)
		v := new(big.Rat).Sub(o.volume, c.volume)
		requireVecCovered(t, centroid, [3]*big.Rat{
			new(big.Rat).Quo(new(big.Rat).Sub(o.mu, c.mu), v),
			new(big.Rat).Quo(new(big.Rat).Sub(o.mv, c.mv), v),
			new(big.Rat).Quo(new(big.Rat).Sub(o.mz, c.mz), v),
		})
	})
}
