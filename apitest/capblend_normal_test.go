package apitest_test

import (
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

// TestCapBlendPlanePatchNormalOutwardBothCaps checks the sign of the Z
// component of every straight-wall chamfer patch's outward normal on the
// rectangular box, for BOTH the start and the end cap: since a start-cap
// band and an end-cap band tilt toward OPPOSITE caps (the exterior end of
// each band), the two cases must read opposite Z signs — the asymmetry a
// naive "the Plane's own u x v is always outward" assumption misses.
func TestCapBlendPlanePatchNormalOutwardBothCaps(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name    string
		end     bool
		wantPos bool // true: normal Z component must be positive
	}{
		{"end cap tilts toward +Z (the chamfered end)", true, true},
		{"start cap tilts toward -Z (the chamfered end)", false, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			_, box := capBlendBox(t)
			chamfered, err := box.Chamfer(t.Context(), capLoopEdgesOn(box, tc.end), units.Millimeters(5))
			require.NoError(t, err)
			prefix := "chamferCap(end,"
			if !tc.end {
				prefix = "chamferCap(start,"
			}
			normals := patchNormalsByRolePrefix(t, chamfered, prefix)
			require.Len(t, normals, 4, "one Plane patch per rectangle wall")
			for role, n := range normals {
				if tc.wantPos {
					require.Greater(t, n.Z, 0.0, "role %s", role)
				} else {
					require.Less(t, n.Z, 0.0, "role %s", role)
				}
				require.InDelta(t, 1.0, n.Len(), 1e-9, "role %s: NormalAt must be unit", role)
			}
		})
	}
}

// TestCapBlendConePatchNormalOutwardBothCaps is the circular-rim analog of
// the Plane check: the cone chamfer patch's normal must tilt toward whichever
// cap is chamfered, for both caps.
func TestCapBlendConePatchNormalOutwardBothCaps(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name    string
		end     bool
		wantPos bool
	}{
		{"end cap", true, true},
		{"start cap", false, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			const R, H = 30.0, 20.0
			disk := circleProfile(t, R, H)
			chamfered, err := disk.Chamfer(t.Context(), capLoopEdgesOn(disk, tc.end), units.Millimeters(5))
			require.NoError(t, err)
			prefix := "chamferCap(end,"
			if !tc.end {
				prefix = "chamferCap(start,"
			}
			normals := patchNormalsByRolePrefix(t, chamfered, prefix)
			require.Len(t, normals, 1, "one whole-turn Cone patch")
			for role, n := range normals {
				if tc.wantPos {
					require.Greater(t, n.Z, 0.0, "role %s", role)
				} else {
					require.Less(t, n.Z, 0.0, "role %s", role)
				}
				require.InDelta(t, 1.0, n.Len(), 1e-9, "role %s: NormalAt must be unit", role)
			}
		})
	}
}

// TestCapBlendReflexApexNormalOutward checks the reflex corner's Cone-apex
// patch against a reference derived from the topology alone, never from the
// builder's own formula.
//
// The offset is an EROSION, and at a reflex corner the eroded boundary is the
// arc of radius d about the corner with the surviving material radially
// OUTSIDE it — the sector the arc cuts off is exactly what the chamfer
// removes. So this patch's cone has the VOID inside and the solid outside,
// and its outward normal points radially INWARD, at the corner's own axis,
// while tilting toward the chamfered cap like every other patch in the band.
//
// The reference is built from the apex vertex, the arc foot and the cap face's
// own outward normal: with dc = ds = d the cone stands at 45 degrees, so the
// outward normal is (capNormal - radialUnit)/sqrt(2) exactly. The check is a
// vector identity rather than a pair of sign predicates, so an inverted normal
// fails it — and so does the radially-outward reading, which is EXACTLY
// perpendicular to the truth here and passes every sign test written against a
// cone ruling.
func TestCapBlendReflexApexNormalOutward(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name string
		end  bool
	}{
		{"end cap", true},
		{"start cap", false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			const d = 3.0
			body := reflexLBody(t)
			chamfered, err := body.Chamfer(t.Context(), capLoopEdgesOn(body, tc.end), units.Millimeters(d))
			require.NoError(t, err)

			prefix := "chamferCap(end,"
			capRole := roleCapEnd
			if !tc.end {
				prefix = "chamferCap(start,"
				capRole = roleCapStart
			}
			apex := apexPatchOf(t, chamfered, prefix)

			// The cap face's own outward normal is the band's axial reference;
			// it is independently pinned by the plane- and cone-patch tests.
			capFace := faceWithRole(t, chamfered, capRole)
			capN, err := capFace.NormalAt(capFace.Loops()[0].CoEdges()[0].Start().Position().Value)
			require.NoError(t, err)
			axis := capN.Value

			// The apex vertex is the ORIGINAL corner, held where the receiver
			// had it; the arc's own start is one foot of the offset connector.
			coedges := apex.Loops()[0].CoEdges()
			corner := coedges[1].End().Position().Value
			p := coedges[0].Start().Position().Value
			v := p.Sub(corner)
			require.InDelta(t, d, v.Dot(axis), 1e-9, "one setback along the cap's own outward sense")
			radial, ok := v.Sub(axis.Scale(v.Dot(axis))).Normalize()
			require.True(t, ok)

			want, ok := axis.Sub(radial).Normalize()
			require.True(t, ok)
			n, err := apex.NormalAt(p)
			require.NoError(t, err)
			require.InDelta(t, 1.0, n.Value.Dot(want), 1e-9, "the apex patch's outward normal")
			// Restated as the two facts the identity carries, so a failure
			// names which half moved.
			require.Less(t, n.Value.Dot(radial), 0.0, "toward the corner's own axis")
			require.Greater(t, n.Value.Dot(axis), 0.0, "toward the chamfered end")
		})
	}
}

// denotedNormalDistance is the distance from got to the unit vector
// (radial + axial)/√2, the outward normal of a 45° chamfer band whose wall
// leaves along the plane-local radial direction (ru, rv), read in 400-bit
// arithmetic. The radial direction is normalized there too, so a circular
// band's azimuth costs the reference nothing.
func denotedNormalDistance(got r3.Vec, ru, rv float64) float64 {
	const prec = 400
	bf := func(x float64) *big.Float { return new(big.Float).SetPrec(prec).SetFloat64(x) }
	u, v := bf(ru), bf(rv)
	rlen := new(big.Float).SetPrec(prec).Add(new(big.Float).SetPrec(prec).Mul(u, u), new(big.Float).SetPrec(prec).Mul(v, v))
	rlen.Sqrt(rlen)
	half := new(big.Float).SetPrec(prec).Sqrt(bf(0.5))
	scale := new(big.Float).SetPrec(prec).Quo(half, rlen)
	want := []*big.Float{
		new(big.Float).SetPrec(prec).Mul(u, scale),
		new(big.Float).SetPrec(prec).Mul(v, scale),
		half,
	}
	sum := new(big.Float).SetPrec(prec)
	for i, c := range []float64{got.X, got.Y, got.Z} {
		diff := new(big.Float).SetPrec(prec).Sub(bf(c), want[i])
		sum.Add(sum, new(big.Float).SetPrec(prec).Mul(diff, diff))
	}
	dist, _ := sum.Sqrt(sum).Float64()
	return dist
}

// TestCapBlendPatchNormalBoundCoversDenotedWall checks that a band patch's
// published Face.NormalAt bound covers its distance from the normal of the
// wall its records denote, not only from the ruled patch the build assembled.
// The cap contour of a 0.1 mm chamfer drawn 10⁶ mm up the v axis is an offset
// the build solved, held about an ulp of 10⁶ off the denoted one, so every
// patch it bounds stands tilted from the exact 45° wall.
//
// Each section sits at v = 10⁶ on the XY sketch, and the end cap is chamfered
// by 0.1 mm. The square's two walls along u tilt by about 1.2e-10. The disk is
// read at eight azimuths, each against the exact cone normal at that azimuth.
//
// Shown to fail: with capband.DenotedNormalAllow left out of setPatchReadings
// (capblend_geom.go), the square's walls along u publish a 1.7e-16 bound
// against a 1.2e-10 distance. The disk leg passes without the term, because
// its whole-turn departure already exceeds that distance; it guards the
// circular arm of the term.
func TestCapBlendPatchNormalBoundCoversDenotedWall(t *testing.T) {
	t.Parallel()
	const cv, half, h, d = 1e6, 10.0, 10.0, 0.1
	for _, tc := range []struct {
		name string
		draw func(s *sketch.Sketch)
	}{
		{`a square`, func(s *sketch.Sketch) {
			rect := s.CreateRectangle(-half, cv-half, half, cv+half)
			s.Fix(rect.A)
			s.Fix(rect.C)
		}},
		{`a disk`, func(s *sketch.Sketch) {
			c := s.CreatePoint(0, cv)
			s.CreateCircle(c, half)
			s.Fix(c)
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			w := sketch.NewWorld()
			s, err := w.CreateSketch(w.XY())
			require.NoError(t, err)
			tc.draw(s)
			_, err = s.Solve(t.Context())
			require.NoError(t, err)
			require.Len(t, s.Profiles(), 1)
			body, err := decad.New().Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
			require.NoError(t, err)
			chamfered, err := body.Chamfer(t.Context(), capLoopEdges(body), units.Millimeters(d))
			require.NoError(t, err)

			var patches []*decad.Face
			for _, f := range chamfered.Faces() {
				for _, o := range f.Origins() {
					if strings.HasPrefix(o.Role, "chamferCap(end,") {
						patches = append(patches, f)
					}
				}
			}
			require.NotEmpty(t, patches)
			worst := 0.0
			for _, f := range patches {
				var points []r3.Vec
				switch f.Surface().(type) {
				case decad.Plane:
					// The patch's own vertex centroid, which lies on it.
					coedges := f.Loops()[0].CoEdges()
					var sum r3.Vec
					for _, ce := range coedges {
						sum = sum.Add(ce.Start().Position().Value)
					}
					points = append(points, sum.Scale(1/float64(len(coedges))))
				default:
					for k := range 8 {
						sin, cos := math.Sincos(float64(k) * math.Pi / 4)
						points = append(points, r3.NewVec((half-d/2)*cos, cv+(half-d/2)*sin, h-d/2))
					}
				}
				for _, p := range points {
					n, err := f.NormalAt(p)
					require.NoError(t, err)
					// The exact wall leaves along the dominant plane-local
					// axis on the square, and along p's own azimuth on the
					// disk.
					ru, rv := p.X, p.Y-cv
					if _, flat := f.Surface().(decad.Plane); flat {
						ru, rv = 0, 0
						if math.Abs(n.Value.X) > math.Abs(n.Value.Y) {
							ru = math.Copysign(1, n.Value.X)
						} else {
							rv = math.Copysign(1, n.Value.Y)
						}
					}
					dist := denotedNormalDistance(n.Value, ru, rv)
					worst = math.Max(worst, dist)
					require.LessOrEqual(t, dist, n.Bound.Mag(), `the published bound covers the denoted normal at %v`, p)
				}
			}
			require.Positive(t, worst, `the premise: some patch's held normal is off the denoted one`)
		})
	}
}
