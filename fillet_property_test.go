package decad_test

import (
	"encoding/json"
	"errors"
	"math"
	"math/rand"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is randomized property testing for Body.Fillet (fillet.go,
// fillet_audit.go), the same treatment revolve and the boolean got. The rest of
// the fillet suite is fixed hand-chosen cases; the two PR #32 review passes both
// named the §5 audit's geometric space — the accept-then-un-tessellatable class
// (S7 boundary contact) and the shrunk-outer-loop-past-a-hole class (S9 nesting)
// — as the part hard to exhaust by inspection. Here we sample that space at
// scale and assert the invariants that must hold whichever corner, section or
// radius is drawn:
//
//  1. BUILD case: when Fillet returns nil the body is watertight, IsSolid, Verify
//     reads Sound at a sufficient tolerance, Tessellate succeeds, mass properties
//     carry bounds, and the analytic Volume matches an INDEPENDENT closed form — a right-angle
//     corner's radius-r fillet moves exactly (1 − π/4)r² of area per unit height,
//     removed at a convex corner and added back at a concave one. Every blend
//     wall is a Cylinder of the fillet radius.
//  2. The cardinal cross-space implication: for EVERY generated case, a nil
//     return MUST tessellate. An accept-then-un-tessellatable body is the exact
//     bug the audit exists to prevent.
//  3. REFUSE case: a refusal is a known sentinel (ErrDegenerate / ErrUnsupported)
//     and leaves the document untouched — never a silently-wrong body.
//  4. Scale invariance: the same relative geometry decides the same way at 1 mm
//     and 1e6 mm, because the contact tolerance is scale-anchored (δ = ε·D).
//  5. Recipe round-trip: a built body's Recipe marshals and unmarshals to itself.
//
// All randomness is drawn from a fixed, logged seed, so any failure is
// replayable. Every generated section is axis-aligned (right-angle corners only),
// which is exactly what makes the (1 − π/4)r² area oracle exact.

const filletPropertySeed int64 = 0xF111E75EED

// cornerBite is the area a right-angle corner's radius-r fillet moves:
// r² − (π/4)r² = (1 − π/4)r². A convex corner loses it, a concave one gains it.
func cornerBite(r float64) float64 { return (1 - math.Pi/4) * r * r }

// filletBasePrismHeight is the sweep height of every generated prism at k = 1.
const filletBasePrismHeight = 20.0

func filletCaseHeight(k float64) float64 { return filletBasePrismHeight * k }

// convexLateral / concaveLateral select a prism's convex / concave lateral
// edges — the section corners a fillet rounds.
func convexLateral() *decad.EdgeQuery {
	return decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.Convex())
}

func concaveLateral() *decad.EdgeQuery {
	return decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.Concave())
}

// filletCase is one randomly generated straight prism whose section corners are
// all right-angled. build(t, k) constructs it at absolute scale k (coordinates,
// hole and height all ×k); area is the section area at k = 1; convexMaxR /
// concaveMaxR are the self-consistency upper bounds on a valid radius for a
// convex / concave group fillet at k = 1 (0 means that polarity has no matching
// corner).
type filletCase struct {
	name        string
	build       func(t *testing.T, k float64) (*decad.Document, *decad.Body)
	area        float64
	convexMaxR  float64
	concaveMaxR float64
}

func (fc filletCase) sectionArea(k float64) float64 { return fc.area * k * k }

// pickProfile returns the solved profile with the given outer/hole segment
// counts.
func pickProfile(t *testing.T, s *sketch.Sketch, nOuter, nHoles int) *sketch.Profile {
	t.Helper()
	for _, p := range s.Profiles() {
		if len(p.Outer) == nOuter && len(p.Holes) == nHoles {
			return p
		}
	}
	require.Failf(t, "no matching profile", "want outer=%d holes=%d, got %d profiles", nOuter, nHoles, len(s.Profiles()))
	return nil
}

func filletExtrude(t *testing.T, s *sketch.Sketch, p *sketch.Profile, k float64) (*decad.Document, *decad.Body) {
	t.Helper()
	doc := decad.New()
	body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(filletCaseHeight(k)), Dir: decad.Along})
	require.NoError(t, err)
	return doc, body
}

// genRect is a plain rectangle: four convex corners, no concave ones.
func genRect(rng *rand.Rand) filletCase {
	w := uniform(rng, 40, 120)
	h := uniform(rng, 40, 120)
	return filletCase{
		name:       "rect",
		area:       w * h,
		convexMaxR: math.Min(w, h) / 2,
		build: func(t *testing.T, k float64) (*decad.Document, *decad.Body) {
			world := sketch.NewWorld()
			s, err := world.CreateSketch(world.XY())
			require.NoError(t, err)
			rect := s.CreateRectangle(0, 0, w*k, h*k)
			s.Fix(rect.A)
			_, err = s.Solve(t.Context())
			require.NoError(t, err)
			return filletExtrude(t, s, pickProfile(t, s, 4, 0), k)
		},
	}
}

// genL is an L-shaped section: five convex corners and one reflex (concave) one.
func genL(rng *rand.Rand) filletCase {
	w := uniform(rng, 70, 120)
	h := uniform(rng, 70, 120)
	w2 := uniform(rng, 25, w-25)
	h2 := uniform(rng, 25, h-25)
	// Both-convex edges need 2r < len; the two edges meeting the reflex corner
	// need r < len.
	bothConvex := math.Min(math.Min(w, h2), math.Min(w2, h)) / 2
	reflexEdges := math.Min(w-w2, h-h2)
	return filletCase{
		name:        "L",
		area:        w*h - (w-w2)*(h-h2),
		convexMaxR:  math.Min(bothConvex, reflexEdges),
		concaveMaxR: reflexEdges,
		build: func(t *testing.T, k float64) (*decad.Document, *decad.Body) {
			world := sketch.NewWorld()
			s, err := world.CreateSketch(world.XY())
			require.NoError(t, err)
			corners := [][2]float64{{0, 0}, {w, 0}, {w, h2}, {w2, h2}, {w2, h}, {0, h}}
			pts := make([]*sketch.Point, len(corners))
			for i, c := range corners {
				pts[i] = s.CreatePoint(c[0]*k, c[1]*k)
				s.Fix(pts[i])
			}
			for i := range pts {
				s.CreateLine(pts[i], pts[(i+1)%len(pts)])
			}
			_, err = s.Solve(t.Context())
			require.NoError(t, err)
			return filletExtrude(t, s, pickProfile(t, s, 6, 0), k)
		},
	}
}

// genRectHole is a rectangle with a rectangular hole placed near a corner: four
// convex outer corners plus four concave hole corners. The near-corner hole lets
// a large outer fillet shrink the loop past it (the S9 nesting refusal) or pinch
// it (the S7 contact refusal).
func genRectHole(rng *rand.Rand) filletCase {
	w := uniform(rng, 80, 130)
	h := uniform(rng, 80, 130)
	hw := uniform(rng, 8, 16)
	hh := uniform(rng, 8, 16)
	off := uniform(rng, 6, 18)
	return filletCase{
		name:        "rectHole",
		area:        w*h - hw*hh,
		convexMaxR:  math.Min(w, h) / 2,
		concaveMaxR: math.Min(hw, hh) / 2,
		build: func(t *testing.T, k float64) (*decad.Document, *decad.Body) {
			world := sketch.NewWorld()
			s, err := world.CreateSketch(world.XY())
			require.NoError(t, err)
			outer := s.CreateRectangle(0, 0, w*k, h*k)
			s.Fix(outer.A)
			s.CreateRectangle(off*k, off*k, (off+hw)*k, (off+hh)*k)
			_, err = s.Solve(t.Context())
			require.NoError(t, err)
			return filletExtrude(t, s, pickProfile(t, s, 4, 1), k)
		},
	}
}

// genRectDiskHole is a rectangle with a circular hole: four convex outer corners,
// no straight concave corner (the disk rim is one smooth circle). The section
// already carries one cylinder wall, so the blend-count check keys on the fillet
// role, not the surface tag.
func genRectDiskHole(rng *rand.Rand) filletCase {
	w := uniform(rng, 80, 130)
	h := uniform(rng, 80, 130)
	rho := uniform(rng, 6, 14)
	cx := uniform(rng, rho+10, w-rho-10)
	cy := uniform(rng, rho+10, h-rho-10)
	return filletCase{
		name:       "rectDiskHole",
		area:       w*h - math.Pi*rho*rho,
		convexMaxR: math.Min(w, h) / 2,
		build: func(t *testing.T, k float64) (*decad.Document, *decad.Body) {
			world := sketch.NewWorld()
			s, err := world.CreateSketch(world.XY())
			require.NoError(t, err)
			outer := s.CreateRectangle(0, 0, w*k, h*k)
			s.Fix(outer.A)
			s.CreateCircle(s.CreatePoint(cx*k, cy*k), rho*k)
			_, err = s.Solve(t.Context())
			require.NoError(t, err)
			return filletExtrude(t, s, pickProfile(t, s, 4, 1), k)
		},
	}
}

// genCornerDiskHole is a rectangle with a SMALL circular hole placed deep in a
// corner region: four convex outer corners and a disk that a large corner fillet
// shrinks the outer loop PAST — the hole ends up outside the rounded material yet
// disjoint from every outer segment. That is the S9 nesting class: S8/S6/S7 all
// pass, only the containment audit catches it (ErrDegenerate). A small fillet
// leaves the hole comfortably inside and builds. This template is what gives the
// suite teeth against the nesting audit — with S9 disabled its large-radius cases
// build a body no tolerance can tessellate.
func genCornerDiskHole(rng *rand.Rand) filletCase {
	w := uniform(rng, 90, 110)
	h := uniform(rng, 90, 110)
	rho := uniform(rng, 1, 2)
	a := uniform(rng, 5, 8) // hole centre on the (0,0) corner diagonal
	return filletCase{
		name:       "cornerDiskHole",
		area:       w*h - math.Pi*rho*rho,
		convexMaxR: math.Min(w, h) / 2,
		build: func(t *testing.T, k float64) (*decad.Document, *decad.Body) {
			world := sketch.NewWorld()
			s, err := world.CreateSketch(world.XY())
			require.NoError(t, err)
			outer := s.CreateRectangle(0, 0, w*k, h*k)
			s.Fix(outer.A)
			s.CreateCircle(s.CreatePoint(a*k, a*k), rho*k)
			_, err = s.Solve(t.Context())
			require.NoError(t, err)
			return filletExtrude(t, s, pickProfile(t, s, 4, 1), k)
		},
	}
}

// filletPlan is one polarity of corner to round: the selector, its sign in the
// area oracle (−1 removes at a convex corner, +1 adds at a concave one), the
// upper bound on a valid radius, and whether these corners are rounded (concave)
// features the min-radius survey must then read.
type filletPlan struct {
	name   string
	sel    func() *decad.EdgeQuery
	sign   float64
	maxR   float64
	survey bool // a concave rectilinear fillet is a concave feature MinRadius reads
}

func filletPlans(fc filletCase) []filletPlan {
	plans := []filletPlan{{name: "convex", sel: convexLateral, sign: -1, maxR: fc.convexMaxR}}
	if fc.concaveMaxR > 0 {
		// rectHole / L have no other concave curved feature, so the survey reads
		// exactly the fillet radius; a disk-hole section would tie its rim in.
		plans = append(plans, filletPlan{name: "concave", sel: concaveLateral, sign: +1, maxR: fc.concaveMaxR, survey: true})
	}
	return plans
}

func TestFilletPropertyInvariants(t *testing.T) {
	rng := rand.New(rand.NewSource(filletPropertySeed))
	t.Logf("fillet property seed = %#x", filletPropertySeed)

	factories := []func(*rand.Rand) filletCase{genRect, genL, genRectHole, genRectDiskHole, genCornerDiskHole}
	fracs := []float64{0.06, 0.22, 0.5, 0.8, 1.05, 1.4}
	const perTemplate = 6

	builds, refuses := 0, 0
	for _, factory := range factories {
		for range perTemplate {
			fc := factory(rng)
			for _, plan := range filletPlans(fc) {
				for _, frac := range fracs {
					r := frac * plan.maxR
					if r <= 0 {
						continue
					}
					if runFilletCase(t, fc, plan, r) {
						builds++
					} else {
						refuses++
					}
				}
			}
		}
	}
	t.Logf("fillet property: %d built, %d refused", builds, refuses)
	// Teeth: both branches must actually fire, or the invariants on one of them
	// never ran.
	require.Positive(t, builds, "no case built — the build-branch invariants never ran")
	require.Positive(t, refuses, "no case refused — the refuse-branch invariants never ran")
}

// runFilletCase runs one (section, polarity, radius) trial and asserts the
// build-or-refuse invariants. It reports whether the fillet built.
func runFilletCase(t *testing.T, fc filletCase, plan filletPlan, r float64) bool {
	t.Helper()
	doc, body := fc.build(t, 1.0)

	// Count the corners this selector will round BEFORE filleting — the oracle
	// needs the count, and every one of them is a right angle.
	edges, selErr := plan.sel().SelectEdges(body)
	require.NoErrorf(t, selErr, "%s/%s selector must match", fc.name, plan.name)
	count := len(edges)
	require.Positivef(t, count, "%s/%s selected no corner", fc.name, plan.name)

	filleted, err := body.Fillet(plan.sel(), units.Millimeters(r))
	if err != nil {
		// Refuse: a known sentinel, and the document is untouched.
		require.Truef(t, errors.Is(err, decad.ErrDegenerate) || errors.Is(err, decad.ErrUnsupported),
			"%s/%s r=%g refused with an unexpected error: %v", fc.name, plan.name, r, err)
		require.Equalf(t, []*decad.Body{body}, doc.Bodies(), "%s/%s a refused fillet retires nothing", fc.name, plan.name)
		return false
	}

	// --- BUILD branch --------------------------------------------------------
	require.Truef(t, filleted.IsSolid(), "%s/%s r=%g built a non-solid", fc.name, plan.name, r)
	requireManifold(t, filleted)
	require.Equalf(t, []*decad.Body{filleted}, doc.Bodies(), "%s/%s the receiver is retired and the fillet registered", fc.name, plan.name)

	// The cardinal implication: a built body MUST tessellate at a sufficiently
	// fine tolerance. An accepted body that cannot — its boundaries actually
	// meet — is the accept-then-un-tessellatable bug the §5 audit exists to
	// prevent.
	requireTessellates(t, filleted, r, fc.name, plan.name)

	// Volume: bounded analytic result matching the independent (1 − π/4)r² oracle.
	vol, err := filleted.Volume()
	require.NoError(t, err)
	require.Equalf(t, decad.Approximate, vol.Exactness, "%s/%s a fillet carries a volume bound", fc.name, plan.name)
	require.Positivef(t, vol.Bound.Base(), "%s/%s a fillet carries a positive bound", fc.name, plan.name)
	gotVol, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	wantVol := (fc.sectionArea(1.0) + plan.sign*float64(count)*cornerBite(r)) * filletCaseHeight(1.0)
	require.InDeltaf(t, wantVol, gotVol, 1e-6*math.Max(1, wantVol),
		"%s/%s r=%g: %d corners × %+.0f bite off the analytic volume", fc.name, plan.name, r, count, plan.sign)

	// Area carries the same circular evaluation bound.
	area, err := filleted.Area()
	require.NoError(t, err)
	require.Equalf(t, decad.Approximate, area.Exactness, "%s/%s a fillet carries an area bound", fc.name, plan.name)

	// Every blend wall is a Cylinder of the fillet radius; there is exactly one
	// per rounded corner.
	requireBlendCylinders(t, filleted, count, r)

	// Zero-tolerance verification surfaces the bounded mass results.
	rep, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equalf(t, decad.Suspect, rep.Status, "%s/%s has bounded mass results", fc.name, plan.name)
	require.False(t, rep.Trustworthy())

	// A concave rectilinear fillet is the section's only concave curved feature,
	// so the min-radius survey reads exactly the fillet radius.
	if plan.survey {
		mr, err := doc.Verify(t.Context(), decad.WithMinRadius())
		require.NoError(t, err)
		require.Len(t, mr.Bodies, 1)
		require.NotNilf(t, mr.Bodies[0].MinRadius, "%s/%s a concave fillet is a concave feature", fc.name, plan.name)
		got, err := mr.Bodies[0].MinRadius.Value.In(units.Millimeter)
		require.NoError(t, err)
		require.InDeltaf(t, r, got, 1e-6*math.Max(1, r), "%s/%s the survey reads the fillet radius", fc.name, plan.name)
	}

	requireRecipeRoundTrip(t, doc)
	return true
}

// requireTessellates asserts a built fillet tessellates at a sufficiently fine
// chord tolerance. Two disjoint-but-close boundaries (a corner fillet near a
// hole) can have OVERLAPPING chord approximations at a coarse tolerance, so
// Tessellate conservatively refuses (reject-never-wrong); halving the tolerance
// clears it. A genuinely pinched or contacting body — the accept-then-
// un-tessellatable bug — has boundaries that actually meet, so no tolerance ever
// clears and the retries exhaust: that is the teeth. Only the chord-proximity
// guard (ErrDegenerate) may refuse; any other error is an unexpected failure.
func requireTessellates(t *testing.T, body *decad.Body, r float64, name, plan string) {
	t.Helper()
	// From 0.1·r, sixteen quarterings reach ~1e-10·r — below the audit's own
	// scale-anchored contact floor (δ = ε·D, ε = 1e-9), so any gap the audit
	// admits clears within them, while a true pinch (gap ≈ 0) never does.
	tol := 0.1 * r
	for range 16 {
		mesh, err := body.Tessellate(units.Millimeters(tol))
		if err == nil {
			require.NotEmptyf(t, mesh.Triangles(), "%s/%s produced an empty mesh", name, plan)
			return
		}
		require.ErrorIsf(t, err, decad.ErrDegenerate, "%s/%s r=%g tessellation refused unexpectedly: %v", name, plan, r, err)
		tol /= 4
	}
	require.Failf(t, "accept-then-un-tessellatable",
		"%s/%s r=%g built but did not tessellate even at a very fine tolerance", name, plan, r)
}

// requireBlendCylinders asserts every fillet-role face is a Cylinder of radius r
// and there is exactly one per rounded corner. Keying on the fillet role (not the
// surface tag) excludes a section's own pre-existing cylinder walls.
func requireBlendCylinders(t *testing.T, body *decad.Body, count int, r float64) {
	t.Helper()
	blends := 0
	for _, f := range body.Faces() {
		cyl, ok := f.Surface().(decad.Cylinder)
		if !ok {
			continue
		}
		fillet := false
		for _, o := range f.Origins() {
			if strings.HasPrefix(o.Role, "fillet(") {
				fillet = true
				break
			}
		}
		if !fillet {
			continue
		}
		blends++
		require.Truef(t, cyl.Radius.Equal(units.Millimeters(r), 1e-9*math.Max(1, r)),
			"a blend wall is a cylinder of the fillet radius, got %s", cyl.Radius)
	}
	require.Equalf(t, count, blends, "one blend cylinder per rounded corner")
}

// requireRecipeRoundTrip asserts the recorded recipe marshals and unmarshals back
// to itself — the fillet Step (op, input, unresolved selector, radius) survives
// the wire codec.
func requireRecipeRoundTrip(t *testing.T, doc *decad.Document) {
	t.Helper()
	recipe := doc.Recipe()
	buf, err := json.Marshal(recipe)
	require.NoError(t, err)
	var got decad.Recipe
	require.NoError(t, json.Unmarshal(buf, &got))
	require.Equal(t, recipe, got, "the recorded fillet recipe round-trips")
}

// TestFilletScaleInvariant proves the verdict tracks the RELATIVE geometry, not
// the absolute size: the same rectangle at a 1 mm part (k = 0.01) and a 1e6 mm
// part (k = 1e4) builds or refuses identically. A radius clearly inside the valid
// range builds at both scales; one clearly past it refuses at both. The contact
// tolerance is scale-anchored (δ = ε·D), so a fixed absolute band — which would
// flip the verdict between these two scales — is exactly what this rejects.
func TestFilletScaleInvariant(t *testing.T) {
	rng := rand.New(rand.NewSource(filletPropertySeed ^ 0x5ca1e))
	t.Logf("fillet scale-invariance seed = %#x", filletPropertySeed^0x5ca1e)

	scales := []float64{0.01, 1e4}
	specs := []struct {
		frac      float64
		wantBuild bool
	}{
		{0.3, true},  // comfortably inside the valid range
		{1.3, false}, // comfortably past it: the two corner fillets overlap
	}

	const cases = 10
	for range cases {
		fc := genRect(rng) // a plain rectangle: the valid bound is exactly min(w,h)/2
		for _, spec := range specs {
			verdicts := make([]bool, len(scales))
			for i, k := range scales {
				_, body := fc.build(t, k)
				r := spec.frac * fc.convexMaxR * k
				_, err := body.Fillet(convexLateral(), units.Millimeters(r))
				verdicts[i] = err == nil
				if err != nil {
					require.Truef(t, errors.Is(err, decad.ErrDegenerate) || errors.Is(err, decad.ErrUnsupported),
						"scale k=%g frac=%g refused with an unexpected error: %v", k, spec.frac, err)
				}
			}
			require.Equalf(t, verdicts[0], verdicts[1],
				"frac=%g: the same relative geometry must decide the same way at k=%g and k=%g", spec.frac, scales[0], scales[1])
			for i, k := range scales {
				require.Equalf(t, spec.wantBuild, verdicts[i], "frac=%g at k=%g", spec.frac, k)
			}
		}
	}
}

// FuzzFillet drives the accept-then-un-tessellatable guard over a rectangle with
// arbitrary dimensions and radius: whenever Fillet returns nil the body must be a
// solid that tessellates, and its volume must match the (1 − π/4)r² oracle. A
// refusal must be a known sentinel. No input may produce an accepted body that
// then fails to tessellate.
func FuzzFillet(f *testing.F) {
	f.Add(100.0, 60.0, 10.0)
	f.Add(40.0, 40.0, 25.0)  // r past the valid bound: must refuse
	f.Add(80.0, 50.0, 0.001) // a tiny radius
	f.Fuzz(func(t *testing.T, w, h, r float64) {
		// Clamp to a sane, buildable rectangle; the fuzzer explores r freely.
		if !(w >= 10 && w <= 1e4) || !(h >= 10 && h <= 1e4) {
			t.Skip()
		}
		if !(r > 0 && r <= 1e4) || math.IsNaN(r) {
			t.Skip()
		}

		world := sketch.NewWorld()
		s, err := world.CreateSketch(world.XY())
		require.NoError(t, err)
		rect := s.CreateRectangle(0, 0, w, h)
		s.Fix(rect.A)
		if _, err := s.Solve(t.Context()); err != nil {
			t.Skip() // an un-solvable degenerate rectangle is not the target
		}
		doc := decad.New()
		body, err := doc.Extrude(s, pickProfile(t, s, 4, 0), decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
		require.NoError(t, err)

		filleted, err := body.Fillet(convexLateral(), units.Millimeters(r))
		if err != nil {
			require.Truef(t, errors.Is(err, decad.ErrDegenerate) || errors.Is(err, decad.ErrUnsupported),
				"w=%g h=%g r=%g refused with an unexpected error: %v", w, h, r, err)
			require.Equal(t, []*decad.Body{body}, doc.Bodies())
			return
		}
		require.True(t, filleted.IsSolid())
		requireManifold(t, filleted)
		requireTessellates(t, filleted, r, "fuzz", "convex")

		vol, err := filleted.Volume()
		require.NoError(t, err)
		require.Equal(t, decad.Approximate, vol.Exactness)
		require.Positive(t, vol.Bound.Base())
		got, err := vol.Value.In(units.CubicMillimeter)
		require.NoError(t, err)
		want := (w*h - 4*cornerBite(r)) * 20
		require.InDeltaf(t, want, got, 1e-6*math.Max(1, math.Abs(want)), "w=%g h=%g r=%g volume off oracle", w, h, r)
	})
}
