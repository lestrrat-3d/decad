package decad_test

import (
	"errors"
	"math"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// capBlendPatchFaces returns every chamfer band patch of a cap-loop chamfer
// result, in the body's own face order.
func capBlendPatchFaces(b *decad.Body) []*decad.Face {
	var out []*decad.Face
	for _, f := range b.Faces() {
		for _, o := range f.Origins() {
			if strings.HasPrefix(o.Role, "chamferCap(") {
				out = append(out, f)
				break
			}
		}
	}
	return out
}

// chamferedPlateSetback is the setback chamferedPlate applies.
const chamferedPlateSetback = 5.0

// chamferedPlate is the 100x60 plate swept 20 mm with its end cap loop
// chamfered — an all-Plane band whose every coordinate is an exact integer, so
// nothing in it is a computed float.
func chamferedPlate(t *testing.T) *decad.Body {
	t.Helper()
	_, box := capBlendBox(t)
	chamfered, err := box.Chamfer(t.Context(), capLoopEdges(box), units.Millimeters(chamferedPlateSetback))
	require.NoError(t, err)
	return chamfered
}

// TestTessellateCapBlendPlateIsExact meshes an all-Plane cap chamfer whose
// every held coordinate is a recorded integer. Nothing is chorded, no contour
// displacement arises from an inexact offset, and both sweep levels are exact,
// so every face's published displacement is zero — the one configuration
// docs/tessellation-design.md §2 lets a mesh claim exactness in.
func TestTessellateCapBlendPlateIsExact(t *testing.T) {
	t.Parallel()
	const d = chamferedPlateSetback
	chamfered := chamferedPlate(t)
	mesh, err := chamfered.Tessellate(t.Context(), units.Millimeters(1))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	require.Len(t, mesh.SourceFaces(), len(mesh.Triangles()))
	require.Equal(t, 0.0, mesh.Bound().Mag(), `an exact all-Plane chamfer chords nothing and rounds nothing`)

	// The mesh IS the solid here, so its enclosed volume is the body's own: the
	// straight slab plus the integral of the eroded section over the setback,
	// int_0^d (100-2t)(60-2t) dt.
	want := 100.0*60.0*(20.0-d) + (6000*d - 320*d*d/2 + 4*d*d*d/3)
	vol, err := chamfered.Volume()
	require.NoError(t, err)
	require.InDelta(t, want, vol.Value.Mag(), 1e-9)
	require.InDelta(t, want, meshVolume(mesh), 1e-9)
	require.InDelta(t, vol.Value.Mag(), meshVolume(mesh), 1e-9)
}

// TestTessellateCapBlendPlateExportsDeterministically pins the export writers
// over the new payload class: repeated STL and OBJ output is byte identical
// (docs/tessellation-design.md §14).
func TestTessellateCapBlendPlateExportsDeterministically(t *testing.T) {
	t.Parallel()
	chamfered := chamferedPlate(t)
	var stlA, stlB, objA, objB strings.Builder
	require.NoError(t, export.STL(t.Context(), &stlA, chamfered, units.Millimeters(0.1)))
	require.NoError(t, export.STL(t.Context(), &stlB, chamfered, units.Millimeters(0.1)))
	require.NoError(t, export.OBJ(t.Context(), &objA, chamfered, units.Millimeters(0.1)))
	require.NoError(t, export.OBJ(t.Context(), &objB, chamfered, units.Millimeters(0.1)))
	require.NotEmpty(t, stlA.String())
	require.NotEmpty(t, objA.String())
	require.Equal(t, stlA.String(), stlB.String())
	require.Equal(t, objA.String(), objB.String())
}

// TestTessellateCapBlendDiskFrustum meshes the one cornerless band — a whole
// closed circle offset into a concentric one — where both directrices sweep the
// SAME window. Its cells are therefore planar quads, and the true frustum
// between the two circles must lie within the published Bound of the mesh.
func TestTessellateCapBlendDiskFrustum(t *testing.T) {
	t.Parallel()
	const (
		r   = 10.0
		h   = 20.0
		d   = 2.0
		tol = 0.25
	)
	body := circleProfile(t, r, h)
	chamfered, err := body.Chamfer(t.Context(), capLoopEdges(body), units.Millimeters(d))
	require.NoError(t, err)
	mesh, err := chamfered.Tessellate(t.Context(), units.Millimeters(tol))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	bound := mesh.Bound().Mag()
	require.Positive(t, bound, `a chorded circle carries a real sagitta`)

	// The two rings carry the same azimuths, so every band cell's four corners
	// are coplanar to within what the coordinates themselves round by — which
	// the mesh's own Bound covers. A band sampled at two densities would put the
	// fourth corner far outside it.
	verts := mesh.Vertices()
	src := mesh.SourceFaces()
	tris := mesh.Triangles()
	patch := map[*decad.Face]bool{}
	for _, f := range capBlendPatchFaces(chamfered) {
		patch[f] = true
	}
	quads := 0
	for i := 0; i+1 < len(tris); i += 2 {
		if !patch[src[i]] || src[i] != src[i+1] {
			continue
		}
		a, b, c := verts[tris[i][0]], verts[tris[i][1]], verts[tris[i][2]]
		fourth := verts[tris[i+1][2]]
		n := b.Sub(a).Cross(c.Sub(a))
		require.Positive(t, n.Len())
		off := math.Abs(fourth.Sub(a).Dot(n)) / n.Len()
		require.LessOrEqual(t, off, bound,
			`band cell %d is not planar to within the published bound`, i)
		quads++
	}
	require.Positive(t, quads, `the disk's band must emit quads`)

	// Falsifier: sample the true chamfer frustum densely. Any point farther from
	// the mesh than Bound disproves the bound; passing samples prove nothing.
	for i := range 41 {
		s := float64(i) / 40
		rr := r - d*s
		z := h - d + d*s
		for j := range 97 {
			th := 2 * math.Pi * float64(j) / 97
			p := r3.Vec{X: rr * math.Cos(th), Y: rr * math.Sin(th), Z: z}
			require.LessOrEqual(t, distanceToMesh(mesh, p), bound,
				`frustum sample %v is farther from the mesh than Bound`, p)
		}
	}
}

// TestTessellateCapBlendHoleWidensWithTheSetback meshes a plate whose circular
// HOLE is chamfered on the end cap. A hole's cap contour is the WIDER
// concentric circle, so the contour ring — not the wall ring — is the directrix
// whose sagitta decides the shared count, and the true widened frustum must
// still lie within the published Bound.
func TestTessellateCapBlendHoleWidensWithTheSetback(t *testing.T) {
	t.Parallel()
	const (
		r   = 10.0
		h   = 8.0
		d   = 2.0
		tol = 0.2
	)
	body := holedPlateBody(t)
	chamfered, err := body.Chamfer(t.Context(), capLoopEdges(body), units.Millimeters(d))
	require.NoError(t, err)
	mesh, err := chamfered.Tessellate(t.Context(), units.Millimeters(tol))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	bound := mesh.Bound().Mag()
	require.Positive(t, bound)

	// The hole sits at (70, 30) and widens from r to r+d as the band rises from
	// h-d to h. Any sample of that true frustum farther from the mesh than Bound
	// disproves the bound.
	for i := range 33 {
		s := float64(i) / 32
		rr := r + d*s
		z := h - d + d*s
		for j := range 91 {
			th := 2 * math.Pi * float64(j) / 91
			p := r3.Vec{X: 70 + rr*math.Cos(th), Y: 30 + rr*math.Sin(th), Z: z}
			require.LessOrEqual(t, distanceToMesh(mesh, p), bound,
				`widened hole sample %v is farther from the mesh than Bound`, p)
		}
	}
}

// TestTessellateCapBlendRoundedRectMiters meshes a band whose corner arcs are
// TRIMMED at mitered offset feet, so the side and cap directrices sweep
// different windows. The mesh must still close, every band patch must own
// facets, and the published bound must exceed the pure chording a coincident
// window would have cost.
func TestTessellateCapBlendRoundedRectMiters(t *testing.T) {
	t.Parallel()
	const (
		rho = 6.0
		d   = 2.0
		tol = 0.4
	)
	body := roundedRectBody(t, 40, 30, 20, rho)
	chamfered, err := body.Chamfer(t.Context(), capLoopEdges(body), units.Millimeters(d))
	require.NoError(t, err)
	mesh, err := chamfered.Tessellate(t.Context(), units.Millimeters(tol))
	require.NoError(t, err)
	requireWatertight(t, mesh)

	// Every chamferCap face the body holds must own facets in the mesh: a patch
	// missing from SourceFaces would mean the band left a hole the closure audit
	// happened not to see.
	held := map[*decad.Face]bool{}
	for _, f := range mesh.SourceFaces() {
		held[f] = true
	}
	patches := capBlendPatchFaces(chamfered)
	require.NotEmpty(t, patches)
	for _, f := range patches {
		require.True(t, held[f], `band patch %s owns no facet`, f.Origins()[0].Role)
	}

	// The mitered corner patches carry a window-skew term on top of their own
	// chording, so the mesh reads strictly above the larger of the two rings'
	// sagitta. chordSagitta is not reachable from here; the internal test
	// TestCapBlendMeshChargesTheWindowSkew pins the term itself.
	require.Positive(t, mesh.Bound().Mag())
	require.Greater(t, mesh.Bound().Mag(), chordSagittaOfRoundedCorner(rho, tol),
		`a mitered band bound must exceed the side ring's own sagitta alone`)
}

// chordSagittaOfRoundedCorner restates docs/tessellation-design.md §3's
// published sagitta for a quarter-turn corner arc chorded at the smallest count
// that fits tol — the figure a coincident-window band would have published on
// its own.
func chordSagittaOfRoundedCorner(radius, tol float64) float64 {
	sweep := math.Pi / 2
	for n := 1; n < 1<<14; n++ {
		s := radius * sweep * sweep / (8 * float64(n) * float64(n))
		if s <= tol {
			return s
		}
	}
	return 0
}

// TestTessellateCapBlendReflexApexFan meshes an L section, whose one reflex
// corner grows an apex patch: a cone whose side directrix has collapsed to the
// original corner point. Its facets must fan from ONE interned vertex, and the
// mesh's own vertex links must stay single cycles there.
func TestTessellateCapBlendReflexApexFan(t *testing.T) {
	t.Parallel()
	const d = 3.0
	body := reflexLBody(t)
	chamfered, err := body.Chamfer(t.Context(), capLoopEdges(body), units.Millimeters(d))
	require.NoError(t, err)
	mesh, err := chamfered.Tessellate(t.Context(), units.Millimeters(0.02))
	require.NoError(t, err)
	requireWatertight(t, mesh)

	apex := apexPatchOf(t, chamfered, "chamferCap(end,")
	shared := map[int]int{}
	fan := 0
	for i, f := range mesh.SourceFaces() {
		if f != apex {
			continue
		}
		fan++
		for _, v := range mesh.Triangles()[i] {
			shared[v]++
		}
	}
	require.Greater(t, fan, 2, `the fan must carry enough triangles for one shared vertex to stand out`)
	var interned []int
	for v, n := range shared {
		if n == fan {
			interned = append(interned, v)
		}
	}
	require.Len(t, interned, 1, `an apex fan closes on exactly one interned vertex`)
	// That vertex is the ORIGINAL reflex corner (20, 40 is convex; the notch sits
	// at (20, 20)) carried to the band's own side level, reflexLHeight - d.
	require.Equal(t, r3.Vec{X: 20, Y: 20, Z: reflexLHeight - d}, mesh.Vertices()[interned[0]])
}

func TestTessellateCapBlendRectangularHoleApexFans(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	outer := s.CreateRectangle(0, 0, 60, 40)
	s.Fix(outer.A)
	s.CreateRectangle(15, 10, 45, 30)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var profile *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			profile = p
		}
	}
	require.NotNil(t, profile)
	doc := decad.New()
	body, err := doc.Extrude(s, profile, decad.Distance{D: units.Millimeters(14), Dir: decad.Along})
	require.NoError(t, err)
	const d = 1.5
	chamfered, err := body.Chamfer(t.Context(), capLoopEdges(body), units.Millimeters(d))
	require.NoError(t, err)
	mesh, err := chamfered.Tessellate(t.Context(), units.Millimeters(0.05))
	require.NoError(t, err)
	requireWatertight(t, mesh)

	apex := map[*decad.Face]int{}
	for _, f := range capBlendPatchFaces(chamfered) {
		if len(f.Loops()[0].CoEdges()) == 3 {
			apex[f] = 0
		}
	}
	require.Len(t, apex, 4, "the four hole corners each own one apex patch")
	for _, f := range mesh.SourceFaces() {
		if _, ok := apex[f]; ok {
			apex[f]++
		}
	}
	for _, n := range apex {
		require.Equal(t, 4, n, "a quarter-turn connector has four facets at 0.05 mm tolerance")
	}

	// The independent section integral subtracts the widening rectangular
	// hole and its four rounded corners from the outer eroded rectangle.
	want := 1800*14 - 300*d*d/2 + (4-math.Pi)*d*d*d/3
	require.InDelta(t, want, meshVolume(mesh), 0.5)

	// Mid-chord points on each true connector arc must fit the published
	// surface displacement. The internal face-bound test pins its sagitta term.
	for _, corner := range []r3.Vec{
		{X: 15, Y: 10}, {X: 45, Y: 10}, {X: 45, Y: 30}, {X: 15, Y: 30},
	} {
		sx, sy := 1.0, 1.0
		if corner.X == 15 {
			sx = -1
		}
		if corner.Y == 10 {
			sy = -1
		}
		th := math.Pi / 16
		point := r3.Vec{X: corner.X + sx*d*math.Cos(th), Y: corner.Y + sy*d*math.Sin(th), Z: 14}
		require.LessOrEqual(t, distanceToMesh(mesh, point), mesh.Bound().Mag(),
			"connector point %v exceeds the mesh bound", point)
	}
}

// TestTessellateCapBlendPlacedStaysWatertight meshes a chamfer under a
// non-identity motion. Every coordinate is then computed, so the published
// bound must be positive and the mesh must still close.
func TestTessellateCapBlendPlacedStaysWatertight(t *testing.T) {
	t.Parallel()
	chamfered := chamferedPlate(t)
	rot, err := r3.Rotation(r3.Vec{X: 1, Y: 2, Z: 3}, units.Radians(0.7))
	require.NoError(t, err)
	moved, err := chamfered.Placed(t.Context(), rot)
	require.NoError(t, err)
	mesh, err := moved.Tessellate(t.Context(), units.Millimeters(1))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	require.Positive(t, mesh.Bound().Mag(),
		`a placed chamfer rounds every coordinate it writes, and says so`)
	require.True(t, mesh.VolumeVerified(),
		`placement rounding is charged into the occupied-volume proof, not a reason to withhold it`)
}

// Cap-loop chamfer bodies as boolean operands (docs/tessellation-reach-design.md
// §7). The mesh publishes the slice-wise occupied-volume proof for a band of
// whole turns, line-line miters and exactly G1 joins, and every boolean then
// takes the body as an operand; any other band stays export-only.
//
// Each leg of that proof was shown to fail: the term was deleted, the named
// test went red, and the term was restored.
//
//	Mchord trimmed term (productUpper(hTrimUpper, sideSegs))
//	    TestCapBlendChamferedPinAsTool: the result volume leaves its bound.
//	Mchord band term (productUpper(dUpper, bandSegs))
//	    TestCapBlendDiskIntersectBandDominated: the result volume leaves its bound.
//	cap station enclosure (lm.capMotion[j] in the cap vertex's motion)
//	    TestCapBlendMeshPublishesVolumeProofForAnAdmittedBand (internal).
//	side level bound (L.bound) and cap level bound (capBandLevel(...).bound)
//	    TestCapBlendMeshMotionCarriesEachLevelBound (internal), one assertion each.
//	exactPrismPointRound (round in the motion)
//	    TestCapBlendMeshMotionCarriesPlacementRounding (internal).
//	line-line miter displacement (the bandDelta arm of capBlendCapMotion)
//	    TestCapBlendMeshMotionCarriesTheMiterDisplacement (internal) and
//	    TestCapBlendBooleanChargesTheMiterDisplacement: the Cut turns Exact.
//	admission arms (whole turn, line-line miter, capJoinIsG1)
//	    TestCapBlendFlangeCutAfterChamfer (whole turn and G1 joins) and
//	    TestCapBlendBooleanAdmitsPlanarBand (line-line miter) refuse when the
//	    arm is removed.

// chamferedPlateVolume is chamferedPlate's own volume: the straight slab plus
// the integral of the eroded section over the setback.
const chamferedPlateVolume = 100.0*60.0*(20.0-chamferedPlateSetback) +
	(6000*chamferedPlateSetback - 320*chamferedPlateSetback*chamferedPlateSetback/2 +
		4*chamferedPlateSetback*chamferedPlateSetback*chamferedPlateSetback/3)

// planarBandOverlap is the volume chamferedPlate shares with the box
// x∈[90,110], y∈[20,40], z∈[10,30]: over y's 20 mm, a flat 5 mm strip of the
// full 10 mm above z = 10, then the band from x = 95 to 100 where the plate's
// top falls from 20 to 15.
const planarBandOverlap = 20 * (5*10 + 37.5)

// TestCapBlendBooleanAdmitsPlanarBand composes an all-Plane chamfer with a box
// crossing its band. Both operands are all-planar and every proof term is
// zero, so each result is the exact closed form and Exact.
func TestCapBlendBooleanAdmitsPlanarBand(t *testing.T) {
	t.Parallel()
	{
		chamfered := chamferedPlate(t)
		mesh, err := chamfered.Tessellate(t.Context(), units.Millimeters(1), decad.WithVerification(decad.VerifyAll))
		require.NoError(t, err)
		require.True(t, mesh.VolumeVerified(), `a band of line-line miters publishes its occupied-volume proof`)
	}
	for _, tc := range []struct {
		name string
		want float64
		run  func(chamfered, tool *decad.Body) (*decad.Body, error)
	}{
		{"intersect", planarBandOverlap, func(a, b *decad.Body) (*decad.Body, error) { return decad.Intersect(t.Context(), a, b) }},
		{"cut", chamferedPlateVolume - planarBandOverlap, func(a, b *decad.Body) (*decad.Body, error) { return decad.Cut(t.Context(), a, b) }},
		{"union", chamferedPlateVolume + 8000 - planarBandOverlap, func(a, b *decad.Body) (*decad.Body, error) { return decad.Union(t.Context(), a, b) }},
		{"union as second operand", chamferedPlateVolume + 8000 - planarBandOverlap, func(a, b *decad.Body) (*decad.Body, error) { return decad.Union(t.Context(), b, a) }},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			chamfered := chamferedPlate(t)
			tool := boxBodyAtZ(t, chamfered.Document(), 90, 20, 110, 40, 10, 20)
			got, err := tc.run(chamfered, tool)
			require.NoError(t, err)
			vol, err := got.Volume()
			require.NoError(t, err)
			require.InDelta(t, tc.want, volumeMM(t, vol), 1e-9)
			require.Equal(t, decad.Approximate, vol.Exactness)
			require.Less(t, boundMM3(t, vol), 1e-8)
			requireBodyWatertight(t, got)
		})
	}
}

// requireCapBlendBooleanRefused asserts every boolean over body refuses it as
// a staging limit naming its loop and the given cause, never a contact
// refusal.
func requireCapBlendBooleanRefused(t *testing.T, build func(*testing.T) *decad.Body, cause string) {
	t.Helper()
	for _, tc := range []struct {
		name string
		run  func(body, tool *decad.Body) (*decad.Body, error)
	}{
		{"union", func(a, b *decad.Body) (*decad.Body, error) { return decad.Union(t.Context(), a, b) }},
		{"cut", func(a, b *decad.Body) (*decad.Body, error) { return decad.Cut(t.Context(), a, b) }},
		{"intersect", func(a, b *decad.Body) (*decad.Body, error) { return decad.Intersect(t.Context(), a, b) }},
		{"tool first", func(a, b *decad.Body) (*decad.Body, error) { return decad.Union(t.Context(), b, a) }},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			body := build(t)
			tool := boxBodyAtZ(t, body.Document(), -5, 5, 5, 25, -5, 30)
			got, err := tc.run(body, tool)
			require.Nil(t, got)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.ErrorContains(t, err, "no proof of the volume")
			require.ErrorContains(t, err, "loop 0")
			require.ErrorContains(t, err, cause)
			var be *decad.BooleanError
			require.False(t, errors.As(err, &be), `an operand the proof does not cover is a staging limit, not a contact refusal`)
		})
	}
}

// TestCapBlendBooleanRefusesMiteredBand keeps the bands §7 does not cover out
// of every boolean: a circular wall at a genuine miter, and a reflex corner.
func TestCapBlendBooleanRefusesMiteredBand(t *testing.T) {
	t.Parallel()
	t.Run("mitered circular wall", func(t *testing.T) {
		t.Parallel()
		requireCapBlendBooleanRefused(t, func(t *testing.T) *decad.Body {
			body := circularSegmentBody(t, 20, 8, 16)
			chamfered, err := body.Chamfer(t.Context(), capLoopEdges(body), units.Millimeters(1))
			require.NoError(t, err)
			return chamfered
		}, "line-line miter or an exactly tangent join")
	})
	t.Run("reflex corner", func(t *testing.T) {
		t.Parallel()
		requireCapBlendBooleanRefused(t, func(t *testing.T) *decad.Body {
			body := reflexLBody(t)
			chamfered, err := body.Chamfer(t.Context(), capLoopEdges(body), units.Millimeters(3))
			require.NoError(t, err)
			return chamfered
		}, "reflex")
	})
}

// TestCapBlendOverlapReadsSuspect pins the Verify half of the same boundary: a
// pair whose overlap cannot be measured is undecided, never silently sound.
func TestCapBlendOverlapReadsSuspect(t *testing.T) {
	t.Parallel()
	body := circularSegmentBody(t, 20, 8, 16)
	chamfered, err := body.Chamfer(t.Context(), capLoopEdges(body), units.Millimeters(1))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.Vec{X: 5, Y: 2, Z: 0})
	require.NoError(t, err)
	overlapping, err := chamfered.PlacedCopy(t.Context(), shift)
	require.NoError(t, err)
	require.NotNil(t, overlapping)

	report, err := chamfered.Document().Verify(t.Context())
	require.NoError(t, err, `an unmeasurable pair reports Suspect, it never fails Verify`)
	require.Equal(t, decad.Suspect, report.Status)
	require.True(t, hasDiagnostic(report, decad.DiagUnsupportedPairPayload),
		`an overlapping cap-blend pair is undecided on its payload, never measured`)
	d, ok := findDiagnostic(report.Diagnostics, decad.DiagUnsupportedPairPayload)
	require.True(t, ok)
	require.Contains(t, d.Message, `no proof of the volume`,
		`a cap-loop chamfer operand names its own cause, never a tessellation limit it does not have`)
	require.Empty(t, report.Interferences,
		`no overlap volume may be published for a pair the boolean refuses`)
}

// TestCapBlendOverlapIsMeasured is the admitted twin: Verify measures an
// admitted cap-blend body's overlap with a box and publishes it as an
// Interference row.
func TestCapBlendOverlapIsMeasured(t *testing.T) {
	t.Parallel()
	chamfered := chamferedPlate(t)
	tool := boxBodyAtZ(t, chamfered.Document(), 90, 20, 110, 40, 10, 20)
	report, err := chamfered.Document().Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Interfering, report.Status, `the pair is measured, not left undecided`)
	require.False(t, hasDiagnostic(report, decad.DiagUnsupportedPairPayload))
	require.Len(t, report.Interferences, 1)
	row := report.Interferences[0]
	require.ElementsMatch(t, []*decad.Body{chamfered, tool}, []*decad.Body{row.A, row.B})
	require.LessOrEqual(t, math.Abs(volumeMM(t, row.Volume)-planarBandOverlap), boundMM3(t, row.Volume))
}

// chamferedFlangeBody builds the drilled filleted flange through the public
// API in one document: a 96×68×16 plate centred at the origin, an analytic
// Cut of an r18 bore, a 12 mm Fillet of the vertical edges and a 1 mm
// cap-loop Chamfer on the end cap.
func chamferedFlangeBody(t *testing.T) (*sketch.World, *decad.Body) {
	t.Helper()
	w := sketch.NewWorld()
	doc := decad.New()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(-48, -34, 48, 34)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	plate, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(16), Dir: decad.Along})
	require.NoError(t, err)
	drilled, err := decad.Cut(t.Context(), plate, cylinderAt(t, w, doc, 0, 18))
	require.NoError(t, err)
	filleted, err := drilled.Fillet(t.Context(), decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1))), units.Millimeters(12))
	require.NoError(t, err)
	chamfered, err := filleted.Chamfer(t.Context(), capLoopEdges(filleted), units.Millimeters(1))
	require.NoError(t, err)
	return w, chamfered
}

// cylinderAt extrudes an r-radius disk centred at (x, 0) symmetrically 32 mm
// each way, so it spans z ∈ [−32, 32] and neither cap lies in a 16 mm plate's
// face.
func cylinderAt(t *testing.T, w *sketch.World, doc *decad.Document, x, r float64) *decad.Body {
	t.Helper()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(x, 0)
	s.Fix(o)
	s.CreateCircle(o, r)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Symmetric{D: units.Millimeters(32)})
	require.NoError(t, err)
	return body
}

// TestCapBlendFlangeCutAfterChamfer drills a bolt hole through the finished
// flange. Its outline joins four lines to four fillet arcs at exactly tangent
// corners and its bore is a whole turn, so the chamfered flange is an ordinary
// operand, and the bolt circle clears every band.
func TestCapBlendFlangeCutAfterChamfer(t *testing.T) {
	t.Parallel()
	w, flange := chamferedFlangeBody(t)
	got, err := decad.Cut(t.Context(), flange, cylinderAt(t, w, flange.Document(), 36, 7))
	require.NoError(t, err)
	vol, err := got.Volume()
	require.NoError(t, err)
	// 16·(6528 − (4 − π)·144 − 324π) is the drilled filleted slab, 116 + 30π
	// the chamfer band removes, and 784π the r7 bolt through 16 mm.
	want := 95116 - 3694*math.Pi
	bound := boundMM3(t, vol)
	require.LessOrEqual(t, math.Abs(volumeMM(t, vol)-want), bound)
	require.Equal(t, decad.Approximate, vol.Exactness)
	require.Positive(t, bound)
	require.Less(t, bound, 50.0, `a ceiling on the published bound, not a pin`)
	require.Len(t, got.Faces(), 21)
	requireBodyWatertight(t, got)
}

// TestCapBlendChamferedPinAsTool cuts a plate with a chamfered pin. The pin's
// chamfer sits 16 mm above the plate, so the plate loses a plain cylinder; the
// pin's trimmed wall runs 63 of its 64 mm, so its chord deficit there is what
// the proof must charge.
func TestCapBlendChamferedPinAsTool(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	doc := decad.New()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(-48, -34, 48, 34)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	plate, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(16), Dir: decad.Along})
	require.NoError(t, err)
	pin := cylinderAt(t, w, doc, 0, 10)
	pin, err = pin.Chamfer(t.Context(), capLoopEdges(pin), units.Millimeters(1))
	require.NoError(t, err)

	got, err := decad.Cut(t.Context(), plate, pin)
	require.NoError(t, err)
	vol, err := got.Volume()
	require.NoError(t, err)
	want := 104448 - 1600*math.Pi
	require.LessOrEqual(t, math.Abs(volumeMM(t, vol)-want), boundMM3(t, vol))
	require.Equal(t, decad.Approximate, vol.Exactness)
	require.Len(t, got.Faces(), 7)
}

// TestCapBlendDiskIntersectBandDominated intersects a short chamfered disk
// with a box enclosing it. The band is 3 mm and the trimmed wall 1 mm, so most
// of the chording deficit sits in the band, which the proof must charge at
// the larger of its two radii.
func TestCapBlendDiskIntersectBandDominated(t *testing.T) {
	t.Parallel()
	body := circleProfile(t, 10, 4)
	chamfered, err := body.Chamfer(t.Context(), capLoopEdges(body), units.Millimeters(3))
	require.NoError(t, err)
	box := boxBodyAtZ(t, chamfered.Document(), -20, -20, 20, 20, -5, 15)
	got, err := decad.Intersect(t.Context(), chamfered, box)
	require.NoError(t, err)
	vol, err := got.Volume()
	require.NoError(t, err)
	// A 1 mm r10 slab under the frustum from r10 to r7 over 3 mm.
	want := 100*math.Pi + math.Pi*(1000-343)/3
	require.LessOrEqual(t, math.Abs(volumeMM(t, vol)-want), boundMM3(t, vol))
}

// TestCapBlendBooleanChargesTheMiterDisplacement cuts a through pocket in a
// chamfered hexagon. Its miter feet sit at irrational points, held only within
// the band's contour displacement, so the result is Approximate rather than an
// exactness the proof never earned.
func TestCapBlendBooleanChargesTheMiterDisplacement(t *testing.T) {
	t.Parallel()
	doc, hexagon := manySidedPrism(t, 6)
	chamfered, err := hexagon.Chamfer(t.Context(), capLoopEdges(hexagon), units.Millimeters(2))
	require.NoError(t, err)
	before, err := chamfered.Volume()
	require.NoError(t, err)
	pocket := boxBodyAtZ(t, doc, -10, -10, 10, 10, -5, 30)
	got, err := decad.Cut(t.Context(), chamfered, pocket)
	require.NoError(t, err)
	vol, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, vol.Exactness)
	want := volumeMM(t, before) - 20*20*filletBoxHeight
	require.LessOrEqual(t, math.Abs(volumeMM(t, vol)-want), boundMM3(t, vol)+boundMM3(t, before))
}
