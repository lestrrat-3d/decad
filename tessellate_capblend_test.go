package decad_test

import (
	"math"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
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
	chamfered, err := box.Chamfer(capLoopEdges(box), units.Millimeters(chamferedPlateSetback))
	require.NoError(t, err)
	return chamfered
}

// TestTessellateCapBlendPlateIsExact meshes an all-Plane cap chamfer whose
// every held coordinate is a recorded integer. Nothing is chorded, no contour
// displacement arises from an inexact offset, and both sweep levels are exact,
// so every face's published displacement is zero — the one configuration
// docs/tessellation-design.md §2 lets a mesh claim exactness in.
func TestTessellateCapBlendPlateIsExact(t *testing.T) {
	const d = chamferedPlateSetback
	chamfered := chamferedPlate(t)
	mesh, err := chamfered.Tessellate(units.Millimeters(1))
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
	chamfered := chamferedPlate(t)
	var stlA, stlB, objA, objB strings.Builder
	require.NoError(t, chamfered.STL(&stlA))
	require.NoError(t, chamfered.STL(&stlB))
	require.NoError(t, chamfered.OBJ(&objA))
	require.NoError(t, chamfered.OBJ(&objB))
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
	const (
		r   = 10.0
		h   = 20.0
		d   = 2.0
		tol = 0.25
	)
	body := circleProfile(t, r, h)
	chamfered, err := body.Chamfer(capLoopEdges(body), units.Millimeters(d))
	require.NoError(t, err)
	mesh, err := chamfered.Tessellate(units.Millimeters(tol))
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
	const (
		r   = 10.0
		h   = 8.0
		d   = 2.0
		tol = 0.2
	)
	body := holedPlateBody(t)
	chamfered, err := body.Chamfer(capLoopEdges(body), units.Millimeters(d))
	require.NoError(t, err)
	mesh, err := chamfered.Tessellate(units.Millimeters(tol))
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
	const (
		rho = 6.0
		d   = 2.0
		tol = 0.4
	)
	body := roundedRectBody(t, 40, 30, 20, rho)
	chamfered, err := body.Chamfer(capLoopEdges(body), units.Millimeters(d))
	require.NoError(t, err)
	mesh, err := chamfered.Tessellate(units.Millimeters(tol))
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
	const d = 3.0
	body := reflexLBody(t)
	chamfered, err := body.Chamfer(capLoopEdges(body), units.Millimeters(d))
	require.NoError(t, err)
	mesh, err := chamfered.Tessellate(units.Millimeters(0.02))
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

// TestTessellateCapBlendPlacedStaysWatertight meshes a chamfer under a
// non-identity motion. Every coordinate is then computed, so the published
// bound must be positive and the mesh must still close.
func TestTessellateCapBlendPlacedStaysWatertight(t *testing.T) {
	chamfered := chamferedPlate(t)
	rot, err := r3.Rotation(r3.Vec{X: 1, Y: 2, Z: 3}, units.Radians(0.7))
	require.NoError(t, err)
	moved, err := chamfered.Placed(rot)
	require.NoError(t, err)
	mesh, err := moved.Tessellate(units.Millimeters(1))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	require.Positive(t, mesh.Bound().Mag(),
		`a placed chamfer rounds every coordinate it writes, and says so`)
}

// TestCapBlendMeshIsExportOnly is the increment's boundary: the mesh exists and
// exports, and every boolean still refuses it, because no proof of the volume
// it and the body it stands for differ by has been built
// (docs/tessellation-reach-design.md §7, §9).
func TestCapBlendMeshIsExportOnly(t *testing.T) {
	chamfered := chamferedPlate(t)
	mesh, err := chamfered.Tessellate(units.Millimeters(1))
	require.NoError(t, err)
	require.NotEmpty(t, mesh.Triangles())

	tool := boxBody(t, chamfered.Document(), 10, 10, 30, 30, 40)
	for _, tc := range []struct {
		name string
		run  func() (*decad.Body, error)
	}{
		{"union", func() (*decad.Body, error) { return decad.Union(chamfered, tool) }},
		{"cut", func() (*decad.Body, error) { return decad.Cut(chamfered, tool) }},
		{"intersect", func() (*decad.Body, error) { return decad.Intersect(chamfered, tool) }},
		{"union as second operand", func() (*decad.Body, error) { return decad.Union(tool, chamfered) }},
	} {
		t.Run(tc.name, func(t *testing.T) {
			body, err := tc.run()
			require.Nil(t, body)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.ErrorContains(t, err, "no proof of the volume")
		})
	}
}

// TestCapBlendOverlapReadsSuspect pins the Verify half of the same boundary: a
// pair whose overlap cannot be measured is undecided, never silently sound.
func TestCapBlendOverlapReadsSuspect(t *testing.T) {
	_, box := capBlendBox(t)
	chamfered, err := box.Chamfer(capLoopEdges(box), units.Millimeters(5))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.Vec{X: 20, Y: 10, Z: 0})
	require.NoError(t, err)
	overlapping, err := chamfered.PlacedCopy(shift)
	require.NoError(t, err)
	require.NotNil(t, overlapping)

	report, err := chamfered.Document().Verify(t.Context())
	require.NoError(t, err, `an unmeasurable pair reports Suspect, it never fails Verify`)
	require.Equal(t, decad.Suspect, report.Status)
	require.True(t, hasDiagnostic(report, decad.DiagUnsupportedPairPayload),
		`an overlapping cap-blend pair is undecided on its payload, never measured`)
	require.Empty(t, report.Interferences,
		`no overlap volume may be published for a pair the boolean refuses`)
}
