package decad_test

import (
	"bytes"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is the public half of docs/tessellation-reach-design.md §4 (tess
// §13's increment T6): a lofted body meshes by restating the triangle set its
// own construction built, and enters the mesh boolean on the strength of the
// proof record that restatement publishes.

// loftBoxAt is the 40 mm-square axis-aligned box a congruent-square loft
// builds. The two
// sections are identical, so every wall is vertical and every held coordinate
// is an exactly represented integer. That is what lets the assertions below
// pin an EXACT zero rather than a small bound — docs/loft-design.md §5.2's
// facet-departure row grants the zero only where both its terms are zero by
// value, which an unplaced LineSeg-only pairing whose stations are all PINNED
// is the case for.
func loftBoxAt(t *testing.T, doc *decad.Document, z0, height float64) *decad.Body {
	t.Helper()
	const half = 20.0
	s0, p0, s1, p1 := loftSquaresAt(t, r3.NewVec(0, 0, z0), half, half, height)
	body, err := doc.Loft(s0, p0, s1, p1)
	require.NoError(t, err)
	return body
}

// TestLoftTessellate is docs/tessellation-reach-design.md §4's own test list
// for the restatement: the mesh IS the payload's triangle set, face for face
// and vertex for vertex, with no chording of its own.
func TestLoftTessellate(t *testing.T) {
	doc := decad.New()
	body := loftBoxAt(t, doc, 0, 10)

	mesh, err := body.Tessellate(units.Millimeters(1))
	require.NoError(t, err)
	require.NotNil(t, mesh)

	// Four LineSeg cells on the single loop, each chorded at exactly one
	// station and split into two wall triangles, plus a square cap's own two
	// triangles at each end (docs/loft-design.md §5.1's Table C and §7's
	// count).
	const stations = 4
	const capTris = 2
	require.Len(t, mesh.Triangles(), 2*stations+2*capTris)
	require.Len(t, mesh.SourceFaces(), len(mesh.Triangles()))
	requireWatertight(t, mesh)

	// The restatement moves nothing, so the signed volume the facets enclose
	// must land inside the body's own published Volume interval.
	vol, err := body.Volume()
	require.NoError(t, err)
	value, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.InDelta(t, value, meshVolume(mesh), vol.Bound.Base()+1e-9)

	live := map[*decad.Face]struct{}{}
	for _, f := range body.Faces() {
		live[f] = struct{}{}
	}
	for i, f := range mesh.SourceFaces() {
		require.NotNil(t, f)
		require.Containsf(t, live, f, "facet %d names a face the body does not carry", i)
	}

	// Every mesh vertex is a held loft vertex, unrounded: the two coordinate
	// sets must match exactly, not merely closely.
	held := map[r3.Vec]struct{}{}
	for _, v := range body.Vertices() {
		held[v.Position().Value] = struct{}{}
	}
	require.Len(t, held, len(mesh.Vertices()))
	for _, p := range mesh.Vertices() {
		require.Containsf(t, held, p, "mesh vertex %v is not a held loft vertex", p)
	}
}

// TestLoftTessellatePinnedLineSegLoftIsExact is docs/tessellation-design.md
// §14's zero-bound line: an unplaced LineSeg-only loft whose every station is
// PINNED publishes a zero on all four §2 proofs, and the boolean admits it
// through the all-planar zero-bound path.
func TestLoftTessellatePinnedLineSegLoftIsExact(t *testing.T) {
	doc := decad.New()
	loft := loftBoxAt(t, doc, 2, 6)

	mesh, err := loft.Tessellate(units.Millimeters(1))
	require.NoError(t, err)
	require.Zero(t, mesh.Bound().Base(), "a pinned, unplaced LineSeg-only loft holds its own boundary exactly")

	// The loft occupies x,y in [-20, 20] and z in [2, 8]; the prism occupies
	// x,y in [0, 40] and z in [0, 10]. Every crossing coordinate is an
	// exactly represented integer and no face pair is coplanar, so the union
	// is decided on exact contacts alone.
	prism := boxBody(t, doc, 0, 0, 40, 40, 10)
	union, err := decad.Union(loft, prism)
	require.NoError(t, err)

	vol, err := union.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, vol.Exactness,
		"two zero-bound all-planar operands meeting at exact contacts compose an exact volume")
	// 40*40*6 + 40*40*10 - 20*20*6 = 23200 mm^3.
	got, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.InDelta(t, 23200.0, got, 1e-9)
}

// TestLoftTessellatePlacedLoftUsesThePositiveBoundPath is the same fixture
// under a non-identity motion: docs/loft-design.md §5.2's facet-departure row
// reduces to the payload's own delta on a LineSeg-only build, so the mesh
// publishes that displacement and the boolean takes the ordinary
// positive-bound path instead.
func TestLoftTessellatePlacedLoftUsesThePositiveBoundPath(t *testing.T) {
	doc := decad.New()
	loft := loftBoxAt(t, doc, 2, 6)

	rot, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.NewVec(3, -2, 1))
	require.NoError(t, err)
	motion, err := rot.Then(shift)
	require.NoError(t, err)
	placed, err := loft.Placed(motion)
	require.NoError(t, err)

	mesh, err := placed.Tessellate(units.Millimeters(1))
	require.NoError(t, err)
	require.Positive(t, mesh.Bound().Base(), "a placed loft's every held vertex carries the motion's own rounding")

	// A LineSeg-only build charges no chord-to-curve term at all, so its
	// facet departure reduces to delta — the same term Bounds.Bound carries
	// beside a zero sectionDelta. The two readings are composed apart
	// (docs/loft-design.md §5.2's two rows) and each rounds outward through
	// its own chain, so the mesh's may sit an ulp above the box's; it may
	// never sit below it, and it may never be a different quantity.
	box, err := placed.Bounds()
	require.NoError(t, err)
	require.GreaterOrEqual(t, mesh.Bound().Base(), box.Bound.Base())
	require.InEpsilon(t, box.Bound.Base(), mesh.Bound().Base(), 1e-12,
		"a LineSeg-only build's facet departure is its delta, up to each composition's own outward rounding")

	prism := boxBody(t, doc, 0, 0, 40, 40, 10)
	cut, err := decad.Cut(prism, placed)
	require.NoError(t, err)
	vol, err := cut.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, vol.Exactness)
	require.Positive(t, vol.Bound.Base(),
		"a positive-bound operand hands the result the displacement rimDelta composes")
	value, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.Positive(t, value)
	require.Less(t, value, 16000.0, "cutting the loft away can only remove material from the 16000 mm^3 prism")
}

// TestLoftExportIsDeterministic pins docs/tessellation-design.md §1's
// Determinism row over the restatement path: the same payload and tolerance
// must produce the same bytes.
func TestLoftExportIsDeterministic(t *testing.T) {
	doc := decad.New()
	body := loftBoxAt(t, doc, 0, 10)

	var first, second bytes.Buffer
	require.NoError(t, body.STL(&first))
	require.NoError(t, body.STL(&second))
	require.NotEmpty(t, first.Bytes())
	require.True(t, bytes.Equal(first.Bytes(), second.Bytes()), "two STL writes of one loft must agree byte for byte")
}
