package decad

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// boxAt is an sx x sy x sz box with its low corner at (x, y, z), in doc. A
// non-identity translation keeps a Boolean on the mesh path.
func boxAt(t *testing.T, w *sketch.World, doc *Document, x, y, z, sx, sy, sz float64) *Body {
	t.Helper()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	r := s.CreateRectangle(0, 0, sx, sy)
	s.Fix(r.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	box, err := Extrude(t.Context(), doc, s, s.Profiles()[0], Distance{D: units.Millimeters(sz), Dir: Along})
	require.NoError(t, err)
	move, err := r3.Translation(r3.Vec{X: x, Y: y, Z: z})
	require.NoError(t, err)
	placed, err := Transform(t.Context(), box, move)
	require.NoError(t, err)
	return placed
}

// planeFaceAt returns the face of b whose retained tag is the plane at height z.
func planeFaceAt(t *testing.T, b *Body, z float64) *Face {
	t.Helper()
	for _, f := range b.Faces() {
		pl, ok := f.n.Surface.(surfacegeom.Plane)
		if ok && f.n.RetainedTag && pl.Frame.Origin().Z > z-1e-9 && pl.Frame.Origin().Z < z+1e-9 {
			return f
		}
	}
	require.FailNow(t, "no retained plane at that height")
	return nil
}

// patchesOf returns the faces of b whose record descends directly from parent.
func patchesOf(b *Body, parent *Face) []*Face {
	var out []*Face
	for _, f := range b.Faces() {
		if f.n.Source != nil && f.n.Source.Parent == parent.n.Source {
			out = append(out, f)
		}
	}
	return out
}

func TestSplitPlaneSharesOneSourceRecord(t *testing.T) {
	sc := newSourceScene(t)
	bored, err := Cut(t.Context(), sc.plate, sc.tool(t, 20))
	require.NoError(t, err)
	boredTop := planeFaceAt(t, bored, 20)
	boredBottom := planeFaceAt(t, bored, 0)
	require.NotNil(t, boredTop.n.Source)

	// A slot across the full width splits the top into two patches.
	slot := boxAt(t, sc.w, sc.doc, 28, -5, 10, 4, 30, 15)
	split, err := Cut(t.Context(), bored, slot)
	require.NoError(t, err)
	patches := patchesOf(split, boredTop)
	require.Len(t, patches, 2)
	require.Same(t, patches[0].n.Source, patches[1].n.Source, "both patches carry one record")
	require.Equal(t, 0, patches[0].n.Source.Operand)
	require.Equal(t, bored.n.Origin.Producer, patches[0].n.Source.Producer)
	var sum, bound float64
	for _, p := range patches {
		a, err := p.Area()
		require.NoError(t, err)
		sum += a.Value.Mag()
		bound += a.Bound.Mag()
	}
	const want = 40*20 - 4*20 - 3.141592653589793*16
	require.InDelta(t, want, sum, bound+1e-9)

	// Cutting again turns the two patches into separate operand faces with
	// equal Origins, which get distinct records with one shared Parent.
	again, err := Cut(t.Context(), split, sc.tool(t, 8))
	require.NoError(t, err)
	faces := topobuild.Faces(&split.n)
	var records []*facetedrecord.SourcePatch
	for i, f := range faces {
		if f.Source != patches[0].n.Source {
			continue
		}
		records = append(records, facetedGroups(t, again)[i].Source)
	}
	require.Len(t, records, 2)
	require.NotSame(t, records[0], records[1], "equal Origins must not merge records")
	require.Same(t, records[0].Parent, records[1].Parent)
	require.Same(t, patches[0].n.Source, records[0].Parent)
	require.NotEqual(t, records[0].Ordinal, records[1].Ordinal)

	// An untouched plane two Cuts later has the first Cut's record as grandparent
	// and a bit-equal frame.
	splitBottom := patchesOf(split, boredBottom)
	require.Len(t, splitBottom, 1)
	lastBottom := patchesOf(again, splitBottom[0])
	require.Len(t, lastBottom, 1)
	require.Same(t, boredBottom.n.Source, lastBottom[0].n.Source.Parent.Parent)
	require.True(t, boredBottom.n.Surface.(surfacegeom.Plane) == lastBottom[0].n.Surface.(surfacegeom.Plane))

	// A Transform keeps the pointers.
	move, err := r3.Translation(r3.Vec{X: 3})
	require.NoError(t, err)
	moved, err := Transform(t.Context(), again, move)
	require.NoError(t, err)
	before, after := facetedGroups(t, again), facetedGroups(t, moved)
	require.Len(t, after, len(before))
	for i := range before {
		require.Same(t, before[i].Source, after[i].Source, "group %d", i)
	}

	// The public record still reports an unknown, empty correspondence.
	var cut *Operation
	for _, op := range sc.doc.OperationHistory().Operations() {
		if outs := op.Outputs(); op.Kind() == OperationCut && len(outs) == 1 && outs[0] == split {
			cut = op
		}
	}
	require.NotNil(t, cut)
	h, err := cut.FaceSuccessors(bored.Faces()[0])
	require.NoError(t, err)
	require.Equal(t, HistoryUnknown, h.Coverage())
	require.Empty(t, h.Faces())
}

func TestShiftedRetryGroupsKeepSource(t *testing.T) {
	w := sketch.NewWorld()
	doc := New()
	a := boxAt(t, w, doc, 0, 0, 0, 10, 10, 10)
	b := boxAt(t, w, doc, 0.5, 0, 10, 10, 10, 10)
	joined, err := Union(t.Context(), a, b)
	require.NoError(t, err)
	shifted := false
	for _, d := range joined.OperationDiagnostics() {
		shifted = shifted || d.Code == OpDiagGapShiftApplied
	}
	require.True(t, shifted, "the pair must take the shifted retry")

	groups := facetedGroups(t, joined)
	nA := len(topobuild.Faces(&a.n))
	facesB := topobuild.Faces(&b.n)
	require.Len(t, groups, nA+len(facesB))
	// The moved box's planes carry their tags translated by the one shift s
	// the retry applied, exactly: every moved origin is the rational o + s.
	first, ok := groups[nA].Surface.(surfacegeom.Plane)
	require.True(t, ok, "the moved operand keeps its tag, translated")
	unmoved, ok := facesB[0].Surface.(surfacegeom.Plane)
	require.True(t, ok)
	s := first.Frame.Origin().Sub(unmoved.Frame.Origin())
	require.Positive(t, s.Len())
	require.LessOrEqual(t, s.Len(), 0.01, "the shift stays within the default gap tolerance")
	for i, g := range groups {
		require.NotNil(t, g.Source, "group %d", i)
		if i < nA {
			require.Equal(t, 0, g.Source.Operand)
			require.Equal(t, i, g.Source.Ordinal)
			require.Equal(t, a.n.Origin.Producer, g.Source.Producer)
			continue
		}
		require.Equal(t, 1, g.Source.Operand, "group %d", i)
		require.Equal(t, i-nA, g.Source.Ordinal)
		require.Equal(t, b.n.Origin.Producer, g.Source.Producer)
		tag, ok := g.Surface.(surfacegeom.Plane)
		require.True(t, ok, "group %d keeps its plane, translated", i)
		src, ok := facesB[i-nA].Surface.(surfacegeom.Plane)
		require.True(t, ok)
		requireExactTranslation(t, src.Frame.Origin(), s, tag.Frame.Origin())
		require.Equal(t, src.Frame.N(), tag.Frame.N())
		require.True(t, g.Retained)
		require.Zero(t, g.NormalBound, "an exact translation states no departure")
		require.Nil(t, g.CapRecord, "the record states the unmoved plane")
	}
}

// requireExactTranslation asserts got is exactly the rational o + s in every
// coordinate.
func requireExactTranslation(t *testing.T, o, s, got r3.Vec) {
	t.Helper()
	for _, c := range [][3]float64{{o.X, s.X, got.X}, {o.Y, s.Y, got.Y}, {o.Z, s.Z, got.Z}} {
		sum := new(big.Rat).Add(new(big.Rat).SetFloat64(c[0]), new(big.Rat).SetFloat64(c[1]))
		require.Zero(t, sum.Cmp(new(big.Rat).SetFloat64(c[2])), "%v + %v is not exactly %v", c[0], c[1], c[2])
	}
}
