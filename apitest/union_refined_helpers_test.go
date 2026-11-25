package apitest_test

import (
	"crypto/sha256"
	"encoding/binary"
	"fmt"
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// unionRefinedOperands returns the operands of refineSectorHubAndLugs's join,
// unjoined: a revolved hub and six copies of one wide point-Loft lug.
func unionRefinedOperands(t testing.TB) (*decad.Document, *decad.Body, []*decad.Body) {
	t.Helper()
	return unionRefinedSet(t, false)
}

// unionRefinedSet is unionRefinedOperands with lugs whose flanks are fitted
// splines when fitted is set. That join's first construction attempt meets a
// cap and root contact in the crossing audit, quarters the sag allowance and
// builds again, so a build that skipped the audit would differ from one that
// ran it.
func unionRefinedSet(t testing.TB, fitted bool) (*decad.Document, *decad.Body, []*decad.Body) {
	t.Helper()
	const count = 6
	doc, hub, lug, _, _ := taperedHubAndRadialLugWidth(t, 9, 7.5, 4.3, 0.8, fitted)
	lugs := []*decad.Body{lug}
	for i := 1; i < count; i++ {
		turn, err := r3.Rotation(r3.Vec{X: 1}, units.Radians(2*math.Pi*float64(i)/float64(count)))
		require.NoError(t, err)
		copied, err := decad.Copy(t.Context(), lug, decad.WithCopyPlacement(turn))
		require.NoError(t, err)
		lugs = append(lugs, copied)
	}
	return doc, hub, lugs
}

// unionRefinedFingerprint holds what two builds of one join must agree on, as
// exact bits: the held mesh, volume, area, bounds and published bound.
type unionRefinedFingerprint struct {
	vertices, triangles int
	meshHash            [sha256.Size]byte
	volume, volumeBound uint64
	area, areaBound     uint64
	box                 [7]uint64
	floor               uint64
	euler               int
	warnings            []string
}

func fingerprintUnionRefined(t *testing.T, body *decad.Body) unionRefinedFingerprint {
	t.Helper()
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.2))
	require.NoError(t, err)
	h := sha256.New()
	for _, v := range mesh.Vertices() {
		require.NoError(t, binary.Write(h, binary.LittleEndian, [3]uint64{
			math.Float64bits(v.X), math.Float64bits(v.Y), math.Float64bits(v.Z)}))
	}
	for _, tri := range mesh.Triangles() {
		require.NoError(t, binary.Write(h, binary.LittleEndian, [3]int64{int64(tri[0]), int64(tri[1]), int64(tri[2])}))
	}
	var fp unionRefinedFingerprint
	fp.vertices, fp.triangles = len(mesh.Vertices()), len(mesh.Triangles())
	copy(fp.meshHash[:], h.Sum(nil))
	volume, err := body.Volume()
	require.NoError(t, err)
	fp.volume, fp.volumeBound = math.Float64bits(volume.Value.Base()), math.Float64bits(volume.Bound.Base())
	area, err := body.Area()
	require.NoError(t, err)
	fp.area, fp.areaBound = math.Float64bits(area.Value.Base()), math.Float64bits(area.Bound.Base())
	box, err := body.Bounds()
	require.NoError(t, err)
	fp.box = [7]uint64{math.Float64bits(box.Min.X), math.Float64bits(box.Min.Y), math.Float64bits(box.Min.Z),
		math.Float64bits(box.Max.X), math.Float64bits(box.Max.Y), math.Float64bits(box.Max.Z),
		math.Float64bits(box.Bound.Base())}
	fp.floor = math.Float64bits(refineSectorFloor(t, body))
	fp.euler = revolveLugEuler(mesh)
	for _, d := range body.OperationDiagnostics() {
		fp.warnings = append(fp.warnings, fmt.Sprintf("%s/%s/%s", d.Code, d.Severity, d.Message))
	}
	return fp
}

// unionThenRefine is the pair UnionRefined replaces, on a fresh set of
// operands.
func unionThenRefine(t *testing.T, fitted bool, bound units.Value) (*decad.Document, *decad.Body, error) {
	t.Helper()
	doc, hub, lugs := unionRefinedSet(t, fitted)
	joined, err := decad.Union(t.Context(), hub, lugs...)
	require.NoError(t, err)
	refined, err := decad.Refine(t.Context(), joined, bound)
	return doc, refined, err
}

func requireOperationKinds(t *testing.T, doc *decad.Document, after int, want ...decad.OperationKind) {
	t.Helper()
	operations := doc.OperationHistory().Operations()
	require.GreaterOrEqual(t, len(operations), after)
	got := make([]decad.OperationKind, 0, len(operations)-after)
	for _, op := range operations[after:] {
		got = append(got, op.Kind())
	}
	require.Equal(t, want, got)
}

// requireUnionRefinedUnchanged requires that the document still holds exactly the
// bodies and history it held. It compares the body lists and counts, never a
// body, because a failing comparison of bodies dumps their whole mesh.
func requireUnionRefinedUnchanged(t *testing.T, doc *decad.Document, bodies []*decad.Body, history int) {
	t.Helper()
	require.True(t, slices.Equal(bodies, doc.Bodies()), "the live bodies changed")
	require.Equal(t, history, len(doc.OperationHistory().Operations()))
}
