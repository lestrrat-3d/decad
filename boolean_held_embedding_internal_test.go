package decad

import (
	"fmt"
	"math"
	"os"
	"strconv"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// heldMeshFolds is an independent all-pairs oracle for the Embedding row of
// docs/tessellation-design.md §1: every facet pair whose boxes meet is
// classified exactly over its held binary64 corners, and each pair that meets
// beyond the vertex or edge its indices share is reported. It shares no code
// with meshbool.EnforceHeldEmbedding's grid or search.
func heldMeshFolds(t *testing.T, verts []r3.Vec, tris [][3]int) []string {
	t.Helper()
	x := make([]proofbound.Xpt, len(verts))
	for i, v := range verts {
		x[i] = proofbound.XptOf(v)
	}
	norms := make([]proofbound.Xpt, len(tris))
	boxes := make([][2]r3.Vec, len(tris))
	for i, tri := range tris {
		norms[i] = meshbool.Xcross(proofbound.Xsub(x[tri[1]], x[tri[0]]), proofbound.Xsub(x[tri[2]], x[tri[0]]))
		boxes[i] = meshbool.TriBox(verts, tri)
	}
	corners := func(i int) ([3]r3.Vec, [3]proofbound.Xpt) {
		tri := tris[i]
		return [3]r3.Vec{verts[tri[0]], verts[tri[1]], verts[tri[2]]}, [3]proofbound.Xpt{x[tri[0]], x[tri[1]], x[tri[2]]}
	}
	var out []string
	for i := range tris {
		for j := i + 1; j < len(tris); j++ {
			if !meshbool.BoxesOverlap(boxes[i], boxes[j]) {
				continue
			}
			var si, sj []int
			for a, va := range tris[i] {
				for b, vb := range tris[j] {
					if va == vb {
						si, sj = append(si, a), append(sj, b)
					}
				}
			}
			fa, xa := corners(i)
			fb, xb := corners(j)
			c, err := meshbool.TriTriClassify(fa, fb, xa, xb, norms[i], norms[j])
			ok := err == nil
			if ok {
				switch len(si) {
				case 0:
					ok = c.Kind == meshbool.ContactNone
				case 1:
					ok = c.Kind == meshbool.ContactPoint ||
						(c.Kind == meshbool.ContactRegion && !meshbool.CoplanarOverlap(xa, xb, norms[i]))
				case 2:
					apexI, apexJ := xa[3-si[0]-si[1]], xb[3-sj[0]-sj[1]]
					ok = c.Kind == meshbool.ContactSegment ||
						(c.Kind == meshbool.ContactRegion &&
							meshbool.PlaneSide(xa[si[0]], xa[si[1]], apexI, norms[i])*meshbool.PlaneSide(xa[si[0]], xa[si[1]], apexJ, norms[i]) < 0)
				default:
					ok = false
				}
			}
			if !ok {
				out = append(out, fmt.Sprintf("facets %d/%d share %d vertices, meet as %d: %v / %v", i, j, len(si), c.Kind, fa, fb))
			}
		}
	}
	return out
}

func clusterPolygon(t *testing.T, n int, r float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	poly, err := s.CreatePolygon(0, 0, n, r)
	require.NoError(t, err)
	s.Fix(poly.Center)
	for _, v := range poly.Vertices {
		s.Fix(v)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	return s, s.Profiles()[0]
}

func clusterPath(t *testing.T, pts ...r3.Vec) *Path {
	t.Helper()
	segs := make([]PathSegment, 0, len(pts)-1)
	for _, p := range pts[1:] {
		segs = append(segs, LineTo{End: p})
	}
	p, err := NewPath(pts[0], segs...)
	require.NoError(t, err)
	return p
}

// clusteredTree is the overlapping-branch tree that broke the mesh boolean's
// cutter: a tapered mitred 16-gon trunk, and tilted tapered mitred 16-gon
// branches rooted inside it, each turned spread radians about z from the last
// and rooted zstep millimetres higher, so neighbouring branches overlap.
type clusteredTree struct {
	doc           *Document
	trunk         *Body
	branchSketch  *sketch.Sketch
	branchProfile *sketch.Profile
	spread, zstep float64
}

func newClusteredTree(t *testing.T, spreadDeg, zstep float64) clusteredTree {
	t.Helper()
	doc := New()
	s, p := clusterPolygon(t, 16, 5)
	trunk, err := doc.Sweep(t.Context(), s, p,
		clusterPath(t, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 30), r3.NewVec(3, 0, 60)),
		WithMitredJoins(), WithSectionScale(units.Scalar(1), units.Scalar(0.9)))
	require.NoError(t, err)
	bs, bp := clusterPolygon(t, 16, 1.5)
	return clusteredTree{doc: doc, trunk: trunk, branchSketch: bs, branchProfile: bp,
		spread: spreadDeg * math.Pi / 180, zstep: zstep}
}

func (c clusteredTree) branch(t *testing.T, k int) *Body {
	t.Helper()
	unit, err := c.doc.Sweep(t.Context(), c.branchSketch, c.branchProfile,
		clusterPath(t, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 2.5), r3.NewVec(0.8, 0.3, 6), r3.NewVec(2.5, 0.6, 9.5)),
		WithMitredJoins(), WithSectionScale(units.Scalar(1.375/1.5), units.Scalar(1.2/1.5), units.Scalar(1/1.5)))
	require.NoError(t, err)
	phi := c.spread * float64(k)
	u := r3.NewVec(math.Cos(phi), math.Sin(phi), 0)
	alpha := 40 * math.Pi / 180
	d := u.Scale(math.Sin(alpha)).Add(r3.NewVec(0, 0, math.Cos(alpha)))
	fu := r3.NewVec(0, 0, 1).Cross(d)
	fu = fu.Scale(1 / fu.Len())
	frame, err := r3.NewFrame(u.Scale(2.6).Add(r3.NewVec(0, 0, 4+c.zstep*float64(k))), fu, d.Cross(fu))
	require.NoError(t, err)
	xf, err := r3.FromFrame(frame)
	require.NoError(t, err)
	placed, err := unit.Placed(t.Context(), xf)
	require.NoError(t, err)
	return placed
}

// TestUnionKeepsTheHeldMeshEmbedded is the union-2 regression: the second
// overlapping branch of the 25° / 1.1 mm cluster puts a rim vertex within an
// ulp of a line of earlier vertices, and its nearest float rounding folds two
// pairs of held facets through each other. Every union's held mesh must stay
// embedded under the exact all-pairs oracle.
func TestUnionKeepsTheHeldMeshEmbedded(t *testing.T) {
	t.Parallel()
	c := newClusteredTree(t, 25, 1.1)
	tree := c.trunk
	for k := range 3 {
		var err error
		tree, err = Union(t.Context(), tree, c.branch(t, k))
		require.NoError(t, err, `union %d`, k+1)
		pp, ok := tree.payload.(facetedPayload)
		require.True(t, ok)
		require.Empty(t, heldMeshFolds(t, pp.verts, pp.tris), `union %d's held mesh is embedded`, k+1)
		require.Len(t, tree.Lumps(), 1)
	}
}

// TestPlacedFacetedBodyStaysEmbedded turns a clustered union result about a
// tilted axis: the motion's float evaluation moves every held vertex, and the
// placed mesh must stay embedded too.
func TestPlacedFacetedBodyStaysEmbedded(t *testing.T) {
	t.Parallel()
	c := newClusteredTree(t, 25, 1.1)
	tree := c.trunk
	for k := range 2 {
		var err error
		tree, err = Union(t.Context(), tree, c.branch(t, k))
		require.NoError(t, err)
	}
	axis := r3.NewVec(1, 2, 3)
	turn, err := r3.Rotation(axis.Scale(1/axis.Len()), units.Radians(0.7))
	require.NoError(t, err)
	placed, err := tree.Placed(t.Context(), turn)
	require.NoError(t, err)
	pp, ok := placed.payload.(facetedPayload)
	require.True(t, ok)
	require.Empty(t, heldMeshFolds(t, pp.verts, pp.tris))
}

// TestClusteredUnionChainSurvey runs a long clustered chain on request and
// reports, per union, the held fold count and the time it took. It is a
// measurement, not a CI test.
func TestClusteredUnionChainSurvey(t *testing.T) {
	if os.Getenv("DECAD_CLUSTER_SURVEY") == "" {
		t.Skip("set DECAD_CLUSTER_SURVEY=spreadDeg,zstep,branches to run")
	}
	var spread, zstep float64
	var n int
	_, err := fmt.Sscanf(os.Getenv("DECAD_CLUSTER_SURVEY"), "%g,%g,%d", &spread, &zstep, &n)
	require.NoError(t, err)
	check, _ := strconv.ParseBool(os.Getenv("DECAD_CLUSTER_ORACLE"))
	c := newClusteredTree(t, spread, zstep)
	tree := c.trunk
	for k := range n {
		b := c.branch(t, k)
		start := time.Now()
		tree, err = Union(t.Context(), tree, b)
		require.NoError(t, err, `union %d`, k+1)
		pp := tree.payload.(facetedPayload)
		folds := -1
		if check {
			folds = len(heldMeshFolds(t, pp.verts, pp.tris))
		}
		t.Logf("union %2d: %v, %d facets, %d folds", k+1, time.Since(start), len(pp.tris), folds)
		require.Len(t, tree.Lumps(), 1)
	}
}
