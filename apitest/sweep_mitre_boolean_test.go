package apitest_test

import (
	"archive/zip"
	"bytes"
	"encoding/xml"
	"io"
	"math"
	"math/big"
	"slices"
	"strconv"
	"strings"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// mtiltTree is the mtilt-shaped union fixture: a tapered 16-gon trunk and
// branches tapered 1.5 → 1 mm, each built at the origin and translated so its
// first millimetre is buried below the trunk's top cap, then leaning outward.
type mtiltTree struct {
	trunkTop  float64
	ring      float64
	firstSpan float64
	factors   []float64
}

func newMtiltTree() mtiltTree {
	return mtiltTree{trunkTop: 20, ring: 7.5, firstSpan: 2, factors: []float64{1.375 / 1.5, 1.2 / 1.5, 1 / 1.5}}
}

func (m mtiltTree) branch(t *testing.T, doc *decad.Document, theta float64) *decad.Body {
	t.Helper()
	s, profile := polygonSweepSketch(t, 16, 1.5)
	u := r3.NewVec(math.Cos(theta), math.Sin(theta), 0)
	path := mustPath(t, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, m.firstSpan),
		u.Scale(1.5).Add(r3.NewVec(0, 0, 5.5)), u.Scale(4).Add(r3.NewVec(0, 0, 9)))
	body, err := doc.Sweep(t.Context(), s, profile, path,
		decad.WithMitredJoins(), decad.WithSectionScale(scalars(m.factors...)...))
	require.NoError(t, err)
	move, err := r3.Translation(u.Scale(m.ring).Add(r3.NewVec(0, 0, m.trunkTop-1)))
	require.NoError(t, err)
	placed, err := body.Placed(t.Context(), move)
	require.NoError(t, err)
	return placed
}

// buriedVolume is the exact volume of a branch's part below the trunk's top
// cap: the first span is a frustum whose section scales linearly from 1 at
// its start to factors[0] at its end, cut one millimetre up.
func (m mtiltTree) buriedVolume(area *big.Rat) *big.Rat {
	f := new(big.Rat).SetFloat64(m.factors[0])
	// s = 1 + (f − 1)·(1 / firstSpan) at the trunk's top.
	s := new(big.Rat).Sub(f, big.NewRat(1, 1))
	s.Quo(s, new(big.Rat).SetFloat64(m.firstSpan))
	s.Add(s, big.NewRat(1, 1))
	sum := new(big.Rat).Add(big.NewRat(1, 1), s)
	sum.Add(sum, new(big.Rat).Mul(s, s))
	v := new(big.Rat).Mul(area, sum)
	return v.Quo(v, big.NewRat(3, 1))
}

// polygonArea is the exact shoelace area of the unplaced branch's start cap.
func polygonArea(t *testing.T, body *decad.Body) *big.Rat {
	t.Helper()
	var startCap *decad.Face
	for _, f := range body.Faces() {
		if f.Origins()[0].Role == roleCapStart {
			startCap = f
		}
	}
	require.NotNil(t, startCap)
	var pts []r3.Vec
	for _, ce := range startCap.Loops()[0].CoEdges() {
		pts = append(pts, ce.Start().Position().Value)
	}
	sum := new(big.Rat)
	for i, p := range pts {
		q := pts[(i+1)%len(pts)]
		term := new(big.Rat).Mul(new(big.Rat).SetFloat64(p.X), new(big.Rat).SetFloat64(q.Y))
		term.Sub(term, new(big.Rat).Mul(new(big.Rat).SetFloat64(q.X), new(big.Rat).SetFloat64(p.Y)))
		sum.Add(sum, term)
	}
	sum.Abs(sum)
	return sum.Quo(sum, big.NewRat(2, 1))
}

func TestSweepMitredTreeUnionChain(t *testing.T) {
	t.Parallel()
	m := newMtiltTree()
	doc := decad.New()
	trunkSketch, trunkProfile := polygonSweepSketch(t, 16, 10)
	trunk, err := doc.Sweep(t.Context(), trunkSketch, trunkProfile,
		mustPath(t, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, m.trunkTop)),
		decad.WithSectionScale(units.Scalar(0.95)))
	require.NoError(t, err)
	trunkVol, err := trunk.Volume()
	require.NoError(t, err)

	branchSketch, branchProfile := polygonSweepSketch(t, 16, 1.5)
	unit, err := decad.New().Sweep(t.Context(), branchSketch, branchProfile, mustPath(t, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1)), decad.WithMitredJoins())
	require.NoError(t, err)
	buried := m.buriedVolume(polygonArea(t, unit))

	const branches = 15
	tree := trunk
	ref := new(big.Rat).SetFloat64(trunkVol.Value.Base())
	slack := trunkVol.Bound.Base()
	for i := range branches {
		b := m.branch(t, doc, 2*math.Pi*float64(i)/branches)
		vol, err := b.Volume()
		require.NoError(t, err)
		ref.Add(ref, new(big.Rat).SetFloat64(vol.Value.Base()))
		ref.Sub(ref, buried)
		slack += vol.Bound.Base()

		start := time.Now()
		tree, err = decad.Union(t.Context(), tree, b)
		require.NoError(t, err, `union %d`, i+1)
		t.Logf("union %2d: %v, %d faces", i+1, time.Since(start), len(tree.Faces()))
		require.Len(t, tree.Lumps(), 1, `union %d is one lump`, i+1)
	}

	got, err := tree.Volume()
	require.NoError(t, err)
	gap := new(big.Rat).Sub(new(big.Rat).SetFloat64(got.Value.Base()), ref)
	gap.Abs(gap)
	allow := new(big.Rat).SetFloat64(got.Bound.Base())
	allow.Add(allow, new(big.Rat).SetFloat64(slack))
	gapF, _ := gap.Float64()
	t.Logf("volume %v ± %v, reference gap %v, operand slack %v", got.Value.Base(), got.Bound.Base(), gapF, slack)
	require.Positive(t, got.Bound.Base())
	require.LessOrEqual(t, gap.Cmp(allow), 0, `the published volume encloses the reference`)

	mesh, err := tree.Tessellate(t.Context(), units.Millimeters(0.01), decad.WithVerification(decad.VerifyBoundary))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	requireWatertight(t, mesh)
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	br, err := report.ForBody(tree)
	require.NoError(t, err)
	require.Equal(t, decad.ValidityValid, br.Validity.Outcome)
	require.Equal(t, 1, br.Topology.Lumps)
}

// TestSweepMitredCrossingUnion is docs/sweep-design.md §16.10's M2 row: two
// branches that cross union into one lump whose volume lies within its bound
// of V1 + V2 − overlap. The second branch is turned to run along x and
// shifted half a millimetre in y, so no two walls are coplanar and the
// overlap is the box 2 × 1.5 × 2.
func TestSweepMitredCrossingUnion(t *testing.T) {
	t.Parallel()
	s, profile := squareSweepSketch(t, 2)
	doc := decad.New()
	path := mustPath(t, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 10))
	a, err := doc.Sweep(t.Context(), s, profile, path, decad.WithMitredJoins())
	require.NoError(t, err)
	b, err := doc.Sweep(t.Context(), s, profile, path, decad.WithMitredJoins())
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.NewVec(-5, 0.5, 5), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	turn, err := r3.FromFrame(frame)
	require.NoError(t, err)
	b, err = b.Placed(t.Context(), turn)
	require.NoError(t, err)

	// Table DM row DM4: Verify reads the same pair's overlap through the
	// mesh path the boolean admits.
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Interfering, report.Status)
	require.Len(t, report.Interferences, 1)
	overlap := report.Interferences[0].Volume
	require.InDelta(t, 2*1.5*2, overlap.Value.Base(), overlap.Bound.Base())

	u, err := decad.Union(t.Context(), a, b)
	require.NoError(t, err)
	require.Len(t, u.Lumps(), 1)
	vol, err := u.Volume()
	require.NoError(t, err)
	require.InDelta(t, 40+40-2*1.5*2, vol.Value.Base(), vol.Bound.Base())
	require.Equal(t, []*decad.Body{u}, doc.Bodies())

	mesh, err := u.Tessellate(t.Context(), units.Millimeters(0.01), decad.WithVerification(decad.VerifyBoundary))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	requireWatertight(t, mesh)
}

// TestSweepMitredTaperedBranchesCross unions two tapered 16-gon branches whose
// leaning spans cross in an X. Every vertex carries delta > 0 and the walls
// cross away from every facet corner, so the hidden-tangency gate admits the
// pair only through the segment-anchored deep witness (Table DM row DM3).
// No closed form states this overlap, so the reading is checked by inclusion
// and exclusion against the intersection and the Verify interference row.
func TestSweepMitredTaperedBranchesCross(t *testing.T) {
	t.Parallel()
	build := func(doc *decad.Document) (*decad.Body, *decad.Body) {
		s, profile := polygonSweepSketch(t, 16, 1)
		path := mustPath(t, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 2), r3.NewVec(8, 0, 10))
		a, err := doc.Sweep(t.Context(), s, profile, path, decad.WithMitredJoins(), decad.WithSectionScale(scalars(0.9, 0.6)...))
		require.NoError(t, err)
		b, err := doc.Sweep(t.Context(), s, profile, path, decad.WithMitredJoins(), decad.WithSectionScale(scalars(0.9, 0.6)...))
		require.NoError(t, err)
		frame, err := r3.NewFrame(r3.NewVec(4, -4, 0), r3.NewVec(0, 1, 0), r3.NewVec(-1, 0, 0))
		require.NoError(t, err)
		turn, err := r3.FromFrame(frame)
		require.NoError(t, err)
		b, err = b.Placed(t.Context(), turn)
		require.NoError(t, err)
		return a, b
	}

	doc := decad.New()
	a, b := build(doc)
	va, err := a.Volume()
	require.NoError(t, err)
	vb, err := b.Volume()
	require.NoError(t, err)
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Len(t, report.Interferences, 1)
	overlap := report.Interferences[0].Volume

	u, err := decad.Union(t.Context(), a, b)
	require.NoError(t, err)
	require.Len(t, u.Lumps(), 1)
	vu, err := u.Volume()
	require.NoError(t, err)
	require.Positive(t, overlap.Value.Base())
	require.Less(t, vu.Value.Base(), va.Value.Base()+vb.Value.Base())

	idoc := decad.New()
	ia, ib := build(idoc)
	inter, err := decad.Intersect(t.Context(), ia, ib)
	require.NoError(t, err)
	vi, err := inter.Volume()
	require.NoError(t, err)
	slack := vu.Bound.Base() + va.Bound.Base() + vb.Bound.Base() + vi.Bound.Base()
	require.InDelta(t, va.Value.Base()+vb.Value.Base()-vi.Value.Base(), vu.Value.Base(), slack+1e-12)
	require.InDelta(t, vi.Value.Base(), overlap.Value.Base(), vi.Bound.Base()+overlap.Bound.Base()+1e-12)

	mesh, err := u.Tessellate(t.Context(), units.Millimeters(0.01), decad.WithVerification(decad.VerifyBoundary))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	requireWatertight(t, mesh)
}

// hollowTrunk is an mtilt trunk with a sealed bore: a 16-gon of circumradius
// 3 tapering to 2.5 along a path leaning 0°, 14° and 28° from the build
// direction, and a 16-gon bore of circumradius 2 tapering to 1.6 through the
// same corners. The bore starts 1 mm above the trunk's start cap and stops a
// quarter of the last span short of its end cap, so it reaches neither end.
type hollowTrunk struct {
	points []r3.Vec
}

func newHollowTrunk() hollowTrunk {
	return hollowTrunk{points: []r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 8), r3.NewVec(2, 0, 16), r3.NewVec(5, 1, 22)}}
}

func (h hollowTrunk) trunk(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	s, profile := polygonSweepSketch(t, 16, 3)
	body, err := doc.Sweep(t.Context(), s, profile, mustPath(t, h.points...),
		decad.WithMitredJoins(), decad.WithSectionScale(scalars(17.0/18, 8.0/9, 5.0/6)...))
	require.NoError(t, err)
	return body
}

// bore sweeps the bore from the origin, since a path starts on its profile's
// plane, and lifts it 1 mm into place.
func (h hollowTrunk) bore(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	up := r3.NewVec(0, 0, 1)
	last := h.points[3].Sub(h.points[2])
	end := h.points[3].Sub(last.Scale(0.25))
	s, profile := polygonSweepSketch(t, 16, 2)
	body, err := doc.Sweep(t.Context(), s, profile,
		mustPath(t, r3.NewVec(0, 0, 0), h.points[1].Sub(up), h.points[2].Sub(up), end.Sub(up)),
		decad.WithMitredJoins(), decad.WithSectionScale(scalars(14.0/15, 13.0/15, 0.8)...))
	require.NoError(t, err)
	lift, err := r3.Translation(up)
	require.NoError(t, err)
	placed, err := body.Placed(t.Context(), lift)
	require.NoError(t, err)
	return placed
}

// branch is a 16-gon branch of circumradius 0.8 whose start cap sits in the
// trunk's wall, 2.4 mm out from the trunk's axis at height 4, between the
// bore (circumradius about 1.94 there) and the trunk's outside (about 2.92).
// It leaves the trunk heading outward and 17° up, then turns upward.
func (h hollowTrunk) branch(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	s, profile := polygonSweepSketch(t, 16, 0.8)
	body, err := doc.Sweep(t.Context(), s, profile, mustPath(t, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 2), r3.NewVec(0, 2, 5)),
		decad.WithMitredJoins(), decad.WithSectionScale(scalars(0.9, 0.7)...))
	require.NoError(t, err)
	// Local z runs along (1, 0, 0.3) and local y along (−0.3, 0, 1).
	frame, err := r3.NewFrame(r3.NewVec(2.4, 0, 4), r3.NewVec(0, 1, 0), r3.NewVec(-0.3, 0, 1))
	require.NoError(t, err)
	turn, err := r3.FromFrame(frame)
	require.NoError(t, err)
	placed, err := body.Placed(t.Context(), turn)
	require.NoError(t, err)
	return placed
}

// faceOrigins collects the feature roles that created a body's faces.
func faceOrigins(body *decad.Body) map[decad.FeatureRef]struct{} {
	out := map[decad.FeatureRef]struct{}{}
	for _, f := range body.Faces() {
		for _, o := range f.Origins() {
			out[o] = struct{}{}
		}
	}
	return out
}

// requireSealedVoid asserts body is one lump of two closed shells, one of
// them a void, and that every void face comes from the bore. It returns the
// outer shell.
func requireSealedVoid(t *testing.T, body *decad.Body, bore map[decad.FeatureRef]struct{}) *decad.Shell {
	t.Helper()
	require.Len(t, body.Lumps(), 1)
	var outer, void []*decad.Shell
	for _, s := range body.Lumps()[0].Shells() {
		require.False(t, s.IsOpen())
		if s.IsVoid() {
			void = append(void, s)
			continue
		}
		outer = append(outer, s)
	}
	require.Len(t, outer, 1)
	require.Len(t, void, 1)
	require.Len(t, void[0].Faces(), 2+3*16, `the void is the bore's two caps and 48 walls`)
	for _, f := range void[0].Faces() {
		for _, o := range f.Origins() {
			require.Contains(t, bore, o, `void face %s comes from the bore`, o.Role)
		}
	}
	return outer[0]
}

// componentVolumes splits a triangle set into its vertex-connected
// components and returns each one's signed enclosed volume, largest first.
// An outward-wound outer shell reads positive and an inward-wound void reads
// negative.
func componentVolumes(verts []r3.Vec, tris [][3]int) []float64 {
	parent := make([]int, len(verts))
	for i := range parent {
		parent[i] = i
	}
	var find func(int) int
	find = func(i int) int {
		if parent[i] != i {
			parent[i] = find(parent[i])
		}
		return parent[i]
	}
	for _, tri := range tris {
		parent[find(tri[1])] = find(tri[0])
		parent[find(tri[2])] = find(tri[0])
	}
	sums := map[int]float64{}
	for _, tri := range tris {
		a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
		sums[find(tri[0])] += a.Dot(b.Cross(c)) / 6
	}
	out := make([]float64, 0, len(sums))
	for _, v := range sums {
		out = append(out, v)
	}
	slices.Sort(out)
	slices.Reverse(out)
	return out
}

// parseSTLSoup reads an ASCII STL's facet corners, welded by their printed
// coordinates, which export writes at full precision.
func parseSTLSoup(t *testing.T, stl string) ([]r3.Vec, [][3]int) {
	t.Helper()
	tris := parseSTLMesh(t, stl)
	index := map[string]int{}
	var verts []r3.Vec
	for line := range strings.SplitSeq(stl, "\n") {
		fields := strings.Fields(line)
		if len(fields) != 4 || fields[0] != "vertex" {
			continue
		}
		key := strings.Join(fields[1:], " ")
		if _, ok := index[key]; ok {
			continue
		}
		index[key] = len(verts)
		var xyz [3]float64
		for k := range 3 {
			v, err := strconv.ParseFloat(fields[1+k], 64)
			require.NoError(t, err)
			xyz[k] = v
		}
		verts = append(verts, r3.NewVec(xyz[0], xyz[1], xyz[2]))
	}
	return verts, tris
}

// parseThreeMFSoup reads the vertices and triangles of a 3MF package's one
// object.
func parseThreeMFSoup(t *testing.T, data []byte) ([]r3.Vec, [][3]int) {
	t.Helper()
	archive, err := zip.NewReader(bytes.NewReader(data), int64(len(data)))
	require.NoError(t, err)
	var model struct {
		Vertices []struct {
			X float64 `xml:"x,attr"`
			Y float64 `xml:"y,attr"`
			Z float64 `xml:"z,attr"`
		} `xml:"resources>object>mesh>vertices>vertex"`
		Triangles []struct {
			V1 int `xml:"v1,attr"`
			V2 int `xml:"v2,attr"`
			V3 int `xml:"v3,attr"`
		} `xml:"resources>object>mesh>triangles>triangle"`
	}
	for _, file := range archive.File {
		if file.Name != "3D/3dmodel.model" {
			continue
		}
		r, err := file.Open()
		require.NoError(t, err)
		raw, err := io.ReadAll(r)
		require.NoError(t, err)
		require.NoError(t, r.Close())
		require.NoError(t, xml.Unmarshal(raw, &model))
	}
	verts := make([]r3.Vec, 0, len(model.Vertices))
	for _, v := range model.Vertices {
		verts = append(verts, r3.NewVec(v.X, v.Y, v.Z))
	}
	tris := make([][3]int, 0, len(model.Triangles))
	for _, tri := range model.Triangles {
		tris = append(tris, [3]int{tri.V1, tri.V2, tri.V3})
	}
	require.NotEmpty(t, tris)
	return verts, tris
}

// requireEncloses asserts got lies within the sum of its own bound and every
// term's bound of the exact sum of plus minus minus, read over big.Rat.
func requireEncloses(t *testing.T, got decad.Measurement, plus, minus []decad.Measurement) {
	t.Helper()
	ref := new(big.Rat)
	allow := new(big.Rat).SetFloat64(got.Bound.Base())
	for _, m := range plus {
		ref.Add(ref, new(big.Rat).SetFloat64(m.Value.Base()))
		allow.Add(allow, new(big.Rat).SetFloat64(m.Bound.Base()))
	}
	for _, m := range minus {
		ref.Sub(ref, new(big.Rat).SetFloat64(m.Value.Base()))
		allow.Add(allow, new(big.Rat).SetFloat64(m.Bound.Base()))
	}
	gap := new(big.Rat).Sub(new(big.Rat).SetFloat64(got.Value.Base()), ref)
	gap.Abs(gap)
	gapF, _ := gap.Float64()
	allowF, _ := allow.Float64()
	require.LessOrEqual(t, gap.Cmp(allow), 0, `gap %v exceeds the bound %v`, gapF, allowF)
}

// requireShellVolumes asserts a triangle set holds exactly two components,
// enclosing outer and −void. The slack is not a proof bound: the vertices sit
// within the mesh's 1e-14-scale bound of the exact body, and summing about
// 250 float tetrahedra of up to 10 mm³ rounds by well under 1e-12 mm³.
func requireShellVolumes(t *testing.T, verts []r3.Vec, tris [][3]int, outer, void float64) {
	t.Helper()
	got := componentVolumes(verts, tris)
	require.Len(t, got, 2, `the outer shell and the void shell`)
	require.InDelta(t, outer, got[0], 1e-9)
	require.InDelta(t, -void, got[1], 1e-9)
}

// TestSweepMitredTrunkSealedBore cuts a bore that reaches neither end out of
// a leaning, tapered trunk (Table DM row DM3). The result is one lump with
// a sealed void shell, and its volume encloses the exact trunk minus bore.
//
// Bound legs: the allowance is the result's own bound plus the two operand
// volumes' roundings, and the exact gap is about 3e-14 mm³. Deleting every
// leg goes red. Deleting the result's own leg alone, or the two operand legs
// alone, stays green, because either part covers a gap that small. Dropping
// the bore from the reference goes red by its whole 205 mm³, and expecting
// the void shell to enclose +bore instead of −bore goes red by 410 mm³.
func TestSweepMitredTrunkSealedBore(t *testing.T) {
	t.Parallel()
	h := newHollowTrunk()
	doc := decad.New()
	trunk := h.trunk(t, doc)
	bore := h.bore(t, doc)
	vt, err := trunk.Volume()
	require.NoError(t, err)
	vb, err := bore.Volume()
	require.NoError(t, err)
	boreOrigins := faceOrigins(bore)

	hollow, err := decad.Cut(t.Context(), trunk, bore)
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{hollow}, doc.Bodies())
	outer := requireSealedVoid(t, hollow, boreOrigins)
	require.Len(t, outer.Faces(), 2+3*16, `the trunk's outside is untouched`)

	vol, err := hollow.Volume()
	require.NoError(t, err)
	requireEncloses(t, vol, []decad.Measurement{vt}, []decad.Measurement{vb})

	mesh, err := hollow.Tessellate(t.Context(), units.Millimeters(0.01), decad.WithVerification(decad.VerifyBoundary))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.Less(t, mesh.Bound().Base(), 1e-12)
	requireWatertight(t, mesh)
	requireShellVolumes(t, mesh.Vertices(), mesh.Triangles(), vt.Value.Base(), vb.Value.Base())

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	br, err := report.ForBody(hollow)
	require.NoError(t, err)
	require.Equal(t, decad.ValidityValid, br.Validity.Outcome)
	require.Equal(t, decad.HeldTopology{Lumps: 1, Voids: 1}, br.Topology)
	require.NotNil(t, br.Region)
	require.InDelta(t, vol.Value.Base(), br.Region.Volume.Value.Base(), vol.Bound.Base()+br.Region.Volume.Bound.Base())

	var stl bytes.Buffer
	require.NoError(t, export.STL(t.Context(), &stl, hollow, units.Millimeters(0.01)))
	verts, tris := parseSTLSoup(t, stl.String())
	require.Len(t, tris, len(mesh.Triangles()))
	requireShellVolumes(t, verts, tris, vt.Value.Base(), vb.Value.Base())
	var tmf bytes.Buffer
	require.NoError(t, export.ThreeMF(t.Context(), &tmf, hollow, units.Millimeters(0.01)))
	verts, tris = parseThreeMFSoup(t, tmf.Bytes())
	require.Len(t, tris, len(mesh.Triangles()))
	requireShellVolumes(t, verts, tris, vt.Value.Base(), vb.Value.Base())
}

// TestSweepMitredHollowTrunkBranch unions a branch whose root is buried in a
// hollow trunk's wall. The bore's void survives whole, the branch's start
// cap is swallowed, and the volume encloses hollow + branch − the overlap,
// the overlap read independently as Intersect(trunk, branch) on fresh
// operands: the root never reaches the bore, so it overlaps the trunk's wall
// alone.
//
// Bound legs: the exact gap is about 6e-15 mm³. Deleting every leg goes red,
// and no single leg is needed alone. Omitting the overlap from the reference
// goes red by its whole 0.89 mm³. Moving the root out to 2.75 mm, where its
// start cap pokes out of the trunk, fails the buried-cap check; moving it in
// to 1.2 mm, where it breaks into the bore, fails the sealed-void check with
// 69 void faces in place of 50.
func TestSweepMitredHollowTrunkBranch(t *testing.T) {
	t.Parallel()
	h := newHollowTrunk()
	doc := decad.New()
	bore := h.bore(t, doc)
	boreOrigins := faceOrigins(bore)
	vb, err := bore.Volume()
	require.NoError(t, err)
	hollow, err := decad.Cut(t.Context(), h.trunk(t, doc), bore)
	require.NoError(t, err)
	vh, err := hollow.Volume()
	require.NoError(t, err)
	branch := h.branch(t, doc)
	vbr, err := branch.Volume()
	require.NoError(t, err)
	var branchStart decad.FeatureRef
	for _, f := range branch.Faces() {
		if f.Origins()[0].Role == roleCapStart {
			branchStart = f.Origins()[0]
		}
	}
	require.NotZero(t, branchStart)

	tree, err := decad.Union(t.Context(), hollow, branch)
	require.NoError(t, err)
	outer := requireSealedVoid(t, tree, boreOrigins)
	require.NotContains(t, faceOrigins(tree), branchStart, `the branch's start cap is buried in the wall`)
	require.Greater(t, len(outer.Faces()), 2+3*16)

	idoc := decad.New()
	inter, err := decad.Intersect(t.Context(), h.trunk(t, idoc), h.branch(t, idoc))
	require.NoError(t, err)
	vi, err := inter.Volume()
	require.NoError(t, err)
	require.Positive(t, vi.Value.Base())
	require.Less(t, vi.Value.Base(), vbr.Value.Base())

	vol, err := tree.Volume()
	require.NoError(t, err)
	requireEncloses(t, vol, []decad.Measurement{vh, vbr}, []decad.Measurement{vi})

	mesh, err := tree.Tessellate(t.Context(), units.Millimeters(0.01), decad.WithVerification(decad.VerifyBoundary))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	requireWatertight(t, mesh)
	requireShellVolumes(t, mesh.Vertices(), mesh.Triangles(), vol.Value.Base()+vb.Value.Base(), vb.Value.Base())

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	r, err := report.ForBody(tree)
	require.NoError(t, err)
	require.Equal(t, decad.ValidityValid, r.Validity.Outcome)
	require.Equal(t, decad.HeldTopology{Lumps: 1, Voids: 1}, r.Topology)
}
