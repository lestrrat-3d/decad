package apitest_test

import (
	"errors"
	"fmt"
	"math/rand/v2"
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad"
)

// coplanarBox is one box of a coplanar fixture: the footprint [x0, x1] x [y0, y1]
// from z0 up by h.
type coplanarBox struct{ x0, y0, x1, y1, z0, h float64 }

func (b coplanarBox) build(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	return meshBoxAt(t, doc, b.x0, b.y0, b.x1, b.y1, b.z0, b.h)
}

// meshBoxAt is a box sketched on the base plane and moved into place by a
// Transform, as TestBooleanGapTolerance builds its boxes: a pair of them is not
// taken by the analytic prism route, so it reaches the mesh evaluator.
// boxBodyAtZ sketches on an offset plane, which the analytic stacked union
// accepts.
func meshBoxAt(t *testing.T, doc *decad.Document, x0, y0, x1, y1, z0, h float64) *decad.Body {
	t.Helper()
	return translated(t, boxBody(t, doc, 0, 0, x1-x0, y1-y0, h), x0, y0, z0)
}

// meshPrismAt is meshBoxAt for a polygon section moved dx along x and raised
// 4 mm: the floor of the slots and tenons below.
func meshPrismAt(t *testing.T, doc *decad.Document, dx, h float64, pts [][2]float64) *decad.Body {
	t.Helper()
	return translated(t, polyPrismBody(t, doc, pts, h), dx, 0, 4)
}

// coplanarModes are the two documents every case runs in.
var coplanarModes = map[string][]decad.DocumentOption{"default": nil, "ExactOnly": {decad.ExactOnly()}}

// coplanarA is the 10 x 10 x 10 box at the origin.
var coplanarA = coplanarBox{0, 0, 10, 10, 0, 10}

// coplanarPair builds a fresh document holding A and b, because every boolean
// retires its operands.
func coplanarPair(t *testing.T, b coplanarBox, opts []decad.DocumentOption) (*decad.Body, *decad.Body, *decad.Document) {
	t.Helper()
	doc := decad.New(opts...)
	return coplanarA.build(t, doc), b.build(t, doc), doc
}

func requireExactVolume(t *testing.T, body *decad.Body, want float64) {
	t.Helper()
	v, err := body.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, v.Exactness)
	require.Zero(t, boundMM3(t, v))
	require.InDelta(t, want, volumeMM(t, v), 1e-9)
}

// requireCoplanarVolume asserts the volume is want within its stated bound, and
// that bound is rounding only. Analytic results read Approximate with a bound
// of about 1e-13, so the exact reading is not asked of them.
func requireCoplanarVolume(t *testing.T, body *decad.Body, want float64) {
	t.Helper()
	v, err := body.Volume()
	require.NoError(t, err)
	require.Less(t, boundMM3(t, v), 1e-9)
	require.InDelta(t, want, volumeMM(t, v), boundMM3(t, v)+1e-9)
}

func requireNoShift(t *testing.T, body *decad.Body) {
	t.Helper()
	for _, d := range body.OperationDiagnostics() {
		require.NotEqual(t, decad.OpDiagGapShiftApplied, d.Code, "the exact classification applies no gap shift")
	}
}

func requireEmptyIntersect(t *testing.T, a, b *decad.Body, noShift bool) {
	t.Helper()
	_, err := decad.Intersect(t.Context(), a, b)
	var be *decad.BooleanError
	require.ErrorAs(t, err, &be)
	require.Equal(t, decad.BooleanEmpty, be.Code)
	var diag decad.OperationDiagnosticError
	if noShift && asDiag(err, &diag) {
		for _, d := range diag.OperationDiagnostics() {
			require.NotEqual(t, decad.OpDiagGapShiftApplied, d.Code)
		}
	}
}

func asDiag(err error, target *decad.OperationDiagnosticError) bool { return errors.As(err, target) }

// coplanarCase states what each operation of one A pair gives.
type coplanarCase struct {
	name  string
	b     coplanarBox
	union float64 // exact volume of the union, 0 when the union is the retried one
	lumps int
	// inter is the exact intersection, 0 when it is empty.
	inter float64
	cut   float64
	// status is what Verify reads; rows is how many interference rows.
	status decad.Status
	rows   int
	// cutAnalytic marks a Cut the analytic stacked route builds, whose volume
	// carries a rounding bound; every other Cut is a mesh result and exact.
	cutAnalytic bool
	// holedTop asks the Cut for a top face with an inner loop.
	holedTop bool
}

// TestCoplanarContactFaceToFace covers A1, A2 and A5 to A7 of
// agent-docs/coplanar-contact-design.md §11.2: boxes meeting face to face, with
// a face resting on another, a flush top, and a separation of 2^-30 either way.
func TestCoplanarContactFaceToFace(t *testing.T) {
	t.Parallel()
	const eps = 1.0 / (1 << 30)
	cases := []coplanarCase{
		{name: "A1 equal footprints shifted in x", b: coplanarBox{0.5, 0, 10.5, 10, 10, 10},
			union: 2000, lumps: 1, cut: 1000, status: decad.Sound},
		{name: "A2 overhang", b: coplanarBox{5.5, 0, 15.5, 4, 10, 10},
			union: 1400, lumps: 1, cut: 1000, status: decad.Sound},
		{name: "A5 same facing", b: coplanarBox{2, 2, 8, 8, 5, 5},
			union: 1000, lumps: 1, inter: 180, cut: 820, status: decad.Interfering, rows: 1,
			cutAnalytic: true, holedTop: true},
		{name: "A7 gap beyond zero", b: coplanarBox{0.5, 0, 10.5, 10, 10 + eps, 10},
			union: 2000, lumps: 2, cut: 1000, status: decad.Sound},
	}
	for _, tc := range cases {
		for mode, opts := range coplanarModes {
			t.Run(tc.name+"/"+mode, func(t *testing.T) {
				t.Parallel()
				a, b, _ := coplanarPair(t, tc.b, opts)
				u, err := decad.Union(t.Context(), a, b)
				require.NoError(t, err)
				require.True(t, tookMeshPath(u), "the pair reaches the mesh evaluator")
				require.Len(t, u.Lumps(), tc.lumps)
				requireNoShift(t, u)
				v, err := u.Volume()
				require.NoError(t, err)
				if tc.lumps == 1 {
					requireExactVolume(t, u, tc.union)
				} else {
					require.InDelta(t, tc.union, volumeMM(t, v), boundMM3(t, v)+1e-9)
				}

				a, b, _ = coplanarPair(t, tc.b, opts)
				if tc.inter == 0 {
					requireEmptyIntersect(t, a, b, true)
				} else {
					common, err := decad.Intersect(t.Context(), a, b)
					require.NoError(t, err)
					requireExactVolume(t, common, tc.inter)
					requireNoShift(t, common)
				}

				a, b, _ = coplanarPair(t, tc.b, opts)
				cut, err := decad.Cut(t.Context(), a, b)
				require.NoError(t, err)
				if tc.cutAnalytic {
					// A blind pocket with a flush top is built analytically, so its
					// volume carries a rounding bound.
					requireCoplanarVolume(t, cut, tc.cut)
				} else {
					requireExactVolume(t, cut, tc.cut)
				}
				requireNoShift(t, cut)
				if tc.holedTop {
					holed := false
					for _, f := range cut.Faces() {
						holed = holed || len(f.Loops()) == 2
					}
					require.True(t, holed, "the pocket's flush top is a face with a hole")
				}

				_, _, doc := coplanarPair(t, tc.b, opts)
				report, err := doc.Verify(t.Context())
				require.NoError(t, err)
				require.Equal(t, tc.status, report.Status)
				require.Len(t, report.Interferences, tc.rows)
				if tc.rows > 0 {
					row := report.Interferences[0]
					require.Greater(t, volumeMM(t, row.Volume)-boundMM3(t, row.Volume), 0.0)
				}
			})
		}
	}
}

// TestCoplanarContactOverlapBeyondZero is A6: an overlap of depth 2^-30 between
// exact faces is a crossing, classified exactly.
func TestCoplanarContactOverlapBeyondZero(t *testing.T) {
	t.Parallel()
	const eps = 1.0 / (1 << 30)
	b := coplanarBox{0.5, 0, 10.5, 10, 10 - eps, 10}
	for mode, opts := range coplanarModes {
		t.Run(mode, func(t *testing.T) {
			t.Parallel()
			a, bb, _ := coplanarPair(t, b, opts)
			u, err := decad.Union(t.Context(), a, bb)
			require.NoError(t, err)
			requireExactVolume(t, u, 2000-95*eps)
			a, bb, _ = coplanarPair(t, b, opts)
			cut, err := decad.Cut(t.Context(), a, bb)
			require.NoError(t, err)
			requireExactVolume(t, cut, 1000-95*eps)
			a, bb, _ = coplanarPair(t, b, opts)
			common, err := decad.Intersect(t.Context(), a, bb)
			require.NoError(t, err)
			v, err := common.Volume()
			require.NoError(t, err)
			require.Greater(t, volumeMM(t, v)-boundMM3(t, v), 0.0)
			require.InDelta(t, 9.5*10*eps, volumeMM(t, v), boundMM3(t, v)+1e-12)

			_, _, doc := coplanarPair(t, b, opts)
			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			require.Equal(t, decad.Interfering, report.Status)
		})
	}
}

// TestCoplanarContactEdgeAndVertexOnly is A3 and A4: boxes meeting along one
// edge or at one corner. The exact classification leaves the result meeting
// itself, so the default document decides it by the gap shift and ExactOnly
// refuses; Intersect is empty, Cut is the target, and Verify reads Sound.
func TestCoplanarContactEdgeAndVertexOnly(t *testing.T) {
	t.Parallel()
	for name, b := range map[string]coplanarBox{
		"A3 edge only":   {10, 0.5, 20, 10.5, 10, 10},
		"A4 vertex only": {10, 10, 20, 20, 10, 10},
	} {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			a, bb, _ := coplanarPair(t, b, nil)
			u, err := decad.Union(t.Context(), a, bb)
			require.NoError(t, err)
			require.Len(t, u.Lumps(), map[string]int{"A3 edge only": 1, "A4 vertex only": 1}[name])
			shifted := false
			for _, d := range u.OperationDiagnostics() {
				shifted = shifted || d.Code == decad.OpDiagGapShiftApplied
			}
			require.True(t, shifted, "the contact the exact pass cannot publish takes the retry")
			v, err := u.Volume()
			require.NoError(t, err)
			require.InDelta(t, 2000, volumeMM(t, v), boundMM3(t, v)+1e-9)

			a, bb, _ = coplanarPair(t, b, []decad.DocumentOption{decad.ExactOnly()})
			_, err = decad.Union(t.Context(), a, bb)
			var be *decad.BooleanError
			require.ErrorAs(t, err, &be)
			require.Equal(t, decad.BooleanUnsupportedContact, be.Code)

			for mode, opts := range coplanarModes {
				a, bb, _ = coplanarPair(t, b, opts)
				requireEmptyIntersect(t, a, bb, false)
				a, bb, _ = coplanarPair(t, b, opts)
				cut, err := decad.Cut(t.Context(), a, bb)
				require.NoError(t, err, mode)
				requireExactVolume(t, cut, 1000)
				_, _, doc := coplanarPair(t, b, opts)
				report, err := doc.Verify(t.Context())
				require.NoError(t, err)
				require.Equal(t, decad.Sound, report.Status, mode)
			}
		})
	}
}

// slotHost is a 20 x 16 x 10 plate.
func slotHost(t *testing.T, doc *decad.Document) *decad.Body {
	return meshBoxAt(t, doc, 0, 0, 20, 16, 0, 10)
}

// dovetailSection is a trapezoid whose flanks slant by 1/8, so every vertex and
// every crossing with the plate's y = 0 and y = 16 faces is a float.
var (
	dovetailTool   = [][2]float64{{6, -4}, {14, -4}, {17, 20}, {3, 20}}
	dovetailTenon  = [][2]float64{{5.5, 0}, {14.5, 0}, {16.5, 16}, {3.5, 16}}
	dovetailTenonL = [][2]float64{{2, 0}, {6.5, 0}, {6.5, 16}, {0, 16}}
	dovetailTenonR = [][2]float64{{0, 0}, {4.5, 0}, {6.5, 16}, {0, 16}}
)

// TestCoplanarContactMadeByCut covers R1 to R4 of §11.2: a made body meets an
// exact body face to face, as a tenon meets its slot. The pair reads Sound,
// Intersect is empty, Cut leaves the target, and Union is the sum.
func TestCoplanarContactMadeByCut(t *testing.T) {
	t.Parallel()
	type fixture struct {
		name  string
		build func(t *testing.T, doc *decad.Document) (target, other *decad.Body)
		// union is the volume of the union.
		union float64
	}
	fixtures := []fixture{
		{name: "R1 tenon in a slot made by two Cuts", build: func(t *testing.T, doc *decad.Document) (*decad.Body, *decad.Body) {
			host := slotHost(t, doc)
			one, err := decad.Cut(t.Context(), host, meshBoxAt(t, doc, 6, -1, 10, 17, 4, 7))
			require.NoError(t, err)
			slot, err := decad.Cut(t.Context(), one, meshBoxAt(t, doc, 10, -1, 14, 17, 4, 7))
			require.NoError(t, err)
			return slot, meshBoxAt(t, doc, 6, 0, 14, 16, 4, 6)
		}, union: 20 * 16 * 10},
		{name: "R2 box under a box made by Cut, flush walls", build: func(t *testing.T, doc *decad.Document) (*decad.Body, *decad.Body) {
			upper, err := decad.Cut(t.Context(), meshBoxAt(t, doc, 0, 0, 10, 10, 10, 10), meshBoxAt(t, doc, 8, 8, 11, 11, 19, 2))
			require.NoError(t, err)
			return upper, meshBoxAt(t, doc, 0, 0, 10, 10, 0, 10)
		}, union: 1000 + 1000 - 4},
		{name: "R3 trapezoid prism in a socket made by Cut", build: func(t *testing.T, doc *decad.Document) (*decad.Body, *decad.Body) {
			socket, err := decad.Cut(t.Context(), slotHost(t, doc), meshPrismAt(t, doc, 0, 7, dovetailTool))
			require.NoError(t, err)
			return socket, meshPrismAt(t, doc, 0, 6, dovetailTenon)
		}, union: 20 * 16 * 10},
		{name: "R4 trapezoid made by a flush Union in a socket made by Cut", build: func(t *testing.T, doc *decad.Document) (*decad.Body, *decad.Body) {
			socket, err := decad.Cut(t.Context(), slotHost(t, doc), meshPrismAt(t, doc, 0, 7, dovetailTool))
			require.NoError(t, err)
			// Each half is drawn at the origin and moved into place, so the Union of
			// the two flush walls is a mesh result.
			tenon, err := decad.Union(t.Context(), meshPrismAt(t, doc, 3.5, 6, dovetailTenonL), meshPrismAt(t, doc, 10, 6, dovetailTenonR))
			require.NoError(t, err)
			require.True(t, tookMeshPath(tenon), "the tenon is a mesh Union result")
			return socket, tenon
		}, union: 20 * 16 * 10},
	}
	for _, f := range fixtures {
		for mode, opts := range coplanarModes {
			t.Run(f.name+"/"+mode, func(t *testing.T) {
				t.Parallel()
				build := func() (*decad.Document, *decad.Body, *decad.Body) {
					doc := decad.New(opts...)
					target, other := f.build(t, doc)
					return doc, target, other
				}
				doc, _, _ := build()
				report, err := doc.Verify(t.Context())
				require.NoError(t, err)
				require.Equal(t, decad.Sound, report.Status)
				require.Empty(t, report.Interferences)

				_, target, other := build()
				requireEmptyIntersect(t, target, other, true)

				_, target, other = build()
				tv, err := target.Volume()
				require.NoError(t, err)
				ov, err := other.Volume()
				require.NoError(t, err)
				u, err := decad.Union(t.Context(), target, other)
				require.NoError(t, err)
				requireNoShift(t, u)
				uv, err := u.Volume()
				require.NoError(t, err)
				bound := boundMM3(t, tv) + boundMM3(t, ov) + boundMM3(t, uv) + 1e-9
				require.InDelta(t, volumeMM(t, tv)+volumeMM(t, ov), volumeMM(t, uv), bound, "the union is the sum of the operands")
				require.InDelta(t, f.union, volumeMM(t, uv), bound)

				_, target, other = build()
				cut, err := decad.Cut(t.Context(), target, other)
				require.NoError(t, err)
				cv, err := cut.Volume()
				require.NoError(t, err)
				require.InDelta(t, volumeMM(t, tv), volumeMM(t, cv), boundMM3(t, tv)+boundMM3(t, cv)+1e-9, "Cut leaves the target")
			})
		}
	}
}

// TestCoplanarContactKeepsRecords is P1: a flush Union keeps every retained
// plane of both operands, publishes no shift warning, and a later Cut over it
// is exact too.
func TestCoplanarContactKeepsRecords(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a := coplanarA.build(t, doc)
	b := coplanarBox{0.5, 0, 10.5, 10, 10, 10}.build(t, doc)
	u, err := decad.Union(t.Context(), a, b)
	require.NoError(t, err)
	require.Empty(t, u.OperationDiagnostics())
	require.Len(t, u.Faces(), 12, "each operand keeps its six faces: neither top or bottom is covered whole")
	for i, f := range u.Faces() {
		_, ok := f.Surface().(decad.Plane)
		require.True(t, ok, "face %d keeps its Plane record", i)
	}
	tool := meshBoxAt(t, doc, 3, 3, 7, 7, 8, 10)
	cut, err := decad.Cut(t.Context(), u, tool)
	require.NoError(t, err)
	requireExactVolume(t, cut, 2000-4*4*10)
	require.Empty(t, cut.OperationDiagnostics())
}

// The classes of a box pair by the extents of its intersection box.
const (
	classOverlap  = "overlap"
	classFace     = "face"
	classEdge     = "edge"
	classVertex   = "vertex"
	classDisjoint = "disjoint"
)

// TestCoplanarContactRandomBoxPairs is P2: 200 seeded box pairs on an integer
// grid, half shifted 0.5 mm in x, classified by their intersection box. Disjoint
// and face-touching pairs read Sound with exact sums; edge and vertex pairs
// join through the retry; overlaps give the inclusion-exclusion volumes and
// Interfering.
func TestCoplanarContactRandomBoxPairs(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(20261118, 7))
	size := func() float64 { return float64(1 + rng.IntN(4)) }
	offset := func() float64 { return float64(rng.IntN(9) - 4) }
	for i := range 200 {
		sa := [3]float64{size(), size(), size()}
		sb := [3]float64{size(), size(), size()}
		off := [3]float64{offset(), offset(), offset()}
		if i%2 == 1 {
			off[0] += 0.5
		}
		a := coplanarBox{0, 0, sa[0], sa[1], 0, sa[2]}
		b := coplanarBox{off[0], off[1], off[0] + sb[0], off[1] + sb[1], off[2], sb[2]}
		ext := [3]float64{
			min(sa[0], off[0]+sb[0]) - max(0, off[0]),
			min(sa[1], off[1]+sb[1]) - max(0, off[1]),
			min(sa[2], off[2]+sb[2]) - max(0, off[2]),
		}
		zero, neg := 0, false
		for _, e := range ext {
			neg = neg || e < 0
			if e == 0 {
				zero++
			}
		}
		class := map[int]string{0: classOverlap, 1: classFace, 2: classEdge, 3: classVertex}[zero]
		if neg {
			class = classDisjoint
		}
		va, vb := sa[0]*sa[1]*sa[2], sb[0]*sb[1]*sb[2]
		overlap := 0.0
		if class == classOverlap {
			overlap = ext[0] * ext[1] * ext[2]
		}
		for mode, opts := range coplanarModes {
			t.Run(fmt.Sprintf("%03d %s/%s", i, class, mode), func(t *testing.T) {
				t.Parallel()
				pair := func() (*decad.Body, *decad.Body, *decad.Document) {
					doc := decad.New(opts...)
					return a.build(t, doc), b.build(t, doc), doc
				}
				x, y, _ := pair()
				u, err := decad.Union(t.Context(), x, y)
				switch class {
				case classEdge, classVertex:
					// A3's outcome: the exact pass leaves the result meeting itself,
					// so the default document joins it by the gap shift and ExactOnly
					// refuses.
					if mode == "ExactOnly" {
						var be *decad.BooleanError
						require.ErrorAs(t, err, &be)
						require.Equal(t, decad.BooleanUnsupportedContact, be.Code)
					} else {
						require.NoError(t, err)
						require.Len(t, u.Lumps(), 1)
						shifted := false
						for _, d := range u.OperationDiagnostics() {
							shifted = shifted || d.Code == decad.OpDiagGapShiftApplied
						}
						require.True(t, shifted)
						uv, err := u.Volume()
						require.NoError(t, err)
						require.InDelta(t, va+vb, volumeMM(t, uv), boundMM3(t, uv)+1e-9)
					}
				case classDisjoint:
					require.NoError(t, err)
					require.Len(t, u.Lumps(), 2)
					requireCoplanarVolume(t, u, va+vb)
				case classFace:
					require.NoError(t, err)
					require.Len(t, u.Lumps(), 1)
					requireCoplanarVolume(t, u, va+vb)
				default:
					require.NoError(t, err)
					requireCoplanarVolume(t, u, va+vb-overlap)
				}

				x, y, _ = pair()
				if class == classOverlap {
					common, err := decad.Intersect(t.Context(), x, y)
					require.NoError(t, err)
					requireCoplanarVolume(t, common, overlap)
				} else {
					_, err := decad.Intersect(t.Context(), x, y)
					var be *decad.BooleanError
					require.ErrorAs(t, err, &be)
					require.Equal(t, decad.BooleanEmpty, be.Code)
				}

				x, y, _ = pair()
				cut, err := decad.Cut(t.Context(), x, y)
				if va == overlap {
					var be *decad.BooleanError
					require.ErrorAs(t, err, &be)
					require.Equal(t, decad.BooleanEmpty, be.Code, "a tool that covers its target leaves nothing")
				} else {
					require.NoError(t, err)
					requireCoplanarVolume(t, cut, va-overlap)
				}

				_, _, doc := pair()
				report, err := doc.Verify(t.Context())
				require.NoError(t, err)
				if class == classOverlap {
					require.Equal(t, decad.Interfering, report.Status)
				} else {
					require.Equal(t, decad.Sound, report.Status)
				}
			})
		}
	}
}
