package decad_test

import (
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/surface-design.md's Table W and docs/sweep-design.md's
// own delivery-table tests for WithSurfaceResult() on Sweep, the last of the
// four features to take the option. Sweep has three distinct build paths, and
// none of them are alike: a one-span straight path reduces to the prism
// builder and a one-span arc reduces to the revolve builder, so each gets the
// whole sheet behaviour for free from prism_build.go/revolve_build.go; a
// composite path has its own join topology (sweep_composite.go) and is the
// only path with real new evaluator work. Every fixture reuses plateSketch
// (extrude_test.go), the arc fixture sweep_arc_test.go's own
// TestSweepArcMatchesQuarterRevolveAndReplaysPlacement proves equivalent to a
// quarter Revolve, and orthogonalSweepPath/orthogonalSweepProfileWithHole
// (sweep_spatial_test.go) — the composite area-composition test swaps that
// fixture's own square profile for a regular pentagon (regularNGonSketch,
// extrude_bounds_test.go), since a square's cap area is exactly representable
// and proves nothing about §4.3's bound composition.

// sweepLinePath is the one-span straight path T1's own shape reduces to: a
// 10 mm rise along the profile plane's positive normal.
func sweepLinePath(t *testing.T) *decad.Path {
	t.Helper()
	path, err := decad.NewPath(
		r3.NewVec(0, 0, 0),
		decad.LineTo{End: r3.NewVec(0, 0, 10)},
	)
	require.NoError(t, err)
	return path
}

// sweepArcPath is sweep_arc_test.go's own quarter-turn arc, proven equivalent
// to a quarter Revolve about plateSketch's own u axis.
func sweepArcPath(t *testing.T) *decad.Path {
	t.Helper()
	path, err := decad.NewPath(
		r3.NewVec(0, 5, 0),
		decad.ArcThrough{
			Through: r3.NewVec(0, 3, 4),
			End:     r3.NewVec(0, 0, 5),
		},
	)
	require.NoError(t, err)
	return path
}

// TestSurfaceSweepLineIsASheet is Table W's line row over the one-span
// straight reduction: the flag rides the reduced prismPayload, so the sheet
// behaviour is prism_build.go's own (docs/surface-design.md §4.3).
func TestSurfaceSweepLineIsASheet(t *testing.T) {
	t.Parallel()
	s, p := plateSketch(t)
	path := sweepLinePath(t)

	solidDoc := decad.New()
	s0, p0 := plateSketch(t)
	solid, err := solidDoc.Sweep(t.Context(), s0, p0, sweepLinePath(t))
	require.NoError(t, err)

	doc := decad.New()
	sheet, err := doc.Sweep(t.Context(), s, p, path, decad.WithSurfaceResult())
	require.NoError(t, err)

	require.Equal(t, decad.BodySheet, sheet.Kind())
	require.False(t, sheet.IsSolid())
	_, err = sheet.Volume()
	require.ErrorIs(t, err, decad.ErrNotSolid)
	_, err = sheet.Centroid()
	require.ErrorIs(t, err, decad.ErrNotSolid)

	require.Len(t, sheet.Faces(), len(solid.Faces())-2)
	free, err := decad.Edges(decad.Free()).Exactly(8).SelectEdges(sheet)
	require.NoError(t, err)
	require.Len(t, free, 8)

	// perimeter (2*(100+60) = 320) times the path's 10 mm length.
	decadtest.MeasuresArea(t, sheet, units.SquareMillimeters(3200), decadtest.Exactly())

	solidBox, err := solid.Bounds()
	require.NoError(t, err)
	sheetBox, err := sheet.Bounds()
	require.NoError(t, err)
	require.Equal(t, solidBox, sheetBox)

	for _, face := range sheet.Faces() {
		origins := face.Origins()
		require.Len(t, origins, 1)
		require.True(t, strings.HasPrefix(origins[0].Role, "side(0,"))
	}
	_, err = decad.Faces(decad.FaceCreatedBy(decad.CapStart(sheet))).SelectFaces(sheet)
	require.ErrorIs(t, err, decad.ErrNoMatch)
	_, err = decad.Faces(decad.FaceCreatedBy(decad.CapEnd(sheet))).SelectFaces(sheet)
	require.ErrorIs(t, err, decad.ErrNoMatch)
}

// TestSurfaceSweepLineSheetVerifiesSound is docs/sweep-design.md's own Table D
// row D1 carve-out: the one-span straight sheet IS a prismPayload build, so it
// transfers verbatim the prism argument docs/surface-design.md §9.1 states,
// and reads Sound.
func TestSurfaceSweepLineSheetVerifiesSound(t *testing.T) {
	t.Parallel()
	s, p := plateSketch(t)
	doc := decad.New()
	sheet, err := doc.Sweep(t.Context(), s, p, sweepLinePath(t), decad.WithSurfaceResult())
	require.NoError(t, err)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	require.True(t, report.Passed())
	require.Empty(t, report.Diagnostics)

	br := decadtest.FindBodyReport(t, report, sheet)
	require.Equal(t, decad.ValidityValid, br.Validity.Outcome)
	require.Nil(t, br.Region)
}

// TestSurfaceSweepArcIsASheet is Table W's arc row over the one-span arc
// reduction: the flag rides the reduced revolvePayload, so the cap omission,
// Kind() and area subtraction are revolve_build.go's own.
func TestSurfaceSweepArcIsASheet(t *testing.T) {
	t.Parallel()
	s0, p0 := plateSketch(t)
	solidDoc := decad.New()
	solid, err := solidDoc.Sweep(t.Context(), s0, p0, sweepArcPath(t))
	require.NoError(t, err)

	capStartFaces, err := decad.Faces(decad.FaceCreatedBy(decad.CapStart(solid))).Exactly(1).SelectFaces(solid)
	require.NoError(t, err)
	capStartArea, err := capStartFaces[0].Area()
	require.NoError(t, err)
	capEndFaces, err := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(solid))).Exactly(1).SelectFaces(solid)
	require.NoError(t, err)
	capEndArea, err := capEndFaces[0].Area()
	require.NoError(t, err)

	s, p := plateSketch(t)
	doc := decad.New()
	sheet, err := doc.Sweep(t.Context(), s, p, sweepArcPath(t), decad.WithSurfaceResult())
	require.NoError(t, err)

	require.Equal(t, decad.BodySheet, sheet.Kind())
	require.False(t, sheet.IsSolid())
	require.Len(t, sheet.Faces(), len(solid.Faces())-2)

	solidArea, err := solid.Area()
	require.NoError(t, err)
	sheetArea, err := sheet.Area()
	require.NoError(t, err)
	wantValue := solidArea.Value.Base() - capStartArea.Value.Base() - capEndArea.Value.Base()
	require.Equal(t, wantValue, sheetArea.Value.Base())

	solidBox, err := solid.Bounds()
	require.NoError(t, err)
	sheetBox, err := sheet.Bounds()
	require.NoError(t, err)
	require.Equal(t, solidBox, sheetBox)

	_, err = decad.Faces(decad.FaceCreatedBy(decad.CapStart(sheet))).SelectFaces(sheet)
	require.ErrorIs(t, err, decad.ErrNoMatch)
	_, err = decad.Faces(decad.FaceCreatedBy(decad.CapEnd(sheet))).SelectFaces(sheet)
	require.ErrorIs(t, err, decad.ErrNoMatch)

	free, err := decad.Edges(decad.Free()).SelectEdges(sheet)
	require.NoError(t, err)
	require.NotEmpty(t, free)
	for _, e := range free {
		require.True(t, e.IsFree())
	}
}

// TestSurfaceSweepArcAndCompositeSheetsVerifyUndecided is the honest answer
// docs/sweep-design.md's Table D row D1 states: neither the arc reduction
// (a revolvePayload build, which carries no construction proof anywhere in
// this evaluator) nor the composite build (whose own boundary/vertex-link
// audit proves assembled topology, not geometric non-self-intersection) earns
// leg 4, unlike the one-span straight sheet.
func TestSurfaceSweepArcAndCompositeSheetsVerifyUndecided(t *testing.T) {
	t.Parallel()

	t.Run("arc", func(t *testing.T) {
		t.Parallel()
		s, p := plateSketch(t)
		doc := decad.New()
		sheet, err := doc.Sweep(t.Context(), s, p, sweepArcPath(t), decad.WithSurfaceResult())
		require.NoError(t, err)
		requireSweepSheetUndecided(t, doc, sheet)
	})
	t.Run("composite", func(t *testing.T) {
		t.Parallel()
		s, p := orthogonalSweepProfile(t)
		doc := decad.New()
		sheet, err := doc.Sweep(t.Context(), s, p, orthogonalSweepPath(t), decad.WithSurfaceResult())
		require.NoError(t, err)
		requireSweepSheetUndecided(t, doc, sheet)
	})
}

func requireSweepSheetUndecided(t *testing.T, doc *decad.Document, sheet *decad.Body) {
	t.Helper()
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	br := decadtest.FindBodyReport(t, report, sheet)
	require.Equal(t, decad.ValidityUndecided, br.Validity.Outcome)
	require.Nil(t, br.Region)
	require.Len(t, br.Validity.Diagnostics, 1)
	require.Equal(t, decad.DiagUndecidedValidity, br.Validity.Diagnostics[0].Code)
	for _, d := range br.Diagnostics {
		require.NotEqual(t, decad.DiagInvalidBody, d.Code)
	}
}

// TestSurfaceSweepCompositeIsASheet is the composite path's own new work: the
// two outer caps are omitted by the assembler (sweep_composite.go), every
// sewn internal join keeps its two adjacent faces, and the published area is
// the composed (sum-in, subtract-out) figure docs/surface-design.md §4.3
// requires rather than a plain walk over the surviving walls alone.
func TestSurfaceSweepCompositeIsASheet(t *testing.T) {
	t.Parallel()
	s0, p0 := regularNGonSketch(t, 5, 1)
	solidDoc := decad.New()
	solid, err := solidDoc.Sweep(t.Context(), s0, p0, orthogonalSweepPath(t))
	require.NoError(t, err)
	require.True(t, solid.IsSolid())

	capStartFaces, err := decad.Faces(decad.FaceCreatedBy(decad.CapStart(solid))).Exactly(1).SelectFaces(solid)
	require.NoError(t, err)
	capStartArea, err := capStartFaces[0].Area()
	require.NoError(t, err)
	capEndFaces, err := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(solid))).Exactly(1).SelectFaces(solid)
	require.NoError(t, err)
	capEndArea, err := capEndFaces[0].Area()
	require.NoError(t, err)

	// Both caps must actually be inexact, or this fixture proves nothing about
	// the composition — an exactly-representable cap (a square, T1's own
	// rectangle) would pass this test vacuously.
	require.Greater(t, capStartArea.Bound.Base(), 0.0)
	require.Greater(t, capEndArea.Bound.Base(), 0.0)

	s1, p1 := regularNGonSketch(t, 5, 1)
	doc := decad.New()
	sheet, err := doc.Sweep(t.Context(), s1, p1, orthogonalSweepPath(t), decad.WithSurfaceResult())
	require.NoError(t, err)

	require.Equal(t, decad.BodySheet, sheet.Kind())
	require.False(t, sheet.IsSolid())
	require.Len(t, sheet.Faces(), len(solid.Faces())-2)

	// The two outer rims (5 edges each, the pentagon's own segment count) are
	// free; every join and longitudinal edge keeps its two faces, or this
	// would resolve to a different count.
	free, err := decad.Edges(decad.Free()).Exactly(10).SelectEdges(sheet)
	require.NoError(t, err)
	require.Len(t, free, 10)
	for _, e := range sheet.Edges() {
		if e.IsFree() {
			continue
		}
		require.Len(t, e.Faces(), 2)
	}

	solidArea, err := solid.Area()
	require.NoError(t, err)
	sheetArea, err := sheet.Area()
	require.NoError(t, err)

	wantValue := solidArea.Value.Base() - capStartArea.Value.Base() - capEndArea.Value.Base()
	require.Equal(t, wantValue, sheetArea.Value.Base())

	wantBound := solidArea.Bound.Base() + capStartArea.Bound.Base() + capEndArea.Bound.Base()
	require.GreaterOrEqual(t, sheetArea.Bound.Base(), wantBound)
	require.InDelta(t, wantBound, sheetArea.Bound.Base(), 1e-9)
}

// TestSurfaceSweepCompositeHoledProfileReportsDisconnectedLumps is decision A
// (docs/surface-design.md §2.2) for the composite path: a holed profile's
// outer wall tube and hole wall tube touch nowhere once the two outer caps
// that used to bridge them are gone, so Body.Lumps() must report both. The
// solid built from the same profile and path keeps its one lump.
func TestSurfaceSweepCompositeHoledProfileReportsDisconnectedLumps(t *testing.T) {
	t.Parallel()
	s0, p0 := orthogonalSweepProfileWithHole(t)
	solidDoc := decad.New()
	solid, err := solidDoc.Sweep(t.Context(), s0, p0, orthogonalSweepPath(t))
	require.NoError(t, err)
	require.Len(t, solid.Lumps(), 1)

	s1, p1 := orthogonalSweepProfileWithHole(t)
	doc := decad.New()
	sheet, err := doc.Sweep(t.Context(), s1, p1, orthogonalSweepPath(t), decad.WithSurfaceResult())
	require.NoError(t, err)
	require.Len(t, sheet.Lumps(), 2)
	for _, l := range sheet.Lumps() {
		require.Len(t, l.Shells(), 1)
		require.False(t, l.Shells()[0].IsVoid())
		require.True(t, l.Shells()[0].IsOpen())
	}
}

// TestSurfaceSweepCompositeRolesResolveThroughFaceCreatedBy checks the
// composite assembler's own rewriteCompositeSideRoles keeps each span's index
// resolvable after cap omission: every wall's side(k,i,j) role still names
// exactly the face it came from, for every span index the three-span path
// visits, while CapStart/CapEnd mint refs that match nothing.
func TestSurfaceSweepCompositeRolesResolveThroughFaceCreatedBy(t *testing.T) {
	t.Parallel()
	s, p := orthogonalSweepProfile(t)
	doc := decad.New()
	sheet, err := doc.Sweep(t.Context(), s, p, orthogonalSweepPath(t), decad.WithSurfaceResult())
	require.NoError(t, err)

	seenSpan := map[string]bool{}
	for _, face := range sheet.Faces() {
		origins := face.Origins()
		require.Len(t, origins, 1)
		matched, err := decad.Faces(decad.FaceCreatedBy(origins[0])).Exactly(1).SelectFaces(sheet)
		require.NoError(t, err)
		require.Same(t, face, matched[0])
		for _, span := range []string{"side(0,", "side(1,", "side(2,"} {
			if strings.HasPrefix(origins[0].Role, span) {
				seenSpan[span] = true
			}
		}
	}
	require.Len(t, seenSpan, 3, "every one of the three spans must keep a resolvable role")

	_, err = decad.Faces(decad.FaceCreatedBy(decad.CapStart(sheet))).SelectFaces(sheet)
	require.ErrorIs(t, err, decad.ErrNoMatch)
	_, err = decad.Faces(decad.FaceCreatedBy(decad.CapEnd(sheet))).SelectFaces(sheet)
	require.ErrorIs(t, err, decad.ErrNoMatch)
}

// TestSurfaceSweepStaysASheetThroughPlacement is §4.2's Placed/Duplicate/
// PlacedCopy claim across all three build paths: each re-evaluates the
// payload and reproduces the sheet with no further code.
func TestSurfaceSweepStaysASheetThroughPlacement(t *testing.T) {
	t.Parallel()

	cases := []struct {
		name  string
		build func(t *testing.T) (*decad.Document, *decad.Body)
	}{
		{"line", func(t *testing.T) (*decad.Document, *decad.Body) {
			s, p := plateSketch(t)
			doc := decad.New()
			b, err := doc.Sweep(t.Context(), s, p, sweepLinePath(t), decad.WithSurfaceResult())
			require.NoError(t, err)
			return doc, b
		}},
		{"arc", func(t *testing.T) (*decad.Document, *decad.Body) {
			s, p := plateSketch(t)
			doc := decad.New()
			b, err := doc.Sweep(t.Context(), s, p, sweepArcPath(t), decad.WithSurfaceResult())
			require.NoError(t, err)
			return doc, b
		}},
		{"composite", func(t *testing.T) (*decad.Document, *decad.Body) {
			s, p := orthogonalSweepProfile(t)
			doc := decad.New()
			b, err := doc.Sweep(t.Context(), s, p, orthogonalSweepPath(t), decad.WithSurfaceResult())
			require.NoError(t, err)
			return doc, b
		}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			_, sheet := tc.build(t)
			wantFree, err := decad.Edges(decad.Free()).SelectEdges(sheet)
			require.NoError(t, err)
			n := len(wantFree)

			motion, err := r3.Translation(r3.NewVec(50, 0, 0))
			require.NoError(t, err)
			placed, err := sheet.Placed(t.Context(), motion)
			require.NoError(t, err)
			requireSweepSheetWithFreeEdges(t, placed, n)

			dup, err := placed.Duplicate(t.Context())
			require.NoError(t, err)
			requireSweepSheetWithFreeEdges(t, dup, n)

			copyMotion, err := r3.Translation(r3.NewVec(0, 50, 0))
			require.NoError(t, err)
			copied, err := dup.PlacedCopy(t.Context(), copyMotion)
			require.NoError(t, err)
			requireSweepSheetWithFreeEdges(t, copied, n)
		})
	}
}

func requireSweepSheetWithFreeEdges(t *testing.T, b *decad.Body, n int) {
	t.Helper()
	require.Equal(t, decad.BodySheet, b.Kind())
	free, err := decad.Edges(decad.Free()).Exactly(n).SelectEdges(b)
	require.NoError(t, err)
	require.Len(t, free, n)
}

// TestSurfaceSweepTessellationRefused is docs/sweep-design.md's Table D row
// D2: every Sweep body, sheet or solid, stages tessellation until the
// shared-span tessellator lands — unlike a surface-extruded prism's, which
// already tessellates.
func TestSurfaceSweepTessellationRefused(t *testing.T) {
	t.Parallel()
	s, p := plateSketch(t)
	doc := decad.New()
	sheet, err := doc.Sweep(t.Context(), s, p, sweepLinePath(t), decad.WithSurfaceResult())
	require.NoError(t, err)

	_, err = sheet.Tessellate(t.Context(), units.Millimeters(0.1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

// TestSurfaceSweepShellOpenAgreesWithFreeEdgeDerivation catches a builder that
// mints a free edge and forgets the Shell.open flag — or marks a shell open
// with none — across all three build paths and each one's solid control.
func TestSurfaceSweepShellOpenAgreesWithFreeEdgeDerivation(t *testing.T) {
	t.Parallel()

	cases := []struct {
		name  string
		build func(t *testing.T, opts ...decad.SweepOption) *decad.Body
	}{
		{"line", func(t *testing.T, opts ...decad.SweepOption) *decad.Body {
			s, p := plateSketch(t)
			doc := decad.New()
			b, err := doc.Sweep(t.Context(), s, p, sweepLinePath(t), opts...)
			require.NoError(t, err)
			return b
		}},
		{"arc", func(t *testing.T, opts ...decad.SweepOption) *decad.Body {
			s, p := plateSketch(t)
			doc := decad.New()
			b, err := doc.Sweep(t.Context(), s, p, sweepArcPath(t), opts...)
			require.NoError(t, err)
			return b
		}},
		{"composite", func(t *testing.T, opts ...decad.SweepOption) *decad.Body {
			s, p := orthogonalSweepProfile(t)
			doc := decad.New()
			b, err := doc.Sweep(t.Context(), s, p, orthogonalSweepPath(t), opts...)
			require.NoError(t, err)
			return b
		}},
	}
	for _, tc := range cases {
		t.Run(tc.name+"/sheet", func(t *testing.T) {
			t.Parallel()
			requireOpenAgreesWithFreeEdges(t, tc.build(t, decad.WithSurfaceResult()))
		})
		t.Run(tc.name+"/solid", func(t *testing.T) {
			t.Parallel()
			requireOpenAgreesWithFreeEdges(t, tc.build(t))
		})
	}
}
